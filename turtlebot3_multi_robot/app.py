import rclpy
from rclpy.node import Node
from flask import Flask, render_template, jsonify, Response, request, send_file
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import threading
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import Image

base_image = cv2.imread('static/images/warehouse.png',cv2.IMREAD_UNCHANGED)
ros_node = None
app = Flask(__name__)


# ROS 2 노드
class TurtleBotController(Node):
    def __init__(self):
        super().__init__("turtlebot_controller")

        self.cart_status = "대기 중"
        self.mo_ma_status = "대기 중"
        self.cart_position = {"x": 0, "y": 0}
        self.mo_ma_position = {"x": 3.7, "y": -3.5}

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.bridge = CvBridge()
        self.latest_frame = None

        # 카트의 포지션
        self.sub_pose_cart = self.create_subscription(PoseWithCovarianceStamped,'/cart/amcl_pose',self.cart_pose_callback,10)
        # 모바일 매니퓰레이터 포지션
        self.sub_pose_moma = self.create_subscription(PoseWithCovarianceStamped,'/moma/amcl_pose',self.moma_pose_callback,10)
        # 캠에서 받아오는 이미지
        self.sub_cam_img = self.create_subscription(Image,'/cart/camera/image_raw',self.cam_callback,10)

        # 수동 조작을 위한 publisher
        self.pub_vel = self.create_publisher(Twist,'/cart/cmd_vel',10)
        # 자동 조작을 위한 publisher
        self.pub_pose = self.create_publisher(PoseStamped,'/cart/goal_pose',10)
        # 자동 조작을 위한 publisher
        self.pub_moma_pose = self.create_publisher(PoseStamped,'/moma/goal_pose',10)

    def cam_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, jpeg = cv2.imencode('.jpg', cv_image)
        self.latest_frame = jpeg.tobytes()

    def cart_pose_callback(self,msg):
        self.cart_position['x'] = msg.pose.pose.position.x
        self.cart_position['y'] = msg.pose.pose.position.y
        self.get_logger().info(f"{msg.pose.pose.position.x}")

    def moma_pose_callback(self,msg):
        self.mo_ma_position['x'] = msg.pose.pose.position.x
        self.mo_ma_position['y'] = msg.pose.pose.position.y


    def update_img(self):
        per_pixel = 0.05
        global base_image
        img = base_image.copy()

        cart_x = -(self.cart_position['y'] - 0.39)
        cart_y = -(self.cart_position['x'] - 4.24)
        cart_map_x = round(cart_x/per_pixel) + 8
        cart_map_y = round(cart_y/per_pixel) + 4

        moma_x = -(self.mo_ma_position['y'] - 0.39)
        moma_y = -(self.mo_ma_position['x'] - 4.24)
        moma_map_x = round(moma_x/per_pixel) + 8
        moma_map_y = round(moma_y/per_pixel) + 4

        cv2.rectangle(img, (cart_map_x,cart_map_y),(cart_map_x+1,cart_map_y+1),(0,0,255,255),-1)
        cv2.rectangle(img, (moma_map_x,moma_map_y),(moma_map_x+1,moma_map_y+1),(255,0,0,255),-1)

        return img
    
    def send_cmd_vel(self, msg):
        if msg == 'up':
            self.linear_x += 0.01
        elif msg == 'down':
            self.linear_x -= 0.01
        elif msg == 'left':
            self.angular_z += 0.01
        elif msg == 'right':
            self.angular_z -= 0.01
        elif msg == 'stop':
            self.linear_x = 0.0
            self.angular_z = 0.0

        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_x
        cmd_vel.angular.z = self.angular_z
        self.pub_vel.publish(cmd_vel)
    
    def send_goal(self,x,y):
        msg = PoseStamped()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.header.frame_id='map'
        self.pub_pose.publish(msg)

        msg_moma = PoseStamped()
        msg_moma.pose.position.x = float(x)
        msg_moma.pose.position.y = float(y)
        msg_moma.header.frame_id = 'map'
        self.pub_moma_pose.publish(msg_moma)



@app.route("/")
def index():
    # 아이템 리스트
    items = [
        {"name": "Item 1", "x": 2.5, "y": -2},
        {"name": "Item 2", "x": 2.7, "y": -0.42},
        {"name": "Item 3", "x": 15, "y": 15},
    ]
    return render_template("index.html", items=items)

@app.route("/get_state", methods=["GET"])
def get_state():
    # 현재 ROS 2 노드 상태 반환
    return jsonify({
        "cart": ros_node.cart_status,
        "mo_ma": ros_node.mo_ma_status,
        "cart_position": ros_node.cart_position,
        "mo_ma_position": ros_node.mo_ma_position,
    })

@app.route("/image")
def image():
    #global img
    img = ros_node.update_img()
    _, encoded_img = cv2.imencode('.png',img)
    return Response(encoded_img.tobytes(), mimetype=('image/png'))

@app.route('/send_vel', methods=['POST'])
def send_vel():
    data = request.json
    ros_node.send_cmd_vel(data['message'])
    return "success", 200

@app.route('/send_goal',methods=['POST'])
def send_goal():
    data = request.json
    ros_node.send_goal(data['pose_x'],data['pose_y'])
    return "success", 200

@app.route('/cam_feed')
def cam_feed():
    def generate_frames():
        while True:
            if ros_node.latest_frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + ros_node.latest_frame + b'\r\n')
            time.sleep(0.1)
    return Response(generate_frames(),mimetype='multipart/x-mixed-replace; boundary=frame')

# def update_img():
#     cartx = 96
#     carty = 92
#     global img

#     img = base_image.copy()
#     cv2.rectangle(img, (cartx,carty),(cartx+1,carty+1),(0,0,255,255),-1)
    
    #while True:
    #    img = base_image.copy()
    #    cv2.rectangle(img, (cartx,carty),(cartx+1,carty+1),(0,0,255,255),-1)
        # print(cartx)
        # cv2.rectangle(img, (cartx,carty),(cartx+1,carty+1),(0,0,255,255),-1)
        # cartx +=1
        # time.sleep(1)
def node_start():
    global ros_node

    rclpy.init()
    ros_node = TurtleBotController()
    rclpy.spin(ros_node)
    rclpy.shutdown()
    ros_node.destroy_node()

if __name__ == "__main__":
    threading.Thread(target=node_start, daemon=True).start()
    threading.Thread(target=cam_feed, daemon=True).start()
    app.run(port='8081', threaded=True)  # Flask 실행 (use_reloader=False: ROS 2와 충돌 방지)


# (4.24 , 0.393)   (8,4 ) 좌상단
# (-0.37, -4.25)  (96, 92) 우하단
# 가로 세로 4.5 4.5 88 88
# per_pixel = 0.05
# pose_x = -(x - 4.24)
# pose_y = -(y - 0.39)
# map_x = round(pose_x/per_pixel)
# map_y = round(pose_y/per_pixel)