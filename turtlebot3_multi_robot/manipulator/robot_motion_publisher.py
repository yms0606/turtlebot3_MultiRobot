import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class RobotMotionPublisher(Node):
    def __init__(self):
        super().__init__('robot_motion_publisher')
        
        # Publisher for joint control
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        self.timer = self.create_timer(1.0, self.run_motion_sequence)  # Timer for motion sequence
        self.current_step = 0
        self.done = False

        self.joint_positions = [
            [0.0, -0.017, 0.070, -0.087],      
            [1.535, 0.0, 0.0, 0.035],          
            [1.500, 1.466, -0.942, -0.645],   
            [1.535, 0.0, 0.0, 0.035],         
            [-1.605, 0.052, -0.052, 0.035],    
            [-1.605, 0.802, -0.610, -0.174],  
            [0.0, -0.017, 0.070, -0.087]      
        ]

    def publish_joints(self, positions, delay=5):
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = delay
        point.time_from_start.nanosec = 0
        msg.points = [point]
        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published Joint Positions: {positions}, Delay: {delay} seconds')
        time.sleep(delay + 1)  # Delay before moving to the next command

    def run_motion_sequence(self):
        if self.done:
            return

        sequence = [
            {'type': 'joints', 'data': self.joint_positions[0], 'delay': 2},  # Step 1: Initial Position
            {'type': 'joints', 'data': self.joint_positions[1], 'delay': 7},  # Step 2
            {'type': 'joints', 'data': self.joint_positions[2], 'delay': 5},  # Step 3
            {'type': 'joints', 'data': self.joint_positions[3], 'delay': 8},  # Step 4: Back to Step 2
            {'type': 'joints', 'data': self.joint_positions[4], 'delay': 8},  # Step 5
            {'type': 'joints', 'data': self.joint_positions[5], 'delay': 8},  # Step 6
            {'type': 'joints', 'data': self.joint_positions[6], 'delay': 8}   # Step 7: Return to Initial Position
        ]

        step = sequence[self.current_step]
        if step['type'] == 'joints':
            self.publish_joints(step['data'], step['delay'])

        self.current_step += 1
        if self.current_step >= len(sequence):
            self.done = True
            self.get_logger().info('Motion sequence completed!')


def main(args=None):
    rclpy.init(args=args)
    node = RobotMotionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
