#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class WheelJointPublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [0.0, 0.0]  # Default positions
        msg.velocity = [0.0, 0.0]  # Default velocities
        msg.effort = [0.0, 0.0]    # Default efforts
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelJointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
