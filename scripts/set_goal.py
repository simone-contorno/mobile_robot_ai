#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler

class SetGoal(Node):

    def __init__(self):
        super().__init__('set_goal')

        # Declare parameters
        self.declare_parameter('x', -1.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('theta', 0.0)
        
        # Get parameters
        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.theta = self.get_parameter('theta').get_parameter_value().double_value
        
        # Create the goal
        self.msg = Pose()
        self.msg.position.x = self.x
        self.msg.position.y = self.y
        self.msg.position.z = 0.0
        (qx, qy, qz, qw) = quaternion_from_euler(0.0, 0.0, self.theta)
        self.msg.orientation.x = qx
        self.msg.orientation.y = qy
        self.msg.orientation.z = qz
        self.msg.orientation.w = qw
        
        # Create the publisher
        self.publisher_ = self.create_publisher(Pose, "/goal", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        self.publisher_.publish(self.msg)
        self.get_logger().info("Publishing: (" + str(self.x) + ", " + str(self.y) + ", " + str(self.theta) + ")")

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

	# Create the node instance
    minimal_publisher = SetGoal()

	# Start the callback function
    rclpy.spin(minimal_publisher)
    
    # Destroy the node
    minimal_publisher.destroy_node()
    
    # Terminate 
    rclpy.shutdown()

if __name__ == '__main__':
    main()