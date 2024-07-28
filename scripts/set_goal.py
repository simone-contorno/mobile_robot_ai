#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
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
            
        """
        # Send the action
        self.feedback_callback = None # Define the function to ged on-going feedback
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.send_goal()
        """

    """
    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = float(self.x)
        goal_msg.pose.pose.position.y = float(self.y)  
        goal_msg.pose.pose.position.z = 0.0         
        goal_msg.pose.pose.orientation.w = float(self.theta)
        goal_msg.pose.header.frame_id = 'map'
        #goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info("Sending goal (x,y,theta): (" + str(self.x) + ", " + str(self.y) + ", " + str(self.theta) + ")")
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
    """
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