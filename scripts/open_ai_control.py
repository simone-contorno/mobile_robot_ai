#!/usr/bin/env python3

from openai import OpenAI

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped 

from tf_transformations import euler_from_quaternion

from nav2_msgs.action import NavigateToPose

"""
start = time.time()

client = OpenAI()

completion = client.chat.completions.create(
  model="gpt-3.5-turbo",
  #model="gpt-4o-mini",
  messages=[
    {"role": "system", "content": "You are a poetic assistant, skilled in explaining complex programming concepts with creative flair."},
    {"role": "user", "content": "Compose a poem that explains the concept of recursion in programming."}
  ]
)

print(completion.choices[0].message)

end = time.time()
diff = end - start

# Print time in milliseconds
print("Time = " + str(diff * 1000) + " ms")
"""

class OpenAIControl(Node):

    def __init__(self):
        super().__init__('open_ai_control')
        
		# Get ranges values
        self.subscription_ranges= self.create_subscription(
            Float32MultiArray,
            'ranges',
            self.ranges_callback,
            10)
        
        # Get odom values
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Get goal values
        self.subscription_goal = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10)
         
    def ranges_callback(self, msg):
        self.get_logger().info("[Ranges] I heard new  " + str(msg.data.size()) + " values")
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (r, p, y) = euler_from_quaternion([qx, qy, qz, qw])
    
        self.get_logger().info("[Odom] I heard new pose: (" + str(x) + ", " + str(y) + ", " + str(z) + ", " + str(y) + ")")

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (r, p, y) = euler_from_quaternion([qx, qy, qz, qw])
        
        self.get_logger().info("[Goal] I heard new goal: (" + str(x) + ", " + str(y) + ", " + str(z) + ", " + str(y) + ")")

def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

	# Create the node instance
    minimal_publisher = OpenAIControl()

	# Start the callback function
    rclpy.spin(minimal_publisher)
    
    # Destroy the node
    minimal_publisher.destroy_node()
    
    # Terminate 
    rclpy.shutdown()

if __name__ == '__main__':
    main()