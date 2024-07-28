#!/usr/bin/env python3

from openai import OpenAI

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist

from tf_transformations import euler_from_quaternion

client = OpenAI()

ai_system = ("You are model to compute a safe control commands for a mobile robot in a static environment, endowed with LiDAR.\n" + 
             "The control commands must be: linear velocities along x and y axis (m/s), and angular velocity (rad/s) around z axis.\n" +
             "You know your actual pose, your goal pose and the data given form the laser scanner.")
print(ai_system)

ai_user = ("Compute the safe trajectory to reach the goal knowing that the robot pose, the goal pose and the laser data.\n" +
           "Return the control commands sequence as 3 variables: linear_x linear_y angular_z, considering a specific time step in seconds.\n" +
           "Return the result without any explanation as: linear_x linear_y angular_z")
print(ai_user)

class OpenAIControl(Node):

    def __init__(self):
        super().__init__('open_ai_control')
        
        # Private variables
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        
        self.ranges = None
        self.pose = None
        self.goal = None
        
        self.time_step = 1
        
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
        
        # Get goal pose values
        self.subscription_goal_pose = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10)
        
        # Get goal values
        self.subscription_goal = self.create_subscription(
            Pose,
            'goal',
            self.goal_callback,
            10)
        
        # Create the publisher
        self.publisher_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        
    def ranges_callback(self, msg):
        self.ranges = msg.data
        #self.get_logger().info("[Ranges] I heard new values")
        # TODO: check if still receiving data
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (roll, pitch, theta) = euler_from_quaternion([qx, qy, qz, qw])

        self.pose = (x, y, theta)
        # TODO: Check if still receiving data
        #self.get_logger().info("[Odom] I heard new pose: (" + str(x) + ", " + str(y) + ", " + str(theta) + ")")

    def goal_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (roll, pitch, theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        self.get_logger().info("[Goal Pose] I heard new goal: (" + str(x) + ", " + str(y) + ", " + str(theta) + ")")
    
    def goal_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        (roll, pitch, theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        if self.goal_x != x or self.goal_y != y or self.goal_theta != theta:
            self.goal_x = x
            self.goal_y = y
            self.goal_theta = theta
            self.goal = (x, y, theta)
            self.get_logger().info("[Goal] I heard new goal: (" + str(x) + ", " + str(y) + ", " + str(theta) + ")")

        if self.pose != None and self.ranges != None:
            print("[Odom] Current pose: (" + str(self.pose[0]) + ", " + str(self.pose[1]) + ", " + str(self.pose[2]) + ")")
            global ai_user
            #print("System = \n" + str(ai_system) + "\n")
            
            # Add goal and pose information to the user
            self.ai_user = ai_user + ("Here your data: " +
                                "Time step: " + str(self.time_step) + " s\n" +
                                "Goal: (" + str(self.goal[0]) + ", " + str(self.goal[1]) + ", " + str(self.goal[2]) + ")\n" +
                                "Pose: (" + str(self.pose[0]) + ", " + str(self.pose[1]) + ", " + str(self.pose[2]) + ")\n" +
                                "Ranges: ("
                                )
            
            # Add ranges information to the user
            for i in range(len(self.ranges)):
                self.ai_user = self.ai_user + str(self.ranges[i]) + ", "
            self.ai_user = self.ai_user + ")\n"
            
            #print("Request = \n" + str(ai_user) + "\n")
            
            start = time.time()

            completion = client.chat.completions.create(
            model="gpt-3.5-turbo",
            #model="gpt-4o-mini",
            messages=[
                    {"role": "system", "content": ai_system},
                    {"role": "user", "content": self.ai_user}
                ]
            )

            cmd_vel = completion.choices[0].message.content
            print("Controls (x, y, theta) = " + cmd_vel)
            # Clear the message
            #cmd_vel = completion.choices[0].message.content.replace("[", "").replace("]", "").replace(" ", "").split(",")
            if ", " in cmd_vel:
                cmd_vel = cmd_vel.replace(",", "")
            if "(" in cmd_vel:
                cmd_vel = cmd_vel.replace("(", "")
            if ")" in cmd_vel:
                cmd_vel = cmd_vel.replace(")", "")
            if "[" in cmd_vel:
                cmd_vel = cmd_vel.replace("[", "")
            if "]" in cmd_vel:
                cmd_vel = cmd_vel.replace("]", "")
                
            cmd_vel = completion.choices[0].message.content.split(" ")
            print("Linear velocity x = " + cmd_vel[0] + " m/s\n" +
                  "Linear velocity y = " + cmd_vel[1] + " m/s\n" +
                  "Angular velocity z = " + cmd_vel[2] + " rad/s\n")
                  

            end = time.time()
            diff = end - start

            # Print time in milliseconds
            print("Time = " + str(diff) + " s")
            self.time_step = diff
            
            # Publish control commands
            cmd = Twist()
            cmd.linear.x = float(cmd_vel[0])
            cmd.linear.y = float(cmd_vel[1])
            cmd.angular.z = float(cmd_vel[2])
            self.publisher_cmd.publish(cmd)

        
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