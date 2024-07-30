#!/usr/bin/env python3

from openai import OpenAI

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Twist

from tf_transformations import euler_from_quaternion, quaternion_from_euler

client = OpenAI()

ai_system = ("You are model to compute a safe control commands for a mobile robot in a static environment, endowed with LiDAR.\n" + 
             "The control commands must be: linear velocities along x and y axis (m/s), and angular velocity (rad/s) around z axis.\n" +
             "You know your actual pose, your goal pose and the data given form the laser scanner.")
print(ai_system)

ai_user = ("Given the time step, thecurrent pose, the goal pose and the laser data of a mobile robot in a static environment, " +
           "you must compute the control commands as linear velocities along x and y axis (m/s), and angular velocity (rad/s) around z axis.\n" +
           "Return the control commands as 3 variables: linear_x linear_y angular_z\n" +
           "Return the result without any explanation as: linear_x linear_y angular_z")
print(ai_user)

class OpenAIControl(Node):

    def __init__(self):
        super().__init__('open_ai_control')
        
        # Private variables
        
        # Odometry
        self.odom_x = None
        self.odom_y = None
        self.odom_theta = None
        self.odom = (self.odom_x, self.odom_y, self.odom_theta)
        self.x_err = 0.5
        self.y_err = 0.5
        self.theta_err = 1
        
        # Goal
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        self.goal = (self.goal_x, self.goal_y, self.goal_theta)
        self.goal_prev = None
        
        # Laser scanner
        self.ranges = None
        
        # Costmap 
        self.costmap = None
        
        # Time step
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
        
        # Get costmap
        self.subscription_costmap = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.costmap_callback,
            10
        )
        # Publisher for the path
        self.publisher_path = self.create_publisher(Path, "/plan", 10)
        
        # Publisher for the control
        self.publisher_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        
    def ranges_callback(self, msg):
        self.ranges = msg.data
        #self.get_logger().info("[Ranges] I heard new values")
        # TODO: check if still receiving data
        
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        #z = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (_, _, self.odom_theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        self.odom = (self.odom_x, self.odom_y, self.odom_theta)
        
        if self.goal != (None, None, None) and self.goal_prev != self.goal:
            if abs(self.odom[0]-self.goal[0]) < self.x_err and abs(self.odom[1]-self.goal[1]) < self.x_err and abs(self.odom[2]-self.goal[2]) < self.theta_err:
                print("Goal reached!")
                self.goal_prev = self.goal # Update old goal
            
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
        
        (_, _, theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        if self.goal_x != x or self.goal_y != y or self.goal_theta != theta:
            self.goal_x = x
            self.goal_y = y
            self.goal_theta = theta
            self.goal = (x, y, theta) # Update new goal
            self.get_logger().info("[Goal] I heard new goal: (" + str(x) + ", " + str(y) + ", " + str(theta) + ")")

        if self.odom != None and self.ranges != None:
            print("[Odom] Current pose: (" + str(self.odom[0]) + ", " + str(self.odom[1]) + ", " + str(self.odom[2]) + ")")
            global ai_user
            #print("System = \n" + str(ai_system) + "\n")
            
            # Add goal and pose information to the user
            self.ai_user = ai_user + ("Here the updated data: " +
                                "Time step: " + str(self.time_step) + " s\n" +
                                "Goal: (" + str(self.goal[0]) + ", " + str(self.goal[1]) + ", " + str(self.goal[2]) + ")\n" +
                                "Pose: (" + str(self.odom[0]) + ", " + str(self.odom[1]) + ", " + str(self.odom[2]) + ")\n" +
                                "Ranges: ("
                                )
            
            # Add ranges information to the user
            for i in range(len(self.ranges)):
                self.ai_user = self.ai_user + str(self.ranges[i]) + ", "
            self.ai_user = self.ai_user + ")\n"
            
            #print("Request = \n" + str(ai_user) + "\n")
            
            start = time.time()

            """
            completion = client.chat.completions.create(
                model="gpt-3.5-turbo",
                #model="gpt-4o-mini",
                messages=[
                        #{"role": "system", "content": ai_system},
                        {"role": "user", "content": self.ai_user}
                    ]
            )

            cmd_vel = completion.choices[0].message.content
            print("Controls (x, y, theta) = " + cmd_vel)
            
            # Clear the message
            #cmd_vel = completion.choices[0].message.content.replace("[", "").replace("]", "").replace(" ", "").split(",")
            if "," in cmd_vel:
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
            """

    def costmap_callback(self, msg):
        if self.goal != (None, None, None) and self.odom != (None, None, None):
            if self.goal_prev != self.goal and self.costmap != msg.data: 
                self.costmap = msg.data
                #print(self.costmap)
                
                path = Path()
                path.header.frame_id = 'map'
                #path.header.stamp = self.get_clock().now().to_msg()
                
                #pose.header.stamp = self.get_clock().now().to_msg()
                    
                steps = 100
                for i in range(steps):
                    waypoint = PoseStamped()
                    waypoint.header.frame_id = 'map'
                    waypoint.pose.position.x = self.odom[0] + (self.goal[0]-self.odom[0]) / steps * i
                    waypoint.pose.position.y = self.odom[1] + (self.goal[1]-self.odom[1]) / steps * i
                    
                    path.poses.append(waypoint)
                
                # Overwrite last waypoint
                """
                waypoint = PoseStamped()
                waypoint.header.frame_id = ''
                waypoint.pose.position.x = self.goal[0]
                waypoint.pose.position.y = self.goal[1]
                
                (qx, qy, qz, qw) = quaternion_from_euler(0.0, 0.0, self.odom_theta)
                
                waypoint.pose.orientation.x = qx
                waypoint.pose.orientation.y = qy
                waypoint.pose.orientation.z = qz
                waypoint.pose.orientation.w = qw
                
                path.poses.append(waypoint)
                """
                    
                """
                for i in range(len(self.costmap)):
                    waypoint = PoseStamped()
                    waypoint.header.frame_id = ''
                    #pose.header.stamp = self.get_clock().now().to_msg()
                    
                    waypoint.pose.position.x = (self.goal_x)
                    waypoint.pose.position.y = self.costmap[i]
                    
                    path.poses.append(waypoint)
                """
                
                self.publisher_path.publish(path)
                #print(path)
        
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