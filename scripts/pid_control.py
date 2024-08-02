#!/usr/bin/env python3

from mobile_robot_ai import utils, controls

import os 
import sys 
import time
import configparser
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist

from tf_transformations import euler_from_quaternion

### Parse configuration file ###
script_directory = os.path.dirname(os.path.abspath(sys.argv[0])) 
parser = configparser.ConfigParser()
parser.read_file(open(script_directory + "/control_config.txt"))

# Get PID gains
Kp_v = float(parser.get('PID', 'Kp_v'))
Kp_w = float(parser.get('PID', 'Kp_w'))

Ki_v = float(parser.get('PID', 'Ki_v'))
Ki_w = float(parser.get('PID', 'Ki_w'))

Kd_v = float(parser.get('PID', 'Kd_v'))
Kd_w = float(parser.get('PID', 'Kd_w'))

# Get GOAL params
goal_threshold = float(parser.get('GOAL', 'goal_threashold'))

# Get CONTROL params
control_mode = int(parser.get('CONTROL', 'control_mode'))
    
print(f"""
Configuration parameters:
Kp_v: {Kp_v}
Kp_w: {Kp_w}

Ki_v: {Ki_v}
Ki_w: {Ki_w}

Kd_v: {Kd_v}
Kd_w: {Kd_w}   

goal_threshold: {goal_threshold}
""")

if control_mode == 0:
    print(f"control_mode: {control_mode} (PID)")

if control_mode == 1:
    print(f"control_mode: {control_mode} (OpenAI)\n")
    
### ROS 2 Class ###

class Control(Node):

    def __init__(self):
        super().__init__('pid_control')
        
        ### Private variables ###
        
        # Iteration counter
        self.iter = 0
        
        # Odometry
        self.odom = (None, None, None)
        self.offset_odom = 0.05
        
        # Goal
        self.new_goal = False
        self.goal = (None, None, None)
        self.goal_prev = self.goal
        
        # Path
        self.path = None
        
        ### Subscribers ###
        
        # Odometry
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Goal
        self.subscription_goal_pose = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10)
    
        # Path waypoints
        self.subscription_plan = self.create_subscription(
            Path,
            'plan',
            self.plan_callback,
            10
        )

        #### Publishers ###
        
        # Control commands
        self.publisher_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
    
    ### Callbacks ###
    
    # Goal
    def goal_pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        (_, _, theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        # Update new goal
        if self.goal[0] != x or self.goal[1] != y or self.goal[2] != theta:
            self.goal = (x, y, theta) 
            self.new_goal = True
            self.get_logger().info("[Goal] I heard new goal: (" + str(x) + ", " + str(y) + ", " + str(theta) + ")")
    
    # Path waypoints
    def plan_callback(self, msg):        
        self.get_logger().info("[Plan] I heard new plan of " + str(len(msg.poses)) + " waypoints")

        # Update the path waypoints
        self.path = []
        for i in range(len(msg.poses)):
            x = msg.poses[i].pose.position.x
            y = msg.poses[i].pose.position.y
            
            qx = msg.poses[i].pose.orientation.x
            qy = msg.poses[i].pose.orientation.y
            qz = msg.poses[i].pose.orientation.z
            qw = msg.poses[i].pose.orientation.w
            
            (_, _, theta) = euler_from_quaternion([qx, qy, qz, qw])
            
            self.path.append((x, y, theta))
    
    # Odometry and Control commands 
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (_, _, theta) = euler_from_quaternion([qx, qy, qz, qw])

        self.odom = (x, y, theta)
        
        # Check if the goal has been reached
        if self.goal != (None, None, None) and self.odom != (None, None, None) and self.goal_prev != self.goal:
            if math.sqrt(math.pow(self.odom[0]-self.goal[0], 2) + math.pow(self.odom[1]-self.goal[1], 2)) < goal_threshold:
                self.get_logger().info("[Odom] Goal reached!")
                self.goal_prev = self.goal # Update old goal
                
                # Set velocities to 0
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                
                self.publisher_cmd.publish(cmd)

            if self.path != None:
                
                print("--------------------------------------------------")
                print("Iteration " + str(self.iter))
                self.iter += 1
                print("--------------------------------------------------")
                
                start = time.time()
                
                ### Compute the next point on the path to reach ###
                
                next_point = utils.compute_next_point(self.path, self.odom, goal_threshold)
                print("Next point on the path: " + str(next_point) + "\n")

                ### Compute the PID control ###
                
                (v, w) = controls.compute_control_commands(self.odom, self.path[next_point], Kp_v, Kp_w, Ki_v, Ki_w, Kd_v, Kd_w, control_mode=control_mode)
            
                print(f"""
                      v_x = {v[0]} [m/s]
                      v_y = {v[0]} [m/s]
                      w_w = {w} [rad/s]
                      """)   
                    
                ### Publish control commands ###
                
                cmd = Twist()
                cmd.linear.x = v[0]
                cmd.linear.y = v[1]
                cmd.angular.z = w
                
                self.publisher_cmd.publish(cmd)

                ### Compute control commands computation time ###
                
                end = time.time()
                diff = end - start
                print("Time = " + str(diff) + " [s]\n")    
                
def main(args=None):
    # Initialize the node
    rclpy.init(args=args)

	# Create the node instance
    minimal_publisher = Control()

	# Start the callback function
    rclpy.spin(minimal_publisher)
    
    # Destroy the node
    minimal_publisher.destroy_node()
    
    # Terminate 
    rclpy.shutdown()

if __name__ == '__main__':
    main()