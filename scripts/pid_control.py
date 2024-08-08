#!/usr/bin/env python3

from mobile_robot_ai import utils, controls
from mobile_robot_ai.ai_prompts import *

import os 
import sys 
import time
import configparser
import math
import signal

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import Log

from tf_transformations import euler_from_quaternion

### Parse configuration file ###
script_directory = os.path.dirname(os.path.abspath(sys.argv[0])) 
parser = configparser.ConfigParser()
parser.read_file(open(script_directory + "/control_config.txt"))

# Get PID params
Kp_v = float(parser.get('PID', 'Kp_v'))
Kp_w = float(parser.get('PID', 'Kp_w'))
Kp = (Kp_v, Kp_w)

Ki_v = float(parser.get('PID', 'Ki_v'))
Ki_w = float(parser.get('PID', 'Ki_w'))
Ki = (Ki_v, Ki_w)

Kd_v = float(parser.get('PID', 'Kd_v'))
Kd_w = float(parser.get('PID', 'Kd_w'))
Kd = (Kd_v, Kd_w)

dt = float(parser.get('PID', 'dt'))

# Get GOAL params
goal_threshold = float(parser.get('GOAL', 'goal_threashold'))

# Get CONTROL params
control_mode = int(parser.get('CONTROL', 'control_mode'))
 
# Get AI params
ai_model = str(parser.get('AI', 'ai_model'))
ai_system = str(parser.get('AI', 'ai_system'))

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
    print(f"control_mode: {control_mode} (OpenAI)")
    print(f"AI model: {ai_model}")
    
    if ai_system in globals():
        ai_system = globals()[ai_system]
    else:
        ai_system = pid_proportional_only
        print("No valid AI system provided. PID with only proportional control will be used.")
    
    print(f"AI system: {ai_system}")

### Log prefix ###
log_prefix = "mobile_robot_ai/"

### ROS 2 Class ###

class Control(Node):

    def __init__(self):
        super().__init__('pid_control')
        
        ### Private variables ###
        
        # PID
        self.e = (0.0, 0.0, 0.0) # proportional error
        self.i = (0.0, 0.0, 0.0) # integral error
        
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
        self.path_point = 0
        
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
        
        # Logs
        self.publisher_log = self.create_publisher(Log, "/rosout", 10)
        
        # Manage CTRL+C command
        signal.signal(signal.SIGINT, self.signal_handler)
    
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
        self.get_logger().info("[Plan] I heard new plan of " + str(len(msg.poses)) + " waypoints\n")
        self.path_point = 0
        
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
                print("--------------------------------------------------\n")
                
                start = time.time()
                
                ### Compute the next point on the path to reach ###
                
                next_point = utils.compute_next_point(self.path, self.odom, goal_threshold)
                
                if next_point > self.path_point:
                    self.path_point = next_point
                    print("Next point on the path: " + str(next_point))

                ### Compute the PID control ###
                theta_target = math.atan2(self.path[self.path_point][1]-self.odom[1], self.path[self.path_point][0]-self.odom[0])
                waypoint = (self.path[self.path_point][0], self.path[self.path_point][1], theta_target)
                
                print(f"""
                      odom_x = {self.odom[0]} [m]
                      odom_y = {self.odom[1]} [m]
                      odom_theta = {self.odom[2]} [rad]
                      
                      goal_x = {waypoint[0]} [m]
                      goal_y = {waypoint[1]} [m]
                      goal_theta = {waypoint[2]} [rad]
                      """)
                
                (v, w, self.e, self.i) = controls.compute_control_commands(self.odom, waypoint, Kp, Ki, Kd, dt, self.e, self.i, control_mode=control_mode, ai_model=ai_model, ai_system=ai_system)
            
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
                
                if self.goal_prev !=  self.goal:
                    self.publisher_cmd.publish(cmd)

                ### Compute control commands computation time ###
                
                end = time.time()
                diff = end - start
                print("Time = " + str(diff) + " [s]\n")    
                
                ### Publish logs ###
                log = Log()
                log.name = log_prefix + "v_x"
                log.msg = str(v[0])
                self.publisher_log.publish(log)
                
                log = Log()
                log.name = log_prefix + "v_y"
                log.msg = str(v[1])
                self.publisher_log.publish(log)
                
                log = Log()
                log.name = log_prefix + "w_z"
                log.msg = str(w)
                self.publisher_log.publish(log)
    
    def signal_handler(self, signum, frame):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        self.publisher_cmd.publish(cmd)
        
        # Terminate 
        rclpy.shutdown()
        exit()
    
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
    #rclpy.shutdown()

if __name__ == '__main__':
    main()