#!/usr/bin/env python3

from mobile_robot_ai import utils
from mobile_robot_ai.ai_prompts import *

import os 
import time
import math
import signal

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import Log

from tf_transformations import euler_from_quaternion

# Get GOAL params
goal_threshold = utils.get_config_param("goal", "goal_threshold")

# Get CONTROL params
control_method = utils.get_config_param("control", "control_method")

# Get ML params
dataset_path = utils.get_config_param('ml', 'dataset_path')
dataset = utils.get_config_param('ml', 'dataset')

# If the Linear Regression is enabled, dataset is not created
if control_method == 2:
    dataset = False

if dataset == True:
    print("Dataset enabled")
    
    # Get dataset path
    ds_path = dataset_path + "/datasets"
    
    # Check if the Datasets folder exists
    if not os.path.exists(ds_path):
        os.makedirs(ds_path)
    
    # Get current date and time
    date = time.strftime("%Y-%m-%d_%H-%M-%S")

    # Creat sub-folder
    ds_path = ds_path + "/" + date + "_dataset"
    if not os.path.exists(ds_path):
        os.makedirs(ds_path)
    
    # Create error on x file
    ds_err_x = open(ds_path + '/error_x.csv', 'w')
    ds_err_x.write('Error,Control\n')
    
    # Create error on y file
    ds_err_y = open(ds_path + '/error_y.csv', 'w')
    ds_err_y.write('Error,Control\n')
    
    # Create error on theta file
    ds_err_theta = open(ds_path + '/error_theta.csv', 'w') 
    ds_err_theta.write('Error,Control\n')

# Print configuration parameters
print(f"goal_threshold: {goal_threshold}")

### Log prefix ###
log_prefix = "mobile_robot_ai/"

### ROS 2 Class ###

class Control(Node):

    def __init__(self):
        super().__init__('control_handler')
        
        ### Private variables ###
        
        # Controls
        self.v_x = None
        self.v_y = None
        self.w_z = None
        
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
        
        # Control commands
        self.subscription_cmd = self.create_subscription(
            Twist,
            'cmd_vel_handler',
            self.cmd_vel_callback,
            10
        )

        #### Publishers ###
        
        # Next waypoint
        self.publisher_next_wp = self.create_publisher(PoseStamped, 'next_wp', 10)
        
        # Control commands
        self.publisher_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Logs
        self.publisher_log = self.create_publisher(Log, "/rosout", 10)
        
        # Manage CTRL+C command
        signal.signal(signal.SIGINT, self.signal_handler)
    
    ### Callbacks ###
    
    # Controls
    def cmd_vel_callback(self, msg):
        self.v_x = msg.linear.x
        self.v_y = msg.linear.y
        self.w_z = msg.angular.z
        
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
                
                # Reset local velocities
                self.v_x = 0.0
                self.v_y = 0.0
                self.w_z = 0.0
                
                # Stop the robot
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                
                self.publisher_cmd.publish(cmd)

            if self.path != None and self.goal_prev != self.goal: 
                
                start = time.time()
                
                ### Compute the next point on the path to reach ###
                
                next_point = self.compute_next_point(self.path, self.odom, goal_threshold)
                
                if next_point > self.path_point:
                    self.path_point = next_point
                    print("Next point on the path: " + str(next_point))

                # Get theta and define next waypoint
                theta_target = math.atan2(self.path[self.path_point][1]-self.odom[1], self.path[self.path_point][0]-self.odom[0])
                waypoint = (self.path[self.path_point][0], self.path[self.path_point][1], theta_target)
                
                # Publish next waypoint
                next_wp = PoseStamped()
                next_wp.header.stamp = self.get_clock().now().to_msg()
                next_wp.header.frame_id = 'map'
                next_wp.pose.position.x = waypoint[0]
                next_wp.pose.position.y = waypoint[1]
                next_wp.pose.orientation.z = waypoint[2]
                self.publisher_next_wp.publish(next_wp)
                
                ### Get the control inputs ###
                if self.v_x != None and self.v_y != None and self.w_z != None:
                    
                    print("--------------------------------------------------")
                    print("Iteration " + str(self.iter))
                    self.iter += 1
                    print("--------------------------------------------------\n")

                    print(f"""
                        odom_x = {self.odom[0]} [m]
                        odom_y = {self.odom[1]} [m]
                        odom_theta = {self.odom[2]} [rad]
                        
                        goal_x = {waypoint[0]} [m]
                        goal_y = {waypoint[1]} [m]
                        goal_theta = {waypoint[2]} [rad]
                        """)
                    
                    print(f"""
                        v_x = {self.v_x} [m/s]
                        v_y = {self.v_y} [m/s]
                        w_z = {self.w_z} [rad/s]
                        """)   
                        
                    ### Publish control commands ###
                    
                    cmd = Twist()
                    cmd.linear.x = self.v_x
                    cmd.linear.y = self.v_y
                    cmd.angular.z = self.w_z
                    
                    if self.goal_prev != self.goal:
                        self.publisher_cmd.publish(cmd)

                    ### Compute control commands computation time ###
                    
                    end = time.time()
                    diff = end - start
                    print("Time = " + str(diff) + " [s]\n")    
                    
                    ### Publish logs ###
                    log = Log()
                    log.name = log_prefix + "v_x"
                    log.msg = str(self.v_x)
                    self.publisher_log.publish(log)
                    
                    log = Log()
                    log.name = log_prefix + "v_y"
                    log.msg = str(self.v_y)
                    self.publisher_log.publish(log)
                    
                    log = Log()
                    log.name = log_prefix + "w_z"
                    log.msg = str(self.w_z)
                    self.publisher_log.publish(log)
                    
                    # Update dataset
                    if dataset == True:
                        e_x = waypoint[0] - self.odom[0]
                        e_y = waypoint[1] - self.odom[1]
                        e_theta = math.atan2(math.sin(waypoint[2] - self.odom[2]), math.cos(waypoint[2] - self.odom[2]))
                        
                        ds_err_x.write(str(e_x) + "," + str(self.v_x) + "\n")
                        ds_err_y.write(str(e_y) + "," + str(self.v_y) + "\n")
                        ds_err_theta.write(str(e_theta) + "," + str(self.w_z) + "\n")
    
    # Compute the next point on the path to reach 
    def compute_next_point(self, path, odom, threshold):     
        # 1. Find the closest point on the path to the actual position
        next_point = 0
        min_dist = ((path[0][0] - odom[0])**2 + (path[0][1] - odom[1])**2)**0.5
        for i in range(len(path)):
            dist = ((path[i][0] - odom[0])**2 + (path[i][1] - odom[1])**2)**0.5
            if dist < min_dist:
                next_point = i

        # 2. Set the next point on the path
        euler_dist = ((path[next_point][0] - odom[0])**2 + (path[next_point][1] - odom[1])**2)**0.5
        while euler_dist < threshold:
            euler_dist = ((path[next_point][0] - odom[0])**2 + (path[next_point][1] - odom[1])**2)**0.5
            if next_point < len(path)-1:
                next_point += 1
            else:
                next_point = len(path)-1
                break
            
        return next_point

    # Signal handler
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