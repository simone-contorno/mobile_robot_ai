#!/usr/bin/env python3

from openai import OpenAI

import time
import re
import numpy as np 
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Twist

from tf_transformations import euler_from_quaternion, quaternion_from_euler

client = OpenAI()

### PID ###

# Proportional gains
Kp_v = 0.2
Kp_w = 0.1

# Integral gains
Ki_v = 0.0
Ki_w = 0.0

# Derivative gains
Kd_v = 0.0
Kd_w = 0.0

# Constrains
v_min = -0.5
v_max = 0.5
w_min = -0.5
w_max = 0.5

dv_max = 0.3
dw_max = 3.14

# Integral gains
#Ki_x = 0.0
#Ki_y = 0.0
#Ki_theta = 0.0

# Derivative gains
#Kd_x = 0.0
#Kd_y = 0.0
#Kd_theta = 0.0

# Time step
dt = 0.8

pid_prompt = f"""
You are controlling a unicycle robot.
Using a PID controller, the robot needs to achieve the following velocities:
- Linear velocity along x-axis: v_x [m/s]
- Linear velocity along y-axis: v_y [m/s]
- Angular velocity about z-axis: w_z [rad/s]

The current positions and orientation of the robot are:
- Current x position: x_current [m]
- Current y position: y_current [m]
- Current theta orientation: theta_current [rad]

The target positions and orientation of the robot are:
- Target x position: x_target [m]
- Target y position: y_target [m]

Follow these steps:
1. Calculate the errors:
- Error in x position: diff_x = x_target - x_current
- Error in y position: diff_y = y_target - y_current
- Error in z orientation: diff_w = atan2(diff_y, diff_x) - theta_current

2. Compute the control outputs (velocities):
- Linear velocity along x-axis: v_x = {Kp_v} * diff_x 
- Linear velocity along y-axis: v_y = {Kp_v} * diff_y 
- Angular velocity about z-axis: w_z = {Kp_w} * atan2(sin(diff_w), cos(diff_w)) 

Provide the results (even 0.0 values), without any extra words and headers, in the format (only the numeric values): v_x v_y w_z diff_x diff_y diff_w
"""

ai_system = pid_prompt
print(ai_system)

class OpenAIControl(Node):

    def __init__(self):
        super().__init__('open_ai_control')
        
        # Private variables
        self.iter = 0
        
        # Odometry
        self.odom_x = None
        self.odom_y = None
        self.odom_theta = None
        self.odom = (self.odom_x, self.odom_y, self.odom_theta)
        self.x_err = 0.1
        self.y_err = 0.1
        self.theta_err = 1.0
        self.offset_odom = 0.05
        
        # Goal
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        self.goal = (self.goal_x, self.goal_y, self.goal_theta)
        self.goal_prev = None
        
        # Laser scanner
        self.ranges = None
        
        # Path
        self.compute_path = False 
        self.costmap = None
        self.path = None
        self.path_percentage = 20
        
        # Time step
        self.path_time_step = 1
        
        # PID
        self.ai_res_memory = []
        
        self.oscillation_v = 0.0
        self.oscillation_w = 0.0
        
        self.oscillation_max_v = 0.1
        self.oscillation_max_w = 0.15
        
        self.T_u_start_v = 0.0
        self.T_u_end_v = 0.0
        self.T_u_v = 0.0
        
        self.T_u_start_w = 0.0
        self.T_u_end_w = 0.0
        self.T_u_w = 0.0
        
        self.flag_otime_v = False
        self.flag_otime_w = False
        
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
    
        # Get plan
        self.subscription_plan = self.create_subscription(
            Path,
            'mobile_robot_ai/plan',
            self.plan_callback,
            10
        )
        
        # Publisher for the plan
        self.publisher_plan = self.create_publisher(Path, "/plan", 10)

        # Publisher for the control
        self.publisher_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        
    def ranges_callback(self, msg):
        self.ranges = []
        
        # Clean infinite values
        for i in range(len(msg.data)):
            if msg.data[i] != float('inf'):
                self.ranges.append(msg.data[i])
    
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
            self.get_logger().info("[Goal Pose] I heard new goal: (" + str(x) + ", " + str(y) + ", " + str(theta) + ")")
    
    def plan_callback(self, msg):
        #self.get_logger().info("[Plan] I heard new plan: " + str(len(msg.poses)) + " waypoints")

        self.path = []
        
        # Choose a step to take equidistant waypoints
        num_points = len(msg.poses) / 100 * self.path_percentage
        step = int(len(msg.poses) / num_points)
        
        # Add intermediate waypoints
        i = 0
        while i in range(len(msg.poses)):
            x = msg.poses[i].pose.position.x
            y = msg.poses[i].pose.position.y
            
            qx = msg.poses[i].pose.orientation.x
            qy = msg.poses[i].pose.orientation.y
            qz = msg.poses[i].pose.orientation.z
            qw = msg.poses[i].pose.orientation.w
            
            (_, _, theta) = euler_from_quaternion([qx, qy, qz, qw])
            
            self.path.append((x, y, theta))
            i += step
        
        # Add last waypoint
        x = msg.poses[-1].pose.position.x
        y = msg.poses[-1].pose.position.y
        
        qx = msg.poses[-1].pose.orientation.x
        qy = msg.poses[-1].pose.orientation.y
        qz = msg.poses[-1].pose.orientation.z
        qw = msg.poses[-1].pose.orientation.w
        
        (_, _, theta) = euler_from_quaternion([qx, qy, qz, qw])
            
        self.path.append((x, y, theta))
        
        # Publish the new plan
        plan = Path()
        plan.header.frame_id = 'map'
        
        for i in range(len(self.path)):
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            
            waypoint.pose.position.x = self.path[i][0]
            waypoint.pose.position.y = self.path[i][1]
            
            (qx, qy, qz, qw) = quaternion_from_euler(0.0, 0.0, self.path[i][2])
                    
            waypoint.pose.orientation.x = qx
            waypoint.pose.orientation.y = qy
            waypoint.pose.orientation.z = qz
            waypoint.pose.orientation.w = qw
            
            plan.poses.append(waypoint)
        
        self.publisher_plan.publish(plan)
        #self.get_logger().info("[Plan] Published new plan: " + str(len(plan.poses)) + " waypoints")
             
    def odom_callback(self, msg):
        global Kp_v, Kp_w
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (_, _, self.odom_theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        self.odom_x = msg.pose.pose.position.x# + self.offset_odom * np.cos(self.odom_theta)
        self.odom_y = msg.pose.pose.position.y# + self.offset_odom * np.sin(self.odom_theta)
        
        self.odom = (self.odom_x, self.odom_y, self.odom_theta)

        # Check if the goal has been reached
        if self.goal != (None, None, None) and self.goal_prev != self.goal:
            if abs(self.odom[0]-self.goal[0]) < self.x_err and abs(self.odom[1]-self.goal[1]) < self.x_err and abs(self.odom[2]-self.goal[2]) < self.theta_err:
                self.get_logger().info("[Odom] Goal reached!")
                self.goal_prev = self.goal # Update old goal
                
                # Set velocities to 0
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                
                self.publisher_cmd.publish(cmd)

        # Compute control commands
        if (self.odom != (None, None, None) and self.goal != (None, None, None) and #and self.ranges != None 
                self.path != None and self.goal_prev != self.goal):
            
            ### Compute the next point on the path to reach ###
            
            # 1. Find the closest point on the path to the actual position
            closest_point = 0
            min_dist = ((self.path[0][0] - self.odom[0])**2 + (self.path[0][1] - self.odom[1])**2)**0.5
            for i in range(len(self.path)):
                dist = ((self.path[i][0] - self.odom[0])**2 + (self.path[i][1] - self.odom[1])**2)**0.5
                if dist < min_dist:
                    closest_point = i
            
            # 2. Set the next point on the path
            while ((self.path[closest_point][0] - self.odom[0])**2 + (self.path[closest_point][1] - self.odom[1])**2)**0.5 < (self.x_err**2 + self.y_err**2)**0.5:
                if closest_point < len(self.path)-1:
                    closest_point += 1
                else:
                    closest_point = len(self.path)-1
                    break
            
            # Create the user request
            """
            ai_user = 
            Here the previous response (v_x v_y w_z I_x_prev I_y_prev I_theta_prev e_x_prev e_y_prev e_theta_prev):
            
            for mem in self.ai_res_memory:
                ai_user += 
                {mem}
                
            """
            
            ai_user = f"""
            
            Use the following updated data:
            Position:
            - x_current: {self.odom[0]}
            - y_current: {self.odom[1]}
            - theta_current: {self.odom[2]}
            
            Target:
            - x_target: {self.path[closest_point][0]}
            - y_target: {self.path[closest_point][1]}
            """   
            
            print("--------------------------------------------------")
            print("Request " + str(self.iter))
            self.iter += 1
            print("--------------------------------------------------")
            print(ai_user)
            
            # Compute and print the desired velocities
            theta_target = math.atan2(self.path[closest_point][1]-self.odom[1], self.path[closest_point][0]-self.odom[0])
            e_theta = theta_target - self.odom[2]
            
            w_des = Kp_w * math.atan2(np.sin(e_theta), np.cos(e_theta))
            v_x_des = Kp_v * (self.path[closest_point][0] - self.odom[0])
            v_y_des = Kp_v * (self.path[closest_point][1] - self.odom[1])
            
            """
            1. Calculate the errors:
            - Error in x position: e_x = x_target - x_current
            - Error in y position: e_y = y_target - y_current
            - Error in orientation: e_theta = atan2(e_y, e_x) - theta_current

            2. Compute the control outputs (velocities):
            - Linear velocity along x-axis: v_x = Kp_x * e_x 
            - Linear velocity along y-axis: v_y = Kp_y * e_y 
            - Angular velocity about z-axis: w_z = Kp_theta * atan2(sin(e_theta), cos(e_theta)) 
            """

            print("Vx des: " + str(v_x_des))
            print("Vy des: " + str(v_y_des))
            print("W des: " + str(w_des) + "\n")
            
            print("Next point on the path: " + str(closest_point) + "\n")
            
            
            start = time.time()
            
            completion = client.chat.completions.create(
                #model="gpt-3.5-turbo",
                model="gpt-4o-mini",
                messages=[
                        {"role": "system", "content": ai_system},
                        {"role": "user", "content": ai_user}
                    ]
            )

            # Compute computation time 
            end = time.time()
            diff = end - start
            print("Time = " + str(diff) + " s\n")
            
            # Get the response
            cmd_vel = completion.choices[0].message.content
            #cmd_vel = "0.0 0.0 0.0"
            print("Response: " + cmd_vel + "\n")
            self.ai_res_memory.append(cmd_vel)                
                
            ### Clear the response ###
            
            # Remove characters that are not numbers or dots (for decimal representation)
            cmd_vel = re.sub(r'[^0-9.e-]', ' ', cmd_vel)
            
            # Split for empty spaces
            cmd_vel = cmd_vel.split()
            
            # Remove empty elements
            for el in cmd_vel:
                if el == "":
                    cmd_vel.remove(el)
            
            # Compute the oscillation
            """
            self.oscillation_v = abs((float(cmd_vel[-3])**2 + float(cmd_vel[-2])**2)**0.5)
            self.oscillation_w = abs(float(cmd_vel[-1]))
            print("Oscillation v = " + str(self.oscillation_v) + " m")            
            print("Oscillation w = " + str(self.oscillation_w) + " m")
            
            if self.oscillation_v > self.oscillation_max_v:
                if self.flag_otime_v == False:
                    self.flag_otime_v = True
                    self.T_u_start_v = time.time()
                else:
                    self.flag_otime_v = False
                    self.T_u_end_v = time.time()
                    self.T_u_v = self.T_u_end_v - self.T_u_start_v
                    print("T_u_v = " + str(self.T_u_v) + " s\n")
            else: 
                Kp_v = Kp_v * 2
            
            if self.oscillation_w > self.oscillation_max_w:
                if self.flag_otime_w == False:
                    self.flag_otime_w = True
                    self.T_u_start_w = time.time()
                else:
                    self.flag_otime_w = False
                    self.T_u_end_w = time.time()
                    self.T_u_w = self.T_u_end_w - self.T_u_start_w
                    print("T_u_w = " + str(self.T_u_w) + " s\n")
            else:
                Kp_w = Kp_w * 2
            """
            
            # Publish control commands
            cmd = Twist()
            """
            cmd.linear.x = v_x_des
            cmd.linear.y = v_y_des
            cmd.angular.z = w_des
            """
            cmd.linear.x = float(cmd_vel[0])
            cmd.linear.y = float(cmd_vel[1])
            cmd.angular.z = float(cmd_vel[2])
            
            print("v_x = " + str(cmd.linear.x) + " m/s\n" +
                  "v_y = " + str(cmd.linear.y) + " m/s\n" +
                  "w_z = " + str(cmd.angular.z) + " rad/s\n")
             
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