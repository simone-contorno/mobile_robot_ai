#!/usr/bin/env python3

from openai import OpenAI

import time
import re
import numpy as np 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Twist

from tf_transformations import euler_from_quaternion, quaternion_from_euler

client = OpenAI()

### PID ###

# Proportional gains
Kp_v = 0.5
Kp_w = 0.5

Kp_x = 1.0
Kp_y = 1.0
Kp_theta = 1.0

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
You are controlling a unicycle robot:
- dx = v * cos(theta)
- dy = v * sin(theta)
- dtheta = w

Using a PID controller, the robot needs to achieve the following velocities:
- Linear velocity v_x along x-axis [m/s]
- Linear velocity v_y along y-axis [m/s]
- Angular velocity w_z about z-axis [rad/s]

The current positions and orientation of the robot are:
- Current x position: x_current [m]
- Current y position: y_current [m]
- Current theta orientation: theta_current [rad]

The target positions and orientation of the robot are:
- Target x position: x_target [m]
- Target y position: y_target [m]
- Target theta orientation: atan2(y_target - y_current, x_target - x_current) [rad]

The desired velocities once the target is reached are:
- Linear velocity along x-axis: 0 m/s
- Linear velocity along y-axis: 0 m/s
- Angular velocity about z-axis: 0 rad/s

Use the following gain parameters to compute the velocities:
- Proportional positionx-axis gain (Kp_x): {Kp_x}
- Proportional position y-axis gain (Kp_y): {Kp_y}
- Proportional orientation theta gain (Kp_theta): {Kp_theta}

Firstly, calculate the errors mantaining the correct sign, also negative values are ok:
- Error in x position: e_x = x_target - x_current
- Error in y position: e_y = y_target - y_current
- Error in orientation: e_theta = atan2(e_y, e_x) - theta_current

Then, compute the proportional terms:
- Proportional x-axis term: P_x = Kp_x * e_x
- Proportional y-axis term: P_y = Kp_y * e_y
- Proportional theta-axis term: P_theta = Kp_theta * atan2(sin(e_theta), cos(e_theta))

Finally, provide the control outputs (velocities), mantaining the correct sign, as:
- Linear velocity along x: v_x = P_x
- Linear velocity along y: v_y = P_y
- Angular velocity about z: w_z = P_theta

Do not use the integral and derivative terms, only the proportional ones.

The acceleration constraints are:
- Maximum linear acceleration along x-axis and y-axis (a_min): {dv_max}
- Maximum angular acceleration about z-axis (a_max): {dw_max}
Consider this information to avoid abrupt linear and angular accelations.

Compute the optimal control velocities to achieve the target position.
Provide the results, without any extra words, in the format: v_x v_y w_z
"""

"""
As constaints consider:
- Mimimum linear velocity along x-axis and y-axis (v_min): {v_min}
- Minimum angular velocity about z-axis (w_min): {w_min}
- Maximum linear velocity along x-axis and y-axis (v_max): {v_max}
- Maximum angular velocity about z-axis (w_max): {w_max}
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
        self.x_err = 0.25
        self.y_err = 0.25
        self.theta_err = 1.0
        self.offset_odom = 0.1
        
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
        self.path_percentage = 10
        
        # Time step
        self.path_time_step = 1
        
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
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (_, _, self.odom_theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        self.odom_x = msg.pose.pose.position.x #+ self.offset_odom * np.cos(self.odom_theta)
        self.odom_y = msg.pose.pose.position.y #+ self.offset_odom * np.sin(self.odom_theta)
        
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
            while abs(self.path[closest_point][0] - self.odom[0]) < self.x_err or abs(self.path[closest_point][1] - self.odom[1]) < self.y_err:
                if closest_point < len(self.path)-1:
                    closest_point += 1
                else:
                    closest_point = len(self.path)-1
                    break
            
            # Create the user request
            ai_user = f"""
            Here the current data:
            - x_current: {self.odom[0]}
            - y_current: {self.odom[1]}
            - theta_current: {self.odom[2]}
            
            - x_target: {self.path[closest_point][0]}
            - y_target: {self.path[closest_point][1]}
            """   
            
            print("--------------------------------------------------")
            print("Request " + str(self.iter))
            self.iter += 1
            print("--------------------------------------------------")
            print(ai_user)
            
            print("Next point on the path: " + str(closest_point) + "\n")
            
            start = time.time()
            
            completion = client.chat.completions.create(
                model="gpt-3.5-turbo",
                #model="gpt-4o-mini",
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
            print("Response: " + cmd_vel + "\n")
            
            ### Clear the response ###
            
            # Remove characters that are not numbers or dots (for decimal representation)
            cmd_vel = re.sub(r'[^0-9.-]', ' ', cmd_vel)
            
            # Split for empty spaces
            cmd_vel = cmd_vel.split()
            
            # Remove empty elements
            for el in cmd_vel:
                if el == "":
                    cmd_vel.remove(el)
            
            # Publish control commands
            cmd = Twist()
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