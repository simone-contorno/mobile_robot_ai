#!/usr/bin/env python3

from openai import OpenAI

import time
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Twist

from tf_transformations import euler_from_quaternion, quaternion_from_euler

client = OpenAI()

### PID ###

# Proportional gains
Kp_x = 1.0
Kp_y = 1.0
Kp_theta = 1.0

# Integral gains
#Ki_x = 0.0
#Ki_y = 0.0
#Ki_theta = 0.0

# Derivative gains
#Kd_x = 0.0
#Kd_y = 0.0
#Kd_theta = 0.0

# Time step
dt = 0.5

# Target
x_target = None
y_target = None
theta_target = None

# Current position
x_current = None
y_current = None
theta_current = None

pid_prompt = f"""
You are controlling a unicycle robot using a PID controller with only Proportional term. 
The robot needs to achieve the following velocities:

- Linear velocity along x-axis (m/s)
- Linear velocity along y-axis (m/s)
- Angular velocity about z-axis (rad/s)

The current positions and orientation of the robot are:
- Current x position: x_current in meters
- Current y position: y_current in meters
- Current theta orientation: theta_current in radians

The target positions and orientation of the robot are:
- Target x position: x_target in meters
- Target y position: y_target in meters
- Target orientation (theta): atan2(y_target - y_current, x_target - x_current) in radians

Use the following gain parameters to compute the velocities:
- Proportional x-axis gain (Kp_x): {Kp_x}
- Proportional y-axis gain (Kp_y): {Kp_y}
- Proportional theta-axis gain (Kp_theta): {Kp_theta}

The time step (dt) is {dt} seconds.

Please calculate the errors:
- Error in x position: e_x = x_target - x_current
- Error in y position: e_y = y_target - y_current
- Error in orientation: e_theta = atan2(e_y, e_x) - theta_current

Use these errors to compute the following terms:
- Proportional x-axis term: P_x = {Kp_x} * e_x
- Proportional y-axis term: P_y = {Kp_y} * e_y
- Proportional theta-axis term: P_theta = {Kp_theta} * atan2(sin(e_theta), cos(e_theta))

Finally, provide the control outputs (velocities) as:
- Linear velocity along x: v_x = P_x 
- Linear velocity along y: v_y = P_y 
- Angular velocity about z: omega_z = P_theta

Provide the results without any extra words in the format: v_x v_y omega_z
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
        self.x_err = 0.5
        self.y_err = 0.5
        self.theta_err = 1.0
        
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
        self.path_counter = 1
        
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
        z = msg.pose.position.z
        
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
        self.get_logger().info("[Plan] I heard new plan: " + str(len(msg.poses)) + " waypoints")
        #self.path = msg.poses
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
        self.path_counter = 1
        
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
        self.get_logger().info("[Plan] Published new plan: " + str(len(plan.poses)) + " waypoints")
             
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (_, _, self.odom_theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        self.odom = (self.odom_x, self.odom_y, self.odom_theta)

        # Check if the goal has been reached
        if self.goal != (None, None, None) and self.goal_prev != self.goal:
            if abs(self.odom[0]-self.goal[0]) < self.x_err and abs(self.odom[1]-self.goal[1]) < self.x_err and abs(self.odom[2]-self.goal[2]) < self.theta_err:
                self.get_logger().info("[Odom] Goal reached!")
                self.goal_prev = self.goal # Update old goal

        # Compute control commands
        if (self.odom != (None, None, None) and self.goal != (None, None, None) and
                self.path != None and self.ranges != None and self.goal_prev != self.goal):
            
            ai_user = f"""
            Here the current data:
            - x_target: {self.path[self.path_counter][0]}
            - y_target: {self.path[self.path_counter][1]}
            
            - x_current: {self.odom[0]}
            - y_current: {self.odom[1]}
            - theta_current: {self.odom[2]}
            """        
            #- theta_target: {self.path[self.path_counter][2]}
                        
            # Add ranges information to the user
            """
            for i in range(len(self.ranges)):
                self.ai_user = self.ai_user + str(self.ranges[i]) + ", "
            self.ai_user = self.ai_user + ")\n"
            """
            
            print("Request " + str(self.iter) + "\n" + ai_user + "\n")
            self.iter += 1
            
            start = time.time()
            
            completion = client.chat.completions.create(
                model="gpt-3.5-turbo",
                #model="gpt-4o-mini",
                messages=[
                        {"role": "system", "content": ai_system},
                        {"role": "user", "content": ai_user}
                    ]
            )

            cmd_vel = completion.choices[0].message.content
            print("Response: " + cmd_vel)
            
            # Clear the messages
            
            # Remove characters that are not numbers or dots (for decimal representation)
            cmd_vel = re.sub(r'[^0-9.]', ' ', cmd_vel)
            
            # Split for empty spaces
            cmd_vel = cmd_vel.split()
            
            # Remove empty elements
            for el in cmd_vel:
                if el == "":
                    cmd_vel.remove(el)
            
            print("v_x = " + cmd_vel[0] + " m/s\n" +
                  "v_y = " + cmd_vel[1] + " m/s\n" +
                  "w_z = " + cmd_vel[2] + " rad/s\n")

            # Compute computation time 
            end = time.time()
            diff = end - start
            print("Time = " + str(diff) + " s")
            
            # Publish control commands
            cmd = Twist()
            cmd.linear.x = float(cmd_vel[0])
            cmd.linear.y = float(cmd_vel[1])
            cmd.angular.z = float(cmd_vel[2])
            self.publisher_cmd.publish(cmd)
            self.path_counter += 1
            
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