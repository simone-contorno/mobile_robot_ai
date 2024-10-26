#!/usr/bin/env python3

from openai import OpenAI

import re
import signal 
import time

from mobile_robot_ai.ai_prompts import *
from mobile_robot_ai import utils

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

from tf_transformations import euler_from_quaternion

class Control_OpenAI(Node):
    def __init__(self):
        super().__init__('control_openai')
        
        ### Private variables ###
        
        # AI model parameters
        
        self.openai_model = utils.get_config_param("openai", "model")
        self.openai_system = utils.get_config_param("openai", "system")
        self.openai_llama_api = utils.get_config_param("openai", "llama_api")
        
        print(f"OpenAI model: {self.openai_model}")
        print(f"OpenAI system: {self.openai_system}")
        print(f"OpenAI LLaMA API: {self.openai_llama_api}")
        
        self.openai_prompt = globals()[self.openai_system]
        
        print(self.openai_prompt)
        
        # Tuning coefficients
        self.Kp_vx = utils.get_config_param("pid", "Kp_vx")
        self.Kp_vy = utils.get_config_param("pid", "Kp_vy")
        self.Kp_wz = utils.get_config_param("pid", "Kp_wz")
        
        self.Ki_vx = utils.get_config_param("pid", "Ki_vx")
        self.Ki_vy = utils.get_config_param("pid", "Ki_vy")
        self.Ki_wz = utils.get_config_param("pid", "Ki_wz")
        
        self.Kd_vx = utils.get_config_param("pid", "Kd_vx")
        self.Kd_vy = utils.get_config_param("pid", "Kd_vy")
        self.Kd_wz = utils.get_config_param("pid", "Kd_wz")
        
        self.dt = utils.get_config_param("pid", "dt")
        
        # Print configuration parameters
        print(f"""
        Configuration parameters:
        Kp_v: {self.Kp_vx, self.Kp_vy}
        Kp_w: {self.Kp_wz}
        
        Ki_v: {self.Ki_vx, self.Ki_vy}
        Ki_w: {self.Ki_wz}
        
        Kd_v: {self.Kd_vx, self.Kd_vy}
        Kd_w: {self.Kd_wz}
        """)
        
        self.Kp = (self.Kp_vx, self.Kp_vy, self.Kp_wz)
        self.Ki = (self.Ki_vx, self.Ki_vy, self.Ki_wz)
        self.Kd = (self.Kd_vx, self.Kd_vy, self.Kd_wz)
        
        # Next waypoint
        self.next_wp_x = None
        self.next_wp_y = None
        self.next_wp_theta = None
        self.waypoint = (None, None, None)
        
        # Odometry
        self.odom_x = None
        self.odom_y = None
        self.odom_theta = None
        self.odom = (None, None, None)
        self.offset_odom = 0.05
        
        # PID
        self.e_prev = (0.0, 0.0, 0.0) # proportional error
        self.i = (0.0, 0.0, 0.0) # integral error
        
        # Iteration counter
        self.iter = 0
        
        ### Subscribers ###
        
        # Next waypoint
        self.subscriber_next_wp = self.create_subscription(
            PoseStamped,
            'next_wp',
            self.callback_next_wp,
            10
        )
        
        # Odometry
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.callback_odom,
            10
        )

        #### Publishers ###

        # Control commands
        self.publisher_cmd = self.create_publisher(
            Twist, 
            "/cmd_vel_handler", 
            10
        )

        # Manage CTRL+C command
        signal.signal(signal.SIGINT, self.signal_handler)
    
    ### Callbacks ###
    
    # Next waypoint
    def callback_next_wp(self, msg):
        self.next_wp_x = msg.pose.position.x
        self.next_wp_y = msg.pose.position.y
        self.next_wp_theta = msg.pose.orientation.z
        self.waypoint = (self.next_wp_x, self.next_wp_y, self.next_wp_theta)
    
    # Odometry 
    def callback_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (_, _, self.odom_theta) = euler_from_quaternion([qx, qy, qz, qw])
        self.odom = (self.odom_x, self.odom_y, self.odom_theta)
        
        if self.next_wp_x != None and self.next_wp_y != None and self.next_wp_theta != None and self.odom_x != None and self.odom_y != None and self.odom_theta != None:
            v, w = self.openai_api()
            
            # Publish control commands
            cmd = Twist()
            cmd.linear.x = v[0]
            cmd.linear.y = v[1]
            cmd.angular.z = w
            self.publisher_cmd.publish(cmd)
        
    ### OpenAI API ###
    def openai_api(self):
        
        ### AI client ####
        if "llama" in self.openai_model:
            client = OpenAI(
                api_key = self.openai_llama_api,
                base_url = "https://api.llama-api.com"
            )
        elif "gpt" in self.openai_model:
            client = OpenAI()
        else:
            print("Model not valid.")
            exit()
            
        ### AI user prompt ###
        openai_user = f"""
        Gains:
        - Kp_v: {self.Kp[0], self.Kp[1]}
        - Kp_w: {self.Kp[2]}
        
        - Ki_v: {self.Ki[0], self.Ki[1]}
        - Ki_w: {self.Ki[2]}
        
        - Kd_v: {self.Kd[0], self.Kd[1]}
        - Kd_w: {self.Kd[1]}
        
        Step size:
        - dt: {self.dt}
        
        Current robot's position:
        - x_current: {self.odom[0]}
        - y_current: {self.odom[1]}
        - theta_current: {self.odom[2]}
        
        Target's position:
        - x_target: {self.waypoint[0]}
        - y_target: {self.waypoint[1]}
        - theta_target: {self.waypoint[2]}
        """   
        #print(ai_user)
        
        ### Get the response ###
        
        start = time.time()
        completion = client.chat.completions.create(
            model=self.openai_model, # gpt-4o-mini | gpt-3.5-turbo
            messages=[
                    {"role": "system", "content": self.openai_prompt},
                    {"role": "user", "content": openai_user}
                ]
        )
        
        end = time.time()
        cmd_vel = completion.choices[0].message.content
        print("Response: " + cmd_vel + "\n")      
        print("Computation time: " + str(end - start) + "\n")   
            
        ### Clear the response ###
        
        # Remove characters that are not numbers or dots (for decimal representation)
        cmd_vel = re.sub(r'[^0-9.e-]', ' ', cmd_vel)
        
        # Split for empty spaces
        cmd_vel = cmd_vel.split()
        
        # Remove empty elements
        for el in cmd_vel:
            if el == "":
                cmd_vel.remove(el)
            elif "e" in el:
                cmd_vel[cmd_vel.index(el)] = 0.0
        
        ### Update return variables ###
        
        v = [float(cmd_vel[0]), float(cmd_vel[1])]
        w = float(cmd_vel[2])
        
        if len(cmd_vel) >= 6:
            self.e_prev = [float(cmd_vel[3]), float(cmd_vel[4]), float(cmd_vel[5])]
            
        if len(cmd_vel) >= 9:
            self.i = [float(cmd_vel[6]), float(cmd_vel[7]), float(cmd_vel[8])]
            
        return v, w
    
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
    minimal_publisher = Control_OpenAI()

	# Start the callback function
    rclpy.spin(minimal_publisher)
    
    # Destroy the node
    minimal_publisher.destroy_node()

if __name__ == '__main__':
    main()