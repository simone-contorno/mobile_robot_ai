#!/usr/bin/env python3

import math 
import signal 
import time
from mobile_robot_ai import utils
from mobile_robot_ai import ziegler_nichols as zn

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

from tf_transformations import euler_from_quaternion

class Control_PID(Node):
    def __init__(self):
        super().__init__('control_pid')
        
        ### Private variables ###

        # Ziegler-Nichols tuning
        self.zn = utils.get_config_param("pid", "ziegler_nichols")
        self.zn_vx = zn.ZieglerNichols()
        self.zn_vy = zn.ZieglerNichols()
        self.zn_wz = zn.ZieglerNichols()
        
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
            v, w = self.pid()
            
            # Ziegler-Nichols tuning
            if self.zn == True:
                # Velocity along x
                if self.zn_vx != None:
                    self.zn_vx.ziegler_nichols_tuning(v[0], time.time())
                    if self.zn_vx.Ku != None and self.zn_vx.Pu != None:
                        print("Ku_vx: ", self.zn_vx.Ku)
                        print("Pu_vx: ", self.zn_vx.Pu)
                        print("--------------------")
                        self.zn_vx = None
                    elif self.zn_vx.Ku != None:
                        self.Kp_vx = self.zn_vx.Ku
                
                # Velocity along y
                if self.zn_vy != None:
                    self.zn_vy.ziegler_nichols_tuning(v[1], time.time())
                    if self.zn_vy.Ku != None and self.zn_vy.Pu != None:
                        print("Ku_vy: ", self.zn_vy.Ku)
                        print("Pu_vy: ", self.zn_vy.Pu)
                        print("--------------------")
                        self.zn_vy = None
                    elif self.zn_vy.Ku != None:
                        self.Kp_vy = self.zn_vy.Ku
                
                # Angular velocity
                if self.zn_wz != None:
                    self.zn_wz.ziegler_nichols_tuning(w, time.time())
                    if self.zn_wz.Ku != None and self.zn_wz.Pu != None:
                        print("Ku_wz: ", self.zn_wz.Ku)
                        print("Pu_wz: ", self.zn_wz.Pu)
                        print("--------------------")
                        self.zn_wz = None
                    elif self.zn_wz.Ku != None:
                        self.Kp_wz = self.zn_wz.Ku
                
                # Update Proportional terms
                self.Kp = (self.Kp_vx, self.Kp_vy, self.Kp_wz)
            
            # Publish control commands
            cmd = Twist()
            cmd.linear.x = v[0]
            cmd.linear.y = v[1]
            cmd.angular.z = w
            self.publisher_cmd.publish(cmd)
            
    ### PID ###
    def pid(self):    
        # 1. Compute proportional term
        e_x = self.waypoint[0] - self.odom[0]
        e_y = self.waypoint[1] - self.odom[1]
        e_theta = math.atan2(math.sin(self.waypoint[2] - self.odom[2]), math.cos(self.waypoint[2] - self.odom[2]))

        # 2. Compute the desired velocities
        v_x = self.Kp[0] * e_x 
        v_y = self.Kp[1] * e_y 
        w_z = self.Kp[2] * e_theta 
        
        if self.dt > 0.0:
            # 3. Compute integral term
            i_x = self.i[0] + e_x * self.dt
            i_y = self.i[1] + e_y * self.dt
            i_theta = self.i[2] + e_theta * self.dt

            # 4. Compute derivative term
            d_x = (e_x - self.e_prev[0]) / self.dt
            d_y = (e_y - self.e_prev[1]) / self.dt
            d_theta = (e_theta - self.e_prev[2]) / self.dt
            
            # 5. Update the desired velocities
            v_x += self.Ki[0] * i_x + self.Kd[0] * d_x
            v_y += self.Ki[1] * i_y + self.Kd[1] * d_y
            w_z += self.Ki[2] * i_theta + self.Kd[2] * d_theta
            
        # 6. Update return values
        v = [abs(v_x), abs(v_y)] 
        w = w_z
        self.e_prev = [e_x, e_y, e_theta]
        self.i = [i_x, i_y, i_theta]
        
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
    minimal_publisher = Control_PID()

	# Start the callback function
    rclpy.spin(minimal_publisher)
    
    # Destroy the node
    minimal_publisher.destroy_node()

if __name__ == '__main__':
    main()