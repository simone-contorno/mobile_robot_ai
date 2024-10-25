import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

import signal
import time 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

# DataSet
dataset_folder = "" # TODO: add absolute path to the created folder by the pid_control script
ds_err_x = pd.read_csv(dataset_folder + "/error_x.csv")
ds_err_y = pd.read_csv(dataset_folder + "/error_y.csv")
ds_err_theta = pd.read_csv(dataset_folder + "/error_theta.csv")

# Take a look at the dataset
#df.head()

# Summarize the data
#df.describe()

# Create train dataset for x
cdf_x = ds_err_x[['Error','Control']]
cdf_x.head(1500)
train_x = cdf_x
#print("Train set x dimension: ", len(train_x))

# Create train dataset for y
cdf_y = ds_err_y[['Error','Control']]
cdf_y.head(1500)
train_y = cdf_y
#print("Train set y dimension: ", len(train_y))

# Create train dataset for theta
cdf_theta = ds_err_theta[['Error','Control']]
cdf_theta.head(1500)
train_theta = cdf_theta
#print("Train set theta dimension: ", len(train_theta))

# Using sklearn package to model data
from sklearn import linear_model

# X
regr_x = linear_model.LinearRegression()
train_x_in = np.asanyarray(train_x[['Error']])
train_x_out = np.asanyarray(train_x[['Control']])
regr_x.fit(train_x_in, train_x_out)

print('X Coefficients: ', regr_x.coef_)
print('X Intercept: ', regr_x.intercept_)

# Y
regr_y = linear_model.LinearRegression()
train_y_in = np.asanyarray(train_y[['Error']])
train_y_out = np.asanyarray(train_y[['Control']])
regr_y.fit(train_y_in, train_y_out)

print('Y Coefficients: ', regr_y.coef_)
print('Y Intercept: ', regr_y.intercept_)

# Theta
regr_theta = linear_model.LinearRegression()
train_theta_in = np.asanyarray(train_theta[['Error']])
train_theta_out = np.asanyarray(train_theta[['Control']])
regr_theta.fit(train_theta_in, train_theta_out)

print('Theta Coefficients: ', regr_theta.coef_)
print('Theta Intercept: ', regr_theta.intercept_)

class Simple_Lin_Reg_Control(Node):

    def __init__(self):
        super().__init__('simple_lin_reg_control')
        
        # Private variables
        self.next_wp_x = None
        self.next_wp_y = None
        self.next_wp_theta = None
        
        self.odom_x = None
        self.odom_y = None
        self.odom_theta = None
        
        # Create subscriber for next waypoint
        self.subscriber_next_wp = self.create_subscription(
            PoseStamped,
            'next_wp',
            self.callback_next_wp,
            10
        )
        
        # Create subscriber for odometry
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.callback_odom,
            10
        )
        
        # Create publisher for control commands
        self.publisher_cmd = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Manage CTRL+C command
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def callback_next_wp(self, msg):
        self.next_wp_x = msg.pose.position.x
        self.next_wp_y = msg.pose.position.y
        self.next_wp_theta = msg.pose.orientation.z
        
    def callback_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (_, _, self.odom_theta) = euler_from_quaternion([qx, qy, qz, qw])
        
        if self.next_wp_x != None and self.next_wp_y != None and self.next_wp_theta != None and self.odom_x != None and self.odom_y != None and self.odom_theta != None:
            # Calculate error
            err_x = np.asanyarray(self.next_wp_x - self.odom_x).reshape(-1, 1)
            err_y = np.asanyarray(self.next_wp_y - self.odom_y).reshape(-1, 1)
            err_theta = np.asanyarray(self.next_wp_theta - self.odom_theta).reshape(-1, 1)
            
            print("Odom: ", self.odom_x, self.odom_y, self.odom_theta)
            print("Next wp: ", self.next_wp_x, self.next_wp_y, self.next_wp_theta)
            print("Error: ", err_x, err_y, err_theta)
            
            # Calculate control 
            start = time.time()
            v_x = regr_x.predict(err_x)
            v_y = regr_y.predict(err_y)
            w_theta = regr_theta.predict(err_theta)

            # Compute prediction computation time 
            end = time.time()
            diff = end - start
                    
            # Publish control commands
                            
            cmd = Twist()
            cmd.linear.x = float(v_x)
            cmd.linear.y = float(v_y)
            cmd.angular.z = float(w_theta)
            
            self.publisher_cmd.publish(cmd)
            
            print(f"""
                    v_x = {v_x} [m/s]
                    v_y = {v_y} [m/s]
                    w_z = {w_theta} [rad/s]
                    """)   
            
            print("Time = " + str(diff) + " [s]\n")    
            
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
    minimal_publisher = Simple_Lin_Reg_Control()

	# Start the callback function
    rclpy.spin(minimal_publisher)
    
    # Destroy the node
    minimal_publisher.destroy_node()
    
    # Terminate 
    #rclpy.shutdown()

if __name__ == '__main__':
    main()