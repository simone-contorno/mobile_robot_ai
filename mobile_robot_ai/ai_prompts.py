start = f"""
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
- Target theta orientation: theta_target [rad]

The gains are:
- Proportional gain for linear velocities along x and y axis: Kp_v [m/s]
- Integral gain for linear velocities along x and y axis: Ki_v [m/s]
- Derivative gain for linear velocities along x and y axis: Kd_v [m/s]

- Proportional gain for angular velocity about z axis: Kp_w [rad/s]
- Integral gain for angular velocity about z axis: Ki_w [rad/s]
- Derivative gain for angular velocity about z axis: Kd_w [rad/s]

The step size is: dt [s]
"""

pid_proportional_only = start + f"""
Follow these steps:
1. Calculate the proportional terms:
- Proportional error in x position: p_x = x_target - x_current
- Proportional error in y position: p_y = y_target - y_current
- Proportional error in z orientation: p_theta = theta_target - theta_current

2. Compute the control outputs (velocities):
- Linear velocity along x-axis: v_x = Kp_v* p_x + Ki_v * i_x + Kd_v * d_x
- Linear velocity along y-axis: v_y = Kp_v * p_y + Ki_v * i_y + Kd_v * d_y
- Angular velocity about z-axis: w_z = Kp_w * atan2(sin(p_theta), cos(p_theta)) + Ki_w * i_theta + Kd_w * d_theta
    
Provide numeric results with only numbers; if the number requires the 'e' letter for the magnitude, approximate it to 0.0.
Provide the results (even 0.0 values), without any extra words and headers, in the format (only the numeric values): 
v_x v_y w_z p_x p_y p_theta 
"""

pid_complete = start + f"""
Follow these steps:
1. Calculate the proportional terms:
- Proportional error in x position: p_x = x_target - x_current
- Proportional error in y position: p_y = y_target - y_current
- Proportional error in z orientation: p_theta = theta_target - theta_current

2. Calculate the intergral terms:
- Integral error in x position: i_x = i_x_prev + p_x * dt
- Integral error in y position: i_y = i_y_prev + p_y * dt
- Integral error in z orientation: i_theta = i_theta_prev + p_theta * dt

3. Calculate the derivative terms:
- Derivative error in x position: d_x = (p_x - p_x_prev) / dt
- Derivative error in y position: d_y = (p_y - p_y_prev) / dt
- Derivative error in z orientation: d_theta = (p_theta - p_theta_prev) / dt
    
4. Compute the control outputs (velocities):
- Linear velocity along x-axis: v_x = Kp_v* p_x + Ki_v * i_x + Kd_v * d_x
- Linear velocity along y-axis: v_y = Kp_v * p_y + Ki_v * i_y + Kd_v * d_y
- Angular velocity about z-axis: w_z = Kp_w * atan2(sin(p_theta), cos(p_theta)) + Ki_w * i_theta + Kd_w * d_theta
    
Provide numeric results with only numbers; if the number requires the 'e' letter for the magnitude, approximate it to 0.0.
Provide the results (even 0.0 values), without any extra words and headers, in the format (only the numeric values): 
v_x v_y w_z p_x p_y p_theta i_x i_y i_theta 
"""

pid_automatic = start + f"""
Compute the control outputs (velocities).
Provide numeric results with only numbers; if the number requires the 'e' letter for the magnitude, approximate it to 0.0.
Provide the results (even 0.0 values), without any extra words and headers, in the format (only the numeric values): 
v_x v_y w_z p_x p_y p_theta i_x i_y i_theta 
"""