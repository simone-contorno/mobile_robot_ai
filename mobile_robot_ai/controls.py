from openai import OpenAI
import math
import re

### OpenAI API ###

def openAI_api(odom, waypoint, Kp_v=0.0, Kp_w=0.0, Ki_v=0.0, Ki_w=0.0, Kd_v=0.0, Kd_w=0.0):
    client = OpenAI()
    ai_system = f"""
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
    - Error in z orientation: diff_w = theta_target - theta_current
    
    2. Compute the control outputs (velocities):
    - Linear velocity along x-axis: v_x = {Kp_v} * diff_x 
    - Linear velocity along y-axis: v_y = {Kp_v} * diff_y 
    - Angular velocity about z-axis: w_z = {Kp_w} * atan2(sin(diff_w), cos(diff_w)) 
    
    Provide the results (even 0.0 values), without any extra words and headers, in the format (only the numeric values): v_x v_y w_z
    """
    
    # Add the theta target to the waypoint
    theta_target = math.atan2(waypoint[1]-odom[1], waypoint[0]-odom[0])
    
    ai_user = f"""
    Use the following updated data:
    Position:
    - x_current: {odom[0]}
    - y_current: {odom[1]}
    - theta_current: {odom[2]}
    
    Target:
    - x_target: {waypoint[0]}
    - y_target: {waypoint[1]}
    - theta_target: {theta_target}
    """   
    print(ai_user)
    
    ### Get the response ###
    
    completion = client.chat.completions.create(
        #model="gpt-3.5-turbo",
        model="gpt-4o-mini",
        messages=[
                {"role": "system", "content": ai_system},
                {"role": "user", "content": ai_user}
            ]
    )
    
    cmd_vel = completion.choices[0].message.content
    #print("Response: " + cmd_vel + "\n")         
        
    ### Clear the response ###
    
    # Remove characters that are not numbers or dots (for decimal representation)
    cmd_vel = re.sub(r'[^0-9.e-]', ' ', cmd_vel)
    
    # Split for empty spaces
    cmd_vel = cmd_vel.split()
    
    # Remove empty elements
    for el in cmd_vel:
        if el == "":
            cmd_vel.remove(el)
    
    v = [float(cmd_vel[0]), float(cmd_vel[1])]
    w = float(cmd_vel[2])
    
    # TODO: remove
    #e_theta = theta_target - odom[2]
    #w = Kp_w * math.atan2(math.sin(e_theta), math.cos(e_theta))
    
    return v, w
            
### PID ###

def pid(odom, waypoint, Kp_v=0.0, Kp_w=0.0, Ki_v=0.0, Ki_w=0.0, Kd_v=0.0, Kd_w=0.0):
    # 1. Compute proportional term
                    
    # Compute and print the desired velocities
    theta_target = math.atan2(waypoint[1]-odom[1], waypoint[0]-odom[0])
    e_theta = theta_target - odom[2]
    
    v_x = Kp_v * (waypoint[0] - odom[0])
    v_y = Kp_v * (waypoint[1] - odom[1])
    v = [v_x, v_y]
    w = Kp_w * math.atan2(math.sin(e_theta), math.cos(e_theta))
    
    return v, w
        
### Compute the control commands ###

def compute_control_commands(odom, waypoint, Kp_v=0.0, Kp_w=0.0, Ki_v=0.0, Ki_w=0.0, Kd_v=0.0, Kd_w=0.0, control_mode=0):
    if control_mode == 0:
        v, w = pid(odom, waypoint, Kp_v, Kp_w, Ki_v, Ki_w, Kd_v, Kd_w)
    elif control_mode == 1:
        v, w = openAI_api(odom, waypoint, Kp_v, Kp_w, Ki_v, Ki_w, Kd_v, Kd_w)
    
    return v, w