from openai import OpenAI
import math
import re

### OpenAI API ###

def openAI_api(odom, waypoint, Kp=(0.0, 0.0), Ki=(0.0, 0.0), Kd=(0.0, 0.0), dt=0.0, e=(0.0, 0.0, 0.0), i=(0.0, 0.0, 0.0), ai_model="gpt-4o-mini"):
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
    1. Calculate the proportional terms:
    - Proportional error in x position: diff_x = x_target - x_current
    - Proportional error in y position: diff_y = y_target - y_current
    - Proportional error in z orientation: diff_theta = theta_target - theta_current
    """
    
    if dt > 0.0:
        ai_system += f"""
    
    2. Calculate the intergral terms:
    - Integral error in x position: i_x = {i[0]} + diff_x * dt
    - Integral error in y position: i_y = {i[1]} + diff_y * dt
    - Integral error in z orientation: i_theta = {i[2]} + diff_theta * dt
    
    3. Calculate the derivative terms:
    - Derivative error in x position: d_x = (diff_x - {e[0]}) / dt
    - Derivative error in y position: d_y = (diff_y - {e[1]}) / dt
    - Derivative error in z orientation: d_theta = (diff_theta - {e[2]}) / dt
    
    4. Compute the control outputs (velocities):
    - Linear velocity along x-axis: v_x = {Kp[0]} * diff_x + {Ki[0]} * i_x + {Kd[0]} * d_x
    - Linear velocity along y-axis: v_y = {Kp[0]} * diff_y + {Ki[0]} * i_y + {Kd[0]} * d_y
    - Angular velocity about z-axis: w_z = {Kp[1]} * atan2(sin(diff_theta), cos(diff_theta)) + {Ki[1]} * i_theta + {Kd[1]} * d_theta
    
    Provide numeric results with only numbers; if the number requires the 'e' letter for the magnitude, approximate it to 0.0.
    Provide the results (even 0.0 values), without any extra words and headers, in the format (only the numeric values): v_x v_y w_z diff_x diff_y diff_theta i_x i_y i_theta 
        """
    
    else:
        ai_system += f"""
    4. Compute the control outputs (velocities):
    - Linear velocity along x-axis: v_x = {Kp[0]} * diff_x 
    - Linear velocity along y-axis: v_y = {Kp[0]} * diff_y 
    - Angular velocity about z-axis: w_z = {Kp[1]} * atan2(sin(diff_theta), cos(diff_theta)) 
    
    Provide the results (even 0.0 values), without any extra words and headers, in the format (only the numeric values): v_x v_y w_z
        """

    ai_user = f"""
    Use the following updated data:
    Position:
    - x_current: {odom[0]}
    - y_current: {odom[1]}
    - theta_current: {odom[2]}
    
    Target:
    - x_target: {waypoint[0]}
    - y_target: {waypoint[1]}
    - theta_target: {waypoint[2]}
    """   
    #print(ai_user)
    
    ### Get the response ###
    
    completion = client.chat.completions.create(
        #model="gpt-3.5-turbo",
        #model="gpt-4o-mini",
        model=ai_model,
        messages=[
                {"role": "system", "content": ai_system},
                {"role": "user", "content": ai_user}
            ]
    )
    
    cmd_vel = completion.choices[0].message.content
    print("Response: " + cmd_vel + "\n")         
        
    ### Clear the response ###
    
    # Remove characters that are not numbers or dots (for decimal representation)
    cmd_vel = re.sub(r'[^0-9.e-]', ' ', cmd_vel)
    
    # Split for empty spaces
    cmd_vel = cmd_vel.split()
    
    # Remove empty elements
    for el in cmd_vel:
        if el == "":
            cmd_vel.remove(el)
    
    # Update return variables
    v = [float(cmd_vel[0]), float(cmd_vel[1])]
    w = float(cmd_vel[2])
    
    if len(cmd_vel) >= 6:
        e = [float(cmd_vel[3]), float(cmd_vel[4]), float(cmd_vel[5])]
        
    if len(cmd_vel) >= 9:
        i = [float(cmd_vel[6]), float(cmd_vel[7]), float(cmd_vel[8])]
        
    # TODO: remove
    #e_theta = theta_target - odom[2]
    #w = Kp_w * math.atan2(math.sin(e_theta), math.cos(e_theta))
    
    return v, w, e, i
            
### PID ###

def pid(odom, waypoint, Kp=(0.0, 0.0), Ki=(0.0, 0.0), Kd=(0.0, 0.0), dt=0.0, e=(0.0, 0.0, 0.0), i=(0.0, 0.0, 0.0)):    
    # 1. Compute proportional term
    e_x = waypoint[0] - odom[0]
    e_y = waypoint[1] - odom[1]
    e_theta = math.atan2(math.sin(waypoint[2] - odom[2]), math.cos(waypoint[2] - odom[2]))

    # 2. Compute the desired velocities
    v_x = Kp[0] * e_x 
    v_y = Kp[0] * e_y 
    w_z = Kp[1] * e_theta 
    
    if dt > 0.0:
        # 3. Compute integral term
        i_x = i[0] + e_x * dt
        i_y = i[1] + e_y * dt
        i_theta = i[2] + e_theta * dt

        # 4. Compute derivative term
        d_x = (e_x - e[0]) / dt
        d_y = (e_y - e[1]) / dt
        d_theta = (e_theta - e[2]) / dt
        
        # 5. Update the desired velocities
        v_x += Ki[0] * i_x + Kd[0] * d_x
        v_y += Ki[0] * i_y + Kd[0] * d_y
        w_z += Ki[1] * i_theta + Kd[1] * d_theta
        
    # 6. Update return values
    v = [v_x, v_y]
    w = w_z
    e = [e_x, e_y, e_theta]
    i = [i_x, i_y, i_theta]
    
    return v, w, e, i
        
### Compute the control commands ###

def compute_control_commands(odom, waypoint, Kp=(0.0, 0.0), Ki=(0.0, 0.0), Kd=(0.0, 0.0), dt=0.0, e=(0.0, 0.0, 0.0), i=(0.0, 0.0, 0.0), control_mode=0, ai_model="gpt-4o-mini"):
    if control_mode == 0:
        v, w, e, i = pid(odom, waypoint, Kp, Ki, Kd, dt, e, i)
    elif control_mode == 1:
        v, w, e, i = openAI_api(odom, waypoint, Kp, Ki, Kd, dt, e, i, ai_model)
    
    return v, w, e, i