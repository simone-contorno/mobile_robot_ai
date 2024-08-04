from openai import OpenAI
from mobile_robot_ai.ai_prompts import *
import math
import re

### OpenAI API ###

client = OpenAI()

def openAI_api(odom, waypoint, Kp, Ki, Kd, dt, e, i, ai_model, ai_system):
    
    ### AI user prompt ###
    
    ai_user = f"""
    Gains:
    - Kp_v: {Kp[0]}
    - Kp_w: {Kp[1]}
    
    - Ki_v: {Ki[0]}
    - Ki_w: {Ki[1]}
    
    - Kd_v: {Kd[0]}
    - Kd_w: {Kd[1]}
    
    Step size:
    - dt: {dt}
    
    Current robot's position:
    - x_current: {odom[0]}
    - y_current: {odom[1]}
    - theta_current: {odom[2]}
    
    Target's position:
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
    
    ### Update return variables ###
    
    v = [float(cmd_vel[0]), float(cmd_vel[1])]
    w = float(cmd_vel[2])
    
    if len(cmd_vel) >= 6:
        e = [float(cmd_vel[3]), float(cmd_vel[4]), float(cmd_vel[5])]
        
    if len(cmd_vel) >= 9:
        i = [float(cmd_vel[6]), float(cmd_vel[7]), float(cmd_vel[8])]
        
    return v, w, e, i
            
### PID ###

def pid(odom, waypoint, Kp, Ki, Kd, dt, e, i):    
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

def compute_control_commands(odom=(0.0, 0.0, 0.0), 
                             waypoint=(0.0, 0.0, 0.0), 
                             Kp=(0.0, 0.0), 
                             Ki=(0.0, 0.0), 
                             Kd=(0.0, 0.0), 
                             dt=0.0, 
                             e=(0.0, 0.0, 0.0), 
                             i=(0.0, 0.0, 0.0), 
                             control_mode=0, 
                             ai_model="gpt-4o-mini", 
                             ai_system=pid_proportional_only):
    
    if control_mode == 0:
        v, w, e, i = pid(odom, waypoint, Kp, Ki, Kd, dt, e, i)
    elif control_mode == 1:
        v, w, e, i = openAI_api(odom, waypoint, Kp, Ki, Kd, dt, e, i, ai_model, ai_system)
    
    return v, w, e, i