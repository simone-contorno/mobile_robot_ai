import math

### Compute the next point on the path to reach ###
def compute_next_point(path, odom, threshold):     
    # 1. Find the closest point on the path to the actual position
    next_point = 0
    min_dist = ((path[0][0] - odom[0])**2 + (path[0][1] - odom[1])**2)**0.5
    for i in range(len(path)):
        dist = ((path[i][0] - odom[0])**2 + (path[i][1] - odom[1])**2)**0.5
        if dist < min_dist:
            next_point = i

    # 2. Set the next point on the path
    euler_dist = ((path[next_point][0] - odom[0])**2 + (path[next_point][1] - odom[1])**2)**0.5
    while euler_dist < threshold:
        if next_point < len(path)-1:
            next_point += 1
        else:
            next_point = len(path)-1
            break
        
    return next_point

def compute_control_commands(odom, waypoint, Kp_v=0.0, Kp_w=0.0, Ki_v=0.0, Ki_w=0.0, Kd_v=0.0, Kd_w=0.0):
    # 1. Compute proportional term
                
    # Compute and print the desired velocities
    theta_target = math.atan2(waypoint[1]-odom[1], waypoint[0]-odom[0])
    e_theta = theta_target - odom[2]
    
    v_x = Kp_v * (waypoint[0] - odom[0])
    v_y = Kp_v * (waypoint[1] - odom[1])
    v = [v_x, v_y]
    w = Kp_w * math.atan2(math.sin(e_theta), math.cos(e_theta))
    
    return v, w