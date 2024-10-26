#!/usr/bin/env python3

import subprocess
import os
from mobile_robot_ai import utils

result = subprocess.run(['ros2', 'pkg', 'prefix', 'mobile_robot_ai'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
workspace_path = result.stdout.decode('utf-8').strip()

current_path = workspace_path
while current_path != '/':
    if 'src' in os.listdir(current_path):
        workspace_path = current_path
        break
    current_path = os.path.dirname(current_path)
            
# Define the commands to run
cmds = []

cmd_build = (f"""
         cd {workspace_path} ;
         colcon build --packages-select mobile_robot_ai ;
         ; read -p \'Press any key to exit...\'
         """)

process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', cmd_build], shell=False)

cmds.append(f"""
            export TURTLEBOT3_MODEL=waffle ;
            export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix mobile_robot_ai)/share/mobile_robot_ai/worlds/ ;
            ros2 launch mobile_robot_ai simulation_launch.py cmd_vel_remap:=/mobile_robot_ai/cmd_vel plan_remap:=/mobile_robot_ai/plan ;
            read -p \'Press any key to exit...\'
            """)

cmds.append(f"""
            echo 'Running data_subscriber...' ; 
            ros2 run mobile_robot_ai data_subscriber ; 
            read -p \'Press any key to exit...\'
            """)

cmds.append("""
            echo 'Running control_handler...' ; 
            ros2 run mobile_robot_ai control_handler.py ; 
            read -p \'Press any key to exit...\'
            """)

# Run control method
control_method = utils.get_config_param("control", "control_method")

if control_method == 0:
    cmds.append("""
                echo 'Running control_pid...' ; 
                ros2 run mobile_robot_ai control_pid.py ; 
                read -p \'Press any key to exit...\'
                """)
elif control_method == 1:
    cmds.append("""
                echo 'Running control_openai...' ; 
                ros2 run mobile_robot_ai control_openai.py ; 
                read -p \'Press any key to exit...\'
                """)
elif control_method == 2:
    cmds.append("""
                echo 'Running control_simple_lin_reg...' ; 
                ros2 run mobile_robot_ai control_simple_lin_reg.py ; 
                read -p \'Press any key to exit...\'
                """)

# Start the processes
processes = []
for cmd in cmds:
    process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', cmd], shell=False)
    processes.append(process)
