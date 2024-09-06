import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
        
    imu_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'imu.yaml')


    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
    )                                               
    
    imu_filter_node =  launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config]
    )

    #select a robot model,the default model is mini_mec 
    #minibot.launch.py contains commonly used robot models
    #launch_arguments choices:mini_mec/mini_akm/mini_tank/mini_4wd/mini_diff/mini_omni/brushless_senior_diff
    #!!!At the same time, you need to modify ld.add_action(minibot_type) and #ld.add_action(flagship_type)
    minibot_type = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description_minibot.launch.py')),
            launch_arguments={'mini_mec': 'true'}.items(),
    )

    ld = LaunchDescription()

    ld.add_action(minibot_type)
    ld.add_action(wheeltec_robot)
    ld.add_action(imu_filter_node)    

    return ld

