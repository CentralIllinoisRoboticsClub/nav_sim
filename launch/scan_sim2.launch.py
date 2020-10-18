from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number

from launch.actions import ExecuteProcess

# https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py

# https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/launch/zed.launch.py
# Also need to add config to the CMakeLists.txt install(DIRECTORY
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    scan_sim_node = Node(
        package="light_scan_sim",
        executable="light_scan_sim_node",
        parameters=[{'freq_hz': 10.0},
                    {'range_min': 0.1},
                    {'range_max': 50.0},
                    {'range_noise': 0.05},
                    {'angle_min': -0.39},
                    {'angle_max': 0.39},
                    {'angle_increment': 0.052}
        ]
    )
    
    ld.add_action(scan_sim_node)
    
    return ld