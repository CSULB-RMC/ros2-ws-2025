from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from glob import glob

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("lidar_slam"), "config", "unilidar_l1.yaml"
    )
    
    lidarNode = Node(    
        package="lidar_slam",
        name="laserMapping",
        executable="pointlio_mapping",
        output="screen",
        parameters=[
            config,
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("lidar_slam"), "rviz_cfg", "loam_livox.rviz")],
        output="screen"  # "screen" or "log" to print output in the terminal or log file respectively.  # Optional; defaults to "screen".
    )
    
    ld.add_action(lidarNode)
    ld.add_action(rviz)
    return ld
