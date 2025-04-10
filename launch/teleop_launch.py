import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    controllerConfig = os.path.join(
        get_package_share_directory("teleop"), "config", "controller.yaml"
    )

    joyNode = Node(
        package="joy",
        executable="joy_node",
        parameters=[
            {"autorepeat_rate": 0.0},
        ],
    )
    controllerNode = Node(
        package="teleop",
        executable="controller",
        name="controller",
        parameters=[controllerConfig],
    )
    ld.add_action(joyNode)
    print(controllerConfig)
    ld.add_action(controllerNode)
    return ld
