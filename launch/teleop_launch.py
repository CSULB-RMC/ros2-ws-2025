import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

'''
This is the launch file for the teleop computer controlling the rover.
package:
    node: <script_alias>
    parameters: ["<parameter_file>"]

joy:    # ros library node for handling joystick inputs
    joy_node: "joy_node"
    parameters: [autorepeat_rate: 0.0]  # disables auto-repeat
    
teleop:
    controller: "controller"
    parameters: ["teleop/config/controller.yaml"]
...
'''
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
        parameters=[controllerConfig],
    )
    ld.add_action(joyNode)
    print(controllerConfig)
    ld.add_action(controllerNode)
    return ld
