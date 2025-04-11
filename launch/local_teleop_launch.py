import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

'''
This is the launch file for testing CAN and teleop controls on a single system.
package:
    node: <script_alias>
    parameters: ["<parameter_file>"]

    joy:    # ros library node for handling joystick inputs
        joy_node: "joy_node"
        parameters: [autorepeat_rate: 0.0]  # disables auto-repeat
        
    teleop:
        controller: "controller"
        parameters: ["teleop/config/controller.yaml"]

    canbus:
        motor_controller: "motor_controller"
        paremeters: ["canbus/config/topics.yaml"]
...
'''
def generate_launch_description():
    ld = LaunchDescription()
    controllerConfig = os.path.join(
        get_package_share_directory("teleop"), "config", "controller.yaml"
    )
    
    config = os.path.join(
        get_package_share_directory("canbus"), "config", "topics.yaml"
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

    canbusNode = Node(
        package="canbus",
        executable="motor_controller",
        parameters=[config],
    )

    ld.add_action(canbusNode)
    ld.add_action(joyNode)
    ld.add_action(controllerNode)
    return ld
