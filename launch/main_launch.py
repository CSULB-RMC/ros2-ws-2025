from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

'''
This is the launch file for the raspberry pi on the main rover.
package:
    node: "<script_alias>"
    parameters: ["<parameter_file>"]
    
canbus:
    motor_controller: "motor_controller"
    paremeters: ["canbus/config/topics.yaml"]
...
'''
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("canbus"), "config", "topics.yaml"
    )

    canbusNode = Node(
        package="canbus",
        executable="motor_controller",
        parameters=[config],
    )

    ld.add_action(canbusNode)
    return ld
