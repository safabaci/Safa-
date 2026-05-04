import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory("dmg_mori_5axis")
    xacro_file = os.path.join(pkg, "urdf", "dmg_mori.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False
            }]
        ),
    ])
