from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    share_dir = get_package_share_directory('ucar_controller')

    parameter_file = os.path.join(
        share_dir, 'config', 'driver_params_mini.yaml')

    controller_node = Node(package='ucar_controller',
                           executable='ucar_controller_node',
                           output='screen',
                           emulate_tty=True,
                           parameters=[parameter_file],
                           )

    return LaunchDescription([
        controller_node,
    ])
