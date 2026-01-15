from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_kdl_package')
    param_file = os.path.join(pkg_share, 'config', 'kdl_params.yaml')

    ctrl_arg = DeclareLaunchArgument(
        'ctrl', default_value='velocity_ctrl', description='Controller mode'
    )

    return LaunchDescription([
        ctrl_arg,
        Node(
            package='ros2_kdl_package',
            executable='ros2_kdl_node',
            name='ros2_kdl_node',
            output='screen',
            parameters=[
                param_file,
                {'ctrl': LaunchConfiguration('ctrl')}
            ]
        )
    ])

