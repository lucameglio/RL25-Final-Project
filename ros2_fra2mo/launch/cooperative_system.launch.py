from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    fra2mo_launch = os.path.join(
        get_package_share_directory('ros2_fra2mo'),
        'launch',
        'fra2mo_navigation.launch.py'
    )

    fra2mo_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fra2mo_launch),
        launch_arguments={'log_level': 'FATAL'}.items() 
    )

    kdl_server_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        name='ros2_kdl_node',
        output='log',
        parameters=[
            {'cmd_interface': 'velocity'},
            {'ctrl': 'velocity_ctrl'},
            {'only_server': True},
        ]
    )

    object_dispatcher_node = Node(
        package='ros2_kdl_package',
        executable='object_dispatcher',
        name='object_dispatcher',
        output='screen'
    )

    fra2mo_object_manager_node = Node(
        package='ros2_fra2mo',
        executable='fra2mo_object_manager',
        name='fra2mo_object_manager',
        output='screen'
    )

    return LaunchDescription([

        fra2mo_navigation,
        kdl_server_node,
        object_dispatcher_node,

        TimerAction(
            period=7.0,
            actions=[fra2mo_object_manager_node]
        ),
    ])
