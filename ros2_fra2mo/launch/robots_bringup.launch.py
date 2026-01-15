import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_file = os.path.join(get_package_share_directory('ros2_fra2mo'), "worlds", "coop_world.sdf")
    models_path = os.path.join(get_package_share_directory('ros2_fra2mo'), 'models')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': [world_file, ' -r']}.items()
    )

    iiwa_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('iiwa_description'),
            '/launch/gazebo_iiwa_coop.launch.py'
        ]),
        launch_arguments={
            'use_sim': 'true',
            'start_rviz': 'false',
            'robot_controller': 'velocity_controller',
            'command_interface': 'velocity',
        }.items()
    )

    fra2mo_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros2_fra2mo'),
            '/launch/gazebo_fra2mo_coop.launch.py'
        ]),
        launch_arguments={}.items()
    )

    delayed_fra2mo = TimerAction(
        period=5.0,  
        actions=[fra2mo_gazebo]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", 
            value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        gazebo_launch,
        iiwa_gazebo,
        delayed_fra2mo  
    ])