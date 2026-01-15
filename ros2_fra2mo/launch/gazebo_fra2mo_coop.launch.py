import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():

    pkg_share = get_package_share_directory('ros2_fra2mo')
    xacro_file = os.path.join(pkg_share, 'urdf', 'fra2mo.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'coop_world.sdf')
    controllers_file = os.path.join(pkg_share, 'config', 'fra2mo_controllers.yaml')
    models_path = os.path.join(pkg_share, 'models')

    robot_description = {"robot_description": ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='fra2mo',
        parameters=[robot_description, {"use_sim_time": True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'fra2mo/robot_description',
            '-name', 'fra2mo',
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '-3.0',
            '-z', '0.1'
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='fra2mo',
        arguments=['joint_state_broadcaster', '--controller-manager', '/fra2mo/controller_manager'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='fra2mo',
        arguments=['fra2mo_gripper_controller', '--controller-manager', '/fra2mo/controller_manager'],
        output='screen'
    )

    delay_jsb_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    delay_gripper_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner]
        )
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='fra2mo',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            'image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            'camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        namespace='fra2mo',
        parameters=[{"use_sim_time": True}]
    )

    ign_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=["fra2mo/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="screen",
        namespace="fra2mo"
    )

    nodes_to_start = [
        robot_state_publisher_node,
        spawn_entity,
        delay_jsb_after_control_node,
        delay_gripper_after_jsb,
        bridge,
        odom_tf,
        ign_clock_bridge
    ]

    return LaunchDescription(
        [SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))] + nodes_to_start
    )