"""Launch Gazebo, the ROS-Gazebo bridge, EKF-SLAM, control, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('aovp_ekf_slam')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    world = os.path.join(package_share, 'worlds', 'ekf_slam_demo.sdf')
    urdf = os.path.join(package_share, 'urdf', 'front_drive_cart.urdf')
    params = os.path.join(package_share, 'config', 'params.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'demo.rviz')

    with open(urdf, 'r', encoding='utf-8') as robot_file:
        robot_description = robot_file.read()

    rviz_argument = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz2')
    gz_gui_argument = DeclareLaunchArgument(
        'gz_gui', default_value='true', description='Start Gazebo GUI')

    # -s starts the server, -g adds the graphical client.  Two conditional
    # includes keep headless CI and a normal desktop launch equally simple.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v 3 {world}'}.items(),
    )
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v 3'}.items(),
        condition=IfCondition(LaunchConfiguration('gz_gui')),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/scan', '/scan')],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen',
    )
    slam = Node(
        package='aovp_ekf_slam',
        executable='ekf_slam_node',
        name='ekf_slam',
        parameters=[params],
        output='screen',
    )
    controller = Node(
        package='aovp_ekf_slam',
        executable='motion_controller',
        name='motion_controller',
        parameters=[params],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_argument,
        gz_gui_argument,
        gazebo_server,
        gazebo_gui,
        bridge,
        robot_state_publisher,
        slam,
        controller,
        rviz,
    ])
