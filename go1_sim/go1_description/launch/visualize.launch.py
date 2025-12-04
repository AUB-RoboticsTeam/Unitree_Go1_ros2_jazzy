import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    package_description = "go1_description"
    pkg_path = get_package_share_directory(package_description)

    # ================= Launch Arguments =================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher', default='True')
    sim_backend = LaunchConfiguration('sim_backend', default='gz')
    debug_mode = LaunchConfiguration('DEBUG', default='false')

    # ================= Paths =================
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot2.xacro')

    # ================= Robot Description =================
    # Correctly separate each part of the xacro command
    robot_description_content = Command([
        "xacro", xacro_file,
        "sim_backend:=", sim_backend,
        "DEBUG:=", debug_mode
    ])

    robot_description_param = {
        'robot_description': robot_description_content,
        'use_sim_time': use_sim_time
    }

    # ================= Nodes =================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[robot_description_param]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_node',
        output='screen',
        condition=IfCondition(use_joint_state_publisher)
    )

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'go1_vis.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    # ================= Launch Description =================
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('use_joint_state_publisher', default_value='True', description='Start joint_state_publisher'),
        DeclareLaunchArgument('sim_backend', default_value='gz', description='Simulation backend: gz or classic'),
        DeclareLaunchArgument('DEBUG', default_value='false', description='Enable debug mode'),

        # Launch nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])

