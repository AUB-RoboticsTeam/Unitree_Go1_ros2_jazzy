from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('go1_gazebo'), 'worlds', 'simple_rooms.world'
        ]),
        description='SDF world file for Gazebo Sim (gz)'
    )

    # Make gz see both go1_gazebo (for worlds) and go1_description (for model:// meshes)
    gz_res = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[FindPackageShare('go1_description'),
               TextSubstitution(text=':'),
               FindPackageShare('go1_gazebo')]
    )
    gz_res_compat = SetEnvironmentVariable(
        name='GZ_RESOURCE_PATH',
        value=[FindPackageShare('go1_description'),
               TextSubstitution(text=':'),
               FindPackageShare('go1_gazebo')]
    )

    gz_cmd = ExecuteProcess(cmd=['gz', 'sim', world, '-v', '4'], output='screen')

    return LaunchDescription([world_arg, gz_res, gz_res_compat, gz_cmd])

