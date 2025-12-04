#!/usr/bin/env python3
import os
import pathlib
import tempfile
import subprocess
import textwrap

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# For optional gating on robot settling before starting controllers
import rclpy
from rclpy.node import Node as RclNode
from nav_msgs.msg import Odometry
import time


# ------------------ helpers ---------------------------------------------------
def _generate_abs_urdf_from_pkg(context):
    urdf_arg = LaunchConfiguration('urdf_file').perform(context)
    pkg_dir = pathlib.Path(FindPackageShare('go1_description').perform(context))
    candidate = pathlib.Path(urdf_arg)
    if not candidate.is_absolute():
        candidate = (pkg_dir / 'xacro' / urdf_arg
                     if urdf_arg.endswith('.xacro')
                     else pkg_dir / 'urdf' / urdf_arg)
    if not candidate.exists():
        raise FileNotFoundError(f"URDF/XACRO not found: {candidate}")

    if candidate.suffix == '.xacro':
        sim_backend = LaunchConfiguration('sim_backend').perform(context) or 'gz'
        debug_mode = LaunchConfiguration('DEBUG').perform(context) or 'false'
        cmd = ['xacro', '--inorder', str(candidate),
               f"sim_backend:={sim_backend}", f"DEBUG:={debug_mode}"]
        urdf_text = subprocess.check_output(cmd, encoding='utf-8')
    else:
        urdf_text = candidate.read_text(encoding='utf-8')

    pkg_dir_fs = get_package_share_directory('go1_description')
    urdf_text = urdf_text.replace('package://go1_description', f'file://{pkg_dir_fs}')

    fd, tmp_path = tempfile.mkstemp(prefix='go1_abs_', suffix='.urdf')
    os.close(fd)
    pathlib.Path(tmp_path).write_text(urdf_text, encoding='utf-8')
    return tmp_path


def _write_robot_description_yaml(urdf_text: str) -> str:
    # Global block (/**) so any node, including controllers, can see robot_description
    global_block = textwrap.dedent("""\
    /**:
      ros__parameters:
        robot_description: |-
    """) + textwrap.indent(urdf_text, "      ") + "\n"

    # Per-controller sections (spawner also matches by controller name)
    controllers = [f"{leg}_{j}_controller" for leg in ("FR", "FL", "RR", "RL") for j in ("hip", "thigh", "calf")]
    sections = []
    for c in controllers:
        sections.append(
            f"{c}:\n  ros__parameters:\n    robot_description: |-\n" + textwrap.indent(urdf_text, "      ")
        )
    body = global_block + "\n".join(sections) + "\n"

    fd, tmp_path = tempfile.mkstemp(prefix='rd_', suffix='.yaml')
    os.close(fd)
    pathlib.Path(tmp_path).write_text(body, encoding='utf-8')
    return tmp_path


def _msgs_ns(context):
    use_ign = LaunchConfiguration('use_ignition_msgs').perform(context).lower() in ('1', 'true', 'yes', 'on')
    return 'ignition.msgs' if use_ign else 'gz.msgs'


def _build_gz(context, *args, **kwargs):
    world_filename = LaunchConfiguration('world_file_name').perform(context)
    world_abs = os.path.join(get_package_share_directory('go1_gazebo'), 'worlds', world_filename)
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': TextSubstitution(text=f" -r {world_abs}"),
            'gui': 'true',
            'verbose': 'false'
        }.items()
    )
    return [gz]


def _build_nodes(context, *args, **kwargs):
    # pose/name
    name = LaunchConfiguration('name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)

    # toggles & delays
    start_leg_ctrls = LaunchConfiguration('start_leg_controllers').perform(context).lower() in ('1', 'true', 'yes', 'on')
    delay_load_cfg = float(LaunchConfiguration('leg_ctrl_load_config_delay').perform(context))
    jsb_delay = float(LaunchConfiguration('joint_state_broadcaster_delay').perform(context))
    auto_after_fall = LaunchConfiguration('auto_start_after_fall').perform(context).lower() in ('1','true','yes','on')
    settle_z = float(LaunchConfiguration('settle_z').perform(context))
    settle_vz = float(LaunchConfiguration('settle_vz').perform(context))
    settle_time = float(LaunchConfiguration('settle_time').perform(context))

    world_name = LaunchConfiguration('world_name').perform(context)
    _ = _msgs_ns(context)

    # generate URDF (absolute) and create a param file exposing robot_description
    abs_urdf = _generate_abs_urdf_from_pkg(context)
    urdf_text = pathlib.Path(abs_urdf).read_text(encoding='utf-8')
    rd_yaml = _write_robot_description_yaml(urdf_text)

    # 1) robot_state_publisher (skip when visualize brings its own)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 100.0,
            'robot_description': urdf_text
        }],
        condition=UnlessCondition(LaunchConfiguration('with_visualize'))
    )

    # 2) spawn robot (slightly delayed to ensure /robot_description is ready)
    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_go1', output='screen',
        arguments=['-file', abs_urdf, '-name', name, '-x', x, '-y', y, '-z', z],
    )
    delayed_spawn = TimerAction(period=0.0, actions=[spawn])

    bridge_args = [
        # Clock (world time)
        f'/world/{world_name}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',

        # Odom from plugin (already in ROS, no /world prefix)
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',

        # Body IMU plugin
        '/imu/imu_sensor@sensor_msgs/msg/Imu@gz.msgs.IMU',

        # 2D lidar (if you have it)
        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    ]

    # === D435i camera bridge topics (Gazebo Sim -> ROS 2) ===
    d435i_topics = [
        # Color image
        '/gz/d435i_front/color/image'
        '@sensor_msgs/msg/Image@gz.msgs.Image',

        # Infrared left image
        '/gz/d435i_front/infra1/image'
        '@sensor_msgs/msg/Image@gz.msgs.Image',

        # Infrared right image
        '/gz/d435i_front/infra2/image'
        '@sensor_msgs/msg/Image@gz.msgs.Image',

        # Depth image
        '/gz/d435i_front/depth/image'
        '@sensor_msgs/msg/Image@gz.msgs.Image',

        # Color camera info
        '/gz/d435i_front/color/camera_info'
        '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

        # Depth camera info
        '/gz/d435i_front/depth/camera_info'
        '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

        # D435i IMU
        '/gz/d435i_front/imu'
        '@sensor_msgs/msg/Imu@gz.msgs.IMU',
    ]
    bridge_args.extend(d435i_topics)

    # Contact sensors on foot links for ground contact detection
    contact_topics = [
        f'/world/{world_name}/model/test_block/link/link/sensor/contact_sensor/contact'
        '@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
    ]
    bridge_args.extend(contact_topics)

    # Also bridge world-level contacts as a robust fallback
    bridge_args.append(
        f'/world/{world_name}/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='go1_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=[
            # Let everyone use /clock
            (f'/world/{world_name}/clock', '/clock'),

            # IMU remap for your existing controller (body IMU)
            ('/imu/imu_sensor', '/imu_plugin/out'),

            # === D435i camera topic remaps (gz -> ROS standard names) ===

            # Color
            ('/gz/d435i_front/color/image', '/camera/color/image_raw'),
            ('/gz/d435i_front/color/camera_info', '/camera/color/camera_info'),

            # Infrared stereo
            ('/gz/d435i_front/infra1/image', '/camera/infra1/image_rect_raw'),
            ('/gz/d435i_front/infra2/image', '/camera/infra2/image_rect_raw'),

            # Depth
            ('/gz/d435i_front/depth/image', '/camera/depth/image_rect_raw'),
            ('/gz/d435i_front/depth/camera_info', '/camera/depth/camera_info'),

            # D435i IMU
            ('/gz/d435i_front/imu', '/camera/imu'),
        ],
        parameters=[{'use_sim_time': True}],
    )



    # 4) joint_state_broadcaster (delayed)
    spawner_jsb = Node(
        package='controller_manager', executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', rd_yaml,
        ],
        output='screen',
    )
    delayed_jsb = TimerAction(period=jsb_delay, actions=[spawner_jsb])

    # 5) leg controllers (load + activate all at once like manual command)
    leg_controllers = [f'{leg}_{j}_controller' for leg in ('FR', 'FL', 'RR', 'RL') for j in ('hip', 'thigh', 'calf')]
    load_cfg_actions = [
        Node(
            package='controller_manager', executable='spawner',
            name='spawner_all_leg_controllers',
            arguments=leg_controllers + [
                '--controller-manager', '/controller_manager',
                '--param-file', rd_yaml,
                '--activate',
            ],
            output='screen',
            condition=IfCondition(str(start_leg_ctrls).lower())
        )
    ]
    delayed_load_cfg = TimerAction(
        period=delay_load_cfg,
        actions=load_cfg_actions,
        condition=IfCondition(str(start_leg_ctrls).lower()),
    )
    # Static TF for the RGB-D camera frame used by the point cloud
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_rgbd_tf',
        arguments=[
            # xyz (pose of the camera frame relative to TRUNK)
            '0.25', '0.0', '0.20',
            # rpy (roll pitch yaw in radians)
            '0', '0', '0',
            # parent frame  (this *does* exist in your URDF)
            'trunk',
            # child frame   (must match the frame_id from the point cloud!)
            'go1/base_link/realsense_rgbd',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 5b) Optional: wait until the robot has fallen/settled, then start JSB and controllers
    def _wait_then_return_actions(context):
        if not auto_after_fall:
            return []
        # Wait for /odom and for z and vz to be under thresholds for settle_time seconds
        rclpy.init(args=[])
        n = RclNode('wait_until_settled')
        last_ok = None
        done = False
        def cb(msg):
            nonlocal last_ok, done
            zpos = float(msg.pose.pose.position.z)
            vz = float(msg.twist.twist.linear.z)
            ok = (zpos <= settle_z) and (abs(vz) <= settle_vz)
            now = time.monotonic()
            if ok:
                if last_ok is None:
                    last_ok = now
                elif now - last_ok >= settle_time:
                    done = True
            else:
                last_ok = None
        sub = n.create_subscription(Odometry, '/odom', cb, 10)
        start = time.monotonic()
        timeout = 5.0
        while rclpy.ok() and not done and (time.monotonic() - start) < timeout:
            rclpy.spin_once(n, timeout_sec=0.1)
        n.destroy_subscription(sub)
        n.destroy_node()
        rclpy.shutdown()
        # After settle, start JSB and controllers
        actions = [spawner_jsb]
        if start_leg_ctrls:
            actions.extend(load_cfg_actions)
        return actions

    after_fall = OpaqueFunction(function=_wait_then_return_actions)

    # 6) wait until /clock is publishing once, then start TF helper and RViz
    wait_for_clock = ExecuteProcess(
        cmd=['bash', '-lc', 'until ros2 topic echo /clock -n 1 >/dev/null 2>&1; do sleep 0.2; done'],
        output='screen',
    )

    odom_tf = Node(
        package='go1_navigation', executable='nav_tf_publisher',
        name='odom_transform_publisher', output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    start_odom_tf = TimerAction(period=0.1, actions=[odom_tf])

    visualize_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('go1_description').perform(context), 'launch', 'go1_visualize.launch.py')
        ]),
        launch_arguments={
            'use_joint_state_publisher': LaunchConfiguration('use_joint_state_publisher'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'urdf_file': LaunchConfiguration('urdf_file')
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_visualize')),
    )
    start_visualize = TimerAction(period=0.2, actions=[visualize_robot])

    # Add unitree_guide2 controller for coordinated robot behavior
   # unitree_guide = Node(
    #   package='unitree_guide2',
     #   executable='junior_ctrl', 
     #   name='unitree_guide_controller',
     #   output='screen',
     #   parameters=[{
     #       'use_sim_time': LaunchConfiguration('use_sim_time'),
     #       'robot_name': 'go1',  # Fixed to match the actual robot
     #       'control_freq': 200.0  # Reduce from default 500Hz to 200Hz for stability
    #    }]
    #)
    #start_guide = TimerAction(period=2.5, actions=[unitree_guide])  # Start after controllers are ready
    
    # Sequence: bridge -> wait for /clock -> (odom_tf, rviz, guide)
    after_bridge = [
        wait_for_clock,
        start_odom_tf,
        start_visualize,
       # start_guide,
    ]

    # 3b) Contact conversion: Contacts → WrenchStamped expected by controller
    contacts_to_wrench = Node(
        package='go1_gazebo', executable='contacts_to_wrench.py', name='contacts_to_wrench', output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'world_name': LaunchConfiguration('world_name')}],
    )

    actions = [rsp, delayed_spawn, bridge, contacts_to_wrench, camera_tf]
    if auto_after_fall:
        actions.append(after_fall)
    else:
        actions.extend([delayed_jsb, delayed_load_cfg])
    # after the bridge starts, gate rviz/tf on the clock being present
    actions.extend(after_bridge)
    return actions


# ------------------ main launch description ----------------------------------
def generate_launch_description():
    world = DeclareLaunchArgument('world_file_name', default_value='default.world')
    world_name = DeclareLaunchArgument('world_name', default_value='default')
    urdf = DeclareLaunchArgument('urdf_file', default_value='robot.xacro')
    name = DeclareLaunchArgument('name', default_value='go1')
    x = DeclareLaunchArgument('x', default_value='0.0')
    y = DeclareLaunchArgument('y', default_value='0.0')
    z = DeclareLaunchArgument('z', default_value='0.6')
    start_leg_controllers = DeclareLaunchArgument('start_leg_controllers', default_value='true')

    joint_state_broadcaster_delay = DeclareLaunchArgument('joint_state_broadcaster_delay', default_value='0.5')
    leg_ctrl_load_config_delay = DeclareLaunchArgument('leg_ctrl_load_config_delay', default_value='0.8')

    # Auto-start controllers after the robot has fallen/settled (uses /odom) 
    auto_start_after_fall = DeclareLaunchArgument('auto_start_after_fall', default_value='false')
    settle_z = DeclareLaunchArgument('settle_z', default_value='0.25')
    settle_vz = DeclareLaunchArgument('settle_vz', default_value='0.05')
    settle_time = DeclareLaunchArgument('settle_time', default_value='0.5')

    use_joint_state_publisher = DeclareLaunchArgument('use_joint_state_publisher', default_value='false')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    sim_backend = DeclareLaunchArgument('sim_backend', default_value='gz')
    debug_mode = DeclareLaunchArgument('DEBUG', default_value='false')
    use_ignition_msgs = DeclareLaunchArgument('use_ignition_msgs', default_value='false')
    with_visualize = DeclareLaunchArgument('with_visualize', default_value='true')

    res_list = [FindPackageShare('go1_description'), TextSubstitution(text=':'), FindPackageShare('go1_gazebo')]
    gz_res = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=res_list)
    gz_res_compat = SetEnvironmentVariable(name='GZ_RESOURCE_PATH', value=res_list)

    return LaunchDescription([
        world, world_name, urdf, name, x, y, z,
        start_leg_controllers,
        joint_state_broadcaster_delay, leg_ctrl_load_config_delay,
        auto_start_after_fall, settle_z, settle_vz, settle_time,
        use_joint_state_publisher, use_sim_time, sim_backend, debug_mode, use_ignition_msgs,
        with_visualize,
        gz_res, gz_res_compat,
        OpaqueFunction(function=_build_gz),
        OpaqueFunction(function=_build_nodes),
    ])
