"""
warehouse_full.launch.py
========================
Complete warehouse simulation: Gazebo Harmonic + ROS 2 Humble + Web Dashboard.

Usage:
  ros2 launch warehouse_simulation warehouse_full.launch.py
  ros2 launch warehouse_simulation warehouse_full.launch.py headless:=true
  ros2 launch warehouse_simulation warehouse_full.launch.py dashboard_only:=true

What this launches:
  1. Gazebo Harmonic (gz sim) with the warehouse world
  2. ROS–GZ bridge for each robot (cmd_vel ↔ odom ↔ scan ↔ tf)
  3. gazebo_sim.py — Flask web dashboard + Gazebo pose bridge
  4. RViz2 (optional, set rviz:=true)

Requirements (Ubuntu 22.04):
  sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
  sudo apt install ros-humble-ros-gz-interfaces
  sudo apt install ros-humble-nav2-bringup
  pip3 install flask flask-socketio eventlet
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    GroupAction, LogInfo, TimerAction, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg        = get_package_share_directory('warehouse_simulation')
    world      = os.path.join(pkg, 'worlds', 'warehouse.world')
    sim        = os.path.join(pkg, 'warehouse_simulation', 'gazebo_sim.py')
    models_dir = os.path.join(pkg, 'models')

    # ── Model resource path ─────────────────────────────────────
    # Gazebo searches GZ_SIM_RESOURCE_PATH for <include uri="model://...">
    existing_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_path = models_dir if not existing_path else f'{models_dir}:{existing_path}'
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_path,
    )

    # ── Declare arguments ───────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_sim_time',   default_value='true',
                              description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('headless',       default_value='false',
                              description='Run Gazebo without GUI'),
        DeclareLaunchArgument('dashboard_only', default_value='false',
                              description='Launch only the web dashboard (no Gazebo)'),
        DeclareLaunchArgument('rviz',           default_value='false',
                              description='Launch RViz2 visualisation'),
        DeclareLaunchArgument('dashboard_port', default_value='5001',
                              description='Flask web dashboard port'),
        DeclareLaunchArgument('num_tugbots',    default_value='4',
                              description='Number of TugBot robots (1-6)'),
    ]

    use_sim_time   = LaunchConfiguration('use_sim_time')
    headless       = LaunchConfiguration('headless')
    dashboard_only = LaunchConfiguration('dashboard_only')
    rviz           = LaunchConfiguration('rviz')
    port           = LaunchConfiguration('dashboard_port')

    # ── 1. Gazebo Harmonic ──────────────────────────────────────
    gz_args = [world]
    gz_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r'] + gz_args,
        output='screen',
        condition=IfCondition(headless),
    )
    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r'] + gz_args,
        output='screen',
        condition=UnlessCondition(headless),
    )

    # ── 2. ROS–GZ bridges ───────────────────────────────────────
    # Each robot gets its own bridge node for cmd_vel, odom, scan, tf
    robot_configs = [
        ('tugbot_1',  'tugbot'),
        ('tugbot_2',  'tugbot'),
        ('tugbot_3',  'tugbot'),
        ('forklift_1','forklift'),
        ('forklift_2','forklift'),
        ('tugbot_manual', 'tugbot'),
    ]

    bridge_nodes = []
    for name, rtype in robot_configs:
        bridge_args = [
            # cmd_vel: ROS → Gazebo
            f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # odometry: Gazebo → ROS
            f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # tf: Gazebo → ROS
            f'/model/{name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ]
        # TugBots also get laser scan
        if rtype == 'tugbot':
            bridge_args.append(
                f'/model/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan')
        # Forklifts get fork_height command bridge
        if rtype == 'forklift':
            bridge_args.append(
                f'/model/{name}/fork_height_cmd@std_msgs/msg/Float64]gz.msgs.Double')

        bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{name}',
            arguments=bridge_args,
            output='screen',
            condition=UnlessCondition(dashboard_only),
        ))

    # Arm joint bridges
    for jnt in ['turret', 'shoulder', 'elbow', 'wrist', 'gripper']:
        bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_arm_{jnt}',
            arguments=[
                f'/arm/{jnt}_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            ],
            output='screen',
            condition=UnlessCondition(dashboard_only),
        ))

    # Joint states bridge for arm
    bridge_nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_arm_joints',
        arguments=[
            '/arm/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
        condition=UnlessCondition(dashboard_only),
    ))

    # ── 3. gazebo_sim.py (web dashboard + Gazebo bridge) ────────
    # Run with --ros flag when Gazebo is also running
    dashboard_with_gz = ExecuteProcess(
        cmd=['python3', sim, '--ros', '--port', port],
        output='screen',
        condition=UnlessCondition(dashboard_only),
    )
    # Standalone dashboard (no Gazebo, no ROS)
    dashboard_standalone = ExecuteProcess(
        cmd=['python3', sim, '--port', port],
        output='screen',
        condition=IfCondition(dashboard_only),
    )

    # ── 4. RViz2 (optional) ─────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
    )

    # ── 5. Fleet status monitor ──────────────────────────────────
    fleet_monitor = Node(
        package='warehouse_simulation',
        executable='warehouse_manager',
        name='warehouse_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(dashboard_only),
    )

    # ── Startup info ─────────────────────────────────────────────
    info = LogInfo(msg=(
        "\n" + "="*60 +
        "\n  Warehouse Simulation v5 — Gazebo + ROS 2 + Dashboard" +
        "\n  Web Dashboard : http://localhost:5001" +
        "\n  Gazebo GUI    : gz topic -e -t /stats" +
        "\n  Fleet status  : ros2 topic echo /fleet/status" +
        "\n  Send command  : ros2 topic pub /fleet/command std_msgs/msg/String" +
        '\n                  \'{"data": "robot 1 go to A3"}\'' +
        "\n" + "="*60
    ))

    return LaunchDescription(
        [set_gz_resource_path] + args + [
            info,
            # Gazebo (delayed 1s to let ROS finish starting)
            TimerAction(period=1.0, actions=[gz_sim_headless, gz_sim_gui]),
            # Bridges (delayed 4s to let Gazebo load the world)
            TimerAction(period=4.0, actions=bridge_nodes),
            # Dashboard + bridge (delayed 5s)
            TimerAction(period=5.0, actions=[
                dashboard_with_gz,
                dashboard_standalone,
                fleet_monitor,
            ]),
            rviz2,
        ]
    )
