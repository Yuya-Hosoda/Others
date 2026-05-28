"""
farm_sim.launch.py
農地シミュレーション全体の起動ファイル

起動順序:
  1. Gazebo Harmonic (gz sim) + 農地ワールド
  2. robot_state_publisher (3台分)
  3. ロボットスポーン (3台)
  4. ros_gz_bridge (センサ/制御トピックのブリッジ)
  5. 雑草スポーン
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PKG_DESC = None  # OpaqueFunction内で参照するためモジュールレベルで保持
PKG_GZ   = None


def launch_setup(context, *args, **kwargs):
    pkg_desc = get_package_share_directory('farm_description')
    pkg_gz   = get_package_share_directory('farm_gazebo')

    use_sim_time = {'use_sim_time': True}

    world_file = os.path.join(pkg_gz, 'worlds', 'farm_field.sdf')
    bridge_cfg  = os.path.join(pkg_gz, 'config', 'ros_gz_bridge.yaml')

    # ---- 1. Gazebo Harmonic 起動 ----
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    # ---- ロボットURDFをxacroで展開 ----
    urdf_files = {
        'robot1': os.path.join(pkg_desc, 'urdf', 'robot1_scout.urdf.xacro'),
        'robot2': os.path.join(pkg_desc, 'urdf', 'robot2_weeder.urdf.xacro'),
        'robot3': os.path.join(pkg_desc, 'urdf', 'robot3_collector.urdf.xacro'),
    }
    robot_descs = {
        name: xacro.process_file(path).toxml()
        for name, path in urdf_files.items()
    }

    # ---- 2. robot_state_publisher (3台) ----
    # Gazebo起動から3秒後に起動
    rsp_nodes = [
        TimerAction(period=3.0, actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'{name}_rsp',
                parameters=[
                    {'robot_description': desc},
                    use_sim_time,
                ],
                remappings=[
                    ('/joint_states', f'/{name}/joint_states'),
                    ('/robot_description', f'/{name}/robot_description'),
                ],
                output='screen',
            )
        ])
        for name, desc in robot_descs.items()
    ]

    # ---- 3. ロボットスポーン ----
    # Gazebo起動から5秒後にスポーン
    spawn_positions = {
        'robot1': (-3.0,  0.0, 0.1),
        'robot2': ( 0.0,  0.0, 0.1),
        'robot3': ( 3.0,  0.0, 0.1),
    }
    spawn_nodes = [
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{name}',
                arguments=[
                    '-name',  name,
                    '-topic', f'/{name}/robot_description',
                    '-x', str(pos[0]),
                    '-y', str(pos[1]),
                    '-z', str(pos[2]),
                ],
                output='screen',
            )
        ])
        for name, pos in spawn_positions.items()
    ]

    # ---- 4. ros_gz_bridge ----
    # Gazebo起動から6秒後に起動
    bridge = TimerAction(period=6.0, actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[
                {'config_file': bridge_cfg},
                use_sim_time,
            ],
            output='screen',
        )
    ])

    # ---- 5. 雑草スポーン ----
    # Node()はlibexecの実行権限が必要なため python3 で直接起動する
    spawn_script = os.path.join(pkg_gz, 'scripts', 'spawn_weeds.py')
    weed_spawner = TimerAction(period=10.0, actions=[
        ExecuteProcess(
            cmd=['python3', spawn_script],
            output='screen',
        )
    ])

    # ---- 共有マップノード ----
    shared_map = TimerAction(period=8.0, actions=[
        Node(
            package='farm_communication',
            executable='shared_map_node',
            name='shared_map_node',
            parameters=[use_sim_time],
            output='screen',
        )
    ])

    return [
        gz_sim,
        *rsp_nodes,
        *spawn_nodes,
        bridge,
        weed_spawner,
        shared_map,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
