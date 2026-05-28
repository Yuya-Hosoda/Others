"""
farm_sim.launch.py
路肩除草シミュレーション全体の起動ファイル

ロボット初期配置 (縦列隊形):
  全ロボットが路肩中央 (Y=5.5) に一列配置、X軸+方向が進行方向

  Robot ①: x=-4,  y=5.5  (先頭: 先導ロボット)
  Robot ②: x=-8,  y=5.5  (中間: 除草ロボット、左側面に除草刃)
  Robot ③: x=-12, y=5.5  (後尾: 回収ロボット)

起動順序:
  1. Gazebo Harmonic (gz sim) + 路肩ワールド
  2. robot_state_publisher (3台分)
  3. ロボットスポーン (縦列隊形)
  4. ros_gz_bridge
  5. 共有マップノード
  6. 雑草スポーン (路肩エリアのみ)
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_desc = get_package_share_directory('farm_description')
    pkg_gz   = get_package_share_directory('farm_gazebo')

    use_sim_time = {'use_sim_time': True}

    world_file = os.path.join(pkg_gz, 'worlds', 'road_shoulder.sdf')
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

    # ---- 2. robot_state_publisher (3台、3秒後) ----
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

    # ---- 3. ロボットスポーン (縦列隊形、5秒後) ----
    # 全ロボットを路肩中央 (Y=5.5) にX軸方向で一列配置
    # yaw=0 でX軸+方向 (道路進行方向) を向く
    spawn_configs = [
        # (name,     x,     y,   z,   yaw)
        ('robot1',  -4.0,  5.5, 0.1, 0.0),   # 先頭: 先導ロボット
        ('robot2',  -8.0,  5.5, 0.1, 0.0),   # 中間: 除草ロボット (左側面に除草刃)
        ('robot3', -12.0,  5.5, 0.1, 0.0),   # 後尾: 回収ロボット
    ]
    spawn_nodes = [
        TimerAction(period=5.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{name}',
                arguments=[
                    '-name',  name,
                    '-topic', f'/{name}/robot_description',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                    '-Y', str(yaw),
                ],
                output='screen',
            )
        ])
        for name, x, y, z, yaw in spawn_configs
    ]

    # ---- 4. ros_gz_bridge (6秒後) ----
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

    # ---- 5. 共有マップノード (8秒後) ----
    shared_map = TimerAction(period=8.0, actions=[
        Node(
            package='farm_communication',
            executable='shared_map_node',
            name='shared_map_node',
            parameters=[use_sim_time],
            output='screen',
        )
    ])

    # ---- 6. 雑草スポーン: 路肩エリアのみ (10秒後) ----
    spawn_script = os.path.join(pkg_gz, 'scripts', 'spawn_weeds.py')
    weed_spawner = TimerAction(period=10.0, actions=[
        ExecuteProcess(
            cmd=['python3', spawn_script],
            output='screen',
        )
    ])

    return [
        gz_sim,
        *rsp_nodes,
        *spawn_nodes,
        bridge,
        shared_map,
        weed_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
