"""
swarm_comm.launch.py
通信確認用の単体起動launchファイル (Gazebo不要)

共有マップノードと通信テストツールのみを起動する。
Gazeboシミュレーションが別途起動していることを前提とする。

使用方法:
  ros2 launch farm_communication swarm_comm.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = {'use_sim_time': True}

    shared_map_node = Node(
        package='farm_communication',
        executable='shared_map_node',
        name='shared_map_node',
        parameters=[
            use_sim_time,
            {'map_size':   200},
            {'map_res':    0.10},
            {'origin_x': -10.0},
            {'origin_y': -10.0},
            {'publish_hz':  1.0},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        shared_map_node,
    ])
