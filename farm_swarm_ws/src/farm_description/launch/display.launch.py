"""
display.launch.py
RViz2でURDFモデルを単体表示・確認するためのlaunchファイル
Gazebo不要 - URDF構造の確認に使用する

使用方法:
  ros2 launch farm_description display.launch.py robot:=robot1
  ros2 launch farm_description display.launch.py robot:=robot2
  ros2 launch farm_description display.launch.py robot:=robot3
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_desc = get_package_share_directory('farm_description')
    robot = context.launch_configurations['robot']

    urdf_map = {
        'robot1': 'robot1_scout.urdf.xacro',
        'robot2': 'robot2_weeder.urdf.xacro',
        'robot3': 'robot3_collector.urdf.xacro',
    }
    urdf_file = os.path.join(pkg_desc, 'urdf', urdf_map[robot])
    robot_desc = xacro.process_file(urdf_file).toxml()

    # robot_state_publisher: /robot_description にパブリッシュ
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 50.0,
        }],
        remappings=[
            ('/robot_description', '/robot_description'),
        ],
        output='screen',
    )

    # joint_state_publisher_gui: スライダーでジョイント操作
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # RViz2設定ファイル
    rviz_cfg = os.path.join(pkg_desc, 'config', 'display.rviz')
    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen',
    )
    return [rsp, jsp_gui, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='robot1',
            description='表示するロボット名 (robot1 / robot2 / robot3)',
            choices=['robot1', 'robot2', 'robot3'],
        ),
        OpaqueFunction(function=launch_setup),
    ])
