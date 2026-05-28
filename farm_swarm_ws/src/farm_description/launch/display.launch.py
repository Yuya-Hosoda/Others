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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_desc = get_package_share_directory('farm_description')

    # 表示するロボットを引数で選択
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='robot1',
        description='表示するロボット名 (robot1 / robot2 / robot3)',
        choices=['robot1', 'robot2', 'robot3'],
    )
    robot_name = LaunchConfiguration('robot')

    def get_robot_description(context, *args, **kwargs):
        robot = context.launch_configurations['robot']
        urdf_file = os.path.join(
            pkg_desc, 'urdf', f'{robot}_scout.urdf.xacro'
            if robot == 'robot1' else
            f'{robot}_weeder.urdf.xacro'
            if robot == 'robot2' else
            f'{robot}_collector.urdf.xacro'
        )
        doc = xacro.process_file(urdf_file)
        return doc.toxml()

    from launch.actions import OpaqueFunction

    def launch_setup(context):
        robot = context.launch_configurations['robot']
        urdf_map = {
            'robot1': 'robot1_scout.urdf.xacro',
            'robot2': 'robot2_weeder.urdf.xacro',
            'robot3': 'robot3_collector.urdf.xacro',
        }
        urdf_file = os.path.join(pkg_desc, 'urdf', urdf_map[robot])
        robot_desc = xacro.process_file(urdf_file).toxml()

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        )
        jsp_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )
        rviz_cfg = os.path.join(pkg_desc, 'config', 'display.rviz')
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
            output='screen',
        )
        return [rsp, jsp_gui, rviz]

    return LaunchDescription([
        robot_arg,
        OpaqueFunction(function=launch_setup),
    ])
