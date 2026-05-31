"""
perception.launch.py
全知覚モジュールの統合起動ファイル

起動するノード:
  1. SLAM            (Robot① LiDAR → OccupancyGridマップ)
  2. Robot①自律航法  (障害物回避 + レーン維持 + 折り返し)
  3. 縦列制御         (Robot②③ がRobot①を追従)
  4. 雑草検出         (Robot② 側方RGBD → MarkerArray)
  5. 雑草除去         (Robot② 除草刃近接 → gz service で削除)
  6. 境界拘束         (危険ゾーン到達時のみ緊急停止)

使用方法:
  ros2 launch farm_perception perception.launch.py
  (farm_sim.launch.py から自動で呼び出される)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('farm_perception')
    use_sim_time = {'use_sim_time': True}

    # ---- 1. SLAM ----
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'slam.launch.py')
        )
    )

    # ---- 2. Robot① 自律航法 (障害物回避 + レーン維持) ----
    robot1_nav = Node(
        package='farm_perception',
        executable='robot1_navigator_node',
        name='robot1_navigator',
        parameters=[use_sim_time],
        output='screen',
    )

    # ---- 3. 縦列制御 (Robot②③がRobot①を追従) ----
    convoy = Node(
        package='farm_perception',
        executable='convoy_controller_node',
        name='convoy_controller',
        parameters=[use_sim_time],
        output='screen',
    )

    # ---- 4. 雑草検出 (Robot②) ----
    weed_detector = Node(
        package='farm_perception',
        executable='weed_detector_node',
        name='weed_detector_robot2',
        parameters=[{
            **use_sim_time,
            'robot_id':          'robot2',
            'use_sim_detection': True,
            'min_weed_area_px':  400,
        }],
        output='screen',
    )

    # ---- 5. 雑草除去 ----
    weed_removal = Node(
        package='farm_perception',
        executable='weed_removal_node',
        name='weed_removal_node',
        parameters=[use_sim_time],
        output='screen',
    )

    # ---- 6. 境界拘束 (危険ゾーンのみ緊急停止) ----
    boundary = Node(
        package='farm_perception',
        executable='boundary_enforcer_node',
        name='boundary_enforcer',
        parameters=[use_sim_time],
        output='screen',
    )

    return LaunchDescription([
        slam_launch,
        robot1_nav,
        convoy,
        weed_detector,
        weed_removal,
        boundary,
    ])
