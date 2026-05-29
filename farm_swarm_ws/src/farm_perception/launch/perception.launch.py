"""
perception.launch.py
全知覚モジュールの統合起動ファイル

起動するノード:
  1. SLAM (Robot ① LiDAR → OccupancyGridマップ)
  2. 雑草検出 (Robot ② 側方RGBD → MarkerArray)
  3. 雑草除去 (Robot ② 除草刃近接 → gz service で削除)
  4. 境界拘束 (全ロボット → 作業エリア外に出た場合に速度補正)

使用方法:
  ros2 launch farm_perception perception.launch.py
  (farm_sim.launch.py から自動で呼び出される)

個別起動:
  ros2 launch farm_perception slam.launch.py
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

    # ---- 2. 雑草検出 (Robot ②) ----
    weed_detector = Node(
        package='farm_perception',
        executable='weed_detector_node',
        name='weed_detector_robot2',
        parameters=[{
            **use_sim_time,
            'robot_id':          'robot2',
            'use_sim_detection': True,    # Gazebo内 → HSV検出
            'min_weed_area_px':  400,
        }],
        output='screen',
    )

    # ---- 3. 雑草除去 ----
    weed_removal = Node(
        package='farm_perception',
        executable='weed_removal_node',
        name='weed_removal_node',
        parameters=[use_sim_time],
        output='screen',
    )

    # ---- 4. 境界拘束 ----
    boundary = Node(
        package='farm_perception',
        executable='boundary_enforcer_node',
        name='boundary_enforcer',
        parameters=[use_sim_time],
        output='screen',
    )

    return LaunchDescription([
        slam_launch,
        weed_detector,
        weed_removal,
        boundary,
    ])
