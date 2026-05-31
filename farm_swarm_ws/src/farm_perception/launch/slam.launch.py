"""
slam.launch.py
Robot ① (Scout) のLiDARデータを用いたSLAM起動ファイル

処理パイプライン:
  /robot1/lidar/points (PointCloud2)
      ↓ pointcloud_to_laserscan
  /robot1/scan_2d (LaserScan)
      ↓ slam_toolbox (async mapping)
  /map (OccupancyGrid)

使用方法:
  ros2 launch farm_perception slam.launch.py
  (farm_sim.launch.py から自動で呼び出される)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('farm_perception')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    use_sim_time = {'use_sim_time': True}

    # ---- pointcloud_to_laserscan ----
    # 3D LiDAR点群 → 2D LaserScan に変換 (slam_toolbox入力用)
    pc2scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pc_to_laserscan',
        parameters=[{
            **use_sim_time,
            'target_frame':         'robot1/base_link',
            'transform_tolerance':  0.05,
            'min_height':           0.05,   # 地面反射を除去
            'max_height':           1.50,   # 頭上の障害物を含む
            'angle_min':           -3.14159,
            'angle_max':            3.14159,
            'angle_increment':      0.01745,  # 1.0 deg
            'scan_time':            0.1,
            'range_min':            0.1,
            'range_max':           100.0,
            'use_inf':              True,
        }],
        remappings=[
            ('cloud_in', '/robot1/lidar/points'),
            ('scan',     '/robot1/scan_2d'),
        ],
        output='screen',
    )

    # ---- slam_toolbox (async mapping mode) ----
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params,
            use_sim_time,
        ],
        output='screen',
    )

    return LaunchDescription([pc2scan, slam])
