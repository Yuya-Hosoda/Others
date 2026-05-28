#!/usr/bin/env python3
"""
spawn_weeds.py
Gazebo Harmonic上に雑草モデルをランダムに配置するROS 2ノード

使用方法:
  ros2 run farm_gazebo spawn_weeds.py
  ros2 run farm_gazebo spawn_weeds.py --ros-args -p num_weeds:=60 -p field_size:=16.0
"""
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
import random
import math
import os


class WeedSpawner(Node):
    def __init__(self):
        super().__init__('weed_spawner')

        # パラメータ宣言
        self.declare_parameter('num_weeds',  40)
        self.declare_parameter('field_size', 18.0)   # フィールド有効範囲 [m]
        self.declare_parameter('small_ratio', 0.6)   # 小型雑草の割合
        self.declare_parameter('exclusion_radius', 2.0)  # 原点周辺のスポーン除外半径
        self.declare_parameter('model_base_path', '')    # カスタムモデルパス

        # SpawnEntityサービスクライアント
        self.spawn_client = self.create_client(
            SpawnEntity, '/world/farm_field/create'
        )
        self.get_logger().info('SpawnEntityサービス待機中...')
        if not self.spawn_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                'SpawnEntityサービスが見つかりません。Gazeboが起動しているか確認してください。'
            )
            return
        self.get_logger().info('SpawnEntityサービス接続完了')

    def spawn_all_weeds(self):
        num_weeds   = self.get_parameter('num_weeds').value
        field_size  = self.get_parameter('field_size').value
        small_ratio = self.get_parameter('small_ratio').value
        excl_r      = self.get_parameter('exclusion_radius').value

        spawned = 0
        attempts = 0
        max_attempts = num_weeds * 5

        while spawned < num_weeds and attempts < max_attempts:
            attempts += 1
            half = field_size / 2.0
            x = random.uniform(-half, half)
            y = random.uniform(-half, half)

            # 原点周辺 (ロボット初期位置) には配置しない
            if math.sqrt(x**2 + y**2) < excl_r:
                continue

            weed_type = 'weed_small' if random.random() < small_ratio else 'weed_large'
            yaw = random.uniform(0, math.pi * 2)
            name = f'weed_{spawned:03d}'

            success = self._spawn_model(name, weed_type, x, y, yaw)
            if success:
                spawned += 1
                if spawned % 10 == 0:
                    self.get_logger().info(f'{spawned}/{num_weeds} 本の雑草を配置しました')

        self.get_logger().info(
            f'雑草配置完了: {spawned}/{num_weeds} 本 (試行回数: {attempts})'
        )

    def _spawn_model(self, name: str, model_type: str, x: float, y: float, yaw: float) -> bool:
        """SDFモデルをGazeboにスポーンする"""
        # モデルパスを探索
        model_dirs = [
            self.get_parameter('model_base_path').value,
            os.path.join(
                os.getenv('AMENT_PREFIX_PATH', '').split(':')[0],
                'share', 'farm_gazebo', 'models'
            ),
            os.path.expanduser('~/.gz/models'),
        ]

        sdf_content = None
        for base in model_dirs:
            if not base:
                continue
            sdf_path = os.path.join(base, model_type, 'model.sdf')
            if os.path.exists(sdf_path):
                with open(sdf_path, 'r') as f:
                    sdf_content = f.read()
                break

        if sdf_content is None:
            # フォールバック: インラインSDF
            sdf_content = self._generate_inline_sdf(name, model_type)

        req = SpawnEntity.Request()
        req.xml = sdf_content
        req.name = name
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.0
        req.initial_pose.orientation.z = math.sin(yaw / 2)
        req.initial_pose.orientation.w = math.cos(yaw / 2)

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            return True
        else:
            self.get_logger().warn(f'{name} のスポーンに失敗しました')
            return False

    def _generate_inline_sdf(self, name: str, model_type: str) -> str:
        """モデルファイルが見つからない場合のフォールバックSDF"""
        if model_type == 'weed_small':
            radius, height = 0.06, 0.12
        else:
            radius, height = 0.12, 0.25

        return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="weed_link">
      <visual name="weed_visual">
        <pose>0 0 {height/2} 0 0 0</pose>
        <geometry>
          <cylinder><radius>{radius}</radius><length>{height}</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.6 0.1 1</ambient>
          <diffuse>0.15 0.7 0.15 1</diffuse>
        </material>
      </visual>
      <collision name="weed_collision">
        <pose>0 0 {height/2} 0 0 0</pose>
        <geometry>
          <cylinder><radius>{radius}</radius><length>{height}</length></cylinder>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""


def main(args=None):
    rclpy.init(args=args)
    node = WeedSpawner()
    node.spawn_all_weeds()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
