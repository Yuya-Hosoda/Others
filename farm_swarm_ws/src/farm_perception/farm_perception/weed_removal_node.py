#!/usr/bin/env python3
"""
weed_removal_node.py
Robot ②の除草刃位置と登録済み雑草位置の近接判定により、
触れた雑草をGazeboから削除する。

購読トピック:
  /robot2/odom           (nav_msgs/Odometry)  - robot2の位置・姿勢
  /swarm/weed_registry   (std_msgs/String)     - 雑草位置JSON

パブリッシュ:
  /swarm/weed_removed    (std_msgs/String)     - 除草完了イベントJSON
  /swarm/weed_registry   (std_msgs/String)     - 更新後の雑草レジストリ
"""
import json
import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String


# Robot ②のURDFで定義した除草刃の取付オフセット (base_linkからの距離)
BLADE_OFFSET_X =  0.05   # [m]
BLADE_OFFSET_Y =  0.28   # [m]  (+Y: 路肩方向)
BLADE_OFFSET_Z = -0.04   # [m]
BLADE_RADIUS   =  0.14   # [m]  回転刃半径
REMOVAL_DIST   =  BLADE_RADIUS + 0.08   # 有効除草距離 [m]

WORLD_NAME = 'road_shoulder'


class WeedRemovalNode(Node):

    def __init__(self):
        super().__init__('weed_removal_node')

        self._weeds: dict[str, tuple[float, float]] = {}   # {name: (x, y)}
        self._removed: set[str]  = set()
        self._lock = threading.Lock()
        self._robot2_pose: tuple[float, float, float] | None = None  # (x, y, yaw)

        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # weed_registry は TransientLocal で受信:
        # spawn_weeds.py が終了した後でも最後のメッセージを受け取れる
        qos_registry = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Odometry, '/robot2/odom', self._odom_callback, qos_sensor
        )
        self.create_subscription(
            String, '/swarm/weed_registry', self._registry_callback, qos_registry
        )

        self.removed_pub  = self.create_publisher(String, '/swarm/weed_removed', 10)
        self.registry_pub = self.create_publisher(String, '/swarm/weed_registry', qos_registry)

        # 10Hzで近接チェック
        self.create_timer(0.1, self._check_proximity)

        self.get_logger().info(
            f'WeedRemovalNode起動: 除草距離={REMOVAL_DIST:.2f}m, '
            f'ワールド={WORLD_NAME}'
        )

    # ================================================================
    #  コールバック
    # ================================================================

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self._robot2_pose = (x, y, yaw)

    def _registry_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._weeds = {
                    w['name']: (float(w['x']), float(w['y']))
                    for w in data.get('weeds', [])
                    if w['name'] not in self._removed
                }
        except Exception as e:
            self.get_logger().warn(f'weed_registry parse error: {e}')

    # ================================================================
    #  近接チェック・除草実行
    # ================================================================

    def _check_proximity(self) -> None:
        if self._robot2_pose is None:
            return

        rx, ry, yaw = self._robot2_pose

        # base_link座標系でのblade位置 → ワールド座標へ変換
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        blade_wx = rx + cos_y * BLADE_OFFSET_X - sin_y * BLADE_OFFSET_Y
        blade_wy = ry + sin_y * BLADE_OFFSET_X + cos_y * BLADE_OFFSET_Y

        to_remove = []
        with self._lock:
            for name, (wx, wy) in self._weeds.items():
                dist = math.sqrt((blade_wx - wx)**2 + (blade_wy - wy)**2)
                if dist < REMOVAL_DIST:
                    to_remove.append(name)

        for name in to_remove:
            self._remove_weed(name, blade_wx, blade_wy)

    def _remove_weed(self, weed_name: str, blade_x: float, blade_y: float) -> None:
        """gz serviceでGazeboからモデルを削除し、レジストリを更新する"""
        if weed_name in self._removed:
            return

        cmd = [
            'gz', 'service',
            '-s', f'/world/{WORLD_NAME}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--req',     f'name: "{weed_name}" type: MODEL',
            '--timeout', '1000',
        ]
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=2.0
            )
            if result.returncode == 0:
                with self._lock:
                    self._removed.add(weed_name)
                    self._weeds.pop(weed_name, None)

                self.get_logger().info(
                    f'除草完了: {weed_name} at ({blade_x:.2f}, {blade_y:.2f})'
                )
                self._publish_removed_event(weed_name, blade_x, blade_y)
                self._publish_updated_registry()
            else:
                self.get_logger().warn(
                    f'削除失敗 {weed_name}: {result.stderr.strip()}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'gz service タイムアウト: {weed_name}')
        except Exception as e:
            self.get_logger().warn(f'削除エラー {weed_name}: {e}')

    # ================================================================
    #  パブリッシュ
    # ================================================================

    def _publish_removed_event(self, name: str,
                                bx: float, by: float) -> None:
        msg = String()
        msg.data = json.dumps({
            'event':  'weed_removed',
            'name':    name,
            'blade_x': round(bx, 3),
            'blade_y': round(by, 3),
            'timestamp': self.get_clock().now().nanoseconds * 1e-9,
        })
        self.removed_pub.publish(msg)

    def _publish_updated_registry(self) -> None:
        with self._lock:
            weeds_list = [
                {'name': n, 'x': round(x, 3), 'y': round(y, 3)}
                for n, (x, y) in self._weeds.items()
            ]
        msg = String()
        msg.data = json.dumps({'weeds': weeds_list})
        self.registry_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WeedRemovalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
