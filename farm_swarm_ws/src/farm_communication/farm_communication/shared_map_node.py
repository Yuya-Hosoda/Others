#!/usr/bin/env python3
"""
shared_map_node.py
3台ロボットの観測を統合するBayesian共有セマンティックマップノード

購読トピック:
  /robot1/odom, /robot2/odom, /robot3/odom  (nav_msgs/Odometry)

パブリッシュトピック:
  /swarm/shared_map      (nav_msgs/OccupancyGrid)  - 統合占有グリッドマップ
  /swarm/robot_status    (std_msgs/String)          - 全ロボット状態JSON
"""
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class SharedMapNode(Node):

    ROBOT_IDS = ['robot1', 'robot2', 'robot3']

    def __init__(self):
        super().__init__('shared_map_node')

        # ---- マップパラメータ ----
        self.declare_parameter('map_size',   200)    # グリッド数 (N×N)
        self.declare_parameter('map_res',    0.10)   # 解像度 [m/cell]
        self.declare_parameter('origin_x', -10.0)   # マップ原点X [m]
        self.declare_parameter('origin_y', -10.0)   # マップ原点Y [m]
        self.declare_parameter('publish_hz',   1.0)  # パブリッシュ周波数 [Hz]

        self.map_size  = self.get_parameter('map_size').value
        self.map_res   = self.get_parameter('map_res').value
        self.origin_x  = self.get_parameter('origin_x').value
        self.origin_y  = self.get_parameter('origin_y').value
        publish_hz     = self.get_parameter('publish_hz').value

        # ---- 確率マップ (log-odds表現) ----
        # チャンネル: 0=free, 1=occupied/weed, 2=unknown
        # 値: log-odds (0.0 = 50/50, 正=occupied, 負=free)
        self.log_odds_map = np.zeros(
            (self.map_size, self.map_size), dtype=np.float32
        )
        # ロボット位置追跡
        self.robot_poses: dict[str, tuple[float, float, float]] = {
            rid: (0.0, 0.0, 0.0) for rid in self.ROBOT_IDS
        }
        self.robot_states: dict[str, str] = {
            rid: 'initializing' for rid in self.ROBOT_IDS
        }
        self.explored_cells = 0

        # ---- QoS設定 ----
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---- 購読: 各ロボットのオドメトリ ----
        for robot_id in self.ROBOT_IDS:
            self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self._odom_callback(msg, rid),
                sensor_qos,
            )

        # ---- パブリッシュ ----
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/swarm/shared_map', map_qos
        )
        self.status_pub = self.create_publisher(
            String, '/swarm/robot_status', 10
        )

        # ---- 定期パブリッシュタイマー ----
        self.create_timer(1.0 / publish_hz, self._publish_all)

        self.get_logger().info(
            f'SharedMapNode 起動完了: マップサイズ={self.map_size}x{self.map_size}, '
            f'解像度={self.map_res}m/cell'
        )

    # ================================================================
    #  コールバック
    # ================================================================

    def _odom_callback(self, msg: Odometry, robot_id: str) -> None:
        """オドメトリ受信時: ロボット位置を更新し、周辺セルを探索済みにマーク"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # クォータニオン → yaw 変換
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)

        self.robot_poses[robot_id] = (x, y, yaw)

        # ロボット周辺2mを探索済みにマーク (log-odds を下げる)
        self._mark_explored(x, y, radius_m=2.0, log_odds_delta=-0.5)

        # 状態更新
        if self.robot_states[robot_id] == 'initializing':
            self.robot_states[robot_id] = 'active'

    # ================================================================
    #  マップ更新
    # ================================================================

    def _mark_explored(self, wx: float, wy: float,
                       radius_m: float, log_odds_delta: float) -> None:
        """指定ワールド座標の周辺セルをlog-odds更新"""
        cx, cy = self._world_to_grid(wx, wy)
        r_cells = int(radius_m / self.map_res)

        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy > r_cells * r_cells:
                    continue
                gx, gy = cx + dx, cy + dy
                if self._in_bounds(gx, gy):
                    prev = self.log_odds_map[gy, gx]
                    self.log_odds_map[gy, gx] = float(
                        np.clip(prev + log_odds_delta, -5.0, 5.0)
                    )

    # ================================================================
    #  パブリッシュ
    # ================================================================

    def _publish_all(self) -> None:
        now = self.get_clock().now().to_msg()
        self._publish_shared_map(now)
        self._publish_robot_status(now)

    def _publish_shared_map(self, stamp) -> None:
        """log-oddsマップをOccupancyGridに変換してパブリッシュ"""
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'

        msg.info.resolution = self.map_res
        msg.info.width  = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        # log-odds → 確率 → [0,100] の整数値
        # -1: unknown, 0: free, 100: occupied
        prob = 1.0 / (1.0 + np.exp(-self.log_odds_map))
        grid_data = (prob * 100).astype(np.int8).flatten().tolist()
        msg.data = grid_data

        self.map_pub.publish(msg)

        # 探索済みセル数を定期ログ
        explored = int(np.sum(self.log_odds_map < -0.1))
        total    = self.map_size * self.map_size
        if explored != self.explored_cells:
            self.explored_cells = explored
            coverage = explored / total * 100
            self.get_logger().info(
                f'探索カバレッジ: {coverage:.1f}% ({explored}/{total} cells)'
            )

    def _publish_robot_status(self, stamp) -> None:
        """全ロボットの状態をJSON文字列でパブリッシュ"""
        status = {
            'timestamp': stamp.sec + stamp.nanosec * 1e-9,
            'robots': {
                rid: {
                    'x':    round(pose[0], 3),
                    'y':    round(pose[1], 3),
                    'yaw':  round(pose[2], 3),
                    'state': self.robot_states[rid],
                }
                for rid, pose in self.robot_poses.items()
            },
            'coverage_pct': round(
                np.sum(self.log_odds_map < -0.1) / (self.map_size ** 2) * 100, 2
            ),
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    # ================================================================
    #  ユーティリティ
    # ================================================================

    def _world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        gx = int((wx - self.origin_x) / self.map_res)
        gy = int((wy - self.origin_y) / self.map_res)
        return gx, gy

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.map_size and 0 <= gy < self.map_size


def main(args=None):
    rclpy.init(args=args)
    node = SharedMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
