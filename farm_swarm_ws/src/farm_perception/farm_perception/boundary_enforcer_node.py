#!/usr/bin/env python3
"""
boundary_enforcer_node.py
各ロボットが路肩作業エリア外に出ないよう速度補正を行うノード

作業エリア境界:
  X: -18.5 〜 +18.5m  (道路方向)
  Y:  +4.3 〜  +7.3m  (路肩幅方向)

補正ロジック:
  - 警戒ゾーン (境界から1m以内) に入ったら緩やかに補正速度を重畳
  - 危険ゾーン (境界から0.3m以内) に入ったら強制停止・反転
  - ロボットが正常範囲内にいる場合は補正しない (teleop/自律走行を妨げない)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ---- 作業エリア境界 ----
X_MIN, X_MAX = -18.5,  18.5
Y_MIN, Y_MAX =   4.3,   7.3

# 警戒ゾーン幅 (ここから補正開始)
WARN_DIST = 1.0
# 危険ゾーン幅 (ここから強制停止)
DANGER_DIST = 0.3

# 補正速度パラメータ
MAX_CORRECTION_LINEAR  = 0.30   # [m/s]
MAX_CORRECTION_ANGULAR = 0.60   # [rad/s]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class BoundaryEnforcerNode(Node):

    ROBOT_IDS = ['robot1', 'robot2', 'robot3']

    def __init__(self):
        super().__init__('boundary_enforcer')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._poses: dict[str, tuple[float, float, float]] = {
            rid: (0.0, 0.0, 0.0) for rid in self.ROBOT_IDS
        }
        self._cmd_pubs: dict[str, object] = {}

        for rid in self.ROBOT_IDS:
            self.create_subscription(
                Odometry,
                f'/{rid}/odom',
                lambda msg, r=rid: self._odom_callback(msg, r),
                qos,
            )
            self._cmd_pubs[rid] = self.create_publisher(
                Twist, f'/{rid}/cmd_vel', 10
            )

        # 20Hzで境界チェック
        self.create_timer(0.05, self._enforce_all)

        self.get_logger().info(
            f'BoundaryEnforcer起動: '
            f'X=[{X_MIN},{X_MAX}] Y=[{Y_MIN},{Y_MAX}]'
        )

    def _odom_callback(self, msg: Odometry, robot_id: str) -> None:
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self._poses[robot_id] = (x, y, yaw)

    def _enforce_all(self) -> None:
        for rid in self.ROBOT_IDS:
            x, y, yaw = self._poses[rid]
            correction = self._compute_correction(x, y, yaw)
            if correction is not None:
                self._cmd_pubs[rid].publish(correction)

    def _compute_correction(self, x: float, y: float,
                             yaw: float) -> Twist | None:
        """
        危険ゾーン (境界から 0.3m 以内) に入った場合のみ強制停止・反転。
        警戒ゾーンでの補正は行わない (ナビゲータ/縦列制御と競合しないため)。
        """
        dist_x_min = x - X_MIN
        dist_x_max = X_MAX - x
        dist_y_min = y - Y_MIN
        dist_y_max = Y_MAX - y

        in_danger = (dist_x_min < DANGER_DIST or dist_x_max < DANGER_DIST or
                     dist_y_min < DANGER_DIST or dist_y_max < DANGER_DIST)

        if not in_danger:
            return None

        # 世界座標系での反発ベクトルを計算
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        fx, fy = 0.0, 0.0

        if dist_x_min < DANGER_DIST:
            fx += MAX_CORRECTION_LINEAR
        if dist_x_max < DANGER_DIST:
            fx -= MAX_CORRECTION_LINEAR
        if dist_y_min < DANGER_DIST:
            fy += MAX_CORRECTION_LINEAR
        if dist_y_max < DANGER_DIST:
            fy -= MAX_CORRECTION_LINEAR

        vx_robot =  cos_y * fx + sin_y * fy
        vy_robot = -sin_y * fx + cos_y * fy

        twist = Twist()
        twist.linear.x  = _clamp(vx_robot * 2.0,
                                  -MAX_CORRECTION_LINEAR, MAX_CORRECTION_LINEAR)
        twist.angular.z = _clamp(vy_robot * 3.0,
                                  -MAX_CORRECTION_ANGULAR, MAX_CORRECTION_ANGULAR)

        self.get_logger().warn(
            f'境界危険ゾーン: ({x:.1f},{y:.1f}) '
            f'補正: vx={twist.linear.x:.2f} wz={twist.angular.z:.2f}',
            throttle_duration_sec=1.0,
        )
        return twist


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryEnforcerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
