#!/usr/bin/env python3
"""
robot1_navigator_node.py
Robot①が路肩 (Y=5.5) に沿って障害物を避けながら自律走行するノード

アルゴリズム:
  1. LaserScan前方扇形内の障害物を検出
  2. 空いている方向へリアクティブ回避
  3. Y=5.5のレーン位置をP制御で維持
  4. X境界で自動折り返し
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

TARGET_Y        = 5.5    # [m] 路肩中央レーン
CRUISE_SPEED    = 0.6    # [m/s] 巡航速度
MAX_ANG_SPEED   = 1.0    # [rad/s]
OBSTACLE_RANGE  = 1.8    # [m] この距離以内を障害物とみなす
FRONT_HALF_DEG  = 35.0   # 前方扇形 ±35°
LANE_P_GAIN     = 0.8    # 横偏差→角速度のP制御ゲイン
X_TURN_EAST     =  17.0  # [m] 東端折り返し
X_TURN_WEST     = -17.0  # [m] 西端折り返し


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


class Robot1Navigator(Node):

    def __init__(self):
        super().__init__('robot1_navigator')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._scan: LaserScan | None = None
        self._x   = -4.0
        self._y   =  5.5
        self._yaw =  0.0
        self._dir =  1.0   # +1=東進, -1=西進

        self.create_subscription(LaserScan, '/robot1/scan_2d', self._scan_cb, qos)
        self.create_subscription(Odometry,  '/robot1/odom',    self._odom_cb, qos)
        self._pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        self.create_timer(0.05, self._control)   # 20 Hz
        self.get_logger().info(f'Robot1Navigator 起動 (cruise={CRUISE_SPEED} m/s)')

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self._x   = msg.pose.pose.position.x
        self._y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w
        self._yaw = 2.0 * math.atan2(qz, qw)

    def _control(self) -> None:
        # 境界折り返し
        if self._x >= X_TURN_EAST and self._dir > 0:
            self._dir = -1.0
            self.get_logger().info('東端到達 → 西進')
        elif self._x <= X_TURN_WEST and self._dir < 0:
            self._dir =  1.0
            self.get_logger().info('西端到達 → 東進')

        obstacle, turn_sign = self._detect_obstacle()

        twist = Twist()
        if obstacle:
            # 障害物回避: 減速 + 空いている方向へ旋回
            twist.linear.x  = CRUISE_SPEED * 0.25
            twist.angular.z = turn_sign * MAX_ANG_SPEED
        else:
            # 巡航: 目標方位 (進行方向) + レーン維持補正
            target_yaw = 0.0 if self._dir > 0 else math.pi
            yaw_err    = _normalize_angle(target_yaw - self._yaw)
            y_err      = TARGET_Y - self._y
            # 進行方向が反転するときレーン補正の符号も反転
            lane_corr  = LANE_P_GAIN * y_err * self._dir

            twist.linear.x  = CRUISE_SPEED
            twist.angular.z = _clamp(
                1.5 * yaw_err + lane_corr, -MAX_ANG_SPEED, MAX_ANG_SPEED
            )

        self._pub.publish(twist)

    def _detect_obstacle(self) -> tuple[bool, float]:
        """前方扇形内の最小距離を調べ、障害物有無と回避方向を返す。"""
        if self._scan is None:
            return False, 0.0

        scan      = self._scan
        half_rad  = math.radians(FRONT_HALF_DEG)
        left_min  = float('inf')
        right_min = float('inf')

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= scan.range_min:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > half_rad:
                continue
            if angle >= 0:
                left_min  = min(left_min,  r)
            else:
                right_min = min(right_min, r)

        front_min = min(left_min, right_min)
        if front_min > OBSTACLE_RANGE:
            return False, 0.0

        # より空いている方向へ旋回 (右が広ければ右旋回)
        turn_sign = 1.0 if right_min > left_min else -1.0
        return True, turn_sign


def main(args=None):
    rclpy.init(args=args)
    node = Robot1Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
