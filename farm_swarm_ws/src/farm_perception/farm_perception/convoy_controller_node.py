#!/usr/bin/env python3
"""
convoy_controller_node.py
Robot②③がRobot①の後方を一定間隔で追従する縦列制御ノード

制御方式:
  Robot② → Robot①の CONVOY_GAP m 後方目標点へ go-to-goal 制御
  Robot③ → Robot②の CONVOY_GAP m 後方目標点へ go-to-goal 制御

  目標点はリーダーの進行方向後方なのでスムーズな曲線追従が可能。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CONVOY_GAP    = 4.0   # [m] 前ロボットとの目標追従距離
MAX_LIN_SPEED = 0.7   # [m/s] 最大前進速度
MAX_ANG_SPEED = 1.2   # [rad/s] 最大角速度
KP_LIN        = 1.0   # 距離誤差P制御ゲイン
KP_ANG        = 2.0   # 方位誤差P制御ゲイン
STOP_DIST     = 0.4   # [m] これより近ければ一時停止


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


class ConvoyController(Node):

    def __init__(self):
        super().__init__('convoy_controller')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._poses: dict[str, tuple[float, float, float]] = {
            'robot1': (-4.0,  5.5, 0.0),
            'robot2': (-8.0,  5.5, 0.0),
            'robot3': (-12.0, 5.5, 0.0),
        }
        self._pubs = {
            'robot2': self.create_publisher(Twist, '/robot2/cmd_vel', 10),
            'robot3': self.create_publisher(Twist, '/robot3/cmd_vel', 10),
        }

        for rid in ('robot1', 'robot2', 'robot3'):
            self.create_subscription(
                Odometry,
                f'/{rid}/odom',
                lambda msg, r=rid: self._odom_cb(msg, r),
                qos,
            )

        self.create_timer(0.05, self._control)  # 20 Hz
        self.get_logger().info(
            f'ConvoyController 起動: gap={CONVOY_GAP}m, max={MAX_LIN_SPEED}m/s'
        )

    def _odom_cb(self, msg: Odometry, robot_id: str) -> None:
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        qz  = msg.pose.pose.orientation.z
        qw  = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self._poses[robot_id] = (x, y, yaw)

    def _control(self) -> None:
        r1 = self._poses['robot1']
        r2 = self._poses['robot2']
        r3 = self._poses['robot3']

        # Robot②はRobot①の後方目標点へ
        twist2 = self._goto(r2, self._target_behind(r1))
        # Robot③はRobot②の後方目標点へ
        twist3 = self._goto(r3, self._target_behind(r2))

        self._pubs['robot2'].publish(twist2)
        self._pubs['robot3'].publish(twist3)

    def _target_behind(self, leader: tuple[float, float, float]) -> tuple[float, float]:
        """リーダーの yaw に沿った後方 CONVOY_GAP m 地点を返す。"""
        lx, ly, lyaw = leader
        tx = lx - CONVOY_GAP * math.cos(lyaw)
        ty = ly - CONVOY_GAP * math.sin(lyaw)
        return tx, ty

    def _goto(self,
              current: tuple[float, float, float],
              target:  tuple[float, float]) -> Twist:
        cx, cy, cyaw = current
        tx, ty       = target

        dx   = tx - cx
        dy   = ty - cy
        dist = math.hypot(dx, dy)

        twist = Twist()
        if dist < STOP_DIST:
            return twist  # 目標に十分近い → 停止

        target_yaw = math.atan2(dy, dx)
        yaw_err    = _normalize_angle(target_yaw - cyaw)

        # 大きく向きがずれている場合は前進を抑制して先に向きを修正
        vx_scale   = max(0.0, math.cos(yaw_err))
        twist.linear.x  = _clamp(KP_LIN * dist * vx_scale, 0.0, MAX_LIN_SPEED)
        twist.angular.z = _clamp(KP_ANG * yaw_err, -MAX_ANG_SPEED, MAX_ANG_SPEED)

        return twist


def main(args=None):
    rclpy.init(args=args)
    node = ConvoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
