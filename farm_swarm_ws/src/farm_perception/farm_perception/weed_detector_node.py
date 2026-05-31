#!/usr/bin/env python3
"""
weed_detector_node.py
Robot ②の側方RGBDカメラでリアルタイム雑草検出を行うノード

検出方式:
  - シミュレーション: HSV緑色マスキング (use_sim_detection=True)
  - 実機: YOLOv8ファインチューニングモデル (use_sim_detection=False)

パブリッシュ:
  /{robot_id}/weed_detections    (visualization_msgs/MarkerArray) - 3D検出位置
  /{robot_id}/weed_detect_image  (sensor_msgs/Image) - デバッグ画像
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

try:
    from cv_bridge import CvBridge
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class WeedDetectorNode(Node):

    def __init__(self):
        super().__init__('weed_detector')

        self.declare_parameter('robot_id',           'robot2')
        self.declare_parameter('use_sim_detection',  True)
        self.declare_parameter('model_path',         'yolov8n.pt')
        self.declare_parameter('conf_threshold',     0.45)
        self.declare_parameter('min_weed_area_px',   500)    # 最小検出面積 [px²]
        self.declare_parameter('camera_fx',          554.26) # 焦点距離 x [px]
        self.declare_parameter('camera_fy',          554.26) # 焦点距離 y [px]
        self.declare_parameter('camera_cx',          320.0)  # 主点 x
        self.declare_parameter('camera_cy',          240.0)  # 主点 y

        robot_id    = self.get_parameter('robot_id').value
        use_sim     = self.get_parameter('use_sim_detection').value
        model_path  = self.get_parameter('model_path').value

        if not CV2_AVAILABLE:
            self.get_logger().error('cv2/cv_bridge が見つかりません。pip install opencv-python を実行してください')
            return

        # YOLOv8 モデルロード (実機用)
        self.model = None
        self.use_sim = use_sim
        if not use_sim:
            try:
                from ultralytics import YOLO
                self.model = YOLO(model_path)
                self.get_logger().info(f'YOLOv8モデル読み込み完了: {model_path}')
            except Exception as e:
                self.get_logger().warn(
                    f'YOLOv8読み込み失敗 ({e}) → HSV検出モードに切替'
                )
                self.use_sim = True

        self.bridge       = CvBridge()
        self.latest_depth = None
        self.robot_id     = robot_id

        # カメラ内部パラメータ
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Robot②: 側方カメラ / Robot①③: 前方カメラ
        if robot_id == 'robot2':
            img_topic   = f'/{robot_id}/rgbd/robot2_side/image'
            depth_topic = f'/{robot_id}/rgbd/robot2_side/depth_image'
        else:
            img_topic   = f'/{robot_id}/rgbd/{robot_id}_front/image'
            depth_topic = f'/{robot_id}/rgbd/{robot_id}_front/depth_image'

        self.create_subscription(Image, img_topic,   self._image_callback, qos)
        self.create_subscription(Image, depth_topic, self._depth_callback, qos)

        self.detect_pub   = self.create_publisher(
            MarkerArray, f'/{robot_id}/weed_detections', 10
        )
        self.debug_pub    = self.create_publisher(
            Image, f'/{robot_id}/weed_detect_image', qos
        )

        self.get_logger().info(
            f'WeedDetector起動: robot={robot_id}, '
            f'mode={"シミュレーション(HSV)" if self.use_sim else "YOLOv8"}'
        )

    # ================================================================
    #  コールバック
    # ================================================================

    def _depth_callback(self, msg: Image) -> None:
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception:
            pass

    def _image_callback(self, msg: Image) -> None:
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        if self.use_sim:
            detections = self._detect_hsv(cv_img)
        else:
            detections = self._detect_yolo(cv_img)

        self._publish_markers(detections, msg.header)
        self._publish_debug(cv_img, detections, msg.header)

    # ================================================================
    #  検出ロジック
    # ================================================================

    def _detect_hsv(self, img: np.ndarray) -> list[dict]:
        """HSV緑色マスキングによるシミュレーション用雑草検出"""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # 緑色 (H: 35-85, 鮮やか, 明るめ)
        mask = cv2.inRange(hsv,
                           np.array([35, 60, 60]),
                           np.array([85, 255, 255]))
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        min_area = self.get_parameter('min_weed_area_px').value
        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            depth_m = self._get_depth_at(x + w // 2, y + h // 2)
            detections.append({
                'bbox':       (x, y, w, h),
                'confidence': min(area / 8000.0, 1.0),
                'class':      'weed',
                'depth':      depth_m,
            })
        return detections

    def _detect_yolo(self, img: np.ndarray) -> list[dict]:
        """YOLOv8推論 (ファインチューニング済みモデル使用)"""
        if self.model is None:
            return []
        threshold = self.get_parameter('conf_threshold').value
        results = self.model(img, conf=threshold, verbose=False)
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                cx_px = (x1 + x2) // 2
                cy_px = (y1 + y2) // 2
                detections.append({
                    'bbox':       (x1, y1, x2 - x1, y2 - y1),
                    'confidence': float(box.conf[0]),
                    'class':      self.model.names[int(box.cls[0])],
                    'depth':      self._get_depth_at(cx_px, cy_px),
                })
        return detections

    def _get_depth_at(self, px: int, py: int, patch: int = 5) -> float:
        """深度画像から指定ピクセル付近の有効深度を返す [m]"""
        if self.latest_depth is None:
            return 1.5
        h, w = self.latest_depth.shape
        x0 = max(0, px - patch)
        x1 = min(w, px + patch)
        y0 = max(0, py - patch)
        y1 = min(h, py + patch)
        patch_data = self.latest_depth[y0:y1, x0:x1]
        valid = patch_data[(patch_data > 0.05) & (patch_data < 10.0)]
        return float(np.median(valid)) if len(valid) > 0 else 1.5

    # ================================================================
    #  パブリッシュ
    # ================================================================

    def _publish_markers(self, detections: list[dict], header) -> None:
        """検出結果を3D MarkerArrayとしてパブリッシュ"""
        array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 前フレームの検出をクリア
        clear = Marker()
        clear.header.stamp    = now
        clear.header.frame_id = header.frame_id
        clear.action          = Marker.DELETEALL
        array.markers.append(clear)

        for i, det in enumerate(detections):
            x_bbox, y_bbox, w, h = det['bbox']
            cx_px = x_bbox + w / 2
            cy_px = y_bbox + h / 2
            depth = det['depth']

            # ピクセル座標 → カメラ座標 (3D)
            x_cam = (cx_px - self.cx) * depth / self.fx
            y_cam = (cy_px - self.cy) * depth / self.fy
            z_cam = depth

            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = header.frame_id
            m.ns              = 'weed_detections'
            m.id              = i + 1
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD

            m.pose.position    = Point(x=z_cam, y=-x_cam, z=-y_cam)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.15
            m.scale.z = 0.25
            m.color   = ColorRGBA(r=1.0, g=0.4, b=0.0,
                                  a=0.8 * det['confidence'])
            m.lifetime.sec = 1

            array.markers.append(m)

        self.detect_pub.publish(array)

    def _publish_debug(self, img: np.ndarray,
                       detections: list[dict], header) -> None:
        """検出枠を描画したデバッグ画像をパブリッシュ"""
        debug = img.copy()
        for det in detections:
            x, y, w, h = det['bbox']
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f'{det["class"]} {det["confidence"]:.2f} d={det["depth"]:.1f}m'
            cv2.putText(debug, label, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        try:
            msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            msg.header = header
            self.debug_pub.publish(msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WeedDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
