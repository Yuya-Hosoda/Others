#!/usr/bin/env python3
"""
spawn_weeds.py
路肩エリア (Y=4.5〜7.0) に雑草をランダム配置するスクリプト

座標系:
  X軸: 道路進行方向 (-18〜+18m)
  Y軸: 路肩方向 (4.5〜7.0m が路肩雑草帯)

スポーン完了後、配置した雑草の位置情報を
/swarm/weed_registry (std_msgs/String JSON) へパブリッシュする。
"""
import json
import math
import os
import random
import subprocess
import sys
import time

try:
    from ament_index_python.packages import get_package_share_directory
    _PKG_MODELS = os.path.join(
        get_package_share_directory('farm_gazebo'), 'models'
    )
except Exception:
    _PKG_MODELS = ''

# ---- 路肩エリア設定 ----
NUM_WEEDS     = int(os.getenv('WEED_COUNT',     '50'))
X_MIN         = float(os.getenv('X_MIN',       '-18.0'))
X_MAX         = float(os.getenv('X_MAX',        '18.0'))
Y_MIN         = float(os.getenv('Y_MIN',         '4.5'))   # 縁石より内側
Y_MAX         = float(os.getenv('Y_MAX',         '7.0'))   # ガードレール手前
SMALL_RATIO   = float(os.getenv('SMALL_RATIO',   '0.6'))
SPAWN_TIMEOUT = float(os.getenv('SPAWN_TIMEOUT', '8.0'))

# ロボット初期位置 (スポーン除外円の中心)
ROBOT_INIT_POSITIONS = [
    (-4.0,  5.5),
    (-8.0,  5.5),
    (-12.0, 5.5),
]
EXCL_RADIUS = 2.5


def find_model_sdf(model_type: str) -> str | None:
    candidates = [_PKG_MODELS, os.path.expanduser('~/.gz/models')]
    for prefix in os.getenv('AMENT_PREFIX_PATH', '').split(':'):
        candidates.append(
            os.path.join(prefix, 'share', 'farm_gazebo', 'models')
        )
    for base in candidates:
        if not base:
            continue
        path = os.path.join(base, model_type, 'model.sdf')
        if os.path.isfile(path):
            return path
    return None


def generate_inline_sdf(name: str, model_type: str) -> str:
    radius, height = (0.06, 0.12) if model_type == 'weed_small' else (0.12, 0.25)
    return (
        f'<?xml version="1.0"?><sdf version="1.9">'
        f'<model name="{name}"><static>true</static>'
        f'<link name="weed_link">'
        f'<visual name="v"><pose>0 0 {height/2:.3f} 0 0 0</pose>'
        f'<geometry><cylinder><radius>{radius}</radius>'
        f'<length>{height}</length></cylinder></geometry>'
        f'<material><ambient>0.1 0.6 0.1 1</ambient></material></visual>'
        f'<collision name="c"><pose>0 0 {height/2:.3f} 0 0 0</pose>'
        f'<geometry><cylinder><radius>{radius}</radius>'
        f'<length>{height}</length></cylinder></geometry></collision>'
        f'</link></model></sdf>'
    )


def is_near_robot(x: float, y: float) -> bool:
    return any(
        math.sqrt((x - rx)**2 + (y - ry)**2) < EXCL_RADIUS
        for rx, ry in ROBOT_INIT_POSITIONS
    )


def spawn_entity(name: str, sdf_path: str | None, sdf_str: str | None,
                 x: float, y: float, yaw: float) -> bool:
    import tempfile
    use_tmp = sdf_path is None
    if use_tmp:
        with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as tmp:
            tmp.write(sdf_str)
            tmp_path = tmp.name
    else:
        tmp_path = sdf_path

    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', name, '-file', tmp_path,
        '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', '0.005',
        '-Y', f'{yaw:.4f}',
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=SPAWN_TIMEOUT
        )
        return result.returncode == 0
    except Exception as e:
        print(f'[spawn] エラー {name}: {e}', file=sys.stderr)
        return False
    finally:
        if use_tmp and os.path.exists(tmp_path):
            os.unlink(tmp_path)


def publish_weed_registry(weed_records: list[dict]) -> None:
    """スポーン完了後、雑草レジストリを /swarm/weed_registry へパブリッシュ。"""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        rclpy.init()
        node = Node('weed_registry_publisher')
        pub = node.create_publisher(String, '/swarm/weed_registry', 10)

        payload = json.dumps({'weeds': weed_records})
        msg = String()
        msg.data = payload

        # QoSが安定するまで少し待ち、複数回パブリッシュして確実に届ける
        time.sleep(1.0)
        for _ in range(5):
            pub.publish(msg)
            time.sleep(0.1)

        node.get_logger().info(
            f'[weed_registry] {len(weed_records)}本の雑草位置を /swarm/weed_registry へ送信'
        )
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f'[weed_spawner] registry publish失敗 (非致命的): {e}', file=sys.stderr)


def main() -> None:
    print(
        f'[weed_spawner] 路肩雑草スポーン開始\n'
        f'  対象エリア: X=[{X_MIN}, {X_MAX}]m  Y=[{Y_MIN}, {Y_MAX}]m\n'
        f'  目標本数: {NUM_WEEDS}本'
    )

    sdf_paths = {
        'weed_small': find_model_sdf('weed_small'),
        'weed_large': find_model_sdf('weed_large'),
    }
    for mt, p in sdf_paths.items():
        print(f'  {mt}: {p if p else "インラインSDF使用"}')

    spawned      = 0
    attempts     = 0
    max_att      = NUM_WEEDS * 6
    weed_records: list[dict] = []

    while spawned < NUM_WEEDS and attempts < max_att:
        attempts += 1
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)

        if is_near_robot(x, y):
            continue

        model_type = 'weed_small' if random.random() < SMALL_RATIO else 'weed_large'
        yaw  = random.uniform(0.0, math.pi * 2)
        name = f'weed_{spawned:03d}'

        sdf_path = sdf_paths.get(model_type)
        sdf_str  = None if sdf_path else generate_inline_sdf(name, model_type)

        if spawn_entity(name, sdf_path, sdf_str, x, y, yaw):
            weed_records.append({'name': name, 'x': x, 'y': y, 'removed': False})
            spawned += 1
            if spawned % 10 == 0:
                print(f'[weed_spawner] {spawned}/{NUM_WEEDS} 本配置済み')
            time.sleep(0.05)

    print(f'[weed_spawner] 完了: {spawned}/{NUM_WEEDS}本 (試行{attempts}回)')

    # スポーン完了後にレジストリをパブリッシュ
    publish_weed_registry(weed_records)


if __name__ == '__main__':
    main()
