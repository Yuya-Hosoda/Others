#!/usr/bin/env python3
"""
spawn_weeds.py
Gazebo Harmonic上に雑草モデルをランダムに配置するスクリプト

ros_gz_sim create コマンドをサブプロセスで呼び出すことで
SpawnEntityサービスへの依存を排除し、確実にスポーンを行う。
"""
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

# ---- 設定パラメータ (環境変数で上書き可能) ----
NUM_WEEDS        = int(os.getenv('WEED_COUNT',       '40'))
FIELD_SIZE       = float(os.getenv('FIELD_SIZE',     '18.0'))
SMALL_RATIO      = float(os.getenv('SMALL_RATIO',    '0.65'))
EXCL_RADIUS      = float(os.getenv('EXCL_RADIUS',    '2.5'))
SPAWN_TIMEOUT    = float(os.getenv('SPAWN_TIMEOUT',  '8.0'))


def find_model_sdf(model_type: str) -> str | None:
    """モデルSDFファイルを探索してパスを返す。見つからなければNone。"""
    candidates = [
        _PKG_MODELS,
        os.path.expanduser('~/.gz/models'),
        os.path.expanduser('~/.gazebo/models'),
    ]
    # AMENT_PREFIX_PATH 内のすべてのshareディレクトリを検索
    for prefix in os.getenv('AMENT_PREFIX_PATH', '').split(':'):
        candidates.append(os.path.join(prefix, 'share', 'farm_gazebo', 'models'))

    for base in candidates:
        if not base:
            continue
        path = os.path.join(base, model_type, 'model.sdf')
        if os.path.isfile(path):
            return path
    return None


def generate_inline_sdf(name: str, model_type: str) -> str:
    """モデルファイルが見つからない場合のインラインSDF"""
    if model_type == 'weed_small':
        radius, height = 0.06, 0.12
    else:
        radius, height = 0.12, 0.25
    return (
        f'<?xml version="1.0"?>'
        f'<sdf version="1.9">'
        f'<model name="{name}">'
        f'<static>true</static>'
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


def spawn_entity(name: str, sdf_path: str | None, sdf_str: str | None,
                 x: float, y: float, yaw: float) -> bool:
    """ros2 run ros_gz_sim create でエンティティをスポーンする"""
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', name,
        '-x', f'{x:.4f}',
        '-y', f'{y:.4f}',
        '-z', '0.005',
        '-Y', f'{yaw:.4f}',
    ]

    if sdf_path:
        cmd += ['-file', sdf_path]
    else:
        # インラインSDFを一時ファイルに書き出す
        import tempfile
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.sdf', delete=False
        ) as tmp:
            tmp.write(sdf_str)
            tmp_path = tmp.name
        cmd += ['-file', tmp_path]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=SPAWN_TIMEOUT,
        )
        if sdf_path is None and 'tmp_path' in locals():
            os.unlink(tmp_path)

        if result.returncode == 0:
            return True
        print(f'[spawn] 失敗 {name}: {result.stderr.strip()}', file=sys.stderr)
        return False
    except subprocess.TimeoutExpired:
        print(f'[spawn] タイムアウト {name}', file=sys.stderr)
        return False
    except Exception as e:
        print(f'[spawn] エラー {name}: {e}', file=sys.stderr)
        return False


def main() -> None:
    print(f'[weed_spawner] 雑草スポーン開始: {NUM_WEEDS}本, '
          f'フィールド{FIELD_SIZE}m x {FIELD_SIZE}m')

    sdf_paths = {
        'weed_small': find_model_sdf('weed_small'),
        'weed_large': find_model_sdf('weed_large'),
    }
    for model_type, path in sdf_paths.items():
        if path:
            print(f'[weed_spawner] モデル発見: {model_type} → {path}')
        else:
            print(f'[weed_spawner] モデルファイル未発見 ({model_type}): インラインSDFを使用')

    half      = FIELD_SIZE / 2.0
    spawned   = 0
    attempts  = 0
    max_att   = NUM_WEEDS * 5

    while spawned < NUM_WEEDS and attempts < max_att:
        attempts += 1
        x = random.uniform(-half, half)
        y = random.uniform(-half, half)

        if math.sqrt(x * x + y * y) < EXCL_RADIUS:
            continue

        model_type = 'weed_small' if random.random() < SMALL_RATIO else 'weed_large'
        yaw  = random.uniform(0, math.pi * 2)
        name = f'weed_{spawned:03d}'

        sdf_path = sdf_paths.get(model_type)
        sdf_str  = None if sdf_path else generate_inline_sdf(name, model_type)

        if spawn_entity(name, sdf_path, sdf_str, x, y, yaw):
            spawned += 1
            if spawned % 10 == 0:
                print(f'[weed_spawner] {spawned}/{NUM_WEEDS} 本配置完了')
            time.sleep(0.05)   # Gazeboへの連続リクエストを少し間引く

    print(f'[weed_spawner] 完了: {spawned}/{NUM_WEEDS}本 '
          f'(試行{attempts}回)')


if __name__ == '__main__':
    main()
