# Farm Swarm Robots - Phase 1: シミュレーション環境構築

3台の小型自律移動ロボットによる協調除草システムのシミュレーション環境です。

## システム構成

| ロボット | 役割 | 搭載センサ |
|---------|------|-----------|
| Robot ① (Scout) | 先導・環境計測 | 3D LiDAR (VLP-16相当) + RGBD前方カメラ |
| Robot ② (Weeder) | 雑草検出・除草 | RGBD側方カメラ + 回転除草刃 |
| Robot ③ (Collector) | 除草確認・残滓回収 | RGBD前方カメラ + 回収コンベア |

## 動作環境

| 項目 | 要件 |
|------|------|
| OS (ホスト) | Windows 11 (WSL2対応バージョン) |
| OS (WSL) | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| シミュレータ | Gazebo Harmonic |
| Python | 3.12以上 (Ubuntu 24.04に同梱) |
| RAM | 8GB以上推奨 (16GB推奨) |
| GPU | Gazebo描画のためNVIDIA/AMD GPU推奨 (内蔵GPUでも動作可) |

---

## 手順 1: WSL2の確認・セットアップ

### 1-1. Windows 11でWSL2が有効か確認する

**PowerShell** (管理者権限) を開いて以下を実行:

```powershell
wsl --version
```

以下のような出力が出れば正常です:
```
WSL バージョン: 2.x.x.x
カーネル バージョン: 5.x.x
```

WSLが入っていない場合はインストール:
```powershell
wsl --install
```
> インストール後にPCの再起動が必要です

### 1-2. Ubuntu 24.04 が入っているか確認

```powershell
wsl --list --verbose
```

`Ubuntu-24.04` が表示されていればOKです。ない場合はインストール:

```powershell
wsl --install -d Ubuntu-24.04
```

### 1-3. Ubuntu 24.04 を開く

スタートメニューから「Ubuntu 24.04」を検索して起動するか、PowerShellで:

```powershell
wsl -d Ubuntu-24.04
```

**以降の操作はすべてUbuntu 24.04のターミナル内で行います。**

### 1-4. WSLg (GUIサポート) の確認

Windows 11のWSL2はWSLg (GUI自動対応) が標準で有効です。確認方法:

```bash
echo $DISPLAY
# 出力例: :0  または  wayland-0
```

何も表示されない場合は以下を試す:
```bash
export DISPLAY=:0
```

---

## 手順 2: ROS 2 Jazzy のインストール

> 既にROS 2 Jazzyがインストール済みの場合は手順3へ進んでください

### 2-1. システムのアップデート

```bash
sudo apt update && sudo apt upgrade -y
```

### 2-2. ロケール設定

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2-3. ROS 2 リポジトリの追加

```bash
# 必要なツールのインストール
sudo apt install -y software-properties-common curl

# ROS 2 GPGキーの追加
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# リポジトリの追加
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 2-4. ROS 2 Jazzy インストール

```bash
sudo apt install -y ros-jazzy-desktop
```

> インストールに数分かかります (容量: 約2GB)

### 2-5. 自動環境設定 (毎回 source しなくて済むように)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2-6. インストール確認

```bash
ros2 --version
# 出力例: ros2 1.3.x (jazzy)
```

---

## 手順 3: Gazebo Harmonic のインストール

### 3-1. Gazebo リポジトリの追加

```bash
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
```

### 3-2. Gazebo Harmonic インストール

```bash
sudo apt install -y gz-harmonic
```

### 3-3. ROS 2 - Gazebo ブリッジパッケージのインストール

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces
```

### 3-4. 追加の依存パッケージ

```bash
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-ros \
  python3-colcon-common-extensions \
  python3-pip
```

### 3-5. 動作確認

```bash
gz sim --version
# 出力例: Gazebo Sim, version 8.x.x
```

---

## 手順 4: リポジトリの取得とワークスペース構築

### 4-1. リポジトリをクローン

```bash
# ホームディレクトリに移動
cd ~

# クローン
git clone https://github.com/yuya-hosoda/others.git farm_project
cd farm_project
```

### 4-2. ワークスペースに移動

```bash
cd farm_swarm_ws
ls src/
# 確認: farm_description  farm_gazebo  farm_communication
```

### 4-3. 依存関係の自動解決

```bash
# ワークスペースのルートで実行
rosdep init  # 初回のみ (エラーが出た場合は sudo rosdep init)
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4-4. ビルド

```bash
# ROS 2環境が有効になっていることを確認
source /opt/ros/jazzy/setup.bash

# ビルド実行
colcon build --symlink-install

# 正常完了時の出力例:
# Starting >>> farm_description
# Starting >>> farm_communication
# Starting >>> farm_gazebo
# Finished <<< farm_description ...
# Finished <<< farm_communication ...
# Finished <<< farm_gazebo ...
# Summary: 3 packages finished
```

> エラーが出た場合は「トラブルシューティング」セクションを参照

### 4-5. 環境変数の設定

```bash
# ビルド後の環境設定 (毎回必要)
source install/setup.bash

# 毎回自動で読み込まれるようにする場合
echo "source ~/farm_project/farm_swarm_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 手順 5: シミュレーションの起動

### 5-1. Gazebo モデルパスの設定

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/farm_project/farm_swarm_ws/install/farm_gazebo/share/farm_gazebo/models
```

毎回設定が必要なので `.bashrc` に追記推奨:
```bash
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/farm_project/farm_swarm_ws/install/farm_gazebo/share/farm_gazebo/models' >> ~/.bashrc
source ~/.bashrc
```

### 5-2. 全体シミュレーション起動

新しいターミナルを開いて:

```bash
source /opt/ros/jazzy/setup.bash
source ~/farm_project/farm_swarm_ws/install/setup.bash

ros2 launch farm_gazebo farm_sim.launch.py
```

**起動順序 (自動):**
- `0秒`: Gazebo起動・農地ワールドロード
- `3秒`: robot_state_publisher起動 (3台分)
- `5秒`: 3台のロボットがGazebo上にスポーン
- `6秒`: ros_gz_bridgeが起動 (センサデータのROS2ブリッジ)
- `8秒`: 共有マップノード起動
- `10秒`: 雑草スポーン (40本)

> Gazebo GUIウィンドウが開くまで30秒程度かかる場合があります

---

## 手順 6: 動作確認

### 6-1. ROS 2 トピック確認

別ターミナルで:

```bash
source /opt/ros/jazzy/setup.bash
source ~/farm_project/farm_swarm_ws/install/setup.bash

# 全トピック一覧 (robot1/2/3 と swarm が見えればOK)
ros2 topic list | grep -E "(robot[123]|swarm)"
```

期待される出力例:
```
/robot1/cmd_vel
/robot1/odom
/robot1/joint_states
/robot1/lidar/points
/robot1/rgbd/robot1_front/image
/robot1/rgbd/robot1_front/depth_image
/robot2/cmd_vel
/robot2/odom
...
/swarm/shared_map
/swarm/robot_status
```

### 6-2. 各センサのデータ確認

```bash
# オドメトリ (ロボット位置) 確認
ros2 topic echo /robot1/odom --once

# LiDARデータのHz確認 (10Hz前後が正常)
ros2 topic hz /robot1/lidar/points

# カメラ画像の確認
ros2 topic hz /robot1/rgbd/robot1_front/image

# 共有マップの確認
ros2 topic echo /swarm/shared_map --field info

# ロボット状態JSON確認
ros2 topic echo /swarm/robot_status
```

### 6-3. キーボードでロボットを手動操作

新しいターミナルで:

```bash
source /opt/ros/jazzy/setup.bash

# Robot 1 を操作
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/robot1/cmd_vel
```

キー操作:
```
u  i  o    ← 前進 (左斜め / まっすぐ / 右斜め)
j  k  l    ← 左回転 / 停止 / 右回転
m  ,  .    ← 後退

q/z: 速度を上げる/下げる
スペース: 緊急停止
```

### 6-4. TF (座標変換) ツリーの確認

```bash
ros2 run tf2_tools view_frames
# frames.pdf が生成される

# PDFを開く (WSLgならWindowsのPDFビューアが開く)
explorer.exe frames.pdf
```

### 6-5. URDF単体表示 (Gazebo不要)

シミュレーション起動前にURDFが正しいか確認したい場合:

```bash
# Robot 1の表示
ros2 launch farm_description display.launch.py robot:=robot1

# Robot 2の表示
ros2 launch farm_description display.launch.py robot:=robot2
```

RViz2ウィンドウが開き、ロボットの3Dモデルが表示されます。

---

## フォルダ構成

```
farm_swarm_ws/
├── README.md                         ← このファイル
└── src/
    ├── farm_description/             パッケージ①: ロボットURDF
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── urdf/
    │   │   ├── common/
    │   │   │   ├── base.urdf.xacro       # 共通差動駆動台車
    │   │   │   ├── lidar.urdf.xacro      # VLP-16相当LiDARマクロ
    │   │   │   └── rgbd.urdf.xacro       # RealSense D435相当RGBDマクロ
    │   │   ├── robot1_scout.urdf.xacro   # ① 先導ロボット
    │   │   ├── robot2_weeder.urdf.xacro  # ② 除草ロボット
    │   │   └── robot3_collector.urdf.xacro # ③ 回収ロボット
    │   └── launch/
    │       └── display.launch.py         # URViz確認用
    │
    ├── farm_gazebo/                  パッケージ②: Gazebo環境
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── worlds/
    │   │   └── farm_field.sdf            # 農地ワールド (20x20m)
    │   ├── models/
    │   │   ├── weed_small/               # 小型雑草モデル
    │   │   └── weed_large/               # 大型雑草モデル
    │   ├── config/
    │   │   └── ros_gz_bridge.yaml        # ROS2-Gazeboトピックブリッジ設定
    │   ├── scripts/
    │   │   └── spawn_weeds.py            # 雑草スポーンスクリプト
    │   └── launch/
    │       └── farm_sim.launch.py        # 全体起動launchファイル
    │
    └── farm_communication/           パッケージ③: マルチエージェント通信
        ├── package.xml
        ├── setup.py
        ├── farm_communication/
        │   └── shared_map_node.py        # Bayesian統合共有マップノード
        └── launch/
            └── swarm_comm.launch.py      # 通信ノード単体起動
```

---

## トラブルシューティング

### Q1: `colcon build` でエラーが出る

```
Package 'farm_gazebo' not found
```

**対処:** ROS 2環境の読み込みを確認
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

### Q2: Gazebo GUIが開かない

WSL2でGUIが表示されない場合:

```bash
# WSLgサービスの確認
ls /tmp/.X11-unix/

# 表示変数の確認
echo $DISPLAY
echo $WAYLAND_DISPLAY
```

`WAYLAND_DISPLAY` が空の場合:
```bash
export DISPLAY=:0
```

また、Windows側でWSLgが有効か確認:
```powershell
# PowerShellで実行
wsl --update
```

---

### Q3: `gz sim` コマンドが見つからない

```bash
# Gazeboのインストール確認
which gz
# → 見つからない場合は手順3を再実行

# PATHの確認
echo $PATH | grep -o '[^:]*gz[^:]*'

# 環境変数の明示的な設定
source /opt/ros/jazzy/setup.bash
```

---

### Q4: ロボットがGazebo上に表示されない

スポーンには**GazeboとROSが両方起動している**必要があります。ログを確認:

```bash
# farm_sim.launch.py のログでエラーを探す
# "waiting for service" が続く場合はGazebo起動待ち (最大30秒)

# 手動スポーンで確認
ros2 run ros_gz_sim create -name robot1 \
  -topic /robot1/robot_description -x -3 -y 0 -z 0.1
```

---

### Q5: センサデータ (/robot1/lidar/points など) が来ない

```bash
# Gazebo側のトピックを確認
gz topic -l | grep robot1

# ブリッジの状態確認
ros2 node info /ros_gz_bridge

# ブリッジを手動起動してデバッグ
ros2 run ros_gz_bridge parameter_bridge \
  /robot1/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```

---

### Q6: `rosdep install` でエラーが出る

```bash
# rosdep初期化が必要な場合
sudo rosdep init
rosdep update

# Pythonパッケージの手動インストール
pip install numpy
```

---

### Q7: Gazebo描画が遅い・落ちる

WSL2でのGPU使用を確認:
```bash
# NVIDIA GPUの場合
nvidia-smi

# ソフトウェアレンダリングにフォールバック
export LIBGL_ALWAYS_SOFTWARE=1
gz sim farm_field.sdf
```

---

## 動作確認チェックリスト

Phase 1の完了基準:

- [ ] `colcon build` が3パッケージすべて成功する
- [ ] `ros2 launch farm_gazebo farm_sim.launch.py` でGazeboが起動する
- [ ] 3台のロボットがGazebo上に表示される
- [ ] `ros2 topic list` に `/robot1/odom`, `/robot2/odom`, `/robot3/odom` が現れる
- [ ] `ros2 topic hz /robot1/lidar/points` で約10Hzのデータが来る
- [ ] `teleop_twist_keyboard` で各ロボットを独立して操作できる
- [ ] `/swarm/shared_map` がパブリッシュされている
- [ ] `/swarm/robot_status` に全ロボットの座標が含まれている

---

## 次のステップ (Phase 2以降)

- **Phase 2**: 知覚モジュール実装 (SLAM・YOLOv8による雑草検出)
- **Phase 3**: マルチエージェント強化学習環境構築 (RLlib + MAPPO)
- **Phase 4**: 3台協調学習・情報共有プロトコル統合

---

## ライセンス

Apache License 2.0
