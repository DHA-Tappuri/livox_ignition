# livox_ignition

## 概要
LivoxのLiDARをignition gazebo (fortress)でシミュレートするために必要なROS2用のパッケージ。
https://github.com/LCAS/livox_laser_simulation_ros2
をignition gazebo用に移植したもの．
しかしfortressではGPUベースのカスタムレイキャスト機能がないため，
gpu_lidarで高密度のレーザースキャンを作成し，
Livoxのスキャンパターンにマッチしたレイのみを抽出する形で実装している．
（Gazebo Simの資料の少なさ，仕様のおかしさ，名称の紛らわしさには辟易してます．）

そのため、今後GPUベースのカスタムレイキャストが実装された場合は
そちらに合わせて実装方法を変更する必要あり

## 前提条件
- OS     : Ubuntu 22.04
- ROS    : Humble
- gazebo : ignition gazebo fortress

## パッケージ
- livox_ign_msgs        : ignition gazebo内のLivoxセンサ用のトピック定義用のパッケージ
-                         (トピックのみを別パッケージとしたのはGazebo Simのアップデートに影響しないため)
- livox_ign_ros2_bridge : ignition gazebo内のトピックをROS2用のトピックに橋渡しするためのパッケージ
- livox_lidar_plugin    : ignition gazebo用のLivox LiDARシミュレーションプラグイン

## インストール方法
```bash
cd ~/ros2_ws/src
git clone https://github.com/DHA-Tappuri/livox_ignition
colcon build --symlink-install

## テスト方法
```bash
cd ~/ros2_ws/src/livox_ignition/livox_lidar_plugin/sdf
ignition gazebo ./sample.sdf -v 4


