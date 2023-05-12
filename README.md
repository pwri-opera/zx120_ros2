# zx120_ros2
OPERA対応油圧ショベルzx120の土木研究所公開ROS2パッケージ群

## 概説
- 国立研究開発法人土木研究所が公開するOPERA（Open Platform for Eathwork with Robotics Autonomy）対応の油圧ショベルであるZX120用のROS2パッケージ群である
- 本パッケージに含まれる各launchファイルを起動することで、実機やシミュレータを動作させるのに必要なROSノード群が立ち上がる
- 動作確認済のROS Version : ROS Humble Hawksbil + Ubuntu 22.04 LTS

## ビルド方法
- ワークスペースの作成（既にwsを作成済の場合は不要．以下、新規作成するワークスペースの名称を"ros2_ws"と仮定して表記）
  ```bash
  $ cd ~/
  $ mkdir --parents ros2_ws/src
  $ cd ros2_ws
  $ colcon build 
  ```
- ~/ros2_ws/以下にbuild, install, log, srcディレクトリが作成される

- 依存パッケージ群をインストールした上でパッケージのビルドと自分のワークスペースをインストール環境上にOverlayする  
  [vcstoolに関する参考サイト](https://qiita.com/strv/items/dbde72e20a8efe62ef95)
  ```bash
  $ cd ~/ros2_ws/src
  $ git clone https://github.com/pwri-opera/zx120_ros2.git
  $ sudo apt update
  $ sudo apt install python3-rosdep2 
  $ rosdep update
  $ rosdep install -i --from-path src --rosdistro humble -y 
  <!--
  $ git clone https://github.com/strv/vcstool-utils.git
  $ ./vcstool-utils/import_all.sh -s .rosinstall ~/catkin_ws/src
  -->
  $ colcon build --symlink-install 
  $ . install/setup.bash
  ```

## 含有するサブパッケージ
### zx120_bringup:
- zx120の実機を動作させる際に必要なノード群を一括起動するためのlaunch用のサブパッケージ

### zx120_control_hardware:
- [ros_control](http://wiki.ros.org/ros_control)の枠組みに倣い、作業機（=swing_joint, boom_joint, arm_joint, bucket_joint, bucket_end_joint）の部分をjoint_state_controller(type: joint_state_controller/JointStateController), upper_arm_contoller(position_controllers/JointTrajectoryController)という名称で実装したサブパッケージ

### zx120_description:
- zx120用のロボットモデルファイル(dae, xacro含む)群

### zx120_unity:
- zx120をunityシミュレータ(OperaSim-AGX, OperaSim-PhysX)上で動作させるのに必要なノード群を一括起動するためのlaunch用のサブパッケージ

### zx120_moveit_config:
- zx120の作業機（=swing, boom, arm, bucketの4軸）のモーション制御のためのライブラリ
- [MoveIt2](https://moveit.ros.org/)に準拠しMoveIt Setup Assistantを用いて作成

### zx120_unity:
- OPERAのUnityシミュレータ(OperaSim-AGX, OperaSim-PhysX)と連携するために必要なノード群を一括起動するためのlaunch用のサブパッケージ

## 各ROSノード群の起動方法
- 実機動作に必要なROS2ノード群の起動方法  
注）実機特有の非公開パッケージが含まれるため、実機以外の環境ではlaunchに失敗します
  ```bash
  $ ros2 launch zx120_bringup zx120_vehicle.launch.py
  ```
- Unityシミュレータとの連携に必要なROS2ノード群の起動方法
  ```bash
  $ ros2 launch zx120_unity zx120_standby.launch.py
  ```

## ハードウェアシステム
zx120のハードウェアのシステム構成を以下のブロック図へ示します
![MicrosoftTeams-image (1)](https://github.com/pwri-opera/zx120_ros2/assets/24404939/a49534cc-13b1-461f-9368-152daabae51e)

## ソフトウェアシステム
### roslaunch zx120_unity zx120_standy.launch.py実行時のノード/トピックパイプライン（rqt_graph）
![rosgraph_ros2_sim](https://github.com/pwri-opera/zx120_ros2/assets/24404939/1192aea7-bae1-4220-b8fc-18c0c0e2e3b1)

### roslaunch zx120_bringup zx120_vehicle.launch.py実行時のノード/トピックパイプライン（rqt_graph）  
注）zx120実機上でのみ実行可能です
![rosgraph](https://github.com/pwri-opera/zx120_ros2/assets/24404939/7cb2ddb1-da25-43c3-8b22-58f838081da4)

