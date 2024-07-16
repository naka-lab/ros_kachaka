# インストール

```
cd ~/catkin_ws/src
git clone https://github.com/naka-lab/ros_kachaka.git
git clone https://github.com/naka-lab/ros_msgs.git
cd ~/catkin_ws/
catkin_make
pip install protobuf
pip install grpcio
sudo apt install ros-noetic-move-base

```

# 実行
## Kachaka制御ノードの起動
- `rosrun ros_kachaka kachaka_core.py (kachakaのIP)`
- この状態でサービスを利用して制御可能（例：[service_caller.py](https://github.com/naka-lab/ros_kachaka/blob/master/example/service_caller.py)）

## Rvizによる可視化と操作
- `roslaunch ros_kachaka kachaka_rviz.launch`
