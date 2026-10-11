# Lab3：Kinect 实时建图与自主导航

本目录运行环境为 **Ubuntu 16.04 虚拟机 + ROS Kinetic + Kobuki/TurtleBot 2 + Kinect v1**。
Ubuntu 没有正式的 16.06 版本；请用 `lsb_release -a` 确认课程虚拟机是 16.04。
以下命令全部在虚拟机终端直接执行。无需容器，也不依赖 `roboexp/lab2`。
ROS Kinetic、TurtleBot 和 OpenNI 的安装须已完成；Kinect v2 不能使用本 OpenNI v1 流程。

## 1. 虚拟机、依赖和编译

把 Kobuki USB 串口及 Kinect 的 camera/audio/motor USB 设备连接到虚拟机，接通供电并打开底盘。
宿主机不能同时占用设备；Kinect 重新枚举后可能需要再次连接到虚拟机。

```bash
source /opt/ros/kinetic/setup.bash
sudo apt-get update
sudo apt-get install ros-kinetic-turtlebot-bringup ros-kinetic-turtlebot-navigation \
  ros-kinetic-turtlebot-teleop ros-kinetic-turtlebot-rviz-launchers \
  ros-kinetic-openni-launch ros-kinetic-depthimage-to-laserscan \
  ros-kinetic-gmapping ros-kinetic-map-server ros-kinetic-amcl \
  ros-kinetic-move-base ros-kinetic-navfn ros-kinetic-dwa-local-planner \
  ros-kinetic-actionlib ros-kinetic-move-base-msgs
sudo usermod -aG dialout "$USER"
```

注销并重新登录后，检查设备：

```bash
lsusb
ls -l /dev/kobuki /dev/serial/by-id/ /dev/ttyUSB* 2>/dev/null
```

若无 `/dev/kobuki`，在每个运行底盘的终端把 `TURTLEBOT_SERIAL_PORT` 设为实际 Kobuki 串口，
优先使用 `/dev/serial/by-id/...` 的稳定路径，不要盲目假设它是 `/dev/ttyUSB0`。
若串口权限仍不足，检查 `id` 是否包含 `dialout`；相机权限应使用随驱动安装的 udev 规则。

```bash
git clone https://github.com/yky666/intelligent_robot_system.git
cd intelligent_robot_system
bash slam/lab3.sh sync
```

PR 合并前需切换到 PR 分支。`sync` 在 `~/catkin_ws/src` 链接本包并执行 `catkin_make -j2`，
拒绝覆盖已有其他 `path_planning` 包。也可先 `export LAB3_CATKIN_WS="$HOME/lab3_catkin_ws"` 使用独立工作区；
后续每个终端保持相同设置。源码的 Python 节点使用 Kinetic 默认的 Python 2。

## 2. 启动底盘和相机

以下各终端先 `cd intelligent_robot_system`，分别保持运行。

终端一：

```bash
# 仅当 /dev/kobuki 不存在时设置实际路径：
# export TURTLEBOT_SERIAL_PORT=/dev/serial/by-id/实际Kobuki设备
bash slam/lab3.sh base
```

终端二：

```bash
bash slam/lab3.sh camera
```

对应原生 ROS 命令为 `roslaunch turtlebot_bringup minimal.launch` 和
`roslaunch openni_launch openni.launch publish_tf:=false`。
脚本设置 Kobuki、hexagons、Kinect 和真机模式；底盘 launch 自动启动 ROS master，
发布机器人模型、里程计、速度平滑器与安全控制器。相机 TF 使用机器人模型，
`publish_tf:=false` 避免重复变换。相机应在 TurtleBot 标准位置水平固定；实际安装与 TF 必须一致。

## 3. 建图、显示和键盘探索

终端三：

```bash
bash slam/lab3.sh mapping
```

终端四：

```bash
bash slam/lab3.sh view
```

终端五：

```bash
bash slam/lab3.sh teleop
```

RViz 的 Fixed Frame 设为 `map`，Map 订阅 `/map`，LaserScan 订阅 `/scan`。
键盘终端保持焦点，`i` 前进、`j/l` 转向、`k` 停车、`z` 降速。
缓慢探索并尽量回到经过的位置；建图时不用 `2D Nav Goal`，不搬动机器人。
键盘控制没有自动避障。

链路：`/camera/depth/image_raw` + `/camera/depth/camera_info` → `lab3_depth_scan`
→ `/scan` → `slam_gmapping` → `/map`。扫描取中央 10 行，量程 0.45–5 m，地图约每 5 秒刷新。
启动脚本检查里程计、深度图和相机内参时间戳及 `odom → camera_depth_frame` TF，
检测到已运行的实验建图/导航/运动节点时拒绝重复启动。其他控制程序也应先退出。

## 4. 保存地图

先按 `k` 停车，保留建图节点，在新终端执行：

```bash
bash slam/lab3.sh save room2
```

生成 `slam/maps/room2.yaml` 和 `slam/maps/room2.pgm`，拒绝覆盖同名地图。
不传名称时生成时间戳名称；30 秒未完成则报错。仓库附带 `room1` 示例地图，
仅适用于其采集环境，换场地需要重新建图。

## 5. 自主导航

停车后 Ctrl+C 结束键盘遥控和建图终端，保留底盘、相机及 RViz。
GMapping 和 AMCL 不能同时发布 `map → odom`。

```bash
bash slam/lab3.sh navigation room2
```

先在 RViz 用 `2D Pose Estimate` 设置实际位置与朝向，核对扫描轮廓与地图匹配，
再用 `2D Nav Goal` 发送空闲区域中的目标。此时机器人会运动。
全局规划器为 `navfn/NavfnROS`，前进速度上限 0.15 m/s，无预设导航目标。
接口为 `/move_base` action（`move_base_msgs/MoveBaseAction`），目标坐标系 `map`；
也接受 `/move_base_simple/goal`（`geometry_msgs/PoseStamped`）。

```bash
# 示例坐标必须按实际地图修改；执行会运动。
bash slam/lab3.sh goal --goal 1.0 0.0 0 --timeout 120
bash slam/lab3.sh goal --goal 1.0 0.0 0 --goal 1.0 1.0 90 --goal 0.0 0.0 180
```

每组三个数为 x（米）、y（米）、朝向（度）。按顺序等待到达；失败或超时停止序列。
超时或 Ctrl+C 取消当前客户端目标，实际停止行为需实机验证。
课程原始 `simple_navigation_goal.cpp` 包含硬编码目标，不要将它作为检查命令运行。

## 6. 直接使用 ROS 命令

在每个新终端设置工作区后可直接运行，不必用脚本：

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch path_planning mapping.launch
# 另一个终端：roslaunch turtlebot_rviz_launchers view_navigation.launch
# 另一个终端：roslaunch turtlebot_teleop keyboard_teleop.launch
# 停车，保持建图运行后保存地图：
# rosrun map_server map_saver -f /绝对路径/intelligent_robot_system/slam/maps/room2
# 停止建图和键盘遥控后启动导航：
# roslaunch path_planning navigation.launch map_file:=/绝对路径/intelligent_robot_system/slam/maps/room2.yaml
```

## 7. 排查与验证范围

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic hz /odom
rostopic hz /camera/depth/image_raw
rostopic hz /scan
rostopic hz /map
rosrun tf tf_echo odom camera_depth_frame
```

`/scan` 有频率不代表测距有效，需在 RViz 核对轮廓。Kinect 存在视野限制、近距离盲区，
玻璃/反光物体可能测距失败，二维扫描不能覆盖所有高度的障碍物。

原 lab 于 2026-09-22 在隔离 ROS Kinetic 环境完成 catkin 编译、Python 2 语法检查、
launch 节点解析、无硬件节点启动及模拟 action 的顺序目标、失败中止和超时取消检查。
本次迁移只做静态检查，尚未在 Ubuntu 虚拟机或真机重新验证，不代表地图质量、定位和避障已验证。
`path_planning/tests/navigation_interface_check.py` 只能在无真机、无真实 `move_base` 的隔离 ROS master 中运行。

`reference/` 保存原始容器说明和脚本供追溯，主启动流程使用上述原生 ROS 命令。
原始课程 Word 文档与 `maps/room1.*` 一并保留；Python 缓存和构建产物不上传。
