# 基于 AR 标记的 Turtlebot2 实时跟随实验报告

## 一、实验背景

本项目面向智能机器人多机协同任务，整体目标是让 TonyPi 或前方机器人完成指定路线运动，后方 Turtlebot2 能够根据视觉信息进行自主跟随，并在跟随过程中保持安全距离、避免碰撞。

在前期调试中，固定二维码航路点方案对场地布置、标签位置和路线规划依赖较强。为了提高跟随的灵活性，本实验采用“前车背贴 AR 标记，后车实时识别并跟随”的方案。该方案不需要提前在场地中布置多个固定标签，也不要求后车知道前车在地图中的绝对位置。只要后车能够通过 Kinect 相机持续看到前车背后的 AR 标签，就可以根据相对位姿完成实时跟随。

## 二、实验目的

本实验的主要目的如下：

1. 掌握 Turtlebot2、Kinect 相机和 ROS Kinetic 环境下 AR 标记识别节点的使用方法。
2. 使用 `ar_track_alvar` 识别前车背贴的 ALVAR AR 标签，并通过 `/ar_pose_marker` 获取目标相对位姿。
3. 根据 AR 标记的前后距离和左右偏差，设计 Turtlebot2 的实时跟随控制方法。
4. 理解 `target_id` 或 `marker_id` 参数在目标选择中的作用，实现不同编号 AR 标签的切换跟随。
5. 结合 `velocity_gate.py` 速度门控节点，在目标丢失、急停触发、扫描异常或前方障碍过近时实现安全停车。
6. 总结单车跟随调试方法，并为多台 Turtlebot2 队列跟随提供参数配置思路。

## 三、实验设备与软件环境

| 类别 | 内容 |
|---|---|
| 移动平台 | Turtlebot2 Kobuki 底盘 |
| 前方目标 | TonyPi 或另一台 Turtlebot2，尾部粘贴 AR 标签 |
| 视觉传感器 | Kinect RGB-D 相机 |
| 避障传感器 | 激光雷达或深度转换得到的 `/scan` |
| 操作系统 | Ubuntu 16.04 |
| ROS 版本 | ROS Kinetic |
| 主要 ROS 包 | `ar_track_alvar`、`multi_robot_following`、`turtlebot_bringup`、`openni_launch` |
| 编程语言 | Python 2.7 |

## 四、实验总体方案

本实验采用“前车贴 AR 标签，后车实时检测并跟随”的方式。

整体流程如下：

1. 在前车尾部粘贴指定 ID 的 ALVAR AR 标签。
2. 后车 Turtlebot2 启动底盘、Kinect 相机和 AR 标签识别节点。
3. `ar_track_alvar` 识别图像中的 AR 标签，并将识别结果发布到 `/ar_pose_marker`。
4. 跟随节点从 `AlvarMarkers` 消息中筛选指定 `marker_id` 的目标标签。
5. 根据目标标签相对于机器人本体的位置计算线速度和角速度。
6. 原始速度先发布到 `cmd_vel_raw`。
7. `velocity_gate.py` 检查急停、任务状态、扫描数据和前方安全距离。
8. 若满足安全条件，则输出速度到底盘；若存在风险，则输出零速度停车。

该方案的核心思想是：用 AR 标签的相对位姿代替前车全局坐标。后车不需要知道前车在地图中的绝对位置，只需要知道前车相对于自己的距离和方向。

## 五、关键代码位置

本实验主要参考仓库中的以下文件：

```text
turtlebot/follow2/multi_robot_following/launch/ar_tracker.launch
turtlebot/follow2/multi_robot_following/launch/follow_marker.launch
turtlebot/follow2/multi_robot_following/scripts/marker_follower.py
turtlebot/follow2/multi_robot_following/scripts/velocity_gate.py
```

各文件作用如下：

| 文件 | 作用 |
|---|---|
| `ar_tracker.launch` | 启动 `ar_track_alvar`，识别相机画面中的 AR 标签 |
| `follow_marker.launch` | 同时启动 AR 跟随节点和速度门控节点 |
| `marker_follower.py` | 根据指定 AR 标签的相对位姿生成原始速度指令 |
| `velocity_gate.py` | 检查急停、扫描、速度新鲜度和安全距离，输出安全速度 |

## 六、AR 标签识别

AR 标签识别使用 `ar_track_alvar` 完成。启动识别节点时，重点关注以下参数：

```text
marker_size_cm
image_topic
info_topic
optical_frame
```

其中 `marker_size_cm` 表示 AR 标签的实际边长，单位为 cm。该参数必须与打印出来的标签实际尺寸一致。

例如，如果打印的标签边长是 14 cm，则启动时应设置：

```bash
marker_size_cm:=14.0
```

如果实际标签边长是 9 cm，则应设置：

```bash
marker_size_cm:=9.0
```

如果该参数设置错误，系统虽然可能识别到标签，但估计出的距离会出现偏差，从而影响后车保持的跟随距离。

## 七、目标编号切换方法

实测时如果发现小车只能跟随 0 号 AR 标签，本质原因通常是程序默认目标编号设置为：

```text
target_id = 0
```

或：

```text
marker_id = 0
```

若需要跟随其他编号的 AR 标签，例如 1 号或 2 号，只需要同时满足两个条件：

1. 实际打印的 AR 标签编号正确。
2. 程序启动参数中的 `target_id` 或 `marker_id` 与标签编号一致。

例如，要跟随 1 号 AR 标签，可以先生成 1 号标签：

```bash
rosrun ar_track_alvar createMarker -s 14 1
```

然后启动跟随节点时设置：

```bash
roslaunch multi_robot_following follow_marker.launch marker_id:=1
```

如果使用简化版 `follow_ar.py` 节点，也可以设置：

```bash
rosrun ar_follower follow_ar.py _target_id:=1
```

常见对应关系如下：

| 要跟随的标签 | 标签生成命令 | 程序参数 |
|---|---|---|
| 0 号标签 | `createMarker -s 14 0` | `marker_id:=0` 或 `_target_id:=0` |
| 1 号标签 | `createMarker -s 14 1` | `marker_id:=1` 或 `_target_id:=1` |
| 2 号标签 | `createMarker -s 14 2` | `marker_id:=2` 或 `_target_id:=2` |

因此，更换跟随对象时，不需要重写控制程序，只需要修改目标 ID 参数即可。

## 八、跟随控制原理

AR 标签识别后，系统可以得到标签相对于相机或机器人本体的位置。

在相机坐标系中，可以近似理解为：

```text
z：目标前后距离
x：目标左右偏差
```

在机器人本体坐标系中，可以理解为：

```text
x：目标前方距离
y：目标横向偏差
```

跟随控制的基本思想如下：

1. 如果目标距离大于期望距离，后车向前移动。
2. 如果目标距离接近期望距离，后车减速并停止。
3. 如果目标位于左侧，后车向左转。
4. 如果目标位于右侧，后车向右转。
5. 如果目标丢失，后车停止等待重新识别。

控制律可以简单表示为：

```text
线速度 = 距离比例系数 × 距离误差
角速度 = 角度比例系数 × 横向偏差
```

其中：

```text
距离误差 = 当前目标距离 - 期望跟随距离
```

当距离误差较大时，后车前进速度较大；当距离误差接近 0 时，后车速度逐渐减小。实际程序中还会对线速度和角速度进行限幅，避免机器人运动过快。

## 九、速度门控与安全保护

为了避免机器人在目标丢失或传感器异常时继续运动，仓库中使用 `velocity_gate.py` 作为速度门控节点。

速度门控的作用包括：

1. 限制最大线速度。
2. 限制最大角速度。
3. 检查急停信号。
4. 检查扫描数据是否有效。
5. 检查前方是否存在过近障碍物。
6. 检查控制指令是否过期。
7. 在危险情况下输出零速度。

常见停车条件如下：

| 停车原因 | 说明 |
|---|---|
| 目标丢失 | AR 标签不在相机视野内 |
| 急停触发 | `/competition/estop` 为 true |
| 扫描异常 | `/scan` 数据缺失或过期 |
| 前方过近 | 前方障碍物或前车距离过近 |
| 速度指令过期 | 控制节点没有持续发布新速度 |
| 任务未开始 | 比赛状态未进入允许运动阶段 |

通过速度门控，即使控制节点出现异常，也不会让机器人继续沿旧速度盲目运动。

## 十、实验启动流程

单车调试时，可以按照以下顺序启动。

### 1. 启动 Turtlebot2 底盘

```bash
roslaunch turtlebot_bringup minimal.launch
```

### 2. 启动 Kinect 相机

```bash
roslaunch openni_launch openni.launch
```

### 3. 查看相机画面

```bash
rosrun image_view image_view image:=/camera/rgb/image_color
```

### 4. 启动 AR 标签识别

示例：

```bash
roslaunch multi_robot_following ar_tracker.launch robot_ns:=tb1 marker_size_cm:=14.0
```

如果相机话题名称不同，需要根据实际情况修改 `image_topic` 和 `info_topic`。

### 5. 检查 AR 是否识别成功

```bash
rostopic echo /ar_pose_marker
```

如果能看到类似下面的信息，说明识别正常：

```yaml
markers:
  - id: 1
    pose:
      position:
        x: ...
        y: ...
        z: ...
```

### 6. 启动 AR 跟随

例如跟随 1 号标签：

```bash
roslaunch multi_robot_following follow_marker.launch robot_ns:=tb1 marker_id:=1 follow_distance:=1.0
```

其中：

```text
marker_id:=1
```

表示跟随 1 号 AR 标签。

```text
follow_distance:=1.0
```

表示期望保持 1.0 m 的跟随距离。

## 十一、调试过程与实验现象

在调试过程中，首先需要确认 AR 标签能够被稳定识别。如果 `/ar_pose_marker` 没有输出目标 ID，则后续跟随节点不会运动。

当识别正常后，后车会根据 AR 标签位置进行动作：

1. 当前车距离较远时，后车向前移动。
2. 当前车距离接近期望值时，后车减速。
3. 当前车向左或向右偏移时，后车转向修正。
4. 当前车转弯过快导致标签离开视野时，后车停止。
5. 当前方障碍物过近时，速度门控输出零速度。

如果发现小车不运动，应优先检查以下内容：

1. AR 标签是否被识别。
2. `marker_id` 是否和实际标签 ID 一致。
3. `marker_size_cm` 是否和标签实际尺寸一致。
4. `/scan` 是否有有效数据。
5. 是否有急停信号。
6. 速度门控是否阻止输出。
7. 是否有其他节点同时向底盘发布速度。

## 十二、常见问题与解决方法

### 1. 只能跟随 0 号标签

原因：程序默认 `target_id` 或 `marker_id` 为 0。

解决方法：将参数改成实际要跟随的标签编号，例如：

```bash
marker_id:=1
```

或：

```bash
_target_id:=1
```

### 2. 能识别标签，但距离不准

原因：`marker_size_cm` 和实际打印尺寸不一致。

解决方法：用尺子测量 AR 标签实际边长，并将启动参数改成对应值。

### 3. 小车转向反了

原因：相机坐标系方向或控制符号与实际安装方向不一致。

解决方法：将角速度控制中的符号取反。

### 4. 小车左右晃动明显

原因：角速度比例系数过大。

解决方法：减小角速度增益，降低最大角速度。

### 5. 小车前后距离震荡

原因：线速度比例系数过大，或者前车运动不稳定。

解决方法：减小线速度增益，适当增大期望跟随距离。

### 6. 小车不动

可能原因：

1. 没有识别到 AR 标签。
2. `marker_id` 设置错误。
3. 速度门控因 `/scan` 异常停车。
4. 急停信号没有解除。
5. 任务状态不允许运动。
6. 速度输出话题不正确。

解决方法：分别检查 `/ar_pose_marker`、`/scan`、`/competition/estop`、`cmd_vel_raw` 和最终底盘速度输入话题。

## 十三、多机扩展方案

如果需要多台 Turtlebot2 队列跟随，可以给每台前车粘贴不同 ID 的 AR 标签。

示例：

| 后车 | 直接前车 | 前车 AR 标签 ID | 后车参数 |
|---|---|---|---|
| Turtlebot2-1 | TonyPi | 0 | `marker_id:=0` |
| Turtlebot2-2 | Turtlebot2-1 | 1 | `marker_id:=1` |
| Turtlebot2-3 | Turtlebot2-2 | 2 | `marker_id:=2` |
| Turtlebot2-4 | Turtlebot2-3 | 3 | `marker_id:=3` |

多机运行时需要注意：

1. 每台车使用不同 ROS 命名空间，例如 `/tb1`、`/tb2`。
2. 每台车使用不同 TF 坐标系，例如 `tb1/base_footprint`、`tb2/base_footprint`。
3. 每台车只跟随自己的直接前车。
4. 不同车的速度话题不能混用。
5. 不能让多个控制节点同时向同一底盘速度话题发布速度。

## 十四、实验结论

本实验完成了基于 AR 标记的 Turtlebot2 实时跟随方案。该方案通过识别前车背贴的 AR 标签，获取前车相对于后车的位置，并根据距离误差和横向偏差控制后车运动。

相比固定二维码航路点方案，该方法具有以下优点：

1. 不需要提前在场地布置多个固定标签。
2. 不依赖前车全局坐标。
3. 前车路线可以更加灵活。
4. 可以通过修改 `marker_id` 快速切换跟随目标。
5. 适合扩展为多车队列跟随。

同时，该方案也存在一定限制：

1. AR 标签必须保持在相机视野内。
2. 光照、反光和遮挡会影响识别稳定性。
3. 前车运动过快或转弯过急时，后车容易丢失目标。
4. 直接 AR 跟随只能停车避障，不能主动绕障。
5. 多机运行时需要严格处理命名空间和 TF 坐标系。

总体来看，AR 标记实时跟随是一种实现简单、调试直观、适合课堂实验和多机协同演示的方案。后续可以进一步结合地图导航、RGB-D 目标识别和轨迹预测，提高系统在复杂场景下的鲁棒性。

## 十五、后续改进方向

后续可以从以下几个方面继续优化：

1. 增大 AR 标签尺寸，提高远距离识别稳定性。
2. 改善标签安装位置，减少前车运动造成的抖动。
3. 将 AR 跟随与 `move_base` 结合，实现主动绕障。
4. 增加目标短时丢失后的预测机制。
5. 加入滤波算法，平滑 AR 位姿测量结果。
6. 针对多车队列设计统一的命名空间和启动文件。
7. 在实机测试中记录不同速度、距离和光照条件下的成功率。
# AR Marker Turtlebot2 Following Experiment Report

## 1. Experiment background

This experiment studies a direct visual-following method for a Turtlebot2.
The target robot carries an ALVAR AR marker on its rear side. The following
Turtlebot2 uses a Kinect RGB camera to detect the marker and estimates the
target's relative position from the AR observation. The follower then
generates linear and angular velocity commands from the relative distance and
lateral offset.

Unlike a fixed-marker waypoint method, this approach does not require a set
of AR markers to be placed throughout the whole course, and it does not
require the follower to know the target robot's global map pose. Its main
requirement is that the target marker remains visible to the camera.

## 2. Objectives

The objectives were:

1. Launch `ar_track_alvar` in ROS Kinetic and obtain AR marker observations.
2. Select a target marker by ID from `/ar_pose_marker`.
3. Use the target distance and lateral offset to control Turtlebot2 motion.
4. Stop when the target marker is not visible.
5. Add a simple front laser check for local stop-and-turn behavior.
6. Record the real launch file and Python node used in the virtual machine so
   the experiment can be reproduced from the repository.

## 3. Hardware and software

| Item | Configuration |
|---|---|
| Following platform | Turtlebot2 Kobuki |
| Target | TonyPi or another robot carrying an AR marker |
| Camera | Kinect RGB camera |
| Range sensor | Turtlebot2 laser scan on `/scan` |
| Operating system | Ubuntu 16.04 |
| ROS | ROS Kinetic |
| Python | Python 2.7 |
| Main detection package | `ar_track_alvar` |
| Experiment package | `ar_follower` |

The package source is stored at
`turtlebot/follow1/ar_follower/`.

## 4. System architecture

The experiment consists of two ROS nodes:

1. `ar_track_alvar` receives the RGB image and camera calibration data, then
   publishes detected markers on `/ar_pose_marker`.
2. `follow_ar.py` selects the configured marker ID, reads `/scan`, computes a
   velocity command, and publishes it to
   `/cmd_vel_mux/input/teleop`.

The actual package layout is:

```text
turtlebot/follow1/ar_follower/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── ar_track.launch
└── scripts/
    └── follow_ar.py
```

## 5. Experimental commands

The VM command history confirmed the following launch sequence:

```bash
roslaunch ar_follower ar_track.launch
rosrun ar_follower follow_ar.py
```

The first command starts the AR detector. The second command starts the
closed-loop follower. Before running them, the Turtlebot2 base, Kinect driver,
and laser driver must already be running and the required ROS environment must
be sourced.

The package can be copied into a catkin workspace with:

```bash
mkdir -p ~/catkin_ws/src
cp -r turtlebot/follow1/ar_follower ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 6. AR detection parameters

The launch file is:

```text
turtlebot/follow1/ar_follower/launch/ar_track.launch
```

Its default parameters are:

| Parameter | Default |
|---|---:|
| `marker_size` | `14.0` |
| `max_new_marker_error` | `0.08` |
| `max_track_error` | `0.2` |
| `cam_image_topic` | `/camera/rgb/image_color` |
| `cam_info_topic` | `/camera/rgb/camera_info` |
| `output_frame` | `/camera_rgb_optical_frame` |

`marker_size` must match the physical marker used in the experiment. If the
camera publishes different topic names, the launch arguments can be
overridden, for example:

```bash
roslaunch ar_follower ar_track.launch \
  cam_image_topic:=/camera/rgb/image_raw \
  cam_info_topic:=/camera/rgb/camera_info
```

The detector output can be checked with:

```bash
rostopic echo /ar_pose_marker
```

The target marker ID must appear in the message before the follower can
generate forward motion.

## 7. Following controller

The control node is:

```text
turtlebot/follow1/ar_follower/scripts/follow_ar.py
```

The controller uses the target marker position:

- `marker.pose.pose.position.z` is treated as the forward distance.
- `marker.pose.pose.position.x` is treated as the lateral offset.
- The distance error is `marker_z - target_dist`.
- The lateral error is converted to angular velocity with a proportional
  gain.

The main control equations are:

```text
linear.x  = Kp_linear  * (marker_z - target_dist)
angular.z = -Kp_angular * marker_x
```

Both commands are limited by configurable maximum values. When the marker is
closer than `0.4 m`, the implementation prevents backward motion by setting
the linear velocity to zero.

The default controller parameters are:

| Parameter | Default | Meaning |
|---|---:|---|
| `~target_id` | `1` | Marker ID to follow |
| `~target_dist` | `0.4` m | Desired following distance |
| `~Kp_linear` | `0.5` | Linear proportional gain |
| `~Kp_angular` | `1.0` | Angular proportional gain |
| `~max_linear` | `0.3` m/s | Linear speed limit |
| `~max_angular` | `1.0` rad/s | Angular speed limit |
| `~min_obstacle_dist` | `0.25` m | Front obstacle threshold |

For a different marker ID:

```bash
rosrun ar_follower follow_ar.py _target_id:=2
```

## 8. Local obstacle response

The node subscribes to `/scan` and examines the front 60-degree sector. It
ignores infinite and NaN readings and keeps the smallest valid range. If that
range is below `min_obstacle_dist`, the command is changed to:

```text
linear.x  = 0.0
angular.z = 0.5
```

This behavior is a local stop-and-turn response, not map-based obstacle
avoidance. It does not plan a route around the obstacle.

## 9. Experimental observations

The VM shell history confirms that the AR detector and direct follower were
started with the commands listed in Section 5. The repository now preserves
the launch file and Python node used by that experiment.

The available record does not include a complete measurement table for
tracking error, maximum following time, success rate, or obstacle-clearance
distance. Therefore this report does not claim a quantitative acceptance
result. For a formal comparison, future runs should record:

| Metric | Suggested measurement |
|---|---|
| Marker detection rate | Visible frames with target ID / total frames |
| Distance error | Mean and maximum absolute error |
| Lateral error | Mean and maximum absolute offset |
| Lost-target response | Time from marker loss to zero linear velocity |
| Following success | Completed trials / total trials |
| Obstacle response | Minimum observed range and stop latency |

## 10. Troubleshooting

### No marker message

Check that the Kinect driver is running, the image and camera-info topics
match the launch arguments, and the physical marker is printed at the size
given by `marker_size`.

### Marker is detected but the robot does not move

Check the marker ID, `/scan`, and the final velocity topic:

```bash
rostopic echo /ar_pose_marker
rostopic echo /scan
rostopic echo /cmd_vel_mux/input/teleop
```

Also make sure that another node is not competing for the same velocity
input.

### Distance is inaccurate

Measure the physical marker and update `marker_size`. A mismatch changes the
estimated relative distance.

### Oscillating steering

Reduce `~Kp_angular` or `~max_angular`. If the sign of the turn is reversed
because of the camera installation, verify the coordinate convention before
changing the sign in the controller.

## 11. Safety limitations

The extracted script is a compact teaching experiment and should be treated
as a direct velocity controller:

- It publishes directly to `/cmd_vel_mux/input/teleop`.
- It has no sensor-freshness timeout for `/scan`.
- Before the first valid scan, the obstacle distance is initialized to
  `999.0`.
- On a close obstacle, it stops forward motion but continues to command an
  angular velocity of `0.5`.
- It does not use the follow2 `velocity_gate.py`.
- It does not use AMCL, `move_base`, a costmap, or a mission-state bridge.
- It should not run simultaneously with another follower or navigation
  controller on the same robot.

An operator should remain next to the robot and be ready to stop the base
during all real-hardware tests.

## 12. Conclusion

The experiment demonstrates a compact AR-marker-based direct-following
pipeline for Turtlebot2. The approach is easy to understand and reproduce:
the camera detects a known marker, the follower converts relative position
into bounded velocity commands, and loss of the marker causes the robot to
stop.

Its main advantage is that it follows the target using relative visual
geometry without requiring a shared map. Its main limitations are dependence
on marker visibility, sensitivity to marker-size calibration, and the lack of
full navigation and sensor watchdog functions. The implementation is
therefore suitable as the repository's follow1 experiment record, while the
more complete map-navigation and velocity-gating functions should remain
documented under follow2.

## 13. Future improvements

Future work can add:

1. A sensor-freshness timeout and explicit emergency-stop input.
2. Low-pass filtering of AR pose measurements.
3. A separate safe velocity gate before the base input.
4. A proper local planner or `move_base` integration for obstacle avoidance.
5. Recorded rosbag trials and quantitative acceptance metrics.
6. Namespaced topics and TF frames for multi-robot deployment.
