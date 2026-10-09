# 方式三 RGB-D 实时跟随 TonyPi

本组负责方式三，以比赛满分要求为验收目标：识别正确、持续跟随、主动绕障、无碰撞、终点进入并停稳。现已确认 Kinect/RGB-D 可用，独立雷达待确认。当前只能操作虚拟机，没有设备在身边。

## 当前进度

| 阶段 | 状态 | 完成依据 |
|---|---|---|
| 比赛要求和官方 follower 调研 | 已完成 | 原 follower 没有 TonyPi 身份识别或主动绕障 |
| RGB-D 数据诊断与无设备演练 | 已编写，待虚拟机运行 | 本包、合成数据源、离线核心测试 |
| 颜色与深度候选检测、持续关联 | 已编写，待身份特征标定 | 合成图像和 ROS 消息替身测试；不是通用 TonyPi 识别模型 |
| 跟随控制、主动绕障、终点判定 | 代码与导航配置已接入，待 ROS 联调 | 目标轨迹、move_base、滚动 odom costmap、独立门控 |
| 全队实机满分验收 | 未进行 | 需真实场地反复试跑与完整记录 |

`rgbd_probe` 检查手工选定的矩形区域（ROI），不是 TonyPi 检测器。`roi_depth_ok` 仅代表区域深度通过设定检查，不证明区域内是 TonyPi，更不证明配准、路径或车体周围安全。节点只发布 JSON 状态和标注图像，不输出速度。

## 第一课 在虚拟机完成数据链路演练

以下按 Ubuntu 16.04 / ROS Kinetic / Python 2.7 编写，先在虚拟机核对：

```bash
lsb_release -a
echo "$ROS_DISTRO"
python --version
```

若不是该环境，先记录实际输出，再调整依赖；不要将 Kinetic 的 cv_bridge 直接用 Windows 或 Python 3 运行。

### 1 安装和构建

将 GitHub 最新代码同步到虚拟机 `~/intelligent_robot_system`：已有仓库可执行 `git pull --ff-only`；也可通过共享文件夹复制完整项目。确认存在 `turtlebot/follow3/rgbd_following/package.xml` 后再构建。

```bash
source /opt/ros/kinetic/setup.bash
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-message-filters \
  ros-kinetic-image-view python-numpy python-opencv python-catkin-pkg
mkdir -p ~/catkin_ws/src
```

确认 `~/catkin_ws/src/rgbd_following` 尚不存在后创建链接；已有同名包时先检查路径，不覆盖：

```bash
ln -s ~/intelligent_robot_system/turtlebot/follow3/rgbd_following \
  ~/catkin_ws/src/rgbd_following
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rospack find rgbd_following
```

### 2 启动无设备演练

终端 A：

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch rgbd_following demo.launch
```

终端 B（每个新终端都需 source ROS 和工作空间）：

```bash
rostopic echo /rgbd_demo/rgbd_probe/status
```

正常应看到 `reason: roi_depth_ok`、`median_m` 约 1.2、`valid_ratio` 约 1.0，始终 `motion_enabled: false`、`target_identified: false`。1.2 米是生成器写入的测试值，不是实测结果。

终端 C：

```bash
rosrun image_view image_view image:=/rgbd_demo/rgbd_probe/preview
```

画面中央绿色矩形的深度为 1.2 米，背景为 2.5 米；框的位置由归一化 ROI `[左, 上, 右, 下]` 控制。

### 3 注入故障并解释输出

每次执行一条，观察终端 B；最后切回 `good`：

```bash
rosparam set /rgbd_demo/source/mode invalid
rosparam set /rgbd_demo/source/mode mixed
rosparam set /rgbd_demo/source/mode stale
rosparam set /rgbd_demo/source/mode frame_mismatch
rosparam set /rgbd_demo/source/mode pause
rosparam set /rgbd_demo/source/mode good
```

| mode | 预期现象 |
|---|---|
| invalid | `insufficient_depth`，无有效深度，不能当作空旷 |
| mixed | `mixed_depths`，前景和背景混在框内，不能盲取距离 |
| stale | `stale_or_future_input`，随后可能伴随 `input_timeout` |
| frame_mismatch | `registered_frame_mismatch`，随后可能伴随 `input_timeout` |
| pause | 最后一次有效输入超过 0.5 秒后 `input_timeout` |
| good | 恢复 `roi_depth_ok` |

以上演练不依赖底盘、相机或 Gazebo，也不能证明真实识别或避障。完成后记录 ROS 版本、构建结果和各故障输出。

## 第二课 拿到设备后测真实数据

只启动一套相机驱动，检查 `rostopic list`。要求同一相机的彩色校正图、配准到彩色相机的深度图，以及对应 CameraInfo；仅修改 frame_id 不能代替配准。诊断器要求三个消息的尺寸、坐标系一致且时间差不超过 0.05 秒，CameraInfo 应持续随图像发布。

下面是常见话题名，必须以实际驱动为准：

```bash
roslaunch rgbd_following probe.launch robot_ns:=tb3 \
  rgb_topic:=/camera/rgb/image_rect_color \
  depth_topic:=/camera/depth_registered/image_rect \
  info_topic:=/camera/rgb/camera_info \
  roi:='[0.4, 0.35, 0.6, 0.65]'
rostopic echo /tb3/rgbd_probe/status
rosrun image_view image_view image:=/tb3/rgbd_probe/preview
```

先让 TonyPi 的躯干覆盖框内区域，再测正面、背面、侧面、抱物行走和左右转弯。记录实际距离、深度中位数、有效比例、P90-P10 深度跨度和遮挡情况。用卷尺对照深度，检查物体轮廓与深度边缘对齐；尺寸和 frame_id 一致并不能证明真正配准。

参数默认值仅用于诊断：深度范围 0.4~4.0 米、有效样本至少 30、比例至少 0.5、P90-P10 跨度不超过 0.25 米，不代表已知 Kinect 实测能力或最终跟随距离。0/NaN/Inf 和超范围深度均不作为有效测量。preview 中断时窗口可能显示最后一帧，必须结合 status 判断新鲜度。

## 完整跟随代码

| 文件 | 功能 |
|---|---|
| `scripts/rgbd_target.py` | 同步 RGB/配准深度/CameraInfo，发布原始时间戳点云；颜色与深度筛选后在 odom 中关联同一目标 |
| `src/rgbd_following_core/vision.py` | HSV 多区间、轮廓、有效深度比例、中位数与深度聚类、物理宽度筛选 |
| `src/rgbd_following_core/tracking.py` | 多帧确认、跳变/歧义拒绝、丢失重确认、实测折线轨迹、全车进入终点判定 |
| `scripts/rgbd_follower.py` | 选择沿目标历史路径后退的跟随点，发送 move_base 目标，检查任务状态/TF/地图/目标新鲜度 |
| `config/navigation.yaml`、`config/costmap.yaml` | 点云驱动的滚动 odom 地图，Navfn + DWA 主动规划绕障；无需 map_server 或 AMCL |
| `scripts/rgbd_velocity_gate.py` | 独立壁钟看门狗、车体扫掠和停止距离检查，默认只发布预览速度 |
| `config/follow.yaml` | 识别、跟踪、间距、速度、终点及实机标定开关 |
| `tools/calibrate_rgbd_target.py`（仓库根目录） | 从照片中紧裁的目标色块生成候选 HSV 配置，包括红色色相跨零处理 |

颜色只是可标定的外观特征，不能直接证明身份。默认绿色用于合成输入，不代表 TonyPi 的真实颜色。明天应选择本体上足够大、稳定且区别于背景的色区，或裁判允许的彩色标识；若本体没有足够区分度或侧身时特征消失，需要调整标识/相机布置或替换识别器，不能靠放宽阈值宣称满分条件已经满足。同色目标靠得很近仍可能无法区分，程序在歧义时停车。

### 1 明天先做颜色与深度标定

先使用前述 probe 确认输入，再保存不同角度、步态和光照的彩色图及配准深度。示例（ROI 数字为照片中的像素坐标，应替换为实际裁剪）：

```bash
python3 tools/calibrate_rgbd_target.py tonypi.png \
  --roi 130 90 190 150 --output /tmp/tonypi-color-candidate.yaml
```

工具不会覆盖已有输出，会保留三个实机标定标志为 false。它只估计颜色，深度范围、物理宽度、地面高度、车体半径和制动能力仍要实测。每个配置应在正面/背面/侧面与干扰物图像中复核；不要只测一张正面照片。

该工具可在 Windows 的 `.venv/Scripts/python.exe` 下使用；若在虚拟机运行 `python3`，需另行确认该解释器能导入 cv2、numpy、yaml，ROS Python 2 已安装这些库不代表 Python 3 也有。

### 2 构建并启动影子模式

在原有依赖之外安装课程对应的导航包，再 catkin_make：

```bash
sudo apt-get install ros-kinetic-navigation ros-kinetic-tf2-ros
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch rgbd_following follow.launch execute:=false \
  settings:=/tmp/tonypi-color-candidate.yaml
```

底盘驱动/相机需单独启动。默认针对一台课程环境车辆：TF 为 `odom`、`base_footprint`，传感器话题在 `/camera/...`，里程计 `/odom`，最终底盘输入 `/cmd_vel_mux/input/navi`。如实际名称不同，在 launch 参数中覆盖。不要同时启动旧 follower、旧导航或其他速度控制源。

影子模式计算候选速度但只发布 `/tb3/cmd_vel_preview`，不创建底盘输入发布者。查看：

```bash
rostopic echo /tb3/perception_status
rostopic echo /tb3/follow_status
rostopic echo /tb3/gate_status
rosrun image_view image_view image:=/tb3/target_preview
```

`perception_status: tracking` 表示当前满足配置的多帧关联条件，不等于已经完成识别可靠性验收。启动时目标必须先形成至少 follow_distance 的实测轨迹，`waiting_for_target_trail` 为正常等待。

所有跟随模式都需要持续任务心跳。与负责 TonyPi 的小组协商接入 `/competition/mission`，可复用已有 UDP 心跳桥。进行独立影子测试时，可把 `mission_topic:=/tb3/test_mission` 传给 launch，在另一终端持续发布私有测试信号：

```bash
rostopic pub -r 5 /tb3/test_mission std_msgs/String "data: 'carrying'"
```

停止该发布后应回到 `mission_stopped`。测试 done 前先停止 carrying 发布，然后将 data 改为 done；不要同时发两种状态。私有测试信号只用于隔离调试，正式比赛切回真实任务源。

### 3 通过标定后执行

记录实际颜色识别可靠性、RGB-D 配准、相机 TF、高度过滤、车体半径、制动距离、侧面/低障碍覆盖和完整终点区域。`follow.yaml` 中填入凸多边形 `finish_polygon`，坐标属于本车本次开机的 odom；初始化摆位或 odom 重置后必须重标。

确认完成后再将 `identity_calibrated`、`depth_coverage_calibrated`、`hardware_calibrated` 设为 true。调整限速/半径时，要同步修改 `navigation.yaml` 和 `costmap.yaml`：规划器不能快于门控允许速度；圆形半径应覆盖车体和突出物，不能为穿窄通道而缩小。

```bash
roslaunch rgbd_following follow.launch execute:=true \
  settings:=/path/to/validated-follow.yaml
```

多机时明确指定每车自己的帧、输入和输出，例如：

```bash
roslaunch rgbd_following follow.launch execute:=false robot_ns:=tb3 \
  odom_frame:=tb3/odom base_frame:=tb3/base_footprint odom_topic:=/tb3/odom \
  rgb_topic:=/tb3/camera/rgb/image_rect_color \
  depth_topic:=/tb3/camera/depth_registered/image_rect \
  info_topic:=/tb3/camera/rgb/camera_info \
  cmd_vel_topic:=/tb3/cmd_vel_mux/input/navi
```

这部分只依赖本车里程计和传感器 TF，无需其他组共享地图原点。跟随目标始终来自自身 RGB-D 检测，不能把另一辆车的位姿当作方式三的识别结果。

### 4 停车、绕障与终点的边界

- 目标丢失立即撤销导航、关闭许可，原地多帧重新确认。长时间丢失后只在原目标附近重捕获，避免追上另一个物体；超 60 秒锁定故障，按裁判规则处理并重启。
- ABORTED/REJECTED/LOST、目标大跳变或 odom 大跳变会锁定停车；没有自动旋转找目标/清图恢复。重新定位后重启清除旧轨迹。
- 导航拒绝未知区域；门控检查车体扫过区域和停止距离。没有可用路径时停车，不能保证任意障碍布局都有绕行解。
- Kinect 仍有视野、近距离和遮挡盲区。2D 点云投影不能保证所有高度的空间均被看见；地面清除射线、障碍高度阈值需实测，4 cm 以下障碍尤其要核查。若场地覆盖无法满足要求，补充传感器或调整布置，不能关掉未知区域检查来强行行走。
- done 后还要确认目标停稳 2 秒、到达最终带间距的跟随点、整个圆形车体进入 finish_polygon 才 finished。`finish_zone_not_reached` 意味着当前队形停在终点区之外，需要修正终点布局/领车停车位置；不能把它当作完赛。
- ROS `/competition/estop` 接入门控，持续 true 会停车；它不控制 TonyPi。所有速度控制源必须统一，禁止其他节点绕过门控直接发底盘速度。

与其他组联合验证遮挡、相互碰撞与终点容纳空间。单元测试或单车跑通都不能代替全队满分验收。

## 离线验证与技术依据

当前 Windows 独立虚拟环境中全项目 88 项测试通过，无跳过，其中方式三 56 项；ROS/navigation 与硬件验收尚未完成，详见 [验证记录](../../docs/validation.md)。

```bash
python3 tools/validate_project.py
python -m unittest discover -s tests -p 'test_rgbd*.py' -v
```

深度和控制核心测试不依赖 ROS、OpenCV 或 NumPy，可用 Python 2.7/3；视觉流水线测试需 OpenCV/NumPy/PyYAML；ROS 适配层替身测试使用 Python 3。Windows 可使用仓库 `.venv/Scripts/python.exe`，虚拟机的 ROS 节点仍使用其系统 Python 2.7。真实 ROS 构建、move_base 避障、图像显示和硬件性能仍需虚拟机/设备验证。

接口核对来源：[ROS 深度图单位规范](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst)、[Kinetic 消息同步器](https://github.com/ros/ros_comm/blob/kinetic-devel/utilities/message_filters/src/message_filters/__init__.py)、[Kinetic cv_bridge](https://github.com/ros-perception/vision_opencv/blob/kinetic/cv_bridge/python/cv_bridge/core.py)。
