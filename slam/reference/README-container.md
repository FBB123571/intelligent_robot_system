# Lab3 Kinect 实时建图与导航

沿用 lab2 的 Kobuki、Kinect、ROS Kinetic 和 `handex-lab2-robot` 容器。
所有以下命令均在宿主机终端执行。先完成建图并保存，再切换到导航。

## 1. 准备设备和代码

连接底盘与 Kinect 的 USB 和供电，打开底盘。在宿主机执行：

```bash
cd /home/matrix/Homework/roboexp
bash lab3/lab3.sh prepare
```

复用 lab2 的自动设备识别、权限设置和容器恢复流程，随后备份容器内旧
`path_planning`、同步当前源码并编译。需要 sudo 时在本机输入密码。
这一步会重启 lab2 真机容器，结束其中原有程序；仅启动底盘和相机，不会自动行驶。
设备重新插拔后重新运行此步骤。

若 lab2 底盘和相机已经正常运行，只同步编译本次代码：

```bash
bash lab3/lab3.sh sync
```

## 2. 启动实时建图

终端一（保持运行）：

```bash
cd /home/matrix/Homework/roboexp
bash lab3/lab3.sh mapping
```

启动前检查里程计、深度图、相机内参的时间戳以及 `odom` 到相机的 TF。
检测到本项目的建图、导航或轨迹控制节点时拒绝重复启动。
其他自定义控制程序也应先结束，保证只有键盘遥控控制底盘。

链路为 `/camera/depth/image_raw` + `/camera/depth/camera_info`
→ `lab3_depth_scan` → `/scan` → `slam_gmapping` → `/map`。
使用原有 OpenNI 驱动，不会再启动相机。扫描取中央 10 行，量程设为 0.45–5 m；
相机驱动以 `publish_tf:=false` 启动，统一使用底盘 URDF 的相机 TF，避免两套变换形成环。
相机应按 TurtleBot 标准安装位置水平固定，TF 必须与实际安装一致。
建图 launch 不含 `move_base`，不会自行发送运动指令。

终端二（实时显示）：

```bash
cd /home/matrix/Homework/roboexp
bash lab3/lab3.sh view
```

RViz 的 Fixed Frame 为 `map`；确认 Map 订阅 `/map`，LaserScan 订阅 `/scan`。
地图约每 5 秒刷新一次。建图阶段只观察地图，不使用 `2D Nav Goal`。

终端三（手动探索）：

```bash
cd /home/matrix/Homework/roboexp
bash lab3/lab3.sh teleop
```

终端需保持键盘焦点。`i` 前进，`j/l` 左右转，`k` 停止，`z` 降低速度。
缓慢行驶和转向，让视野覆盖不同方向，条件允许时返回经过的位置帮助校正。
建图中不要搬动机器人；键盘遥控不等同于自动避障。

## 3. 保存地图

先按 `k` 停车，保持建图节点运行，在新终端执行：

```bash
cd /home/matrix/Homework/roboexp
bash lab3/lab3.sh save room1
```

生成容器 `/root/lab3_maps/room1.yaml` 和 `.pgm`，同时复制到宿主机
`lab3/maps/`。YAML 使用相对图像路径，可将两个文件一起移动。
拒绝覆盖同名地图；不传名字时按时间生成新名字。30 秒没有保存完成则报错。

## 4. 接入自主导航

先停车，再 Ctrl+C 结束键盘遥控和建图终端。RViz 可保留，底盘与相机保留运行。
GMapping 和 AMCL 不能同时发布 `map → odom`。

```bash
cd /home/matrix/Homework/roboexp
bash lab3/lab3.sh navigation room1
```

该入口启动深度扫描、地图服务、AMCL 和 `move_base`，复用 TurtleBot 的速度平滑器
及安全控制器。全局规划使用 `navfn/NavfnROS`，前进速度上限 0.15 m/s。
启动时没有预设目标。先在 RViz 用 `2D Pose Estimate` 标出机器人在地图中的实际位置
与朝向，核对扫描轮廓和地图匹配，再用 `2D Nav Goal` 发送目标。

程序接口为标准 `/move_base` action，类型 `move_base_msgs/MoveBaseAction`，
目标坐标系 `map`；也可使用 `/move_base_simple/goal`（`geometry_msgs/PoseStamped`）。

命令行接口示例（这些坐标仅为语法示例，必须改成实际地图中的空闲位置；执行会运动）：

```bash
bash lab3/lab3.sh goal --goal 1.0 0.0 0 --timeout 120
bash lab3/lab3.sh goal --goal 1.0 0.0 0 --goal 1.0 1.0 90 --goal 0.0 0.0 180
```

每组三个数是 x（米）、y（米）、朝向（度）。按顺序等待到达，失败或超时停止序列；
超时或 Ctrl+C 会取消本客户端当前目标。取消依赖 ROS 通信和底盘停止机制，需实机验证。
原课程 `simple_navigation_goal.cpp` 保留，其中硬编码了目标，不要把它当作检查命令运行。

## 排查与验证范围

```bash
podman exec -it handex-lab2-robot bash -lc 'source /root/catkin_ws/devel/setup.bash; rostopic hz /scan'
podman exec -it handex-lab2-robot bash -lc 'source /root/catkin_ws/devel/setup.bash; rostopic hz /map'
podman exec -it handex-lab2-robot bash -lc 'source /root/catkin_ws/devel/setup.bash; rosrun tf tf_echo odom camera_depth_frame'
```

`/scan` 有频率不代表有有效距离，应在 RViz 检查点云轮廓。
Kinect 有窄视野、近距离盲区及玻璃/反光物体测距限制；二维扫描不能保证看到所有高度的障碍物。

2026-09-22：已在无设备映射、无外部网络的 ROS Kinetic 容器中完成 catkin 编译、
Python 2 语法检查、两个 launch 的节点解析，以及深度转换/GMapping 节点无硬件启动。
隔离的模拟 action 服务验证了三个目标顺序发送、坐标/朝向转换、失败中止和超时取消。
真机当时未连接，尚未验证实际深度转换、
地图质量、定位或避障效果。没有启动真机或发送运动目标。
