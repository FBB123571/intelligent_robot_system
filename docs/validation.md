# 验证记录

## 已进行的离线检查

2026-10-09 方式三控制链路开发：在仓库独立 `.venv` 安装 PyYAML 6.0.3、NumPy 2.5.3、OpenCV headless 5.0.0.93，Python 3.14.7 下全项目 **88 项测试通过，无跳过**；其中方式三 56 项。38 个 Python 文件及 XML/YAML/动作结构与方式三配置语义检查通过。覆盖真实 OpenCV 合成图像检测、毫米/米换算、颜色歧义、连续关联、目标丢失与超时、光学坐标变换、停止路径碰撞、影子模式无底盘发布者、未标定拒绝执行、导航失败锁定、最终目标小位移以及整车进入终点。已编写 move_base 滚动 odom 地图与 DWA 配置，但没有运行真实 ROS/navigation，因此不能把核心和消息替身测试称为实际绕障测试。Kinetic 的旧版 OpenCV、Python 2、catkin 构建和整车性能仍待验证；CI 已增加纯核心 Python 2 测试但本次未执行远端 CI。

2026-10-09 方式三开发复查：新增 RGB-D 诊断核心、ROS 回调替身测试、无设备合成数据入口。当前 Windows Python 3.14.7 上共发现 48 项测试，45 项通过，3 项原有视觉测试因缺少 OpenCV/NumPy 跳过；方式三新增 16 项全部通过。静态检查通过 26 个 Python 文件及 launch/package XML、YAML、动作文件。尚未在 ROS 虚拟机运行 catkin_make 或 demo.launch，也未验证 Kinect。下文“32 项全部通过”为此前记录，不能视为本次环境结果。新增 Python 2 核心检查已配置到 CI，本次未实际运行 CI。

在 Windows 开发主机运行 Python 单元测试和项目静态检查；没有连接 TonyPi、Kinect 或 Turtlebot2，也没有 ROS master/导航运行环境。

2026-10-09：Python 3.14 环境下 32 项单元测试通过，含 OpenCV 合成色块/球和无目标图像检查；Python、launch/package XML、YAML、路线与全部五个 d6a 文件结构检查通过。Python 2.7 的核心测试和节点语法检查由 GitHub Actions 单独执行，本机没有 Python 2/ROS Kinetic，不能把 Python 3 通过说成 ROS 集成通过。

检查覆盖：路径长度跟随转角、起步间隔、测量跳变/非有限数拒绝、地图边界/未知区域/占用、定位协方差、扫描有效性、连续观测抓取、动作格式、手臂夹持锁定、夹持信号故障、门控超时/限速/急停、导航失败锁定、目标丢失取消导航与终点停车。节点适配层使用消息替身，不是 ROS 集成测试。

`competition.py --simulate` 已走完整个示例状态序列，动作只打印，没有向真实设备发送命令。生成的两个候选动作仅完成格式验证，不代表机械运动验证。

运行可复查命令：

```bash
python3 tools/validate_project.py
python3 -m unittest discover -s tests -v
python3 humanoid/scripts/competition.py --simulate
```

## 尚待填写的实测值

| 项目 | 测量结果 | 操作者与日期 |
|---|---|---|
| VM ROS 发行版与 Python 版本 | 待确认，Ubuntu 16.04 按 Kinetic 配置 | |
| Pi SDK 与 Python 版本 | 待测 | |
| 抓取物体尺寸、重量、颜色 | 待测 | |
| head pan/tilt 零位 | 待测 | |
| LAB 阈值、grab_bottom、grab_area | 待测 | |
| 单次抱物前进距离与偏航 | 待测 | |
| 单次抱物左转、右转角 | 待测 | |
| 抓取→步态→放下的动作衔接 | 待测 | |
| 夹持手臂舵机编号及锁定效果 | 待测 | |
| 右转/放下候选改动与验证 | 待测 | |
| 实测地图 yaml/pgm 与场地照片 | 待补 | |
| 各车 map 坐标一致性 | 待测 | |
| 车体尺寸、通道净宽、队列间隔 | 待测 | |
| 相机标记 ID、边长与可靠识别距离 | 待测 | |
| 单车及整队丢失目标停车时间 | 待测 | |
| 遮挡后原地重新捕获 | 待测 | |
| 前方障碍/无线断链/扫描断链停车 | 待测 | |
| 两次转弯通过、抱持无掉落 | 待测 | |
| 四车终点到达与停稳、跟随方式分类 | 待测 | |

建议每次试跑记录 Git commit、配置文件、地图版本、成功/失败原因，便于另一台电脑复现。失败记录与成功记录都保留。
