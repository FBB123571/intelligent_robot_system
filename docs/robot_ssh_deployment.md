# TonyPi 实机连接与部署

2026-10-10 实机检查：PC 内置 WLAN 连接 SYSU-SECURE，外接 UGREEN WLAN 2 连接机器人热点。PC 机器人侧地址为 192.168.149.191，机器人地址为 192.168.149.1。地址以实际 DHCP 分配为准；无需交换两张网卡。

```powershell
ssh pi@192.168.149.1
```

密码只在 SSH 登录提示中输入，不保存到仓库。机器人是 Raspbian 10，Python 3.7.3、OpenCV 4.5.3、PyYAML 5.3.1 已安装。

## 硬件接口

SDK 路径：`/home/pi/TonyPi/HiwonderSDK/hiwonder`。实机官方 `ActionGroupControl.py` 调用旧版 `Board.setBusServoPulse`；`BusServoCmd.py` 使用 `/dev/ttyAMA0`、115200 波特率。机器上同时存在新版 `ros_robot_controller_sdk.py`，默认同一串口使用 1000000 波特率，因此不能以文件存在判断实际控制板类型。

本项目 `competition.yaml` 和 `joints.yaml` 明确使用 `hardware_backend: legacy`。未来更换控制板再按实机协议选择 `controller`。`auto` 仅为比赛程序的模块导入兼容模式，不检测控制板协议。

## 独立部署与检查

按团队要求，完整 Git 仓库现已同步到 `/home/pi/intelligent_robot_system` 根目录，分支为 `main`，远程为 `https://github.com/yky666/intelligent_robot_system.git`。保留之前的 `releases` 部署快照；根目录的 `runtime/competition.local.yaml` 指向根目录 `runtime/ActionGroups`，用于现场配置。`runtime` 和 `releases` 加入实机 `.git/info/exclude`，不参与提交，避免覆盖现场标定数据。

日常从根目录运行：

```bash
cd /home/pi/intelligent_robot_system
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --check-actions
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --simulate
# 在 VNC 桌面终端中预览摄像头
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --vision-only --debug
```

机器人能访问 GitHub 时，使用 `git pull --ff-only` 更新代码。只有机器人热点连接时，本次通过 PC 传输 Git bundle 完成同步，仓库仍保留完整 Git 历史；后续可继续由 PC 转发更新，无需让机器人直接上网。

比赛代码部署到 `/home/pi/intelligent_robot_system/releases/<版本目录>`，动作文件放在版本目录的 `runtime/ActionGroups`，通过部署配置的 `action_dir` 指向该目录。保持 `/home/pi/TonyPi` 原有文件完整。抓取、直走、左转和接近动作优先采用实机已有文件；右转和反向抓取放置候选依然需要实测。

本次版本目录为 `/home/pi/intelligent_robot_system/releases/20261010-legacy-preflight`，上传的 23 个文件逐个校验 SHA256。实机通过动作文件检查、完整状态机模拟和 12 项单元测试；本地全部 160 项单元测试通过。这些验证不代表实际抓取、步态、放置或摄像头标定已经完成。

在部署目录执行：

```bash
cd /home/pi/intelligent_robot_system/releases/20261010-legacy-preflight
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --check-actions
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --simulate
```

以上两条命令不初始化舵机驱动、不发送动作；模拟也不打开摄像头、不发送跟随心跳。默认保留 `hardware_calibrated: false` 和未标定的 ROI。

## 图像与正式运行

普通 SSH 会话没有图形 DISPLAY，`--debug` 的 OpenCV 窗口需要在机器人 VNC 桌面的终端运行（实机监听 5900），或使用已经配置好的 X11 图形转发。在该桌面切换到部署目录，然后运行：

```bash
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --vision-only --debug
```

运行前检查摄像头占用。现场同时有官方 TonyPi 服务、Joystick、上位机和网页控制 worker 等进程；不得让多个程序同时发送舵机指令。正式比赛需要逐项完成动作验证、视觉 ROI 和路线标定，释放相机与串口的控制权，然后使用 `--execute --debug`。不能只把标定开关改成 true 就当成验证完成。

实机还存在 `put_down.d6a`、`put_down2.d6a`、`put_up_object.d6a`。当前比赛配置已选择实机 `put_down`，并采用固定抓取手臂的组合转向候选，详见 [组合转向动作](carry_turn_composition.md)；实际抱物姿态与放置起始姿态的连接尚需现场验证。
