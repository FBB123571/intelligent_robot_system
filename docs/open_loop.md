# 开环固定动作版本

入口 `humanoid/scripts/open_loop.py`，配置 `humanoid/config/open_loop.yaml`。直接执行：grab 一次 → grab_go_forward 十次 → 网页控制 left 十次 → grab_go_forward 十次 → 网页控制 right 十次 → grab_go_forward 十次 → 网页控制 release_legacy 一次，共 52 次动作调用。

使用 `/home/pi/TonyPi/ActionGroups` 的抓取、直走文件和 `/home/pi/tonypi_web_control/actions` 的三个指定文件。执行前读取并检查所有文件的 SQLite 帧格式；不打开摄像头、不定位物块、不确认抓取/放置、不重抓、不检测障碍、不调整头部，也不发送 Turtlebot 心跳。所有已确认身体关节 1–16 按文件原值发送，不额外锁定手臂；未确认的 17/18 列不发送。按各帧时长等待，没有额外插入站立动作或动作间停顿。

三个网页控制动作的本次快照保存在 `humanoid/ActionGroup/web_control_reference`，便于同步与追溯；配置仍指向你指定的网页控制目录，未覆盖这些实机文件。

```bash
cd /home/pi/intelligent_robot_system
# 文件检查
python3 humanoid/scripts/open_loop.py --check-actions
# 默认只打印顺序，不发送舵机指令
python3 humanoid/scripts/open_loop.py
# 实际运动：先让机器人对准物块，并确保整条行进路线可通行
python3 humanoid/scripts/open_loop.py --execute
```

正式运动前退出其他上位机、网页动作 worker 等舵机控制程序，避免同时控制串口。Ctrl+C 或 SIGTERM 请求停止，复用旧版 Board 的舵机停止接口；停止步态中间帧仍可能失衡。开环中的 carrying 和 done 仅表示执行阶段、指令序列完成，不代表抓取、转向或放置实际成功。

原闭环入口仍为 `competition.py`：寻找蓝色物块 → 对齐/接近并等待连续稳定识别 → 抓取 → 视觉确认抱持（失败则复位后重新识别，最多三次）→ 路线每个动作后检查抱持 → 等待放置区域确认 → 再查抱持 → 放置 → 确认手中无物且放置区域有蓝色物块 → 完成。闭环运动同时受图像障碍检查约束，反馈缺失或过期则停止；放置区域确认目前由外部/操作员信号提供。
