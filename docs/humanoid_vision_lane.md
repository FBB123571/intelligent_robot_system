# 红色抓取与视觉道路居中

当前前进已切回由 `go_forward` 腿部生成的 `grab_go_forward`，保持原 grab 手臂尾帧。开环顺序不变。新增闭环参数源为 `humanoid/config/competition_vision_lane.yaml`；旧 competition.yaml 保留旧蓝色示例，不用于当前红色场地。

本机闭环入口仍是 competition.py，使用 `runtime/competition.local.yaml`。抓取阶段识别红色、对齐、接近、抓取和确认抱持；抓取成功后视觉道路控制接管原固定步数路线。HSV 同时使用 0–10 和 170–179 两段红色色相，避免漏掉色相跨零的红色。抓取、持物、放置、障碍图中的目标排除均复用同一颜色规则，调试标签显示 RED。

道路阶段每次只执行一个动作组周期，停稳后丢弃动作前画面，再识别下一帧。道路边界检测用 Canny/Hough 提取 ROI 中的左右收敛线，以近处中心偏差和远近中心差修正方向。灰色地面和障碍不能仅靠颜色区分；这里的边缘只是实验几何线索，地砖缝、阴影和非连续障碍可能造成误判，必须用实际画面校验黄色边界线确实对应道路边界。

状态顺序：第一段道路居中 → 前方横向边界且左侧边界消失，连续确认后左转 → 看见并稳定对齐新道路 → 第二段居中 → 对称条件触发右转 → 稳定对齐第三段 → 第三段居中 → 两侧边界均消失且没有前方横向障碍，连续多帧并保持至少 1 秒 → 持物确认 → 放置 → 手中无红块且放置 ROI 出现红块，确认完成。

终点只允许在完成两次转向、最后直行至少指定周期之后判断。前两段丢失边界不表示到终点；相机失效、遮挡、图像质量不足或看见前方挡墙均不会触发这个条件。未校准的“无边界”也可能是漏检，故当前仅交付可视化调试方案，不声称终点识别已经可靠。

转弯采用已生成抱物左/右转步态，至少转过实测的最少周期后，再用新道路居中和方向稳定结果确认完成，并有最大周期限制。**仅凭这些单目边界不能保证精确 90°**；需在地面实际量角标定 turn_min_cycles/turn_max_cycles，若需要角度误差保证，还需 IMU 航向或视觉定位标记反馈。配置中的 3/12 是待标定示例，不是实测 90°数据。居中修正也使用同一转向周期，若单周期幅度过大，需另录较小幅度抱物转向动作后再标定。

## 相机与头部 pitch

头部 pitch 是 PWM 舵机 1，单位微秒（500–2500），与身体总线舵机中位 500 的刻度不同。`head_control.py` 可单独直接发送头部命令；默认只打印，必须带 --execute 才运动。不要与其他舵机控制程序同时使用。

```bash
cd /home/pi/intelligent_robot_system
# 打印指令，不动舵机：
python3 humanoid/scripts/head_control.py --servo 1 --pulse 1550
# 操作员观察方向，在中位附近小步测试，不要直接跳到量程端点：
python3 humanoid/scripts/head_control.py --servo 1 --pulse 1550 --backend controller --execute
```

增加 PWM 是否让此机器更俯视需要实际观察，不能仅凭“pitch”名称判断。若 1550 抬头，则向 1450 方向小步试验。把最终值写入本机配置的 head.pickup_tilt 和 head.carry_tilt；本次仍保留 1500，部署时不自动改动实机头部。俯视后必须重新标定 held_roi、placed_roi、道路 ROI 和近远采样行。抱物和道路需同时可见；视线被物块遮挡时，程序等待/停止，不能凭旧画面继续前进。

## 调试与启用

```bash
cd /home/pi/intelligent_robot_system
# VNC 桌面终端：红色框、目标 mask、黄色道路边界和状态
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --vision-only --debug
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --check-actions
# 开环已恢复 go_forward 下半身，无道路闭环：
python3 humanoid/scripts/open_loop.py --config runtime/open_loop.local.yaml --check-actions
```

现场分别标定红色阈值、held/placed ROI、障碍 guard ROI、道路几何与转角触发区域、抱物转向步数和头部角度。灰色地面通道检查目前同时使用颜色和边缘，不能代替测距。仅在这些检查通过后设置 lane_following.calibrated 和 hardware_calibrated 为 true，再运行：

```bash
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --execute --debug
```

本机配置默认不允许执行；--vision-only 不会移动头部或身体。模拟模式仍演练固定参考路线与抓放状态，不代表完成真实道路检测验证。普通 SSH 不会弹出 OpenCV 窗口，需要 VNC 桌面或图形转发。
