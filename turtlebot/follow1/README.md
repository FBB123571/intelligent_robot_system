# 方式一与背贴 AR 跟随接入

用户方案中的第一车识别 TonyPi 背贴标记，可使用本包的参考控制器：

```bash
roslaunch multi_robot_following ar_tracker.launch robot_ns:=tb1 marker_size_cm:=9.0
roslaunch multi_robot_following follow_marker.launch robot_ns:=tb1 marker_id:=1 follow_distance:=1.0
```

先确保底盘、相机、扫描、TF 和心跳桥已启动，话题和 frame 约定见根 README。控制器把标签真实位姿变换到车体坐标，限制速度，丢失就停车。前方障碍由速度门控阻挡，**此参考控制器没有绕障规划**。不要与 move_base 或其他 follower 共用同一 `cmd_vel_raw`。

实验要求的“方式一”参考方案是场地固定标签给出航路点，与背贴移动标签有区别。负责此部分的队友需要选定最终评分方案；若做固定标签引导，应补充固定标签 ID→地图航路点映射和到点切换程序。不要把参考控制器当作四种方式全部完成的证明。

## 实验报告

本组采用基于 AR 标记的实时跟随方案。后车通过 Kinect 相机识别前车背贴的 AR 标签，根据目标相对位姿进行跟随，并结合速度门控实现安全停车。

详细实验报告：

[基于 AR 标记的 Turtlebot2 实时跟随实验报告](../../docs/ar_marker_follow_experiment_summary.md)
