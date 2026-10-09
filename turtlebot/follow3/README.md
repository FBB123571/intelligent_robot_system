# 方式三由队友接入 RGB-D 实时跟随

课程提供 `roslaunch turtlebot_follower follower.launch`，可以先用于单车观察深度跟随效果。最终队友需完成 TonyPi 身份筛选、目标丢失停车、扫描避障、命名空间适配和终点停车。

新控制器应输出 `/tb3/cmd_vel_raw`，并每 0.1 秒发布 `/tb3/motion_allowed`。只有目标可靠、任务心跳新鲜、状态为 carrying/releasing/done 且无急停时，才允许速度。复用 `velocity_gate.py`，将输出接 `/tb3/cmd_vel_mux/input/navi`。

不应让原 `turtlebot_follower` 直接发布底盘速度并绕过门控。本次代码重点实现 TonyPi 和方式二，**尚未实现独立的 RGB-D TonyPi 识别控制器**。原课程 follower 是否适合 TonyPi 的体型和步态，必须由负责队友实测。
