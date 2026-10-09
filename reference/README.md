# 课程参考资料

`course_docs` 包含从 `D:\Kaiyuan\UGRD\大三\课件\机器人` 复制的第一至四讲以及预备知识实验手册；比赛要求的第五六讲原文保留在仓库根目录。

`ros_packages` 原样复制自 `D:\vmware共享文件夹`：`ar_location`、`path_planning`、`trajectory_following`、`show_trajectory`、`learning_topic`。用于接口比对与课程回顾，不是新的比赛实现。原始文件保留其作者和许可信息；本仓库不对课程原始资料额外声明开源许可。

未复制 VNC 安装程序、录制视频和仿真 world，因为它们不是当前比赛执行代码的依赖。不要把整个 `reference/ros_packages` 加入比赛 catkin workspace 并启动所有控制节点，原代码含全局硬编码话题和直接底盘速度发布，可能与多机门控冲突。

新代码采用课程已有的 `AlvarMarkers`、摄像头话题、`MoveBaseAction` 和 d6a 数据格式；实现未修改这些原始参考包。
