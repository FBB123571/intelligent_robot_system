# 2026-10-19 报告整合记录

## 仓库增量

- 原报告提交：`7bbf588`；拉取后的代码基线：`43a7771`。
- 共 14 个新增提交（包括合并提交）、22 个文件变更。
- 新增方式一独立报告 `latex/follow1/ar_marker_report.tex`（325 行），扩充实验总结。
- 从方式二迁出 `ar_tracker.launch`、直接 AR 跟随和速度门控，建立 `ar_marker_following` 包。
- 新增独立 `competition_core` 包，修改方式二跨包引用、Docker依赖、CI路径和适配器测试路径。
- 没有新增可复核的人形全程数据、方式三/四现场统计、成员名单或照片文件；相应占位继续保留。

## 已整合

- 保留方式一原始投稿；在总报告第5节整理两版本对照、尺寸/ID、坐标与控制律、时效/门控、调试、队列和实验边界。
- 不把旧控制器的 0.4 m、0.3 m/s 参数与新控制器的 1.0 m、0.12 m/s 混合。
- 区分任务状态在上游转换为许可，门控本身检查许可/速度/扫描，而非直接订阅任务字符串。
- 更新方式二跨包依赖、附录路径、证据表、参考来源、日期和代码基线。
- 新投稿中“结果/实测”叙述没有配套原始数据，保留为定性材料，不将参数示例和期望行为包装成新版完整验收。

## 源码核对发现（本次未修改控制代码）

1. `follow1/ar_marker_following/launch/follow_marker.launch` 两个 node 仍写 `pkg="multi_robot_following"`，脚本已迁入新包。原文启动示例不能直接认定可运行。
2. 独立 `competition_core/setup.py` 声明 `packages=['competition_core']`、`package_dir={'': 'src'}`，但源码位于 `src/__init__.py`、`src/planning.py` 等，缺少对应 `src/competition_core/` 目录；旧包仍含同名源码副本。需在目标catkin环境核对安装与实际导入来源。
3. 新 Dockerfile 仅复制 AR 包与独立核心包，未复制方式二/三，且 AR 包没有对应 rostest 目录。工作流步骤名称不能证明导航/方式三集成测试已经执行。
4. `ar_tracker.launch` 默认 `robot_ns=tb2`、直接跟随默认 `tb1`；部署需显式统一，且新旧默认标签尺寸不同。

## 校验

- `tools/validate_project.py` 通过，报告检查了88个Python文件及XML/YAML/动作结构；该检查不验证catkin安装和实机运行。
- `test_ros_guards.py`：11项消息替身测试通过，未使用真实ROS master、底盘、Kinect或机器人。
- PDF编译与页面检查详情见 `VALIDATION.md` 最新记录。
