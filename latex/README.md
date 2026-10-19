# 多机协同实验报告（LaTeX 协作目录）

使用 **XeLaTeX**，入口是 `main.tex`。第一组负责 **TonyPi 人形机器人 + 方式二建图避障跟随**；姓名、学号、实际贡献先占位。第二/三/四组与方式一/三/四的对应关系暂供协作，需各组确认。

本轮按代码基线 `43a7771` 核对（2026-10-19），保留 `humanoid/maze_route` 正文，并整合新提交的方式一报告。完整实机结果尚未收齐，文中明确区分实现、合成测试、局部实机和全队验收。

方式一投稿 `follow1/ar_marker_report.tex` 是独立文档，保留原样；其内容经源码核对后归入 `sections/20_follow1.tex`。总报告区分旧 `ar_follower` 和新 `ar_marker_following`，同步更新方式二对 AR 包、共享门控和 `competition_core` 的依赖。不要把含 `\documentclass` 的独立投稿直接 `\input` 到主文档。

本轮还记录了新包 `follow_marker.launch` 残留旧 node 包名、独立共享核心安装目录与声明不一致，以及 CI 实际范围的限制；仅整合文稿，未顺带改动控制代码。更新范围见 `INTEGRATION_20261019.md`。

## 各组改哪些文件

| 责任 | 正文文件 | 图片目录 |
|---|---|---|
| 第一组：人形机器人 | `sections/10_humanoid.tex` | `figures/humanoid/` |
| 第一组：方式二 | `sections/30_follow2.tex` | `figures/follow2/` |
| 第二组（暂定）：方式一 | `sections/20_follow1.tex` | `figures/follow1/` |
| 第三组（暂定）：方式三 | `sections/40_follow3.tex` | `figures/follow3/` |
| 第四组（暂定）：方式四 | `sections/50_follow4.tex` | `figures/follow4/` |
| 共同分工 | `sections/02_division.tex` | 无 |
| 总体方案/实验/总结 | `sections/01_overview.tex`、`03_architecture.tex`、`60_experiments.tex`、`70_conclusion.tex` | `figures/site/` |
| 封面与代码基线 | `metadata.tex` | 无 |
| 原始结果索引 | `data/experiment_records.csv` | 证据路径写入 CSV |

各组章节内已留成员分工表。第一组成员表统一放在 `02_division.tex`。按真实贡献填写任务和提交/实验记录，不默认给任何人分配已完成工作。

## 编译

安装完整 TeX Live 或 MiKTeX（需 ctex、Fandol、TikZ、latexmk），从本目录执行：

```powershell
latexmk -xelatex -interaction=nonstopmode -halt-on-error -outdir=build main.tex
```

或 Windows 双击 `build.cmd`。产物为 `build/main.pdf`。也可在 Overleaf 上传整个 `latex/` 目录，主文件选 `main.tex`，编译器选 XeLaTeX；不要只上传章节文件。中文字体使用 TeX 随附 Fandol，避免依赖个人 Windows 字体。

`build/` 及辅助文件不提交 Git。仓库保留一份已编译初稿 `report-draft.pdf` 便于直接查看；正文修改后运行 `build.cmd` 或 `latexmk`，检查版式后从 `build/main.pdf` 更新它。封面明确标为协作初稿，不是最终验收报告。

## 增补图片

当前会话所示照片尚无本地文件路径，**没有假定截图已保存或伪造现场图片**。请把原图按 `figures/README.md` 的文件名放入对应目录。已有图位使用 `\reportphoto`，缺图时能正常编译并显示明确占位框；补齐后自动显示图片并移除图题的待归档说明。终稿需补齐图片或删除图位。

所有新图表必须有 `\caption` 和唯一 `\label`；建议前缀 `hum-`、`f1-`、`f2-`、`f3-`、`f4-`，正文使用 `\ref`。TikZ 示意图均标明不是实测图，不把它们当作现场结果。

## 协作与同步

```bash
git pull --ff-only
git switch -c report/group1
# 修改各自章节、图片与真实记录
git add latex
git commit -m "Update group 1 report and experiment evidence"
git push -u origin report/group1
```

通过 PR 合并。旧 Git 用 `git checkout -b report/group1`。各组尽量不同时修改 `main.tex`、`preamble.tex` 或其他组章节，公共分工信息由对接人统一合并。图片用英文文件名，不提交密码、私钥、视频、bag或编译缓存。

## 定稿前的缺项

- 确认第二/三/四组对应关系、各组成员与学号，核实分工和贡献。
- 补齐现场原图、参数与完整实测记录；没有数据时保留“待测”，不能写 0 次失败或 100% 成功。
- 最新标记路线的任务心跳、抓取确认/掉落恢复、严格丢目标处置尚有集成缺口，报告没有宣称已全部实现。
- 方式一实际为移动背贴 AR 直接跟随，与手册固定场地标签参考有区别；各组共同确认方式独立性。
- 原始运行数据模板是空表，不是实验结果；视频/bag放团队共享存储，表内保留证据位置。
