# GCC-PHAT 历史会话索引

这些是 Claude Code 的原始会话记录（JSONL），位于
`C:/Users/KasuganoSora/.claude/projects/<路径编码>/<session-id>.jsonl`。
共 6 个与本项目相关的会话，覆盖 2026-10-02（Python 原型）到 2026-11-04（慢路径、技能包与技术报告）。

> 隐私：S2 的历史消息中含一段 API key 明文，**禁止**复制到文档、代码、提交或对外分享。
> 原始记录还含大量工具输出与本机路径，对外提供前需脱敏。
> 本次重生成（2026-11-04）后对 `skill_distill/*.txt` 做凭据特征扫描（`sk-`/`AIza`/`ghp_`/`Bearer`/`api_key`），0 命中。

## 会话一览

| # | 日期（UTC） | Session | 工作目录 | 主题 | 规模（原始） |
|---|---|---|---|---|---|
| S1 | 10-02 06:48–06:54 | `63883335-fa87-43e4-a74e-72998472f9da` | `Desktop/gcc-phat` | Python 原型 `gcc.py`：修 scipy/matplotlib 环境；定位两处算法 bug（测试信号在窗内周期、分数时延），修复后 50/50 命中。**项目起点** | 0 用户文本 / 6 助手 / 28 工具调用 |
| S2 | 10-02 11:36 → 10-05 12:43 | `efb92714-69a3-4a82-a869-ead490bf24bf` | `.../GCC-PHAT/GCC-PHAT/GCC-PHAT.srcs` | 首轮 RTL 联调：`$unit` 重复声明（Synth 8-988/8-2644）、FFT/CORDIC/UART 通路、综合与时序、静音时数值跳动 | 82 用户 / 517 助手 / 1827 工具调用 |
| S3 | 10-22 11:54 → 10-24 10:44 | `563f95e1-3118-4732-83df-b616c437073c` | `.../GCC-PHAT/GCC-PHAT` | 阵列半径改 20 cm；mp3/`audio1` 真实录音离线验证；投票与逐帧稳定性；天台现场测试；bit/bin 交付 | 134 用户 / 204 助手 / 1952 工具调用 |
| S4 | 10-24 11:02 → 10-31 15:37 | `3bec8b23-e1a9-40ea-ad59-d89e185e3085` | `.../GCC-PHAT/GCC-PHAT` | 5 m → 50 m 远场优化（AGC/掩码/投票）；STM32 串口接线（B13/PA10）与云台；重叠帧 + 64 帧提前锁定；文档重构 | 52 用户 / 87 助手 / 1721 工具调用 |
| S5 | 10-31 16:49 → 11-01 10:04 | `00d141ea-8d8b-49b7-b753-b8741a4da933` | `.../GCC-PHAT` | 版本盘点（10.27 版）；重建 `GCC-PHAT_2` 工程树；bit/bin 归档；清理 10.30 版本痕迹 | 31 用户 / 33 助手 / 748 工具调用 |
| S6 | 11-01 10:10 → 11-04 07:07 | `9b8c01a8-864d-47c1-b7fc-76148f1dc055` | `.../GCC-PHAT_2` | FPGA 直驱 SG90 云台（C19/C18，负方位翻转）；时延优化；256 点 FFT 时分复用（24 DSP）；1024 点慢路径谐波分析（方案二）；技术报告按模板定稿（18 页）；`gcc-phat-4ch` 技能包 | 70 用户 / 57 助手 / 1743 工具调用 |

合计：约 369 条有效用户消息、904 条助手文本、8,019 次工具调用、188 条工具错误。

## 与文档阶段的对应

| 阶段 | 会话 | 产出文档 |
|---|---|---|
| 算法原型 | S1 | `doc/01_定点误差分析报告.md` 的早期素材 |
| RTL 首轮联调 | S2 | `doc/02_接口时序说明.md`、`doc/03_仿真验证记录.md` 初稿 |
| 几何/投票/现场 | S3 | `doc/04_板级调试指南.md`、`doc/08_真实录音仿真与掩码配置对比.md` |
| 远场优化/云台 | S4 | `doc/06`、`doc/07`（用户自执行升级） |
| 版本重建 | S5 | `doc/00_交付说明.md`、`README.md` 的版本段 |
| 时分复用/慢路径 | S6 | `doc/00/03/05` 更新、`doc/09_AI协作工作流.md`、`skill/slow-harmonic-tdoa/`、`skill/gcc-phat-4ch/`、桌面技术报告 |

## 已提炼的产物

- `skill_distill/`：`mine.py`（解析器）、`compact.py`（用户语料压缩）、`sessions_summary.txt`、
  `user_compact.txt`、`assistant_digest.txt`、`tool_actions.txt`、`errors.txt`（188 条工具错误）。
- `skill/slow-harmonic-tdoa/`：当前功能 skill（1024 点慢路径谐波 TDOA），
  含 `SKILL.md` 与 `references/{algorithm,integration,verification,pitfalls}.md`。
- `skill/gcc-phat-4ch/`：四路并行 256 点 FFT 功能包（`SKILL.md`、4 篇 references、
  定向 TB、安装/检查/复跑脚本、验证日志）；根目录 `skill (2).zip` 为打包副本。
- `skill/README.md`：包说明、每项适用范围/使用方法/失效条件/复用验证。
- `doc/09_AI协作工作流.md`：人读版全工程阶段流程（背景参考）。
- `C:\Users\KasuganoSora\Desktop\技术文档\技术报告_基于FPGA的低成本声学追踪系统.docx`：
  18 页技术报告，严格按 `参考.docx` 的章节与字数上限。
- 早期整工程版 skill 已被上述功能包取代；原始提炼数据仍在 `skill_distill/`。

## 重新挖掘

```bash
cd D:/vivado_project/FPGA/GCC-PHAT_2
C:/Users/KasuganoSora/.conda/envs/rknn_opset19/python.exe skill_distill/mine.py
C:/Users/KasuganoSora/.conda/envs/rknn_opset19/python.exe skill_distill/compact.py
```

`compact.py` 从 `user_messages.txt` 生成 `user_compact.txt`：丢弃压缩续写摘要、图片占位、
IDE/命令注入与中断标记，同会话同分钟去重，每条截断 870 字符。

若会话路径变化，改 `mine.py` 顶部的 `ROOTS` 通配（当前匹配 `*GCC*` / `*gcc*` 项目目录）；
不要直接读 100 MB+ 的 JSONL。
