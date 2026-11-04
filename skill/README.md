# gcc-phat-4ch Skill 包

面向四元麦克风阵列 **四路并行 256 点 FFT 预处理链** 的可复用技能包：
4 路同步帧 → Hanning 加窗 → 四颗 xfft_256 同拍变换 → 129 点单边谱突发 → 学习型频点掩码。

包内只有**一个功能**的知识、流程与验证资产，不覆盖整个工程。工程基线：
`C:\Users\KasuganoSora\Desktop\GCC-PHAT`（26 个设计 `.sv`，只读，不在其中改动），
XC7A100T-FGG484-2，Vivado 2018.3，单时钟 50 MHz。

## 1. 目录

```
skill/
├── README.md                        本文件（不随安装副本复制）
└── gcc-phat-4ch/
    ├── SKILL.md                     入口：边界、十条铁律、5 步流程、排障
    ├── references/
    │   ├── 01_algorithm.md          加窗、FFT 配置、129 点谱、掩码与定点
    │   ├── 02_integration.md        端口、六状态时序、顶层接线、资源
    │   ├── 03_verification.md       TB 判据、命令与证据清单
    │   └── 04_pitfalls.md           14 类历史故障
    ├── scripts/
    │   ├── install.sh               安装到 <工程>/.claude/skills/
    │   ├── check.sh                 结构 / frontmatter / 引用 / 凭据自检
    │   └── run_tb_fft.sh            任意工程树上复跑四路 FFT 定向 TB
    ├── assets/
    │   └── tb_fft_chain.sv          可直接复跑的定向 TB
    └── verification/
        ├── clean_deploy_2026-11-04.log
        └── tb_fft_chain_2026-11-04.log
```

共 11 个文件；其中可安装载荷 9 个（不含两份验证日志）。

## 2. 安装与使用

```bash
cd D:/vivado_project/FPGA/GCC-PHAT_2

sh skill/gcc-phat-4ch/scripts/install.sh                  # 安装到本工程
sh skill/gcc-phat-4ch/scripts/install.sh /path/to/proj    # 安装到其他工程树
sh skill/gcc-phat-4ch/scripts/check.sh                    # 自检（含 SHA256）
sh skill/gcc-phat-4ch/scripts/run_tb_fft.sh /path/to/GCC-PHAT
```

安装后 Claude Code 会在 `.claude/skills/gcc-phat-4ch/` 发现该 skill。
使用时可直接描述任务（如“改 129 点突发的起始索引”），或显式说“按 gcc-phat-4ch skill 执行”。

## 3. 每一项的说明

### 3.1 `gcc-phat-4ch/SKILL.md`

| 项 | 内容 |
|---|---|
| 适用范围 | 修改/验证 `fft_parallel_scheduler`、`fft_256_wrapper`、`window_oper_24b`、`hanning_rom_24b`、`freq_noise_mask`、`preprocess_top`；排查频谱缺失、相位不一致、帧同步错误、突发丢点 |
| 使用方法 | 作为入口先读；按 §4 五步流程执行；改动前逐条核对 §3 十条铁律 |
| 失效条件 | N≠256、fs≠10 kHz、通道数≠4；缺 `xfft_256`/`xfft_0`；Vivado 大版本或 IP 配置字变化；工程树不是旧版 26 文件基线 |
| 已验证复用 | 2026-11-04 干净目录安装、`check.sh` PASS、`tb_fft_chain` PASS |

### 3.2 `references/01_algorithm.md`

| 项 | 内容 |
|---|---|
| 适用范围 | 理解/修改 Hann 系数、24×24 截位、FFT 配置字与 `SCALE_SCH`、129 点单边谱、掩码学习判据 |
| 使用方法 | 改算法或定点前先读 §2~§4；用 §7 检查表确认要同步的参数 |
| 失效条件 | 改窗型/点数/采样率；`SCALE_SCH` 或 `FFT_IP_LATENCY` 变化；掩码判据 `8E > ΣE` 被改写 |
| 已验证复用 | 2026-11-04 对照 `project_global_pkg.sv`、`hanning_256.mem` 与 RTL 逐项核对 |

### 3.3 `references/02_integration.md`

| 项 | 内容 |
|---|---|
| 适用范围 | 端口方向/位宽、六状态机、上游帧缓存与下游测向分包的契约、资源预算 |
| 使用方法 | 新增/改端口时对照 §1；集成后按 §6 检查接线；构建前确认 IP 与时钟 |
| 失效条件 | 端口改名/改位宽；帧率或时钟改变；把并行 FFT 改为时分复用；`frame_sync_err` 语义变化 |
| 已验证复用 | 端口表与 `top_gcc_phat.sv`、`preprocess_top.sv` 实例逐项对照；TB 按同一连接跑通 |

### 3.4 `references/03_verification.md`

| 项 | 内容 |
|---|---|
| 适用范围 | `tb_fft_chain` 的激励、断言、容差、运行命令、通过日志样例与覆盖边界 |
| 使用方法 | 按 §2 执行；按 §3 核对 PASS 与相位检查次数；按 §4 判断结论能覆盖到哪 |
| 失效条件 | TB 文件名/断言改变；Vivado 仿真器选项变化；`xfft_256` 网表路径变化 |
| 已验证复用 | 2026-11-04 干净目录 `PASS`，相位检查 2 次、失败 0 次 |

### 3.5 `references/04_pitfalls.md`

| 项 | 内容 |
|---|---|
| 适用范围 | 14 类故障：综合端口位宽、IP 端口名、数组维度、`$signed`、截位、缩放、`.mem` 路径、配置字、帧边界、`m_last`、无反压、帧号、AGC 重学、timescale |
| 使用方法 | 按症状检索；先看“根因”，再执行“处理”；处理后用 `tb_fft_chain` 复验 |
| 失效条件 | Vivado/IP 升级后配置字或端口变化；改用其他点数/窗型；对应 RTL 已重构 |
| 已验证复用 | 第 7、8、10、14 条在本机复跑中实际遇到或逐条核对 |

### 3.6 `scripts/install.sh`

| 项 | 内容 |
|---|---|
| 适用范围 | 把 `SKILL.md`、`references/`、`scripts/`、`assets/` 复制到目标工程的 `.claude/skills/gcc-phat-4ch/` |
| 使用方法 | `sh install.sh [目标工程根]`；默认当前目录；逐文件 `cmp` 校验后输出 `installed copy: OK` |
| 失效条件 | 目标只读；目标不是 Claude Code 工程（仍可复制，但不会被发现）；包内文件名改变 |
| 已验证复用 | 2026-11-04 `mktemp -d` 空目录与本工程各安装一次 |

### 3.7 `scripts/check.sh`

| 项 | 内容 |
|---|---|
| 适用范围 | 11 个文件是否齐全、frontmatter、四份引用、聊天/抓包残留、凭据模式、9 个载荷 SHA256 |
| 使用方法 | `sh check.sh`；退出码 0 为通过，输出 `PACKAGE CHECK PASSED` |
| 失效条件 | 包结构改名；新增文件未登记；日志文件名变化 |
| 已验证复用 | 2026-11-04 空目录与本工程两次 PASS |

### 3.8 `scripts/run_tb_fft.sh` 与 `assets/tb_fft_chain.sv`

| 项 | 内容 |
|---|---|
| 适用范围 | 在只有工程源码与 IP 网表的目录里，复跑四路加窗 + 并行 FFT + 129 点突发 |
| 使用方法 | `sh run_tb_fft.sh [工程根] [日志路径]`；脚本在 `mktemp -d` 中 staging，不污染工程 |
| 失效条件 | 工程缺 `GCC-PHAT.srcs/src` 或 `xfft_256/sim/xfft_256.vhd`；Vivado 不在默认路径且未设 `VIVADO_BIN`；点数/端口改变而 TB 未同步 |
| 已验证复用 | 2026-11-04 干净目录 3 帧仿真，`PASS`，耗时约 29 s |

## 4. 已验证的复用结果

### 4.1 干净环境部署（2026-11-04）

在空临时目录中只复制 `skill/gcc-phat-4ch/`（无工程文件、无 `.claude/`）：

```bash
TMP=$(mktemp -d)
cp -r skill/gcc-phat-4ch "$TMP/pkg"
sh "$TMP/pkg/scripts/install.sh" "$TMP"
sh "$TMP/pkg/scripts/check.sh"
```

结果：`installed copy: OK`、`PACKAGE CHECK PASSED`、9 个载荷 SHA256
（见 `verification/clean_deploy_2026-11-04.log`）。

### 4.2 本工程安装

```bash
sh skill/gcc-phat-4ch/scripts/install.sh .
sh skill/gcc-phat-4ch/scripts/check.sh
```

结果：安装到 `.claude/skills/gcc-phat-4ch/`，`check.sh` 通过，9 个载荷文件与源包逐文件一致。

### 4.3 干净目录定向仿真

只保留 8 个 RTL/参数文件、`tb_fft_chain.sv`、`xfft_256.vhd`、`hanning_256.mem`：

```
=== 3帧完成：通道相位检验 2 次，失败 0 次 ===
PASS
```

日志见 `verification/tb_fft_chain_2026-11-04.log`。

### 4.4 未验证 / 限制

- **未做跨设备验证**：没有在另一台电脑、另一 Vivado 版本、另一块板卡上复跑。
- **未覆盖下游**：六对互谱、1024 点 IFFT、128 帧投票、串口与云台不在本包范围内，`tb_fft_chain` 也不断言它们。
- **掩码学习未单独断言**：TB 只跑 3 帧，`freq_valid_mask` 的正确性靠整机联调，不靠本 TB。
- 完整跑 TB 需要工程源码、Vivado 与 `.mem` 激励；包本身是文档、脚本与一个 TB，不是可独立运行的二进制。
- 包内不含聊天原文、API key、串口抓包；`verification/` 只有两次运行的文本日志。

## 5. 失效条件汇总

出现任一条，本包的结论需要重新验证：

1. FFT 点数 N ≠ 256，或采样率 ≠ 10 kHz。
2. 通道数 ≠ 4，或帧缓存不再输出 129 点单边谱。
3. 工程没有 `xfft_256` / `xfft_0`，或 IP 升级改变配置字、`m_last` 行为。
4. 阵列几何改变而 `COS_K`/`TAU_MAX` 未同步。
5. Vivado 大版本变化，仿真器选项或 IP 仿真库路径改变。
6. 工程树不是旧版 26 文件基线。
