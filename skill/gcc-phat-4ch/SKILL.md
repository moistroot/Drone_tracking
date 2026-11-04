---
name: gcc-phat-4ch
description: 四元麦克风阵列的四路并行 256 点 FFT 预处理链开发与验证技能——4 路同步帧、Hanning 加窗、四颗 xfft_256 同拍变换、129 点单边谱突发与学习型频点掩码。用于改动 mic_frame_buffer 读口、hanning_rom_24b / window_oper_24b、fft_256_wrapper、fft_parallel_scheduler、freq_noise_mask 或 preprocess_top，或排查 129 点频谱缺失、四通道相位不一致、帧同步错误、突发丢点与资源时序问题。
---

# 四路并行 GCC-PHAT 预处理链 Skill

本 skill **只覆盖一个功能**：把四路 24 bit / 10 kHz 麦克风帧变成四路对齐的 129 点单边频谱。
不覆盖 I2S 接收、高通与 AGC 内部算法，也不覆盖六对互谱、IFFT、投票、串口与云台；这些只作为接口与验收约束出现。

工程基线：`C:\Users\KasuganoSora\Desktop\GCC-PHAT`（只读基线，不在其中改动），XC7A100T-FGG484-2，Vivado 2018.3，单时钟 50 MHz。
可复用的验证资产针对该基线的 26 个设计 `.sv` 中的预处理子集。

## 1. 功能边界

| 属于本 skill | 不属于本 skill |
|---|---|
| `mic_frame_buffer` 的读口时序（4 路同址、1 拍延迟） | `mic_serial_rx` 的 I2S 移位与声道选择 |
| `hanning_rom_24b` / `window_oper_24b`：256 点系数与 24×24 乘加 | `highpass_900` / `agc_shared_24b` 的内部算法 |
| `fft_256_wrapper`：xfft_256 配置字、SCALE_SCH、AXI-S 时序 | `complex_mult_24b` / `cordic_phat_norm_24b` 的 PHAT 归一化 |
| `fft_parallel_scheduler`：六状态机、8+4 拍配置、256 拍并行输入、129 点突发 | `pair_scheduler` / `ifft_peak_detect` |
| `freq_noise_mask`：3 帧噪声学习、频段门、`learn_done` | `tdoa_solver` 的 128 帧投票与角度解算 |
| `preprocess_top` 的端口与顶层接线 | `uart_tx_module` / `seg8_hc595` / `servo_pan_tilt` |
| `tb_fft_chain` 与 `run_tb_fft.sh` | 整机联调、机械与声学标定 |

## 2. 数据流（精确到端口）

```
mic_ch_data[0:3] / mic_frame_valid / mic_frame_cnt      ← 来自 mic_collect_top
        │
        ▼
hanning_rom_24b          256×24 系数 ROM，地址 = rd_addr（四路共用）
        │  win_coef（1 拍延迟）
        ▼
fft_parallel_scheduler   S_IDLE → S_CFG_WAIT(8) → S_CFG_GAP(4)
        │                → S_DATA(256 拍，四路同址读) → S_OUT_WAIT(等 4×m_last)
        │                → S_BURST(129 拍)
        │  rd_en / rd_addr / rd_data[0:3]
        ▼
4 × fft_256_wrapper      xfft_256，Pipelined Streaming，24 bit in/out
        │  fft_re[0:3] / fft_im[0:3]（每拍 1 个频点）
        ▼
fft_idx / pre_frame_valid / pre_frame_cnt  →  phat_position_top（六对互谱）
        │
        └─ mask_valid / mask_idx / mask_re / mask_im → freq_noise_mask
                                                        → freq_valid_mask[128:0]
                                                        → mask_learn_done
```

时序量级（50 MHz，一帧 = 25.6 ms = 1,280,000 拍）：

| 事件 | 拍数 | 时间 |
|---|---|---|
| 帧有效到配置 | 8 | 160 ns |
| 配置间隔 | 4 | 80 ns |
| 四路并行输入 256 点 | 256 | 5.12 µs |
| IP 流水（首点进→首点出） | ≈310 | 6.2 µs |
| 129 点突发 | 129 | 2.58 µs |
| 本链合计 | ≈707 | ≈14.1 µs（帧预算 0.055%） |

## 3. 铁律（改动前逐条确认）

1. **四路共用一套系数与一个读地址**。`rd_addr` 同时广播到四路 `rd_data`，任何按通道加偏移的读法都会破坏通道间相位关系；`tb_fft_chain` 的 X1=jX0、X2=−X0、X3=−jX0 就是查这个。
2. **`SCALE_SCH = 8'b1010_1011` 是幅度契约**。四段缩放为 >>2、>>2、>>2、>>3，总计 >>9；改它必须同步下游 PHAT 门限与所有定点期望值。
3. **配置字位域固定**：`{7'b0, SCALE_SCH, fwd_inv=1, 4'b0, NFFT=8}`。`NFFT` 字段是 log2 点数；256 点写 8，写成 256 会被 IP 截断成 0。
4. **配置只在帧边界发**。`cfg_valid` 由 `mic_frame_valid` 触发，`cfg_pending` 保持到 `s_axis_config_tready`；帧中间重配会让本帧输出错位。
5. **配置后必须等 8 拍再送数、再隔 4 拍**。这是 IP 的配置生效窗口；缩短会出现首点被吞或 `m_last` 缺失。
6. **只发 0~128 共 129 个频点，且无反压**。`S_BURST` 每拍推一个频点，下游必须同速消费；加 `tready` 会改变协议，必须同时改 `pair_scheduler`。
7. **突发必须等四路 `m_last` 全部到齐**。只等单通道会截断慢通道；`ch_done` 四位与 `&fft_m_last` 是判据。
8. **帧号是唯一的帧身份**。`mic_frame_cnt` 与内部 `locked_frame_cnt` 不一致时置 `frame_sync_err` 并丢弃该帧，不允许“修补”或跨帧拼接。
9. **AGC 换挡必须让掩码重新学习**。`agc_gain_changed` 清空 `freq_noise_mask` 的累加器并拉低 `learn_done`；否则旧噪声底会把新增益下的频点全判为无效。
10. **证据优先**：`tb_fft_chain` 输出 PASS（相位检查≥2、失败=0）、综合无 8-3848/8-6104、WNS/WHS>0、DRC 0 Error，缺一不算完成。

## 4. 标准改动流程（5 步）

### 第 1 步：确认基线与参数源
- 确认工程根目录含 `GCC-PHAT.srcs/src/project_global_pkg.sv` 与 `project_global_param.vh`。
- 读 `CUR_SAMPLE_FREQ=10000`、`CUR_FFT_POINT=256`、`FFT_BINS=129`、`FFT_IP_LATENCY=310`、`MASK_BIN_LO/HI=26/127`。
- 确认 `xfft_256` 的仿真网表存在：`GCC-PHAT.srcs/sources_1/ip/xfft_256/sim/xfft_256.vhd`。
- 确认 Vivado 未占用工程（GUI 打开 `.xpr` 时 batch 构建会等待）。

### 第 2 步：先改定点模型，再改 RTL
- 用 Python 复现 `>>23` 截位、24 bit 饱和与 `SCALE_SCH` 的总移位，先算清 1 kHz 单音在 bin 25/26 的幅度与相位。
- 任何“看起来只是改位宽”的改动，都要先在模型里跑一遍满量程与 −60 dB 小信号，确认不会饱和或截成零。

### 第 3 步：改 RTL 并保持接口
- 改 `fft_parallel_scheduler` 时同步检查 `S_CFG_WAIT`/`S_CFG_GAP` 计数、`rd_advance = (state==S_DATA) && (&fft_s_ready)`、`ch_done` 与 `out_cnt`。
- 改 `fft_256_wrapper` 时同步检查 `CFG_WORD` 位域与 `cfg_pending` 握手。
- 改 `freq_noise_mask` 时同步检查 `restart`、`learn_cnt` 与频段门 `MASK_BIN_LO/HI`。

### 第 4 步：定向仿真
```bash
sh scripts/run_tb_fft.sh /path/to/GCC-PHAT
```
判据只有一条：日志末尾出现 `PASS`，且 `通道相位检验 N 次，失败 0 次`。出现 `TIMEOUT`、`FAIL` 或相位失败，先查第 5 节。

### 第 5 步：集成与构建
- 在 `preprocess_top` 中确认 `hanning_rom_24b`、`fft_parallel_scheduler`、`freq_noise_mask` 三者端口一一对应。
- 在 `top_gcc_phat` 中确认 `fft_idx/fft_re/fft_im/pre_frame_valid/pre_frame_cnt` 接到 `phat_position_top`，`freq_valid_mask/mask_learn_done` 接到测向分包。
- 构建后检查：WNS/WHS>0、DRC 0 Error、DSP 增量约 +96（四颗 256 点 FFT）、BRAM 增量约 +1（Hann ROM）。
- 最后更新工程文档中的资源、时序与 TB 结果，不把“仿真通过”写成“实测精度”。

## 5. 排障速查

| 症状 | 先看 | 常见根因 |
|---|---|---|
| 无 `pre_frame_valid` | `state`、`mic_frame_valid` | 帧有效脉冲太窄；`rst_n` 未释放；帧号不连续触发 `frame_sync_err` |
| 有突发但频点全零 | `rd_en`、`rd_data`、`hanning_256.mem` | `$readmemh` 路径相对 xsim 运行目录，找不到时 ROM 全零 |
| 只有 128 点或 129 点错位 | `out_cnt`、`S_BURST` | 把 `POINTS_HALF` 当上界；用 `m_idx` 而不是 `out_cnt` 做门控 |
| 四通道相位不一致 | `rd_addr`、通道映射 | 读地址按通道偏移；`fft_re/im` 数组下标接反 |
| 幅度只有一半或饱和 | `SCALE_SCH`、`window_oper` 截位 | 缩放段写错；`prod[46:23]` 改成 `[47:24]` |
| 突发中途停住 | `ch_done`、`&fft_m_last` | IP 的 `m_last` 丢失；配置未在帧边界生效 |
| 掩码全 0 | `learn_done`、`restart` | AGC 换挡后未重学；学习期不足 3 帧；频段门写反 |
| 综合报 8-3848 | 端口位宽 | 改端口后只改了实例一侧；数组端口维度不一致 |
| 综合报 8-6104 | IP 端口名 | 手写 IP 实例与 IP 版本不匹配，xsim 容忍而综合不容忍 |
| xelab 报 43-4100 | timescale | VHDL 网表无 timescale；`xelab --timescale 1ns/1ps` |

## 6. 失效条件（出现任一条，本 skill 的结论不再成立）

- FFT 点数 N≠256，或采样率≠10 kHz（频点间隔、帧长、掩码频段全部改变）。
- 通道数≠4，或帧缓存不再输出 129 点单边谱。
- 工程没有 `xfft_256` 或 `xfft_0` 两颗 IP，或 IP 版本升级导致配置字位域、`m_last` 行为变化。
- 阵列几何改变而 `COS_K`/`TAU_MAX` 未同步（本链不含角度，但下游验收会失配）。
- Vivado 大版本变化，`xvlog/xelab/xsim` 选项或 IP 仿真库路径改变。
- 工程树不是旧版 26 文件基线（其他分支可能已改为时分复用，端口完全不同）。

## 7. 参考

- `references/01_algorithm.md` — 加窗、FFT 配置、129 点单边谱、噪声掩码与定点推导。
- `references/02_integration.md` — 端口、六状态时序、顶层接线与资源。
- `references/03_verification.md` — `tb_fft_chain` 判据、`run_tb_fft.sh` 用法与证据清单。
- `references/04_pitfalls.md` — 14 类历史故障与定位方法。
- 包说明、安装方法与复用边界：`skill/README.md`（不随安装副本复制）。
