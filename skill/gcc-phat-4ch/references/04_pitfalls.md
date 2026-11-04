# 04 历史故障目录（14 类）

每条都按“症状 → 根因 → 处理”记录；前 3 条来自综合/实现，后 11 条来自仿真与整机联调。

## 1. 综合报 8-3848：端口位宽不匹配

**症状**：xsim 能过，综合报 `[Synth 8-3848] Net ... does not match port width`。
**根因**：改了 `fft_idx` 或 `fft_re/im` 的位宽，只改了一侧实例；或 `FFT_BINS` 从 128 改成 129 后下游仍按 128 接。
**处理**：以 `project_global_pkg.sv` 为唯一参数源，改完后 grep 所有实例端口；数组端口连 `[0:3]` 时逐项核对维度。

## 2. 综合报 8-6104：IP 端口名不匹配

**症状**：手写的 `xfft_256` 实例在 xsim 下能跑，综合报端口不存在或未连接。
**根因**：IP 版本升级后端口改名（如 `m_axis_data_tuser` 位宽变化），或把 `event_tlast_unexpected` 之类可选端口漏接。
**处理**：用 IP 的 `_stub.v`/实例模板重新生成实例，不手抄旧代码；未用事件端口显式悬空。

## 3. 数组端口维度不一致

**症状**：`rd_data[0:3]` 在 `preprocess_top` 与 `mic_frame_buffer` 之间位宽或打包方式不同，仿真出现 X。
**根因**：SystemVerilog 的 unpacked array 端口必须两侧完全一致；把 `[3:0]` 写成 `[0:3]` 不会报错，但元素顺序反了。
**处理**：统一用 `[0:MAX_CHAN_NUM-1]`，并在顶层按通道号注释坐标定义。

## 4. 截位没加 `$signed`

**症状**：小信号正常，大信号相位偶尔翻 180°，PHAT 时延跳变。
**根因**：24 bit 有符号数在拼接或比较时被当成无符号，负半周被解释成大正数。
**处理**：所有进入乘加、比较、饱和判断的 24 bit 数都显式 `$signed()`；改完用满量程负值激励复跑。

## 5. `window_oper` 截位改错

**症状**：频谱幅度只有预期的一半，或窗中心削顶。
**根因**：把 `prod[46:23]` 写成 `prod[47:24]`（多移一位，等于除以 2），或漏掉 24 bit 饱和。
**处理**：系数按 2^23 定标，右移 23 位；饱和到 `0x7FFFFF/0x800000`。用 TB 的 8,000,000 LSB 正弦核对 bin 26 幅度。

## 6. `SCALE_SCH` 与下游门限不匹配

**症状**：换缩放方案后，掩码把大部分频点判为无效，或 PHAT 归一化前就饱和。
**根因**：`SCALE_SCH` 从 `1010_1011`（总 >>9）改成其他值，幅度整体缩放，掩码的 `8E > ΣE` 判据与 PHAT 门限没跟着改。
**处理**：缩放方案是接口契约，改它必须同步 TB 容差、掩码门限、下游 PHAT 门。

## 7. `$readmemh` 找不到文件

**症状**：仿真能跑，但频谱全零；`hanning_rom_24b` 输出恒为 0。
**根因**：`$readmemh("hanning_256.mem", rom)` 的相对路径是 **xsim 运行目录**，不是源文件目录。
**处理**：把 `.mem` 复制到运行目录，或用 `run_tb_fft.sh` 统一 staging；日志里出现 `$readmemh: File not found` 时先查这里。

## 8. 配置字 `NFFT` 字段写错

**症状**：IP 输出全零或只输出前几个点。
**根因**：`NFFT` 字段是 log2 点数，256 点应写 8；写成 256 会被截成 0。
**处理**：保持 `{7'b0, SCALE_SCH, 1'b1, 4'b0, 4'd8}`，不要按“点数”直觉填。

## 9. 配置不在帧边界

**症状**：第一帧错位，后续帧正常；或偶发 `m_last` 缺失。
**根因**：`cfg_valid` 在 `S_DATA` 或 `S_BURST` 期间拉高，IP 在帧中间重配。
**处理**：`cfg_valid` 只在 `S_IDLE && mic_frame_valid` 产生，`cfg_pending` 保持到 `config_tready`。

## 10. 只等单通道 `m_last`

**症状**：突发提前开始，慢通道的后几个频点是上一帧的。
**根因**：`S_OUT_WAIT` 用 `|ch_done` 而不是 `&ch_done`。
**处理**：四位 `ch_done` 全部置位才进 `S_BURST`；`ch_done` 在每帧 `S_IDLE` 清零。

## 11. 突发无反压导致丢点

**症状**：下游偶尔少收 1~2 个频点，`fft_idx` 不连续。
**根因**：`S_BURST` 每拍推一个频点，没有 `tready`；下游若插入等待，数据直接丢。
**处理**：下游必须同速消费；确需反压时，同时改 `fft_parallel_scheduler` 与 `pair_scheduler` 的协议，不能只在一侧加 `ready`。

## 12. 把 `frame_sync_err` 当成“已丢帧”

**症状**：帧号跳变后，下游仍收到该帧数据，投票出现坏点。
**根因**：旧版 `frame_sync_err` 只置标志，**不丢帧**；`preprocess_top` 中该信号甚至没有引出使用。
**处理**：要么在下游显式用该标志跳过本帧，要么接受坏帧并在投票层剔除；不能假设 RTL 已经处理。

## 13. AGC 换挡后掩码没重学

**症状**：远距离目标突然“消失”，`freq_valid_mask` 全 0。
**根因**：`agc_gain_changed` 没接到 `freq_noise_mask.restart`，旧噪声底按新增益比较，所有频点都判为无效。
**处理**：`restart` 与 `agc_gain_changed` 同源；换挡后至少 3 帧 `learn_done=0`，下游暂停投票。

## 14. xelab 报 43-4100（timescale）

**症状**：`Module tb_fft_chain has a timescale but at least one module in design doesn't have timescale.`
**根因**：`xfft_256.vhd` 是 VHDL 网表，没有 Verilog timescale。
**处理**：`xelab ... --timescale 1ns/1ps`；不要删 TB 的 `` `timescale ``，否则延时单位会变成默认值。
