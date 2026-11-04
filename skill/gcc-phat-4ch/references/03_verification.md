# 03 验证判据与证据

## 1. `tb_fft_chain` 验什么

| 项 | 设置 |
|---|---|
| 激励 | 四路 1 kHz 正弦，10 kHz 采样，幅度 8,000,000 LSB |
| 通道初相 | ch0 0°、ch1 90°、ch2 180°、ch3 270°（sin / cos / −sin / −cos） |
| 帧数 | 连续 3 帧，每帧 256 点 |
| 帧缓存模型 | 同步读、1 拍延迟，四路同址 |
| 断言点 | `fft_idx == 26`、`pre_frame_cnt >= 1` |
| 断言式 | `X1 = jX0`、`X2 = −X0`、`X3 = −jX0` |
| 容差 | `|X0| 的 2% + 2000 LSB`（覆盖 FFT 各级截断与非整频点泄漏） |
| 合格判据 | `ph_checks >= 2` 且 `ph_errs == 0`，打印 `PASS` |

1 kHz 对应 25.6 个周期，能量落在 bin 25/26；bin 26 的相位关系对四颗 IP 的通道映射、配置生效时刻与 `m_last` 对齐最敏感，因此选它做断言点。

## 2. 怎么跑

### 2.1 一键脚本

```bash
sh scripts/run_tb_fft.sh /path/to/GCC-PHAT
```

脚本做四件事：

1. 在 `mktemp -d` 建空工作目录（不污染工程）。
2. 从工程 `GCC-PHAT.srcs/src` 复制 8 个文件：`project_global_pkg.sv`、`project_global_param.vh`、`hanning_rom_24b.sv`、`window_oper_24b.sv`、`fft_256_wrapper.sv`、`fft_parallel_scheduler.sv`、`freq_noise_mask.sv`、`preprocess_top.sv`；再复制 `hanning_256.mem`。
3. 从包的 `assets/` 复制 `tb_fft_chain.sv`，从工程 IP 目录复制 `xfft_256.vhd`。
4. 依次执行 `xvlog --sv`、`xvhdl`、`xelab --timescale 1ns/1ps -L unisims_ver`、`xsim -R`，把完整输出写到 `verification/tb_fft_chain_<date>.log`，并 grep 最后的 `PASS`。

环境要求：Vivado 2018.3 的 `xvlog/xvhdl/xelab/xsim` 在 `PATH`，或设置 `VIVADO_BIN` 指向 `Vivado/2018.3/bin`。

### 2.2 手工命令（与脚本等价）

```bash
xvlog --sv project_global_pkg.sv hanning_rom_24b.sv window_oper_24b.sv \
      fft_256_wrapper.sv fft_parallel_scheduler.sv freq_noise_mask.sv \
      preprocess_top.sv tb_fft_chain.sv
xvhdl xfft_256.vhd
xelab tb_fft_chain -s sim_fft -L unisims_ver --timescale 1ns/1ps
xsim sim_fft -R
```

`--timescale 1ns/1ps` 不能省：`xfft_256.vhd` 是 VHDL 网表，xelab 会因“TB 有 timescale、设计中有模块没有”报 `XSIM 43-4100`。

## 3. 通过的日志长什么样

```
=== 3帧完成：通道相位检验 2 次，失败 0 次 ===
PASS
$finish called at time : 91390 ns
```

`verification/tb_fft_chain_2026-11-04.log` 还保留了 bin 25/26 的逐通道实/虚部，例如：

```
帧1 idx=26 | ch0 re=-852865 im=-281756 | ch1 re=281749 im=-852868
           | ch2 re=852863 im=281754  | ch3 re=-281750 im=852866
```

可手算验证：`jX0 = 281756 − j852865`，与 ch1 的 `281749 − j852868` 相差 7 LSB；`−X0` 与 ch2、`−jX0` 与 ch3 同样在容差内。这说明四颗 IP 处理的是同一帧、同一频点，且通道顺序没有接反。

## 4. 覆盖边界

| 已覆盖 | 未覆盖 |
|---|---|
| 加窗系数读取与 24×24 截位 | `highpass_900` / AGC 的算法正确性 |
| 四颗 xfft_256 的配置与并行启动 | `freq_noise_mask` 的学习收敛（TB 只跑 3 帧，掩码未单独断言） |
| 129 点突发的索引与帧对齐 | 六对互谱、PHAT、1024 点 IFFT |
| 四通道相位与通道映射 | 128 帧投票、角度解算、串口与云台 |
| 帧缓存读模型的 1 拍延迟 | 真实录音、混响、多声源 |

因此 `tb_fft_chain` 的 PASS 只能证明“预处理链把同一帧正确变换到频域”，不能替代整机联调，也不能写成测向精度。

## 5. 干净目录复现记录

2026-11-04 在一台装有 Vivado 2018.3 的 Windows 机器上，把技能包复制到空目录后：

1. `install.sh` 输出 `installed copy: OK`；
2. `check.sh` 输出 `PACKAGE CHECK PASSED`，列出 9 个载荷文件的 SHA256；
3. 只保留 8 个 RTL/参数文件、`tb_fft_chain.sv`、`xfft_256.vhd`、`hanning_256.mem`，运行 `run_tb_fft.sh`，得到 `PASS`。

三次运行的日志都在 `verification/` 下，未做剪辑。
