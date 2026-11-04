# 02 顶层集成与端口时序

## 1. `preprocess_top` 端口

| 方向 | 端口 | 位宽 | 说明 |
|---|---|---|---|
| in | `clk_50m` | 1 | 唯一时钟，50 MHz |
| in | `rst_n` | 1 | 异步复位、同步释放，低有效 |
| in | `mic_ch_data[0:3]` | 24 | 当前帧四路样本（写侧直通） |
| in | `mic_frame_cnt` | 16 | 帧号，来自 `mic_collect_top` |
| in | `mic_frame_valid` | 1 | 单拍，帧写满 256 点时给出 |
| out | `rd_en` | 1 | 回读帧缓存的读使能 |
| out | `rd_addr` | 10 | 0~255 循环，四路共用 |
| in | `rd_data[0:3]` | 24 | 读数据，`rd_en` 后第 2 拍有效 |
| in | `agc_gain_changed` | 1 | 换挡脉冲，清空掩码学习 |
| out | `fft_idx` | 10 | 突发频点 0~128 |
| out | `fft_re[0:3]` / `fft_im[0:3]` | 24 | 四路频域数据，与 `fft_idx` 同拍 |
| out | `freq_valid_mask` | 129 | 频点有效位 |
| out | `pre_frame_cnt` | 16 | 本帧帧号，锁存于突发首点 |
| out | `pre_frame_valid` | 1 | 突发期间连续 129 拍 |
| out | `mask_learn_done` | 1 | 3 帧学习完成 |
| out | `fft_ip_latency` | 10 | 常量 310，供下游对齐 |

## 2. 内部连接

```
preprocess_top
├─ hanning_rom_24b        addr = rd_addr, dout = win_coef
├─ fft_parallel_scheduler
│   ├─ rd_en/rd_addr → 帧缓存
│   ├─ rd_data[ch] + win_coef → window_oper_24b[ch] → fft_256_wrapper[ch] → xfft_256
│   ├─ freq_bram[ch] 129×48 bit
│   └─ fft_idx/fft_re/fft_im/pre_frame_* → 测向分包
└─ freq_noise_mask        valid/idx/re/im = pre_frame_* 与 ch0，frame_end 由 S_BURST 末尾给出
```

`freq_noise_mask` 只吃 ch0 的频域数据，四路共用一张掩码；这是“声源在阵列附近、四只麦克风看到同一频段”的工程假设。若某只麦克风故障，掩码不会单独反映。

## 3. 六状态机

| 状态 | 停留 | 退出条件 | 关键动作 |
|---|---|---|---|
| `S_IDLE` | 帧间 | `mic_frame_valid` | 锁存 `mic_frame_cnt`，清 `ch_done` |
| `S_CFG_WAIT` | 8 拍 | `cfg_wait_cnt >= 7` | 等待 IP 配置生效 |
| `S_CFG_GAP` | 4 拍 | `gap_cnt == 3` | 配置与数据隔离 |
| `S_DATA` | 256 拍 | `in_cnt == 255 && rd_advance` | `rd_en` 拉高，四路同址读 |
| `S_OUT_WAIT` | 变动 | `&ch_done` | 等四路 `m_valid && m_last` |
| `S_BURST` | 130 拍 | `out_cnt == 129` | 每拍推一个频点，`out_cnt` 0~128 |

`rd_advance = (state == S_DATA) && (&fft_s_ready)`：只有四颗 IP 同时能收，才推进读地址。任何一颗 `s_ready` 掉低，整条链停住而不是丢点。

`pre_frame_valid` 由 `out_cnt_d <= 128` 产生，`out_cnt_d` 是 `out_cnt` 打一拍，正好抵消 `freq_bram` 的读延迟；因此外部看到的 `fft_idx=0` 与 `fft_re/im` 是同一频点，不会错位。

## 4. 与相邻分包的契约

### 4.1 上游 `mic_collect_top`

- `mic_ch_data` 在 `mic_frame_valid` 当拍仍有效，但本链不使用它，只用帧缓存回读。
- 帧缓存是乒乓结构：写侧写 B 区时读侧读 A 区，因此 `rd_en` 必须在本帧 `frame_valid` 之后、下一帧写满之前完成 256 次读。
- `mic_frame_valid` 与 `mic_frame_cnt` 必须同拍更新；帧号回绕按 0xFFFF→0 处理。

### 4.2 下游 `phat_position_top`

- 以 `pre_frame_valid` 为唯一帧同步；在该脉冲内按 `fft_idx` 0~128 逐拍取数。
- `fft_idx` 在突发外保持最后值，不得用作“新数据”标志。
- `mask_learn_done=0` 时下游应跳过本帧投票（前 3 帧没有有效掩码）。
- `fft_ip_latency=310` 是常量，用于下游估算流水深度，不参与数据路径。

## 5. 资源与时序

| 项 | 旧版实现值 | 说明 |
|---|---|---|
| DSP48E1 | 4×24 = 96 | 四颗 256 点 FFT |
| BRAM36 | 约 1（Hann ROM）+ 4×129×48 bit 频域缓存 | 频域缓存可推断为分布式或块 RAM |
| LUT / FF | 见工程 `util_impl.rpt` | 本链约占全设计的一半 |
| WNS / WHS | +2.770 ns / +0.015 ns | 50 MHz，Vivado 2018.3 |
| DRC | 0 Error | `drc.rpt` |

`hanning_rom_24b` 与 `freq_bram` 都带 `ram_style` 属性；改综合策略或换器件时先确认它们仍被推断为 BRAM，否则 Slice 会明显上升。

## 6. 集成检查清单

1. `preprocess_top` 的 `rd_*` 与 `mic_collect_top` 的 `mic_rd_*` 一一对应，没有多驱动。
2. `fft_re/fft_im` 的数组下标顺序与 `mic_coord_x/y` 的通道定义一致（ch0=−x，ch1=+x，ch2=−y，ch3=+y）。
3. `freq_valid_mask` 位宽 129，接到下游时不要截成 128。
4. `mask_learn_done` 与 `agc_gain_changed` 同源同拍，换挡后至少 3 帧不投票。
5. `pre_frame_cnt` 与 `mic_frame_cnt` 的位宽一致（16 bit），跨分包不截位。
6. 顶层没有把 `frame_sync_err` 悬空后又假设“坏帧已丢”——旧版只置标志，不丢帧。
