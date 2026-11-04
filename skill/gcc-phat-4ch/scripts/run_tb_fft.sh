#!/bin/sh
# run_tb_fft.sh - 在任意工程树上复跑四路并行 FFT 定向 TB
# 用法: sh run_tb_fft.sh [工程根目录] [日志路径]
#   工程根目录默认当前目录，需包含 GCC-PHAT.srcs/src 与 xfft_256 的 sim 网表
set -eu

PKG_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PROJ=${1:-$PWD}
LOG=${2:-$PWD/tb_fft_chain_$(date +%Y-%m-%d).log}
VIVADO_BIN=${VIVADO_BIN:-D:/vivado/Vivado/2018.3/bin}

SRC="$PROJ/GCC-PHAT.srcs/src"
IP="$PROJ/GCC-PHAT.srcs/sources_1/ip/xfft_256/sim/xfft_256.vhd"

for f in project_global_pkg.sv project_global_param.vh hanning_rom_24b.sv \
         window_oper_24b.sv fft_256_wrapper.sv fft_parallel_scheduler.sv \
         freq_noise_mask.sv preprocess_top.sv hanning_256.mem; do
    if [ ! -f "$SRC/$f" ]; then
        echo "ERROR: missing $SRC/$f" >&2
        exit 1
    fi
done
if [ ! -f "$IP" ]; then
    echo "ERROR: missing xfft_256 sim netlist: $IP" >&2
    exit 1
fi
for tool in xvlog xvhdl xelab xsim; do
    if [ ! -x "$VIVADO_BIN/$tool" ] && [ ! -x "$VIVADO_BIN/$tool.bat" ]; then
        echo "ERROR: $tool not found under $VIVADO_BIN" >&2
        exit 1
    fi
done

WORK=$(mktemp -d "${TMPDIR:-/tmp}/gcc-phat-4ch.XXXXXX")
trap 'rm -rf "$WORK"' EXIT INT TERM

cp "$SRC/project_global_pkg.sv" "$SRC/project_global_param.vh" \
   "$SRC/hanning_rom_24b.sv" "$SRC/window_oper_24b.sv" \
   "$SRC/fft_256_wrapper.sv" "$SRC/fft_parallel_scheduler.sv" \
   "$SRC/freq_noise_mask.sv" "$SRC/preprocess_top.sv" \
   "$SRC/hanning_256.mem" "$WORK/"
cp "$PKG_DIR/assets/tb_fft_chain.sv" "$WORK/"
cp "$IP" "$WORK/"

mkdir -p "$(dirname "$LOG")"
: > "$LOG"

cd "$WORK"
echo "== xvlog ==" | tee -a "$LOG"
"$VIVADO_BIN/xvlog" --sv project_global_pkg.sv hanning_rom_24b.sv \
    window_oper_24b.sv fft_256_wrapper.sv fft_parallel_scheduler.sv \
    freq_noise_mask.sv preprocess_top.sv tb_fft_chain.sv 2>&1 | tee -a "$LOG"

echo "== xvhdl ==" | tee -a "$LOG"
"$VIVADO_BIN/xvhdl" xfft_256.vhd 2>&1 | tee -a "$LOG"

echo "== xelab ==" | tee -a "$LOG"
"$VIVADO_BIN/xelab" tb_fft_chain -s sim_fft -L unisims_ver \
    --timescale 1ns/1ps 2>&1 | tee -a "$LOG"

echo "== xsim ==" | tee -a "$LOG"
"$VIVADO_BIN/xsim" sim_fft -R 2>&1 | tee -a "$LOG"

if grep -q '^PASS$' "$LOG"; then
    echo "RESULT: PASS"
    echo "log: $LOG"
else
    echo "RESULT: FAIL" >&2
    echo "log: $LOG" >&2
    exit 1
fi
