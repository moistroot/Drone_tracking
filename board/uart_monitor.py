#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口数据记录到 TXT 文件
用法示例:
    python serial_to_txt.py -p COM3 -b 115200 -o output.txt
    python serial_to_txt.py -p /dev/ttyUSB0 -b 9600 --hex --timestamp
"""

import argparse
import sys
import time
import signal
from datetime import datetime

import serial
import serial.tools.list_ports


def list_ports():
    """列出当前可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到任何串口设备")
        return
    print("可用串口:")
    for p in ports:
        print(f"  {p.device}\t{p.description}")


def build_arg_parser():
    ap = argparse.ArgumentParser(description="读取串口数据并写入 TXT 文件")
    ap.add_argument("-p", "--port", help="串口号，例如 COM3 或 /dev/ttyUSB0")
    ap.add_argument("-b", "--baudrate", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--bytesize", type=int, default=8, choices=[5, 6, 7, 8], help="数据位，默认 8")
    ap.add_argument("--parity", default="N", choices=["N", "E", "O", "M", "S"], help="校验位，默认 N")
    ap.add_argument("--stopbits", type=float, default=1, choices=[1, 1.5, 2], help="停止位，默认 1")
    ap.add_argument("-o", "--output", default="serial_log.txt", help="输出文件路径，默认 serial_log.txt")
    ap.add_argument("--hex", action="store_true", help="以十六进制形式记录")
    ap.add_argument("--timestamp", action="store_true", help="每行数据前加时间戳")
    ap.add_argument("--timeout", type=float, default=1.0, help="串口读超时(秒)，默认 1.0")
    ap.add_argument("--list", action="store_true", help="列出可用串口后退出")
    ap.add_argument("--flush-interval", type=float, default=2.0,
                    help="每隔多少秒强制刷新写入磁盘，默认 2.0 秒")
    return ap


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        print("错误: 必须指定 -p/--port 串口号，或使用 --list 查看可用串口")
        parser.print_help()
        sys.exit(1)

    # 打开串口
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=args.bytesize,
            parity=args.parity,
            stopbits=args.stopbits,
            timeout=args.timeout,
        )
    except serial.SerialException as e:
        print(f"打开串口失败: {e}")
        list_ports()
        sys.exit(1)

    print(f"已打开串口 {args.port} @ {args.baudrate}, 记录到 {args.output}")
    print("按 Ctrl+C 停止记录...")

    stop_flag = {"stop": False}

    def handle_signal(signum, frame):
        stop_flag["stop"] = True
        print("\n收到退出信号，正在关闭...")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    last_flush = time.time()
    total_bytes = 0
    line_count = 0

    try:
        with open(args.output, "a", encoding="utf-8", newline="") as f:
            # 写文件头，方便查看
            f.write(f"===== 记录开始: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} "
                    f"port={args.port} baud={args.baudrate} =====\n")

            while not stop_flag["stop"]:
                try:
                    data = ser.read(ser.in_waiting or 1)
                except serial.SerialException as e:
                    print(f"读取串口出错: {e}")
                    break

                if not data:
                    # 超时，顺便检查是否需要 flush
                    if time.time() - last_flush >= args.flush_interval:
                        f.flush()
                        last_flush = time.time()
                    continue

                total_bytes += len(data)

                if args.hex:
                    text = " ".join(f"{b:02X}" for b in data)
                else:
                    # 解码，无法解码的字节用替换字符
                    text = data.decode("utf-8", errors="replace")

                # 时间戳模式下，逐行加时间戳
                if args.timestamp:
                    for line in text.splitlines(keepends=True):
                        if line.strip() == "":
                            continue
                        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        f.write(f"[{ts}] {line if line.endswith(('\n', '\r')) else line + '\n'}")
                        line_count += 1
                else:
                    f.write(text)
                    line_count += text.count("\n")

                # 定时刷新
                if time.time() - last_flush >= args.flush_interval:
                    f.flush()
                    last_flush = time.time()

            f.write(f"===== 记录结束: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} "
                    f"共 {total_bytes} 字节 =====\n")
            f.flush()
    finally:
        ser.close()
        print(f"完成。共接收 {total_bytes} 字节，文件: {args.output}")


if __name__ == "__main__":
    main()