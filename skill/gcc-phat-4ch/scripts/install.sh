#!/bin/sh
# install.sh - 把技能包安装到目标工程的 .claude/skills/gcc-phat-4ch
# 用法: sh install.sh [目标工程根目录]   默认当前目录
set -eu

PKG_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
TARGET=${1:-$PWD}
DEST="$TARGET/.claude/skills/gcc-phat-4ch"

if [ ! -d "$TARGET" ]; then
    echo "ERROR: target not found: $TARGET" >&2
    exit 1
fi

mkdir -p "$DEST"
for item in SKILL.md references scripts assets; do
    rm -rf "$DEST/$item"
    cp -R "$PKG_DIR/$item" "$DEST/$item"
done

# 逐文件核对，避免半拷贝
for f in SKILL.md \
         references/01_algorithm.md references/02_integration.md \
         references/03_verification.md references/04_pitfalls.md \
         scripts/install.sh scripts/check.sh scripts/run_tb_fft.sh \
         assets/tb_fft_chain.sv; do
    if ! cmp -s "$PKG_DIR/$f" "$DEST/$f"; then
        echo "ERROR: copy mismatch: $f" >&2
        exit 1
    fi
done

chmod +x "$DEST"/scripts/*.sh 2>/dev/null || true
echo "installed copy: OK"
echo "destination: $DEST"
