#!/bin/sh
# check.sh - 技能包结构、frontmatter、引用与凭据自检
# 用法: sh check.sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
fail=0

need() {
    if [ ! -f "$ROOT/$1" ]; then
        echo "MISSING: $1"
        fail=1
    fi
}

# 11 个文件：1 入口 + 4 参考 + 3 脚本 + 1 资产 + 2 验证日志
need SKILL.md
need references/01_algorithm.md
need references/02_integration.md
need references/03_verification.md
need references/04_pitfalls.md
need scripts/install.sh
need scripts/check.sh
need scripts/run_tb_fft.sh
need assets/tb_fft_chain.sv
if [ -d "$ROOT/verification" ]; then
    need verification/clean_deploy_2026-11-04.log
    need verification/tb_fft_chain_2026-11-04.log
else
    echo "verification logs: not installed (ok)"
fi

# frontmatter
if ! head -n 5 "$ROOT/SKILL.md" | grep -q '^name: gcc-phat-4ch$'; then
    echo "ERROR: SKILL.md frontmatter name mismatch"
    fail=1
fi
if ! grep -q '^description: ' "$ROOT/SKILL.md"; then
    echo "ERROR: SKILL.md missing description"
    fail=1
fi

# 入口必须引用四份参考
for r in 01_algorithm 02_integration 03_verification 04_pitfalls; do
    if ! grep -q "references/$r.md" "$ROOT/SKILL.md"; then
        echo "ERROR: SKILL.md does not reference references/$r.md"
        fail=1
    fi
done

# 不允许聊天记录、抓包等大体积原始数据
if find "$ROOT" -type f \( -name '*.jsonl' -o -name '*.pcap' -o -name '*.pcapng' -o -name '*.bin' \) | grep -q .; then
    echo "ERROR: raw chat/capture/binary payload found"
    fail=1
fi

# 凭据模式（分片拼接，避免脚本自匹配）
P1="api[_-]?key"
P2="sk-[A-Za-z0-9]{20,}"
P3="BEGIN PRIV""ATE KEY"
for pat in "$P1" "$P2" "$P3"; do
    if grep -rInE "$pat" "$ROOT" \
        --include='*.md' --include='*.sh' --include='*.sv' --include='*.log' 2>/dev/null; then
        echo "ERROR: possible credential material found"
        fail=1
    fi
done

if [ "$fail" -ne 0 ]; then
    echo "PACKAGE CHECK FAILED"
    exit 1
fi

echo "PACKAGE CHECK PASSED"
echo "payload sha256:"
cd "$ROOT"
sha256sum SKILL.md \
    references/01_algorithm.md references/02_integration.md \
    references/03_verification.md references/04_pitfalls.md \
    scripts/install.sh scripts/check.sh scripts/run_tb_fft.sh \
    assets/tb_fft_chain.sv
