"""从 user_messages.txt 生成 user_compact.txt（人读/喂模型的精简用户语料）。

保留：真实用户输入、任务通知（便于还原等待点）。
丢弃：压缩续写摘要、图片占位、IDE/命令注入、技能注入块、中断标记；同一会话同一分钟内的完全重复只留一条。
截断：每条 870 字符（与原文件一致），时间戳保留到分钟。
顺序：与 user_messages.txt 相同（会话按转录路径排序）。
"""
import re

SRC = "skill_distill/user_messages.txt"
DST = "skill_distill/user_compact.txt"
LIMIT = 870

DROP_PREFIX = (
    "This session is being continued",
    "[Image:",
    "[Request interrupted by user]",
    "<ide_opened_file>",
    "<ide_selection>",
    "<local-command-",
    "<command-name>",
    "<command-message>",
    "Base directory for this skill:",
)

text = open(SRC, encoding="utf-8").read()
parts = re.split(r"\n----- \[([^\]]+)\] (\S+)\n", text)

seen = set()
out = []
for i in range(1, len(parts), 3):
    tag, ts, body = parts[i], parts[i + 1], parts[i + 2].strip()
    if not body or body.startswith(DROP_PREFIX):
        continue
    key = (tag, ts[:16], body)
    if key in seen:
        continue
    seen.add(key)
    out.append((tag, ts[:16], body[:LIMIT]))

with open(DST, "w", encoding="utf-8") as fh:
    for tag, ts, body in out:
        fh.write(f"<<{tag} {ts}>>\n{body}\n\n")

print(f"blocks={len(out)} chars={sum(len(b) for _, _, b in out)}")
