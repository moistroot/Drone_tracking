import json, glob, os, re, collections, sys

ROOTS = glob.glob(r"C:/Users/KasuganoSora/.claude/projects/*GCC*") + \
        glob.glob(r"C:/Users/KasuganoSora/.claude/projects/*gcc*")
files = sorted({f for r in ROOTS for f in glob.glob(os.path.join(r, "*.jsonl"))})

def get_text(content):
    if isinstance(content, str):
        return content
    out = []
    if isinstance(content, list):
        for b in content:
            if isinstance(b, dict):
                if b.get("type") == "text":
                    out.append(b.get("text", ""))
                elif b.get("type") == "tool_result":
                    pass
    return "\n".join(out)

def blocks(content):
    return content if isinstance(content, list) else []

usu = open("skill_distill/user_messages.txt", "w", encoding="utf-8")
ast = open("skill_distill/assistant_digest.txt", "w", encoding="utf-8")
too = open("skill_distill/tool_actions.txt", "w", encoding="utf-8")
err = open("skill_distill/errors.txt", "w", encoding="utf-8")
summ = open("skill_distill/sessions_summary.txt", "w", encoding="utf-8")
stat = collections.Counter()

for f in files:
    proj = os.path.basename(os.path.dirname(f))
    sess = os.path.basename(f)[:8]
    tag = f"{proj}/{sess}"
    users, asst, calls, errs, first_t, last_t = [], [], [], [], None, None
    side = 0
    with open(f, encoding="utf-8") as fh:
        for ln, line in enumerate(fh, 1):
            try:
                d = json.loads(line)
            except Exception:
                continue
            t = d.get("timestamp")
            if t:
                first_t = first_t or t
                last_t = t
            if d.get("isSidechain"):
                side += 1
                continue
            typ = d.get("type")
            msg = d.get("message")
            if not isinstance(msg, dict):
                continue
            role = msg.get("role")
            cont = msg.get("content")
            if typ == "user" and role == "user":
                for b in blocks(cont):
                    if isinstance(b, dict) and b.get("type") == "tool_result" and b.get("is_error"):
                        txt = get_text(b.get("content"))
                        errs.append((t, txt[:500]))
                        stat["tool_error"] += 1
                if d.get("toolUseResult") is not None:
                    continue
                txt = get_text(cont).strip()
                # drop pure IDE/context injections
                if not txt:
                    continue
                if txt.startswith("<ide_opened_file>") and len(txt) < 400:
                    continue
                users.append((t, txt))
                stat["user"] += 1
            elif typ == "assistant" and role == "assistant":
                for b in blocks(cont):
                    if not isinstance(b, dict):
                        continue
                    if b.get("type") == "text" and b.get("text", "").strip():
                        asst.append((t, b["text"].strip()))
                        stat["asst"] += 1
                    elif b.get("type") == "tool_use":
                        nm = b.get("name", "?")
                        inp = b.get("input", {}) or {}
                        stat["tool:" + nm] += 1
                        calls.append((t, nm, inp))

    summ.write(f"== {tag}\n  file={os.path.basename(f)}\n  time={first_t} .. {last_t}\n"
               f"  user={len(users)} assistant_text={len(asst)} tool_calls={len(calls)} errors={len(errs)} sidechain_lines={side}\n"
               f"  first_user: {(users[0][1][:160].replace(chr(10),' ') if users else '')}\n"
               f"  last_user:  {(users[-1][1][:160].replace(chr(10),' ') if users else '')}\n\n")
    for t, txt in users:
        usu.write(f"\n----- [{tag}] {t}\n{txt[:4000]}\n")
    for t, txt in asst:
        ast.write(f"\n[{tag}] {t} :: {txt[:280].replace(chr(10),' ')}\n")
    for t, nm, inp in calls:
        if nm == "Bash":
            s = inp.get("command", "")
        elif nm in ("Write", "Edit", "Read"):
            s = " " + str(inp.get("file_path", ""))
        elif nm == "Grep":
            s = f" pattern={inp.get('pattern')!r} path={inp.get('path')} glob={inp.get('glob')}"
        elif nm == "Glob":
            s = f" {inp.get('pattern')} in {inp.get('path')}"
        elif nm == "Task":
            s = f" [{inp.get('subagent_type')}] {inp.get('description')}: {str(inp.get('prompt'))[:200]}"
        elif nm == "Skill":
            s = f" {inp.get('skill')} {inp.get('args')}"
        elif nm == "WebSearch":
            s = f" {inp.get('query')}"
        elif nm == "WebFetch":
            s = f" {inp.get('url')}"
        else:
            s = " " + str(inp)[:200]
        too.write(f"[{tag}] {t} {nm}:{s[:400]}\n")
    for t, txt in errs:
        err.write(f"\n----- [{tag}] {t}\n{txt}\n")

summ.write("\n== TOTALS\n")
for k, v in stat.most_common():
    summ.write(f"  {k}: {v}\n")
for fobj in (usu, ast, too, err, summ):
    fobj.close()
print("files:", len(files))
print("stats:", dict(stat))
