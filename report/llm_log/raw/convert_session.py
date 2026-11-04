"""把一次子代理会话的 stream.jsonl 转成可读 markdown（带脱敏）。
用法：python3 -I convert_session.py --session <会话目录> --rules <脱敏规则.json> --title <标题> --note <导读文件.md> --out <输出.md>
会话目录里有 prompt.md（主控写的任务书全文）、stream.jsonl（逐行事件）、meta.json。
输出顺序：导读 → 提示词全文 → 按时间的模型文字回答与工具调用一行摘要 → 最终回答。
模型的内部思考块不导出（流里本就是空的）；工具输出不贴，只在工具报错时留一行报错摘要。"""
import argparse, json, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "index"))
from sanitize import Sanitizer

def one_line(s, n=180):
    s = " ⏎ ".join(x.strip() for x in str(s).splitlines() if x.strip())
    return s if len(s) <= n else s[:n - 1] + "…"

def tool_summary(name, inp):
    if name == "Bash":
        return f"`Bash` {one_line(inp.get('command', ''))}" + (f"  —— {one_line(inp['description'], 60)}" if inp.get("description") else "")
    if name in ("Read", "Write", "Edit", "NotebookEdit"):
        extra = ""
        if name == "Write": extra = f"（写入 {len(inp.get('content', ''))} 字符）"
        if name == "Edit": extra = f"（替换 {len(inp.get('old_string', ''))}→{len(inp.get('new_string', ''))} 字符）"
        return f"`{name}` {inp.get('file_path', '')}{extra}"
    if name in ("Skill",):
        return f"`Skill` {inp.get('skill', '')}"
    if name in ("Agent", "Task"):
        return f"`{name}` {one_line(inp.get('description', ''), 80)}"
    keys = ", ".join(f"{k}={one_line(v, 60)}" for k, v in list(inp.items())[:3])
    return f"`{name}` {keys}"

def main():
    ap = argparse.ArgumentParser()
    for k in ("session", "rules", "title", "note", "out"):
        ap.add_argument("--" + k, required=True)
    a = ap.parse_args()
    S = Sanitizer(a.rules)
    meta = json.load(open(os.path.join(a.session, "meta.json")))
    prompt = open(os.path.join(a.session, "prompt.md"), encoding="utf-8").read()
    out = [f"# {a.title}\n", open(a.note, encoding="utf-8").read().strip() + "\n",
           f"- 会话规模：{meta.get('num_turns')} 轮，耗时约 {round(meta.get('duration_s', 0) / 60)} 分钟，结束状态 {meta.get('terminal_reason')}。\n",
           "---\n", "## 一、提示词全文（主控写给执行代理的任务书，原样）\n", "````markdown", prompt.rstrip(), "````\n",
           "---\n", "## 二、执行过程（按时间顺序：模型文字回答原文 ＋ 工具调用一行摘要）\n",
           "图例：▶ 模型对外说的话（原文）；· 工具调用（只留工具名和命令/文件名）；⚠ 工具返回报错（只留首行）。\n"]
    final = None
    step = 0
    pending = {}
    last_text = None
    for line in open(os.path.join(a.session, "stream.jsonl"), encoding="utf-8"):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        t = d.get("type")
        sub = "（子任务内）" if d.get("parent_tool_use_id") else ""
        if t == "assistant":
            for c in d["message"].get("content", []):
                if c.get("type") == "text" and c.get("text", "").strip():
                    out.append(f"\n▶ {sub}" + c["text"].strip().replace("\n", "\n  ") + "\n")
                    last_text = (len(out) - 1, c["text"].strip())
                elif c.get("type") == "tool_use":
                    step += 1
                    pending[c.get("id")] = step
                    out.append(f"· [{step}]{sub} " + tool_summary(c.get("name", ""), c.get("input", {}) or {}))
        elif t == "user":
            cont = d.get("message", {}).get("content", [])
            if isinstance(cont, list):
                for c in cont:
                    if isinstance(c, dict) and c.get("type") == "tool_result" and c.get("is_error"):
                        body = c.get("content")
                        if isinstance(body, list):
                            body = " ".join(x.get("text", "") for x in body if isinstance(x, dict))
                        out.append(f"  ⚠ [{pending.get(c.get('tool_use_id'), '?')}] 报错：{one_line(body, 160)}")
        elif t == "result":
            final = d.get("result")
    if final is None and os.path.exists(os.path.join(a.session, "result.md")):
        final = open(os.path.join(a.session, "result.md"), encoding="utf-8").read()
    if final and last_text and last_text[1] == final.strip():
        out[last_text[0]] = "\n▶ （最后一条文字回答就是下方第三节的最终回答，这里不重复。）\n"
    out += ["\n---\n", "## 三、最终回答（执行代理交回主控的报告，原文）\n", (final or "（无）").strip(), ""]
    open(a.out, "w", encoding="utf-8").write(S("\n".join(out)))
    print(a.out, os.path.getsize(a.out), "tools", step)

if __name__ == "__main__":
    main()
