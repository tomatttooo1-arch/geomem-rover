"""生成任务书总索引 briefs_index.md。
用法：python3 build_briefs_index.py --run <run> --rules <脱敏规则.json> --out briefs_index.md
<run> 下每个包一个目录，含 brief*.md，结果在同目录 README.md 或 HANDOVER.md；
主控日志 <run>/CONTROL.md 里记录"交回 → 收/不收/打回/改判/轮换"。"""
import argparse, glob, os, re, datetime, collections, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sanitize import Sanitizer

TYPES = [  # (类型, 包名前缀)
    ("HLS 核", ("MEMK", "SRCHK", "GEO_", "PREP", "OBSK", "GMAP", "PLNPATH", "PERMIT", "DETPOST", "OBSREF", "D16")),
    ("整板构建", ("MERGE", "M7PRE", "OFFLOAD", "CDC", "HOSTM", "BUILD")),
    ("上板测量", ("BOARD", "SMOKE", "M7BOARD", "DPUPIPE", "CAMKV", "E2E", "NANOCPU", "NANOPROF", "A53BENCH", "GOLIVE", "DPU")),
    ("文档", ("DOC", "FIG", "REFCHK", "SOTA", "GUIDE", "SKILL", "LLMLOG", "AUDIT")),
    ("发布", ("REL",)),
]
OTHER = "产品软件/链路"
MODULES = [("记忆核", r"记忆核|MEMK"), ("搜索核", r"搜索核|SRCHK"), ("规划核", r"规划核|GMAP|PLNPATH|取路径"),
           ("几何核", r"几何核|GEO_"), ("观测核", r"观测|OBSK"), ("前处理核", r"前处理核|PREP"),
           ("后处理核", r"后处理核|DETPOST|POSTPL"), ("许可核", r"许可核|permit"), ("DPU", r"DPU"),
           ("整板 BD", r"整板|比特流|BD\b|MERGE"), ("PS 软件", r"\bPS\b|A53|kv_[a-z]+"), ("Nano", r"Nano"),
           ("底盘", r"底盘|里程"), ("相机", r"相机|Orbbec"), ("设计文档", r"文档|报告")]

def ptype(name):
    for t, prefs in TYPES:
        if name.upper().startswith(prefs):
            return t
    return OTHER

def strip_md(s):
    s = re.sub(r"[`*#>|]", "", s)
    s = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", s)
    return re.sub(r"\s+", " ", s).strip()

def short(s, n):
    s = strip_md(s)
    return s if len(s) <= n else s[:n - 1] + "…"

def goal(text):
    lines = [l for l in text.splitlines() if l.strip()]
    first = lines[0] if lines else ""
    m = re.match(r"^\s*(?:#+\s*)?(?:【[^】]*】|[A-Za-z0-9_\- ]+(?:brief|第[一二三]?[阶段棒]+)?\s*[：:]?)?\s*(.*)$", first)
    g = m.group(1) if m and m.group(1) else first
    if len(strip_md(g)) < 8 and len(lines) > 1:
        g = lines[1]
    return short(g, 90)

def pdate(text, path):
    m = re.search(r"20\d\d-\d\d-\d\d", text[:600])
    if m:
        return m.group(0)
    m = re.search(r"\b(1[01])-(\d\d)\b", text[:600])
    if m:
        return f"2026-{m.group(1)}-{m.group(2)}"
    return datetime.date.fromtimestamp(os.path.getmtime(path)).isoformat()

CONCLS = [re.compile(r"^#+\s*.*(结论|一页|先看)"), re.compile(r"^#+\s*.*(结果|摘要|总结|交回|小结)")]
def result(d):
    for fn in ("README.md", "HANDOVER.md"):
        p = os.path.join(d, fn)
        if not os.path.exists(p):
            continue
        lines = open(p, encoding="utf-8", errors="replace").read().splitlines()
        for CONCL in CONCLS:
          for i, l in enumerate(lines):
            if CONCL.match(l) and "预注册" not in l:
                for l2 in lines[i + 1:i + 15]:
                    t = strip_md(re.sub(r"^\s*(\d+\.|[-*])\s*", "", l2))
                    if len(t) > 6 and not l2.lstrip().startswith(("#", "|", "```")):
                        return short(t, 110), fn
        # 无结论节：取标题后第一段正文
        for l2 in lines[1:30]:
            t = strip_md(l2)
            if len(t) > 10 and not l2.startswith(("#", "|", "-->", "```")):
                return short(t, 110), fn
    return "（无 README/HANDOVER）", ""

VERD = re.compile(r"(打回|不收|改判|轮换|验收通过|验收|功能收|面积不收|→\s*收)")
def verdicts(name, ctrl_lines):
    base = re.sub(r"(_r2|_b|_c|_impl|_close|_phaseA)$", "", name)
    pat = re.compile(r"(?<![A-Za-z0-9_])" + re.escape(base) + r"(?![A-Za-z0-9])")
    hits = []
    for l in ctrl_lines:
        if pat.search(l):
            # 只看包名之后 120 字内的判词，避免同一行别的包的判词串过来
            for m in pat.finditer(l):
                seg = l[m.end():m.end() + 120]
                nxt = re.search(r"(?<![A-Za-z0-9_])[A-Z][A-Z0-9_]{2,}\d(?![A-Za-z0-9])", seg)
                if nxt:
                    seg = seg[:nxt.start()]
                hits += [v.replace(" ", "") for v in VERD.findall(seg)]
    return hits

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True); ap.add_argument("--rules", required=True); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    S = Sanitizer(a.rules)
    ctrl = open(os.path.join(a.run, "CONTROL.md"), encoding="utf-8", errors="replace").read().splitlines()
    rows = []
    for bp in sorted(glob.glob(os.path.join(a.run, "*", "brief*.md"))):
        d = os.path.dirname(bp); pkg = os.path.basename(d)
        suf = os.path.basename(bp)[5:-3]  # "", "_b", "_r2" ...
        name = pkg + suf
        text = open(bp, encoding="utf-8", errors="replace").read()
        mods = [m for m, r in MODULES if re.search(r, text[:3000])][:4]
        res, src = result(d) if suf == "" else (f"（同 {pkg} 目录，见主包行）", "")
        v = verdicts(pkg, ctrl)
        flags = []
        if any(x in v for x in ("打回", "不收", "面积不收")): flags.append("不收/打回")
        if "改判" in v: flags.append("改判")
        if "轮换" in v: flags.append("轮换")
        if any(x in v for x in ("→收", "验收", "验收通过", "功能收")): flags.insert(0, "收")
        rows.append(dict(name=name, pkg=pkg, date=pdate(text, bp), goal=S(goal(text)), mods="、".join(mods) or "—",
                         res=S(res), src=src, verdict="、".join(flags) or "未见记录", type=ptype(pkg), rej=bool(set(flags) & {"不收/打回", "改判"})))
    pkgs = collections.OrderedDict()
    for r in rows:
        pkgs.setdefault(r["pkg"], r)
    tc = collections.Counter(r["type"] for r in pkgs.values())
    rej = [p for p, r in pkgs.items() if r["rej"]]
    out = []
    out.append("# 任务书总索引（主控派给执行代理的全部任务书）\n")
    out.append("本表由 `build_briefs_index.py` 从运行目录逐个任务书自动生成，未手改。每个任务书是主控会话写给一个执行代理（子代理）的完整提示词；"
               "同一个包的接力棒（`_b`、`_c`、`_r2` 等后缀）各占一行。\"结果\"取自同目录 README.md 或 HANDOVER.md 的结论段第一句；"
               "\"主控验收\"取自主控日志 CONTROL.md 中该包名之后的判词（收 / 不收或打回 / 改判 / 轮换，轮换指代理上下文用满后换新代理接力）。"
               "已脱敏：人名改为 member-a/b/c，绝对路径改为 `<run>`、`<repo>` 等占位符。\n")
    out.append("## 统计\n")
    out.append(f"- 任务书总数：**{len(rows)}** 份，分属 **{len(pkgs)}** 个工作包（其余为同包接力棒）。")
    out.append("- 按类型（以包计）：" + "；".join(f"{t} {tc.get(t,0)}" for t in [t for t, _ in TYPES] + [OTHER]) + "。")
    out.append(f"- 主控日志里出现过“不收 / 打回 / 改判”的包：**{len(rej)}** 个（{', '.join(rej)}）。")
    out.append("  - 人工复核（2026-11-04，逐条读主控日志原句）：真正被判\"不收\"的是 3 个——MEMK3（功能收、面积不收，随后派 MEMK4 重写架构）、"
               "SRCHK4（LUT 门没过，不收）、SRCHK5（不收面积、只收功能版）；MERGE9P 是\"收（部分）\"，同一条里主控因推算方法失效改了判据，"
               "并打回的是另一成员的存储设计而非本包；NN-CAM2 的\"改判\"是把它从可选项改判为文档必需的数据项，不是否定。")
    nv = sum(1 for r in pkgs.values() if r["verdict"] == "未见记录")
    out.append(f"- 主控日志里出现过“轮换”的包：{sum(1 for r in pkgs.values() if '轮换' in r['verdict'])} 个；主控日志中没有找到判词的包：{nv} 个（多为小包或仍在进行中）。")
    out.append("- 类型按包名前缀归类：HLS 核＝单个 PL 加速核的设计与实现；整板构建＝Vivado 整板集成与比特流；上板测量＝在板子上跑测量或冒烟；"
               "文档＝设计报告与提交材料；发布＝提交代码包与发布标签；产品软件/链路＝PS/Nano 侧软件与产品链接入。\n")
    out.append("## 总表\n")
    out.append("| # | 包名 | 类型 | 日期 | 目标（任务书首段） | 涉及核/模块 | 结果（README/HANDOVER 结论首句） | 主控验收 |")
    out.append("|---|---|---|---|---|---|---|---|")
    esc = lambda s: s.replace("|", "／")
    for i, r in enumerate(rows, 1):
        out.append(f"| {i} | {r['name']} | {r['type']} | {r['date']} | {esc(r['goal'])} | {r['mods']} | {esc(r['res'])} | {r['verdict']} |")
    open(a.out, "w", encoding="utf-8").write(S("\n".join(out)) + "\n")
    print(len(rows), len(pkgs), dict(tc), len(rej))

if __name__ == "__main__":
    main()
