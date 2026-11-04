# 怎样重新生成本目录与 raw/ 下的文件

本目录和 `../raw/` 里的 markdown 全部由脚本生成，生成后未手改（raw/ 每份开头的"导读"段是人工写的，作为 `--note` 文件传入）。

| 文件 | 生成脚本 | 输入 |
|---|---|---|
| `briefs_index.md` | `build_briefs_index.py` | 运行目录下每个包的 `brief*.md`、同目录 `README.md`/`HANDOVER.md`、主控日志 `CONTROL.md` |
| `../raw/session_*.md` | `../raw/convert_session.py` | 一次子代理会话的 `prompt.md`（任务书全文）、`stream.jsonl`（逐行事件流）、`meta.json` |

```bash
python3 -I -B build_briefs_index.py --run <run> --rules <rules.json> --out briefs_index.md
python3 -I -B ../raw/convert_session.py --session <会话目录> --rules <rules.json> \
    --title "<标题>" --note <导读.md> --out ../raw/session_01_xxx.md
```

**脱敏**：两个脚本都调用 `sanitize.py`，按 `<rules.json>` 里的"正则 → 替换"表依次替换。规则表本身含真实人名、路径和主机名，因此**不随提交包提供**；
复用时按下列类别自建一份即可：绝对路径 → `<run>`、`<repo>`、`<worktree>` 等占位符；成员名 → `member-a/b/c`（非本队人员 → `member-x/y`）；
账号名 → `agent-account-a/b/c`；邮箱 → `<email>`；IP → `<ip>`；主机名与 ssh 别名 → `<host>`、`<host-alias>`；登录凭据文件与锁令牌 → `<secret-file>`、`--token <redacted>`；
学校名 → `<univ>`；与本项目无关的其他课题名 → `<other-project>`。

**会话转换取什么、不取什么**：取任务书全文、模型每轮对外的文字回答原文、每次工具调用的一行摘要（工具名＋命令或文件名，截断到约 180 字）、工具报错的首行、最终回答原文；
不取工具输出正文（太长且含大量环境细节）和模型内部思考块（事件流里本就为空）。
