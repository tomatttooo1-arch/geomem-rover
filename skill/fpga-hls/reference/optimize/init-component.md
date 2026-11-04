> 来源：AMD Ross `hls-optimize/reference/init-component.md`（作者：Alex Li, Sai Akhil Ayyagari），MIT 许可，见 `../../upstream.md`。
> **团队共享仓库改造（owner 2026-11-02）**：上游这一步会 `git init`、`git config user.email/user.name`
> （无 `--global`，写进仓库本地配置）、改 `.gitignore`，并在当前分支上做基线提交。在团队仓库里这会建嵌套仓库、
> 覆盖 `work/<名字>` 副本的本地身份（`tools/workspace.sh` 靠"本地 user.name == 目录名、邮箱 <名字>@example.org"
> 判身份）、把每次尝试的提交灌进共享历史。以上步骤**全部删除**，改为由 `session/hlsopt_session.sh` 建隔离实验区。
> 上游"基线可一步退回、过程可追溯"的目的保留。

# init-component（2022.2 + 团队仓库版）

## 禁止事项（任何时候都不做）

`git init`、`git config`（任何作用域的写入）、修改 `.gitignore`、`git push`、在实验区以外 `git reset --hard`、
`git clean`、任何 `--force`。身份沿用仓库已有配置。HLS 构建产物（solution 目录、`agent_logs/`、golden 输出）
不进 git，靠**只 scoped add 源文件和账本**保证，不靠改 `.gitignore`。

## 1. 确认 HLS 工程存在（取代上游的 vitis-comp.json 检查）

2022.2 没有 `vitis-comp.json`。确认的是：

- 有 `<proj_dir>/hls.app`（已跑过的 Tcl 工程），或有一份 `run_hls.tcl`（可由 `scripts/hls_run.sh --tcl` 建工程）；
- `set_top`、源文件、testbench 都能对上（`scripts/hls_report.sh basic-info <solution_dir>`）。

都没有时，按 `reference/run-flow.md` §4 的模板写 `run_hls.tcl`（这是源文件，会随尝试一起提交）。

## 2. 建隔离实验区（取代上游的 git init + 基线提交）

在**用户自己的功能分支副本**里（例如 `work/<本人>/`，分支 `feat|fix|exp/<本人>/<主题>`）运行：

```bash
bash tools/agent_skills/fpga-hls/session/hlsopt_session.sh start \
    --module <模块名> --files "<被优化的源文件，相对仓库根，空格分隔；含 run_hls.tcl 如需改>" \
    [--date YYYYMMDD] [--wt-root <实验区父目录>] [--ledger-dir <账本目录，默认=首个源文件目录/.memory>]
```

脚本做的事与前置检查：

- 原副本必须在本人的功能分支上（不在 `main`、不游离、分支名含本人 `user.name`）；
- 被优化的源文件在原副本里 `git status --porcelain -- <files>` 必须为空，否则**拒绝并告诉用户**
  （那可能是 owner 或队友没提交的修改，不能被覆盖）；
- `git worktree add -b exp/<本人>/hls-opt-<模块>-<YYYYMMDD> <副本外的同级目录>/hls-opt-<模块>-<日期> <当前功能分支>`，
  默认父目录是 `<副本路径>-wt/`（例如 `<worktrees>/`），不在团队副本之内；
- 在实验分支上提交账本初始化，并打本地标签 `hlsopt/<模块>/base`；
- 会话状态记在该 worktree 的私有 git 目录里（`.git/worktrees/<名>/hlsopt-session.env`），不进工作树。

之后**所有**优化工作都 `cd` 到脚本打印的实验区路径里做。HLS 工程（`<proj_dir>`）也建在实验区里，
不要指到原副本的路径上。

## 3. 验证

```bash
cd <实验区>
bash <实验区>/tools/agent_skills/fpga-hls/session/hlsopt_session.sh status
git log --oneline -3
git tag -l 'hlsopt/<模块>/*'
```

## 4. 架构流程同样适用

`reference/architect.md` Step 3d-2（基线快照）之后的提交也在这样的实验区里做：
架构产物先在实验区生成、提交、打 `hlsopt/<模块>/base`，再进入优化。
