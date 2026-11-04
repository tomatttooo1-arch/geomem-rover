
## 10-26 车屏 AR 接入比赛链（D-042，先读）

- **KV 服务默认带 AR**：`geomem-kv-start` → `kv_start.sh <repo> <out> drm`，即 `kv_server --ar drm --ar-source live`、屏模式（DP 有 EDID 用 `--ar-mode 1024x600`，没有用 `ar_screen.sh` 同款 `1024x600_edid` modeline）、`--ar-colour main --ar-colour-port 47801`、快照内固定 tune `kv_ar_tune.json`（colour_main 1、depth_stretch_pct 250 即 k=2.5，布局 depth_dx 192 / depth_dy 140 / mini_x 880 / mini_y 456，取自 AR_COLOUR_MAIN §17 板上原值）、`--expect-build-id 0x00da5041`、`--core-timeout-s 3`，DMA 环境同 ar_screen.sh，不转储。
- **车屏不挡车**：AR 起不来（没有助手二进制、无屏/DRM 错、助手 15 s 未就绪、比特流不对）⇒ 停掉这次的残留进程，按 10-26 前的原命令行在 `<out>-ar-off` 重起，stdout 打 `AR_DISABLED reason=…`；运行目录写 `ar_status.json`。`geomem-kv-start` 的 JSON 带 `ar`（drm/off，按运行中进程的实际参数）与 `ar_reason`；`nano_up.sh` 打印 `AR_STATUS on` 或 `AR_DISABLED reason=…`，`console_remote.sh up` 的进度里也会显示这行。
- **Nano 送彩色**：KV 报 `ar: drm` 时 `nano_up.sh` 给车栈 `CAR_AR_COLOUR=1`，车栈在 producer 就绪后起 `nano_colour.py --lazy`（日志 `$RUN/colour.log`，退出写 `$RUN/colour_stats.json`），随 car_down 停；它只是显示，退出不触发车栈 fail-stop。`--lazy` 只在 KV 来要彩色时才订阅彩色话题，KV 没开 AR 时只发 0.5 s 一个的小 HELLO。显式给 `CAR_AR_COLOUR=0/1` 时以给的为准；通用 `car_up.sh` 默认 0（bench_line4 等路径不变）。
- **KV 快照可更新**：服务器上 `bash tools/car_run/deploy_kv.sh --ref <同 Nano 的 ref>`（`--dry-run` 先看；KV sudo 要密码加 `--sudo-stdin`；服务在跑先收车栈再加 `--stop-kv`）。它 git archive 同一提交、宿主交叉编 AR 助手、经 Nano 送 KV、`install_kv_service.sh --update`（旧快照留在 `releases/*-prev-*` 可回滚，kv_server 在跑时拒绝），最后核已装 COMMIT 与助手 sha256。`board_preflight.sh` 会核 KV 已装 COMMIT 是否等于部署 ref、服务在跑时 AR 开没开。
- 上板未验：真车上整链（AR 起屏＋彩色）只在 10-16/17 手工链实测过，比赛链自动版本待控制器上板确认；Nano CPU 增量见验收手册。

## 10-21 第二趟启动前检查补充

`demo` 必须没有当前车栈指针；包括坏软链均明确拒绝，需先结束并采集第一趟。连接已有趟使用 `bridge`，不再把复用第一趟当第二趟成功。

`map_sha256` 在任何SSH/起栈前校验为完整64位小写摘要。第二趟还会先读完整且已绑定的记忆、地图导出及图片，核对导出sha表声明的数据库摘要与配置/记忆相同，并严格比对绑定时YAML和图片摘要；通过后才启动KV和车栈。此处不重读原始数据库，数据库实际读取/摘要由底图生成步骤负责。未部署新check_demo.py的Nano会明确失败，不会绕过检查。

Nano本机手动调用 `nano_up.sh` 的第7参数为第二趟数据库SHA256（前6参数仍为repo、mode、hub_port、KV配置、地图YAML、记忆文件）。笔记本session.py自动传入。mapping/attach/kv-up无需该摘要。

当前14:29检查Nano远程入口关闭、Tailscale离线，最后在线14:10；本次更改只在服务器隔离测试，尚未部署到Nano，不作为实车两趟验收。

# 电脑比赛控制台 · 2026-10-19

## 10-19 现场启动修订（以下优先）

桌面启动器现在调用 session.py start，电脑直接 SSH Nano，不使用实验室服务器。已有 READY 车栈时复用；未运行时，Nano 先启动 KV，再执行原车栈预检和启动。SSH 同时转发 hub 与真实三维通道，Python 桥和原网站在电脑本机运行。启动不发送行驶指令，车栈相机只打开一次；Ctrl+C 只关闭本次本机服务，车栈继续运行。

一次性配置：电脑到 Nano 和 Nano 到 KV 的独立 SSH 公钥、可信主机公钥、session.json 的实际地址与目录。Nano 使用 kv_ssh.example.conf 配置。KV 管理员安装器 install_kv_service.sh 将固定源码复制为 root 所有，并安装固定无参数的 KV 启动权限；只在用户批准和管理员执行后生效，安装本身不启动/停止硬件。重复或身份不明的现有 KV 进程不会被替换。

现场默认开启车栈 ROS 预览；不直接打开独立相机。Docker 只用于离线底图，check 不再强制检查电脑 Docker。start 无车栈时启动第一趟；第二趟仍用 demo 并提供地图、记忆和 SHA。Nano 本机也可调用 nano_up.sh，参数依次是源码目录、mapping/demo/attach/kv-up、hub端口、KV SSH配置、可选地图和记忆路径。

新增启动专项7项、KV身份专项4项在Python3.13/3.8均通过；Nano直接SSH KV已实测。管理员安装及笔记本同局域网直连仍待完成，以当前实车停点文档为准。下方旧流程中“裸hub地址”“手动分开起KV”“默认关闭预览”“check强制Docker”等描述已由本修订取代。


## 界面

当前使用原 M8 网站 `http://127.0.0.1:8788/#remote`，源码保存在 `ui/geomem/operator/`。保留原有冰蓝玻璃、车模和按钮效果，主界面一屏显示，次要功能用抽屉展开。`session.py` 会启动本地 Node 网站并连接 Python 转发桥。支持转发桥全部八类命令：急停、二次确认解除、按住遥控、找物、选择候选、撤销、相对目标、结束建图。此前 `/competition.html` 页面仅保留历史代码，不是当前交付入口。

- 遥控频率 10 Hz，速度比例默认 25%，最高 0.15 m/s / 0.6 rad/s；松手、失焦、隐藏页、切流程、连接异常时解除驾驶并尝试 stop。重新连接不恢复驾驶。
- 空格请求急停（输入文字时除外）；急停按钮固定在顶部，完整控制台全屏时保留。网页断开时无法保证发送到车上，备用 Nano 终端仍按队长交接准备 `car_pc.sh estop`。
- 第三视角沿用原 M8 三维车模，画真实 1 cm 底图、滚动栅格、位置、物体记忆与轨迹；第一视角和双画面通过同一个桥读取 `/preview.jpg`。
- 页面区分受理、执行结果、超时未知；遥控只等 ACK，结束建图最长等待约 190 秒，急停/解除留 35 秒回执余量。
- 下方五阶段清单为人工确认，不是自动判定；日志可以导出 JSON，不含车栈口令。

当前网站默认只接实车，未连接时锁定控制并明确显示未知。之前的模拟脚本保留为历史开发工具，不用于本次交付和实车验收。

## 现场配置与启动

1. 赛前将本工作分支的完整源码快照部署到 Nano 的独立目录，并写真实提交号 `COMMIT`；KV260 需要 `tools/kv_link` 等完整依赖及已安装的 `m7_pl_merge_a2` 比特流。不要只复制本页面就认为车栈已更新。
2. 将 `session.example.json` 复制为 `session.json`，填写 Nano 的现场 SSH 地址、局域网 IP、两板部署目录和本次 KV 运行目录。示例 IP/目录是占位值。电脑到 Nano、Nano 到 KV260 的 SSH 密钥和无交互 sudo 须在现场预先配置。
3. 笔记本需 Python 3.8+（桥仅标准库）和 Node.js 20+（原网站，无需 npm install）；底图生成另需 Docker Desktop 及交接中指定的 RTAB-Map 镜像。当前这台笔记本尚未确认安装 Docker，不宣称底图生成已实测。

所有脚本默认只打印计划。加 `--execute` 才执行：

```text
python ui/geomem/tools/competition/session.py check --config session.json --execute
python ui/geomem/tools/competition/session.py kv-up --config session.json --execute
python ui/geomem/tools/competition/session.py mapping --config session.json --execute
```

`kv-up` 经 Nano SSH 启动 KV 服务；发现已有 KV 服务或本趟 car.current 时拒绝改动，不擅自替换。新 `kv_run` 目录避免覆盖旧运行证据。`mapping` 先执行车栈 dry-run，再起栈、获取本趟 token 到本机文件、启动桥并打开控制台。若车已运行，仅使用 `bridge` 获取当前 token 并连接。桥保持在终端前台；Ctrl+C 退出桥不会自动收车。

第一趟结束建图并收到完成回执后：

```text
python ui/geomem/tools/competition/session.py down --config session.json --execute
```

核对第一趟 `down_marks.txt` 中的 `memory bind rc=0`，在配置中填写 `database`、`map_yaml`、`memory_file`。复制与生成：

```text
python ui/geomem/tools/competition/session.py collect --config session.json --execute
python ui/geomem/tools/competition/session.py basemap --config session.json --execute
```

底图输出到配置旁 `session-data/basemaps/`，耗时和库 SHA 写入 `basemap-result.json`。将完整 SHA 填入配置的 `map_sha256`。原位、原朝向摆车后启动第二趟：

```text
python ui/geomem/tools/competition/session.py demo --config session.json --execute
```

如 Nano/KV260、SSH、镜像、地图身份不满足要求，脚本停止并保留输出，不跳过预检。现场启动路径尚未上板跑通，不应称为一键实车验收完成。

## 第一视角接入

新增可选 `CAR_PREVIEW=1`（配置 `preview: true`）会在车栈内订阅 `/camera/color/image_raw`，转换为 `/fpga/preview/jpeg`，并为 hub 开启 `--preview`。不直接读取 `/dev/video*`，不用旧的 `rover_agent.py` 视频服务。


桥仅转发合规 JPEG；超过 1.5 秒或链路断开时返回不可用，页面不把旧画面当实时。图像编码已在服务器两套 Python 用真实 OpenCV 测试；ROS 话题到浏览器的实机链路未验。

## 模拟与真实的边界

