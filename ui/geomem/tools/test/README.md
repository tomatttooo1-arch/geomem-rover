# GeoMem 前端阶段 1 测试脚本（只供测试）

- `mock_bridge_v2.mjs`：协议第二版**模拟桥**（Node，非 `bridge/` 的实现）。`node mock_bridge_v2.mjs 19190 <log>`：WS `ws://127.0.0.1:19190/ws`，控制口 `http://127.0.0.1:19191/ctl?...`（参数见文件头注释）。
- `a1_clock.mjs`…`a9_scope.mjs`：设计 §6 判据 A1/A2/A3/A4/A6/A8/A9，各自带阳性对照；`layout_candidates.mjs`（候选不被裁）、`sw_cleanup.mjs`（旧 SW 注销）、`clean_checkout.mjs`（干净检出起页）。
- 页面用 `app/server.mjs` 起（`PORT=18801 node app/server.mjs`），脚本参数给页面地址与截图目录；页面以 `?source=live&ws=ws://127.0.0.1:19190/ws` 直连模拟桥。
- A1/A2 的 v1 对照要 v1 界面（`git archive 3426dde ui/geomem/app` ＋ 官方 `three.core.js`，sha256 `eb077d24…`）与验收时的 v1 模拟桥（控制器 scratch `geomem_review/tools/mock_bridge.mjs`，端口 19090/19091）。
- 结果与口径见 `../../docs/STAGE1_FRONT_RESULTS.md`。

## 联调（真桥，10-15）

- `integ/`：真桥＋真前端判据脚本（T1 回放、T2 hub 夹具），经 `ui/geomem/bridge/testbed/run_tb.sh` 进 `unshare -rn` 隔离网络命名空间跑，页面由真桥托管 `http://127.0.0.1:8788/?source=live`。说明见 `bridge/testbed/README.md`，预注册与结果见 `docs/STAGE1_INTEG_PREREG.md`、`STAGE1_INTEG_RESULTS.md`。
- `integ/kiosk_*.mjs`（10-17）：展台页 `kiosk.html` 的冒烟与判据（puppeteer-core，4090/SwiftShader 两臂），用法见 `bridge/testbed/README.md`，预注册与结果见 `docs/STAGE1_KIOSK_PREREG.md`、`STAGE1_KIOSK_RESULTS.md`。
