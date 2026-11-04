# MERGE8 brief：m5 整板构建＋宿主读产物（主控 11-04，worker-opus）

RUN = <run>。先读 `RUN/MERGE7/README.md`（§5 一键命令与收住判据，**判据照抄、不改**）和 `RUN/MERGE7/HANDOVER.md`。

## 做什么
1. **整板构建**，照 MERGE7 README §5：
   - 硬件提交 f35f07f，开干净的 build worktree；
   - 前处理核用 **s18**：`RUN/PREP1/build/s18ooc/sol1/impl/export.zip`（sha256 以 94b41446ad803679 开头，先核）；
   - 互连 **samd**（主控按裁定 1 采纳 MERGE7 的推荐：SAMD 推算 CLB 90.0–92.2% ≤95%）。
   - 取整板槽 `<locks>/nat_build_whole`（mkdir＋owner 文件写清用途与 pid，结束删 owner 再 rmdir）。MemAvailable ≥12 GB、8 线程。槽 2 上 MERGE7 的 SmartConnect 对照（pid 866633）还可能在跑，不要碰它。
   - 布局后按四条止损判（WNS −0.3 脚本自动停；clk_dsp < −0.05、拥塞 ≥5、CLB >99.5% 要你看 timing_place.rpt / congestion_place.rpt 手判）。触发止损：停下，回报主控，不要自行改设计。
   - clk_dsp 布线后小幅为负：按 MERGE6 裁定 7 先再跑一轮布线后 phys_opt、再调时钟根；**降 DPU 时钟前必须先回报主控**。
   - 收住后 package_app，判据 1–6 逐条写过/不过。
2. **宿主读产物**（参照 `RUN/HOSTM4/` 的脚本与 README 格式）：整板与分模块资源（LUT/FF/BRAM/URAM/DSP/CLB 及占比，按核列：几何、融合、证据、许可、DPU、前处理、后处理、规划、互连）、时序摘要（各时钟 WNS/WHS）、Vivado 功耗报告解析、布局图 PNG。
3. 顺手做 MERGE7 HANDOVER 里两件小事（不影响整板，有空槽才做）：GEO_FX 断言阴性对照；README §3 补 SmartConnect 数。

## 硬约束
- 不碰板子、Nano、车。只按 pid 停进程，禁用 `pkill -f`。被系统拒绝的操作停下写进报告，不绕。
- git：KNOWN_BUILDS 加新 BUILD_ID 可以提交到 `feat/member-a/nat-merge7-20261104`（或新开 `feat/member-a/nat-merge8-20261104`），打 post 标签，只推显式 ref，不 `--tags`、不合 main。
- 等构建用后台＋轮询日志尾（tail/grep），不 cat 大日志；上下文约 300k 写 HANDOVER 交回。

## 交付
`RUN/MERGE8/README.md`：一页结论（BUILD_ID、时序、资源总表＋分模块表、功耗、判据 1–6、包路径与 SHA256、上板清单照 MERGE6 §8 抄）。交回 10 行内摘要。
