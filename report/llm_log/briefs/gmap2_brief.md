# GMAP2 brief：规划核第二段——取路径压 LUT＋全局地图 gmap_op 0–4 进核（主控 11-04，worker-opus-high）

RUN = <run>。同一个规划核（planner_field_dual），一包做完、一起回归、只跑一次最终 OOC。

## 起点与材料
- 代码起点：PLNPATH0 的 post（分支 `feat/member-a/nat-plnpath0-20261104`  at 9bb45cb）；GMAP 的 C 模型与整数版 kv_mapper 在 `feat/member-a/nat-gmap1-20261104`  at 42503fc（合进来）。新开 worktree `<worktree>/nat-gmap2-20261104`，分支 `feat/member-a/nat-gmap2-20261104`。PLNPATH0 的旧 worktree 只读，不在里面写。
- 必读：`RUN/PLNPATH0/HANDOVER.md`（降法 1–4、怎么跑、已知坑）、`RUN/PLNPATH0/README.md`、`RUN/GMAP0/DESIGN.md`、`RUN/GMAP0/DESIGN_ADD_PLGRID.md`、`RUN/GMAP0/README.md`（§0 判据、对照②的合成向量）、`RUN/HWDESIGN_RULES_20261104.md`（**含末尾"补充"一节：取路径核 HLS 实测是手算的 2.5–3 倍，五条写码规矩照做**）。HLS 工作调用 skill fpga-hls，时序与方法学看 vivado-checks。

## 要做（按顺序）
1. **取路径压回去**：照 PLNPATH0 HANDOVER 降法 1、2（必要时 3、4），每步 `make native`（320/320）＋原生 tb 两个 sha（0036fda9… 和 86b2d252…）＋csynth 估计。目标：取路径部分回到 OOC ≤4,347 的水平（用 csynth 估计筛，不单独跑 OOC）。
2. **gmap_op 0–4 进核**（MERGE／SCAN／CLEAR／WINDOW，含设计补充里的车体种子与走过清除）：先按设计写，csynth 后逐函数对手算（GMAP0 约 740 LUT、WINDOW 约 306），超 1.5 倍先改再往下走。
3. **最终一次 OOC**：整核门 **≤5,420 LUT**（= 4,347 ＋ 740 ＋ 330），WNS ≥0  at 100 MHz，BRAM/URAM 照设计（全局图在 DDR，片上不新增 URAM，BRAM36 新增 ≤2）。若超门：WINDOW 只准用设计补充 §5.2 第 1 档（去证据计数）；其他超出写清在哪、怎么降，交回主控，不擅自把逐格计算移回 PS。
4. **判据（照 GMAP0 README §0 与设计补充 §6，测前已写死，不加别的测试）**：对照①已过不重跑；对照②联仿对 C 模型两组向量逐位一致，其中合成向量 1（压边界、窗口完全在图外、帧里出现 100/0/−1 以外的值）同时拿 kv_mapper 整数版 PS 参考跑一遍；WINDOW 的采纳闸（car1015 差异格 ≤0.01%）在 HLS 前先跑；gmap_op 不用时（=0 等价旧行为）原生 tb 两个 sha 不变。
5. 规划驱动（DRV5 分支 d2c5086 的 `hw/planner/drv`）补 0x90 与 gmap_op 的寄存器和参数块；宿主与 qemu 各跑一次已有自检。
6. export.zip 交出来给 m6，写进 README（路径与 sha256）。

## 纪律
构建槽只用 `<locks>/nat_build_slot_{1,2,3}`（mkdir 取、写 owner、用完删 owner 再 rmdir），开前 MemAvailable ≥12 GB、线程 ≤8，`nat_build_whole` 不碰。pre/post 标签、只推显式 ref、不 `--tags`、不合 main。禁 `pkill -f`。不动板子。输出节流。**约 20 万 token 时写 `GMAP2/HANDOVER.md` 交回**，不要撑到 30 万。交付 `RUN/GMAP2/README.md`（改前改后资源表、判据结果、拍数）；交回 10 行内中文摘要。
