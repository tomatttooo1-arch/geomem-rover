# SRCHK3 brief：概率搜索核 HLS 实现（主控 11-04，worker-opus-high，新代理）

RUN = <run>。设计与 C 模型已经完成、关键对照①已全量通过，本棒只做 HLS 到交出 export。

## 起点与材料
- 代码：分支 `feat/member-a/nat-srchk0-20261104`  at 7a6f428（SRCHK2 post）。新开 worktree `<worktree>/nat-srchk3-20261104`，分支 `feat/member-a/nat-srchk3-20261104`；旧 worktree 只读。
- 必读：`RUN/SRCHK0/DESIGN.md`（结构、位宽、存储、手算预算、§10 判据）、`RUN/SRCHK1/HANDOVER.md`（§4 第 3–5 条、§4.3 未经 HLS 验证的点）、`RUN/SRCHK2/HANDOVER.md`（§3 联仿向量与 testbench、§4 下一棒）、`RUN/HWDESIGN_RULES_20261104.md`（**含末尾"补充"：取路径核 HLS 实测是手算的 1.7 倍，五条写码规矩照做**）。HLS 工作调用 skill fpga-hls，时序与方法学看 vivado-checks。

## 要做
1. csim：用 `hw/srch/srch_tb.cpp`，HLS 类型版对 SW 版逐字节（先抓位宽错）。
2. csynth：逐函数对手算（设计 3,510 LUT／84 DSP），**Multiplexer 栏和任何一项超手算 1.5 倍先改再往下**；存储 RAMB36 ≤4、URAM ≤14（整板片上存储已紧）。
3. cosim：`SRCHK1/data/trace_v1.bin`、`trace_v2.bin` 两组向量逐字节（这就是对照②，不加别的测试）。
4. OOC  at 100 MHz：门 **≤4,000 LUT**、WNS ≥0、RAMB36 ≤4、URAM ≤14。超了先按设计里的降法降；要把逐格计算挪回 PS 的降法**不准擅自用**，写清超在哪交回主控。
5. 交出 export.zip（路径与 sha256 写进 README），给 m6 合入用；PS 驱动（SRCHK1 已写）按最终寄存器表核一遍。
6. A53 计时程序：照 `RUN/A53BENCH0` 的格式，搜索每帧观测与一次选点各给一条命令出中位数（板子不碰，由主控排进板测）。

## 纪律
构建槽只用 `<locks>/nat_build_slot_{1,2,3}`（mkdir 取、写 owner、用完删 owner 再 rmdir），开前 MemAvailable ≥12 GB、线程 ≤8，`nat_build_whole` 不碰。pre/post 标签、只推显式 ref、不 `--tags`、不合 main。禁 `pkill -f`。不动板子。输出节流。**约 20 万 token 时写 `SRCHK3/HANDOVER.md` 交回。** 交付 `RUN/SRCHK3/README.md`（改前改后资源表、cosim 结果、拍数）；交回 10 行内中文摘要。
