# BOARDHANG0：m5 上板整机卡死定位（2026-11-04）

你是 FPGA 国赛线执行代理，交回后主控验收。输出节流（tail/grep，长内容落文件），上下文约 200k 收尾写 HANDOVER。

## 背景（先读）
RUN=<run>。读 RUN/SMOKE6/README.md 与 HANDOVER.md：09:25 起冒烟链在 KV260 上跑到 build→load(m5，BUILD_ID 0x0f35f07f)→lpd_afi→dpu 这一段时整机卡死（网口 PHY 有载波、串口全哑、ARP 失败），竞争解释 5 条已列。m6 会沿用同一套加载与 LPD/AFI 前置步骤，所以不定位清楚 m6 无法上板。

## 第一阶段：零板端（现在就做）
1. 查历史：m5a / m5 之前有没有在真板上加载成功过（SMOKE1/2/5、BOARD*、DEPLOY 记录、CONTROL.md 里 grep），哪些步骤（尤其 m5_lpd_afi.py 清 RST_LPD_TOP 第 19 位、读 AFI 宽度、读 sysinfo 0xa0000000）是第一次在真板执行。
2. 静态审查：m5_lpd_afi.py 与 board_bringup.sh 的寄存器操作序列，对照 m5 的块设计（MERGE7/builds/impl-m5-166-f35f07f/ 的 address_map.csv、hwh、CLOCKS.txt）与 Zynq UltraScale+ TRM（RST_LPD_TOP、AFI FM、S_AXI_LPD、PL 时钟/隔离 PCAP 后的复位释放）：找出哪个操作可能让 A53 的总线访问永不返回。重点：清 S_AXI_LPD 复位位时是否有 PL 主设备在途事务；dtbo 里 PL 时钟是否在读 PL 寄存器之前已使能；LPD 主口（PL→PS LPD）挂起会不会连带卡死 PS。
3. 给出最小分段上板方案（每一步只多一个动作，每步之前先 sync 落盘日志，必要时 `echo 1 > /proc/sys/kernel/sysrq` 并开 panic=10 自动重启或硬件看门狗以便卡死后自恢复——评估是否可行），以及对可疑步骤的代码修正（如果静态审查能坐实）。修正放新 worktree（<worktree>/nat-boardhang0-20261104，pre/post 标签，只推显式 ref）。

## 第二阶段：板子恢复后（owner 会安排现场断电重启 KV260；你用 `timeout 10 ssh -o ConnectTimeout=6 -o BatchMode=yes kv260 uptime` 每 2 分钟探一次，最多等 90 分钟，等不到就只交第一阶段）
2. **加载任何比特流之前只读日志**：SMOKE6/README 顶部"区分方法"列出的文件（smoke.log 最后 PHASE、build.txt、bringup-*/dmesg_load.txt、lpd/lpd_afi.txt），`journalctl -b -1`（若持久化）、/var/log/kern.log。据此判定卡在哪一步。
3. 若日志能坐实位置，按第一阶段方案只对那一步做一次最小复现或验证修正（例：只加载 m5、只读 BUILD_ID，不跑 lpd_afi）。**同一步卡死第二次就停**，不要反复试——每次卡死都需要人去现场断电。卡死后立刻在 README 写清，交主控。
4. 车绝对不动，不碰相机设置。用完带令牌释放锁。

## 交付
RUN/BOARDHANG0/README.md：历史核查、静态审查结论、日志判定（若有）、根因（坐实/未坐实分开写）、修正与验证、给 m6 上板的前置步骤建议。决策类格式：观测、竞争解释、区分性证据、已做/未做验证、结论、反向检验。
