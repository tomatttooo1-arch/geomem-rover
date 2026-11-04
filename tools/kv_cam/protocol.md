# kv_cam 共享内存帧环协议 v1（NN-CAM1，2026-11-04）

写方只有一个：`kv_cam`（KV260 上的相机进程）。读方任意多个：kv_server（深度进几何/ROI/融合）、dpu_rx 与 PL 前处理核驱动（彩色进识别）、前端、车屏、预览、地面标定。
读方只映射只读页，永远不阻塞写方，也永远拿不到"写了一半"的帧。字节布局以 `cam_ring.h` 为准（每个偏移都有 `static_assert` 钉死）；本文是给 NN-CAM2 和 PREP 驱动的说明。

## 1. 段

| 项 | 值 |
|---|---|
| 名字 | POSIX 共享内存 `kv_cam_v1`，即 `/dev/shm/kv_cam_v1`（`--name` 可改） |
| 创建者 | kv_cam，**以 root 服务运行**（KV 上 ubuntu 用户注销时会删它名下的 `/dev/shm` 文件，memory `reference_kv260_devshm_removeipc_deletes_ubuntu_files`） |
| 权限 | 0644：任何用户可读，只有写方可写；读方 `shm_open(O_RDONLY)` + `mmap(PROT_READ)` |
| 可选后备 | `--path FILE --phys-base ADDR`：改为 mmap 一个文件或设备（如 udmabuf / 预留物理内存），`phys_base` 写进头里，供 PL 直接按物理地址读（见第 7 节） |
| 大小 | 4096（环头）+ nslots × slot_stride；默认 8 槽 × 1,232,896 B ≈ 9.9 MB |
| 字节序 | 小端（KV260 与宿主 x86 相同） |

## 2. 环头（段内第 0 页，4096 B）

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | u64 | magic | `0x3147524d4143564b`（"KVCAMRG1"）。写方最后写它；读方见 magic 不对就当"写方还没建好"，稍后重开 |
| 8 | u32 | version | 1。布局有任何不兼容改动必须升版本（并换段名 `kv_cam_v2`） |
| 12 | u32 | header_bytes | 4096 |
| 16 | u32 | nslots | 槽数（默认 8） |
| 20 | u32 | slot_hdr_bytes | 4096 |
| 24 | u64 | slot_stride | 每槽字节数（页倍数） |
| 32 | u64 | slots_offset | 4096 |
| 40 | u64 | total_bytes | 段总长；读方用它核对 `fstat` 大小 |
| 48–67 | 5×u32 | colour_w, colour_h, colour_fourcc, colour_stride, colour_bytes | 默认 640, 480, `'YUYV'`, 1280, 614400 |
| 68–87 | 5×u32 | depth_w, depth_h, depth_fourcc, depth_stride, depth_bytes | 默认 640, 480, `'Y16 '`, 1280, 614400 |
| 88 | u32 | colour_off | 彩色载荷在槽内的偏移（4096） |
| 92 | u32 | depth_off | 深度载荷在槽内的偏移（4096 + 614400 = 618496，页对齐） |
| 96 | u64 | phys_base | 段第 0 字节的物理地址；POSIX shm 后备时为 0（不连续） |
| 104 | u32 | writer_pid | |
| 108 | u32 | writer_epoch | 写方每次挂上段就 +1（kv_cam 重启可见） |
| 112 | atomic u64 | heartbeat_ns | 写方 CLOCK_MONOTONIC，至少每 100 ms 刷一次（含重试等待期间） |
| 120 | atomic u64 | latest_seq | 最新已发布序号；0 = 还没有帧 |
| 128 | atomic u32 | futex_word | latest_seq 的低 32 位；读方在它上面 `FUTEX_WAIT`，写方每帧 `FUTEX_WAKE` |
| 132 | atomic u32 | state | 1 STARTING / 2 STREAMING / 3 RESTARTING / 4 STOPPED |
| 136 | atomic u32 | alarm_count | 每次告警 +1（先写 alarm_text 再加） |
| 140 | atomic u32 | alarm_code | 最近一次告警码（见第 6 节） |
| 144 | atomic u32 | restarts | 写方挂上以来的流重启次数 |
| 148 | atomic u32 | rate_events | 起流速率检查判"半速"的次数 |
| 152 | atomic u64 | frames_published | |
| 160 | atomic u64 | near_zero_frames | 深度近乎全 0 的帧数（LDP 嫌疑） |
| 168 | atomic u64 | incomplete_sets | 缺彩色或缺深度的帧组数 |
| 176 | 4×f32 | fx, fy, cx, cy | 彩色相机内参（深度已硬件对齐到彩色，同一套内参） |
| 192 | 8×f32 | k1..k6, p1, p2 | 彩色畸变 |
| 224 | u32 | intr_valid | 1 = 上面内参来自 SDK |
| 228 | u32 | dist_model | OBCameraDistortionModel |
| 232 | f32 | depth_scale_mm | 深度单位（Gemini 2 为 1.0 mm） |
| 236 | u32 | source_kind | 1 orbbec / 2 replay / 3 synth |
| 240 | char[128] | alarm_text | 最近一次告警文字（诊断用，可能读到半截） |
| 368 | char[128] | writer_info | 设备名、序列号、固件、连接类型 |
| 496 | u32 | nominal_fps | |

## 3. 槽（每槽 = 1 页槽头 + 彩色载荷 + 深度载荷，均页对齐）

序号 s（从 1 开始）落在第 `(s − 1) % nslots` 槽：`slot_base = base + slots_offset + ((s−1) % nslots) × slot_stride`。

槽头（4096 B）：

| 偏移 | 类型 | 字段 | 说明 |
|---|---|---|---|
| 0 | atomic u64 | seq_begin | 写方开始写序号 s 时置 s |
| 8 | atomic u64 | seq_end | 写方写完序号 s 后置 s |
| 16 | u64 | seq | 一致时 = seq_begin = seq_end |
| 24 | u64 | colour_index | 相机彩色帧号（回放时 = 回放包帧号） |
| 32 | u64 | depth_index | 相机深度帧号 |
| 40 / 48 | u64 | colour_dev_ts_us / depth_dev_ts_us | 设备时钟 |
| 56 / 64 | u64 | colour_sys_ts_us / depth_sys_ts_us | SDK 主机时间戳 |
| 72 | u64 | arrive_ns | kv_cam 收到这组帧的 CLOCK_MONOTONIC（**单时钟域**，各消费者都用它对齐位姿） |
| 80 | u64 | publish_ns | 发布前一刻的 CLOCK_MONOTONIC |
| 88 | u32 | flags | bit0 有彩色，bit1 有深度，bit2 深度近乎全 0（LDP 嫌疑），bit3 起流速率可疑，bit4 CRC 有效，bit5 深度未对齐 |
| 92 / 96 | u32 | colour_bytes / depth_bytes | 本帧有效字节（无此流时为 0） |
| 100 / 104 | u32 | crc_colour / crc_depth | zlib CRC-32（只在 `kv_cam --crc` 且 bit4 置位时有效；默认关，省 A53） |
| 108 | u32 | epoch | 发布时的 writer_epoch |
| 112 | f32 | depth_scale_mm | |
| 116 | u32 | depth_valid_permille | 每 16 个像素抽 1 个算的非零深度比例（‰） |

载荷：

- 彩色（槽内 `colour_off`）：YUYV 4:2:2，每行 1280 B，像素对 `Y0 U Y1 V`；SDK 原样交出，A53 不做色彩转换（转换在 PL 前处理核里，主控定的方向 C）。`--colour-format rgb` 时为 RGB888、每行 1920 B（非默认）。
- 深度（槽内 `depth_off`）：u16 小端，单位 `depth_scale_mm`，0 = 无效。默认是硬件对齐到彩色的 640×480（与彩色逐像素对应）。若板上没有 YUYV 的硬件对齐档（第 8 节待查），kv_cam 告警后退到未对齐的 640×400：数据占前 400 行、后 80 行置 0、flags bit5 置位——消费者必须看这一位。
- 某一流缺帧时，对应 bit 不置位，载荷是旧内容，**不得使用**。

## 4. 写方步骤（kv_cam，`RingWriter`）

1. `s = latest_seq + 1`；槽 = `(s−1) % nslots`。
2. `seq_begin = s`（relaxed），随后 **release 栅栏**——保证 seq_begin 先于任何载荷字节可见。
3. 写载荷与槽头元数据。
4. `seq_end = s`（release）。
5. `latest_seq = s`（release）；`futex_word = (u32)s`；`FUTEX_WAKE(futex_word, INT_MAX)`。

写方崩在第 3 步：该槽 `seq_begin = s ≠ seq_end`，latest 没前进，读方永远不会接受 s；重启后写方从同一个 s 重写这一槽。

## 5. 读方步骤（`RingReader` / `libcamring.so` / `cam_ring.py`）

拷贝读（推荐，`read_copy`）：

1. `e = seq_end`（acquire）。`e < s` → not_yet（还没写完）；`e > s` → overwritten（已被新帧覆盖）。
2. 拷出槽头与载荷。
3. **acquire 栅栏**，再读 `seq_begin`：≠ s → overwritten（拷贝期间被覆盖，丢弃这份拷贝）。
4. 槽头 `seq` ≠ s → corrupt（写方故障；不应出现）。

零拷贝读（PREP 驱动、kv_server 想省一次 614 KB 拷贝时）：`state(s) == ok` → 直接用 `colour_ptr(s)`/`depth_ptr(s)` → 用完调 `still_valid(s)`，false 就把这一帧的结果作废。槽数 8、30 帧/秒时，一帧在环里能安全停留约 7 × 33 ms ≈ 233 ms；处理比这更慢的消费者必须拷贝。

等待：`wait_newer(after, timeout_ms)` 在 futex_word 上睡（只读映射上 FUTEX_WAIT 可用，宿主内核 5.4 已实测；KV 为 5.15），最多每 100 ms 醒一次复查。

两种消费方式：
- **只要最新帧**（kv_server、dpu_rx、车屏）：`s = latest()`，读它；落后就跳过。
- **逐帧不漏**（录制、前端若需要）：`s = 上一帧 + 1`；遇 overwritten 就跳到 `latest − nslots + 2` 并计数。

写方重启：同几何的段被原样复用（序号接着涨、epoch +1），读方不必重开。几何变了（换格式/槽数）写方会 unlink 后重建；读方的判断是 `stale()`（心跳超过 2 s 没动）→ 关掉重开。

## 6. 健康与告警

kv_cam 只告警、自动重启，**不拒起、不停车**（FPGA 规矩）。告警同时出现在 stderr（`KVCAM ALARM code=N …`）、`--alarm-log` 的 JSONL、环头 alarm_count/alarm_code/alarm_text。

| 码 | 含义 | 动作 |
|---|---|---|
| 1 | 起流失败（找不到设备、SDK 报错） | 等 `--retry-gap-s`（10 s）重试，无上限；每 3 次失败重建 SDK Context（重枚举后能重新找到相机） |
| 2 | 起流后 `--start-wait-s`（20 s）内深度帧不足 `--start-depth-frames`（30） | 停流、等待、重试（Gemini 2 深度流约 2/3 概率起不来的老问题） |
| 3 | 起流速率检查判半速（深度间隔中位数 < 32.8 ms 或彩色 p25 > 50 ms，规则同 `tools/car_run/depth_probe.py`，只在 30 帧/秒时做） | 重启，最多 `--rate-max-restarts`（5）次 |
| 4 | 半速重启 5 次仍半速 | 继续出流，槽 flags bit3 置位 |
| 5 | `--stall-s`（3 s）没有帧组 | 重启流 |
| 6 | 深度近乎全 0 连续 `--ldp-frames`（30）帧（LDP 嫌疑） | 只告警、槽 flags bit2；`--ldp-restart` 才重启（默认关：近距物体造成的激光保护重启也消不掉） |
| 7–11 | 档位不符、非 USB3、无硬件对齐档、帧尺寸不符、源错误 | 只告警 |
| 100 | 信息（LDP 解除、速率检查样本不足等） | — |

## 7. 给 PL 前处理核驱动的说明（DMA）

PL 的 m_axi 按物理地址读 DDR，而 `/dev/shm` 的页物理上不连续，所以默认后备下 PREP 驱动要 **把一帧 YUYV（614,400 B）拷进它自己的连续缓冲**（XRT BO / udmabuf），再刷 cache、启动核。A53 上一次 614 KB 拷贝约 0.3–0.6 ms（推算），30 帧/秒约占 1–2 % 个核。
省掉这次拷贝的办法：kv_cam 用 `--path /dev/udmabufN --phys-base <物理地址>` 把整个环放进连续缓冲，PREP 直接按 `phys_base + slots_offset + 槽号 × slot_stride + colour_off` 读；代价是要在 KV 上加 udmabuf（或预留内存）配置，且读前要做 cache 维护（PL 读口不走一致性端口时）。默认是前者；改后者要主控定（见 NN-CAM1 README 待定事项）。

## 8. 板上还没验证的

- YUYV 640×480@30 下有没有 640×400 Y16 的硬件对齐深度档：CAMKV0 只列了 RGB 彩色档的硬件对齐表。`kv_cam --list` 会打印 YUYV/RGB × 30/15 的对齐表，上板第一步跑它。
- 真相机下的起流重试、半速判定、LDP 抽样阈值都沿用 Nano 车上的规则与数值，A53 上的表现要板上看。

## 9. 宿主测试（`run_ring_tests.py`）

- T1 逐字节一致：回放包（rec30 录像 90 帧，打成 640×480 YUYV + Y16）30 帧/秒发两遍，C++ 读方（memcmp）与 Python 读方（numpy）同时读，每帧按槽头帧号与回放包逐字节比对。阳性对照：写方从第 45 帧起让深度换成下一帧内容（序号错位），两个读方都必须抓到。
- T2 覆盖安全：合成源 1000 帧/秒灌 8 槽，读方在彩色与深度拷贝之间随机睡 0–12 ms；带序号锁复查时不得交出任何错字节。阳性对照：关掉复查（`--no-seqlock`），必须出现撕裂帧。
- 结果见 `runs/nationals-prelim-20261102/NN-CAM1/README.md`。
