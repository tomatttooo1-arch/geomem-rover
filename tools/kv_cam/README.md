# kv_cam：KV260 相机进程（NN-CAM1，2026-11-04）

Orbbec Gemini 2 直插 KV260 USB3，kv_cam 用 OrbbecSDK v2.9.3 取 YUYV 彩色与硬件对齐深度，写进共享内存帧环 `/dev/shm/kv_cam_v1`，
供 kv_server、dpu_rx / PL 前处理核驱动、前端、车屏、预览、地面标定本机取帧。协议见 `protocol.md`。

| 文件 | 作用 |
|---|---|
| `cam_ring.h` | 协议本体：布局（`static_assert` 钉偏移）、写方 `RingWriter`、读方 `RingReader`（序号锁、futex 等待） |
| `kv_cam.cpp` | 进程：帧源（orbbec / replay / synth）、起流重试、半速检查、断流重启、LDP 告警、自检 `--selftest`、故障注入（测试用） |
| `kv_cam_orbbec.cpp` | OrbbecSDK 帧源（`-DKVCAM_ORBBEC` 才编进去；宿主构建只有桩）；`--list`、`--dump-props`、`--set-prop` |
| `kv_cam_common.h` | 帧源接口、回放包 KVCAMPK1 格式、确定性合成帧 |
| `kv_cam_read.cpp` | 参考读方 + 校验器（逐字节比对回放包或合成帧） |
| `cam_ring_c.cpp`、`cam_ring.py` | C 接口与 Python 读方（ctypes，序号锁在 C 里做） |
| `make_replay_pack.py` | 录像目录（frames.jsonl + PNG）→ 回放包 |
| `run_ring_tests.py` | 宿主两项关键测试与阳性对照 |
| `kv_cam.service` | systemd 模板（未安装） |

构建：`make host`（宿主，只有回放/合成源）；`make arm`（aarch64 交叉编译，链 OrbbecSDK，RUNPATH `$ORIGIN/../lib`，部署布局同 camkv0 包）。

默认行为（不加任何开关）：彩色 640×480 YUYV 30 帧/秒、深度 640×400 Y16 30 帧/秒硬件对齐（出 640×480）、帧同步开、聚合 ANY（与 CAMKV0 实测配置一致，只把 RGB 换成 YUYV）、8 槽、不算 CRC、不绑核。
板上用法：`kv_cam --cpu 1 --nice -5 --alarm-log /var/log/kv_cam/alarms.jsonl`；看环：`kv_cam_read --info`。
