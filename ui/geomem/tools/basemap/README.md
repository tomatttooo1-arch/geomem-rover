# 展台底图包工具（10-17；10-21 起默认 TSDF 融合）

把一份 RTAB-Map 地图库变成展台页面第三视角的「F 风格」底图：1 cm 真彩体素格。包格式与桥怎么托管见
`ui/geomem/docs/PROTOCOL_V2_DESIGN.md` §2.10；桥只读成品包，仍是纯标准库。

```bash
python ui/geomem/tools/basemap/make_basemap.py --db <地图库.db> --out-dir <输出目录>
#   -> <输出目录>/basemap-<库 sha256 前 12 位>/{basemap.json, cells.bin}
```

## 比赛现场：服务器上一条命令（10-21 起推荐）


```bash
# 第一趟收车、memory_bind.json 为 bound 之后；换趟重建桥连接后再跑一次（命中缓存，只复制）
bash ui/geomem/tools/basemap/server_basemap.sh --run <Nano 上第一趟运行目录>
```

详见脚本头注释与 `docs/team/BASEMAP_TSDF_HANDOVER_20261021.md` §3。笔记本直连时用 Windows 便携包（`laptop.md`）。

## 10-21：`--method tsdf`（默认）与 `--mount-fix`

owner 10-21 批：底图从「逐帧点云直接叠加」改成 **TSDF 加权融合**，并修相机外参缺 roll。原型与指标见

### CLI（部署脚本按这个写，别改名）

| 参数 | 默认 | 说明 |
|---|---|---|
| `--method tsdf\|pointcloud` | `tsdf`（给了 `--ply` 时为 `pointcloud`） | `pointcloud` = 10-17 的旧路径，输出与 main `0606d11` 的 cells.bin 逐字节相同 |
| `--mount-fix auto\|off` | `auto` | 仅 tsdf：逐关键帧拟合地面，求相机安装 roll/pitch 修正（各帧坡度取中位数），左乘到各节点 local_transform 后再融合；修正量 > 3° 或能拟合的帧 < 10 个就不修并在 json `mount_fix.warning` 报警 |
| `--poses-from auto\|db\|export` | `auto` | 仅 tsdf：`db` = 库 `Admin.opt_poses/opt_ids` 里 rtabmap 关图时存的全局优化位姿（与 `rtabmap-export --opt 0 --poses` 同源，float32 精度），**不需要 rtabmap/docker/ROS**；`export` = 走 `--export docker\|native` 只导位姿（`--opt 0 --poses --poses_format 11`）；`auto` = 库里有就 db，否则 export。也可 `--poses <格式 11 文件>` 直接给 |
| `--thin-fill on\|off` | `off` | 仅 tsdf：细结构补回（`basemap_thin.py`，拷自 G_thin 原型）：融合时累加各关键帧剔点前原始深度的 1 cm 支持格，分格后把离 TSDF 格 > 3 格、竖直细长的块（约 2 cm 椅腿/桌腿）补回；收益中等偏小，7° 新库未验 |
| `--device auto\|cuda\|cpu` | `auto` | open3d 有 CUDA 就 GPU，否则 CPU（Windows pip 轮无 CUDA，CPU 是主路径） |
| `--threads N` | 8 | 写进 `OMP_NUM_THREADS`（不固定时负载高下 integrate 会慢 50 倍，A §2.3） |
| `--min-points` | pointcloud 2 / tsdf 1 | tsdf 下 SOR 关闭（`sor_k` 记 0） |

其余参数（`--cell --crop --z-min --export --rtabmap-export --image --work-dir --keep-export --force --workers`）照旧；
tsdf 的体素 = `--cell`。tsdf 固定参数（`basemap_tsdf.TSDF_DEFAULTS`，写进 json `tsdf.params`）：截断 8 体素（8 cm）、
深度截断 3 m、二阶差分剔飞点（`curv`）、权重 ≥ 2、融合系平移半体素（体素中心 = 格心）、过零边两端体素都出格（双层，无透光缝）、
块 16³、预分配 16000 块（不够时 open3d 自动扩）。

### 依赖（tsdf）

- Python **3.8–3.12**（open3d 0.19 没有 3.13 轮）；`numpy`、`scipy`（裁剪用 k-d 树，同 pointcloud）、`Pillow`（JPEG/PNG 解码）、
  `open3d==0.19.0`。最小装法：`pip install --no-deps open3d==0.19.0` + `pip install numpy scipy pillow`（宿主 Linux 实测 import 与融合可用，见 E_impl 报告）。
- RVL 深度解码是纯 numpy（`basemap_tsdf.rvl_decode`，与 C 版逐像素相同，约 30 ms/帧），不编译、不往源码目录写东西；只读源码目录、无网络可跑。
- 没装 open3d：直接报错并提示 `--method pointcloud`（不自动回落：两条路径画质差别大，且 pointcloud 要 rtabmap-export，静默回落会让人以为用的是新底图）。
- `--poses-from db` 时整条路径不调用任何 rtabmap 可执行文件；pointcloud 路径的依赖照旧（numpy、scipy ≥ 1.6、rtabmap-export）。

### basemap.json 新增字段（已有字段不改不删）

`method`（`tsdf`/`pointcloud`）；tsdf 另有 `tsdf{open3d, device, omp_num_threads, voxel_m, params, poses_source(db|export|file),
frames_*, active_blocks, surface_points}`、`mount_fix{mode, applied, frames_fitted, slope_before_deg, slope_after_deg,
mount_rpy_before_deg, mount_rpy_after_deg, delta_roll_deg, delta_pitch_deg, rotation_deg, warning?}`；`export.mode` 多两个值
`db`（位姿取自库）与 `poses`（`--poses` 给的文件）；`timing_s` 多 `poses_db/tsdf_read_decode/tsdf_preprocess/mount_fix/tsdf_integrate/tsdf_extract`。

### 实测（10-21，10-16 库 `20261015_190638.db`，宿主负载约 20、内存紧，计时偏慢）

| 配置 | 格数 | C 口径地面 / 表面厚度中位 | <50 格碎块 | 进程墙钟（CPU 8 线程，含读库/位姿/写包） |
|---|---|---|---|---|
| main pointcloud（现行） | 1,289,778 | 12.3 / 9.3 mm | 1.64 % | 约 120–146 s（含 docker 导出） |
| tsdf，`--mount-fix off` | 891,265 | 7.5 / 7.3 mm | 0.45 % | 49 s（宿主抖动；同代码 GPU 档 891,261 格） |
| **tsdf，默认（auto）** | **874,118** | **6.8 / 6.6 mm** | **0.41 %** | **26 s** |
| 默认 + `--thin-fill on` | 881,281（补回 7,163） | 6.9 / — | 0.45 % | 26 s |

mount-fix auto 在该库拟合 172/175 帧：roll +0.838°、pitch −0.475°（C 轴离线拟合 +0.863° / −0.40°），地面坡度均值
(−0.47°, −0.74°) → (0.01°, 0.01°)。GPU 档同库格数相同（874,118），逐字节只保证同设备。

### 挂接点

`make_basemap.SURFACE_POSTPROCESS`：`f(xyz, rgb, tsdf_info) -> (xyz, rgb)`，作用在 TSDF 表面点上、分格（`build_cells`）之前，
给「细结构保留」之类后处理用；默认 None。

## 10-17 版（`--method pointcloud`）

## 步骤

| 步 | 做什么 | 与效果稿原型（`render/hq.py`）的差别 |
|---|---|---|
| 0 | 流式 sha256 源库（包的身份＝库的身份） | 新增 |
| 1 | docker 里 `rtabmap-export --opt 0 --cloud --poses --decimation 1 --max_range 3.0 --voxel 0.004 --edge_bleeding_error 0.02`（10-17 起 0.004，原 0.008，供 1 cm 格取足点），库目录只读挂载 | 参数相同；`--opt 0` 与 `tools/car_run/rtabmap_car.sh` 给 Nav2 导 `--map` 的默认相同（全局优化 map 系） |
| 2 | 读 PLY 的 vertex 元素；按 (x,y,z,r,g,b) 排成规范顺序 | 新增排序：之后每步与导出点的先后无关 |
| 3 | 去掉非有限点与 z ≤ −0.15 m 的点 | 相同 |
| 4 | 只留建图轨迹 2 m 以内（xy）的点 | 原型逐点暴力广播（慢的根源），这里 k-d 树，结果相同 |
| 5 | 统计离群：到 8 个近邻的平均距离 < 均值 + 1.5σ | 相同 |
| 6 | 1 cm 格（map 系，10-17 起；原 2 cm、≥3 点），格内 ≥ 2 点才留，颜色取平均并四舍五入成 uint8 | 原型在 three 系分格、存 float 颜色；格集合与格心相同，只差恰在格边界上的点 |

`cells.bin`：N×3 int16 LE 格索引（格心 = (索引 + 0.5) × cell_m（默认 0.01 m，见 basemap.json），map 系 x 前 y 左 z 上），紧跟 N×3 uint8 RGB；按格索引排序。
`basemap.json`：`format: geomem-basemap`、`version: 1`、`source_db{name,sha256,bytes}`、参数、各步点数、包围盒、
建图轨迹 `[[x,y,yaw]…]`、导出命令与镜像、`files.cells{bytes,sha256,count,layout}`、生成脚本 git 提交、
`generated_at`、`timing_s`。

## 实测（2026-10-17，样例库 `20261015_190638.db`，sha256 `fae1c3bb…52575e6`；宿主负载 23–34）

- 一条命令 40.3 s / 44.4 s（导出 30.9 / 32.7 s，其余 ≤ 12 s）；格 259 494 个，`cells.bin` 2 335 446 B。
- 确定性：两次 `cells.bin` 逐字节相同（sha256 `d4c5ed42…`），`basemap.json` 去掉 `generated_at`/`timing_s` 后逐字节相同；
  两次导出的 PLY 与轨迹也逐字节相同，且与效果稿当时的导出（`export/hq_cloud.ply`）逐字节相同；python3.8 与 3.13 输出相同。
- 已知答案对照：与原型 `fv.f32`（259 495 格）比，259 494 格相同、原型多 1 格（边界点）；颜色除 36 格外都在四舍五入误差内（≤ 0.5/255），
  那 36 格最大差 5/255。
- 源库只读挂载导出前后 sha256 不变。



## 10-17 改默认为 1 cm（owner）
- 原因：owner 在 2 / 1.5 / 1 cm 三版对比后选 1 cm；0.5 cm 不做（约 4 倍格数、按比例估 4090 每帧 > 30 ms，且 Gemini 2 深度噪声与位姿误差为厘米级，格再细只放大噪声）。
- 样例库实测（服务器 4090，展台页）：2 cm 26 万格 2.7 ms/帧；1.5 cm 52 万格 4.9 ms；1 cm 129 万格 8.1 ms（含阴影重算 11.8 ms）。
- 旧 2 cm 包要复现：`--cell 0.02 --min-points 3`，并用 0.008 的导出（`--ply` 指定）。
- 默认参数一条命令端到端（含 docker 导出 48.8 s）实测 120.5 s（宿主未独占）：**超过 2 cm 时定的「≤ 60 s」目标（K1 当时按 2 cm 判过）**；cells.bin sha256 `9549886c…` 与 `--ply` 复用导出时逐字节相同。

## 10-18：在车上或笔记本上生成、运行中加载（owner）

比赛单趟流程里底图要在建图结束后才有，现场未必连得上服务器，所以工具可以不用 docker 外壳、直接调本机 `rtabmap-export`
（`--export native`，参数与 docker 档相同，另显式 `--poses_format 11`）。k-d 树查询改为每 100 万点一块（结果逐字节相同，峰值内存小）。
依赖：Python 3.8+、numpy、scipy ≥ 1.6、RTAB-Map 的 `rtabmap-export`（0.23.7 实测）。结果与判据：`ui/geomem/docs/STAGE2_KIOSK_ODOM_RESULTS.md`。

**Orin Nano（车栈镜像里，推荐；建图后 `car_down` 之后跑）**——镜像 `fpga-ros:humble-nav-orbbec-scipy-20261017` 有 RTAB-Map 0.23.7 与 apt numpy/scipy：

```bash
docker run --rm --network none -v <仓库>:/wt:ro -v /data/fpga/rtabmap:/in:ro -v <输出目录>:/out \
  --entrypoint bash fpga-ros:humble-nav-orbbec-scipy-20261017 -c \
  'source /opt/ros/humble/setup.bash && python3 /wt/ui/geomem/tools/basemap/make_basemap.py --db /in/<库>.db --out-dir /out --export native'
# -> <输出目录>/basemap-<库 sha 前 12 位>/ ；把这个目录拷到笔记本桥的 --basemap-watch 目录
```
宿主代理趟（同版本软件的 x86 容器、6 核配额）162 s、最大 RSS 2.24 GB，cells.bin 与服务器 docker 档逐字节相同；
**Nano 上约 4–7 分钟、峰值约 2–2.5 GB（推断，未上板）**。

容器里自带 Python/numpy/scipy，笔记本不用装 Python 依赖；未在笔记本实测。② 原生：装 RTAB-Map 的 Windows 版后
`python make_basemap.py --db 库.db --out-dir 输出 --export native --rtabmap-export "C:\...\rtabmap-export.exe"`（还要 pip 装 numpy、scipy）；
Windows 版 RTAB-Map 的版本与导出结果**未验证**，不保证与 0.23.7 逐字节相同。

**运行中加载**：桥加 `--basemap-watch <目录>`（可与 `--basemap` 同用），每 2 s 看一次；新出现的 `basemap-<sha12>/` 核过就加载，
页面不用刷新。拷包最好先拷到同目录的别的名字再改名；直接拷也行（半截时核不过，文件变完会重试）。

## 比赛现场：owner 笔记本生成（10-18 owner 定；**未实测**）

主路径在 owner 的 Windows 笔记本上用 Docker Desktop 跑；Nano 收车（`car_down`）后在车栈镜像里跑（见上一节）作备选。

1. **装 Docker Desktop**（Windows，WSL 2 后端），启动后在 PowerShell 里 `docker version` 能看到 Server 段。
   拷到笔记本后 `docker load -i rtabmap.tgz`；`docker images` 能看到同名同标签。
3. **拷库**：把仓库副本（至少 `ui/geomem/tools/basemap/`）放到如 `C:\fpga\repo`，建图结束后把车上的 RTAB-Map 库
   （`/data/fpga/rtabmap/<库>.db`）拷到如 `C:\fpga\db\`。
4. **跑命令**（PowerShell，一行）：
   ```powershell
   ```
   产物在 `C:\fpga\out\basemap-<库 sha 前 12 位>\`（`basemap.json`、`cells.bin`）。
5. **交给桥**：把整个 `basemap-<sha12>\` 目录拷进桥 `--basemap-watch` 指向的目录（先拷成别名再改名最稳）；桥每 2 s 扫一次，
   核过就加载，页面不用刷新。

以上步骤与命令未在笔记本上实测（镜像里的入口与 Python 依赖照服务器档推断）；赛前须在笔记本上用样例库跑通一次。
