
只管「建图库 `.db` → `basemap-<sha12>/`」这一步；拷库、起桥、`--basemap-watch` 等沿用控制台现有流程。
10-21 起 `make_basemap.py` 默认 `--method tsdf --mount-fix auto`，位姿直接读库（Admin 表），**不再需要
rtabmap / ROS / Docker**，只要 Python 3.12 + open3d 0.19.0 + numpy + scipy + Pillow。

## 主路径：Windows 便携包（不装 Docker、不要管理员、不联网）

  sha256 `200d87f1…`）。内容：python.org 官方 Python 3.12.10 embeddable、open3d 0.19.0 / numpy 2.3.5 /
  scipy 1.18.1 / Pillow 13.0.0 的 win_amd64 轮子（`--no-deps`）、VC 运行库 `vcomp140.dll` `msvcp140.dll`、
  启动器 `fpga_run.py`、`make_basemap.bat`。版本与哈希见包内 `VERSIONS.txt`。源文件在本目录 `laptop/`。
- 解压到纯英文短路径，例如仓库旁边：`C:\fpga\runtime\fpga-basemap-win64\`。
- 手动跑（cmd 或 PowerShell）：
  ```
  set FPGA_ROOT=C:\fpga\repo
  C:\fpga\runtime\fpga-basemap-win64\make_basemap.bat C:\fpga\session-data\<库>.db C:\fpga\session-data\basemaps
  ```
  不设 `FPGA_ROOT` 时默认包的上一级就是仓库根。产物 `...\basemaps\basemap-<sha12>\{basemap.json,cells.bin}`。
  其余参数原样转给 make_basemap（如 `--threads 12`、`--force`）。
- **必须经 `fpga_run.py` 启动**：嵌入版 Python 不把脚本目录放进 `sys.path`，直接
  `python.exe make_basemap.py` 会报 `No module named 'basemap_tsdf'`（宿主用 `-P` 模拟已复现）。
- Windows 的 open3d 轮子只有 CPU 版（轮内只有 `open3d/cpu/pybind*.pyd`），笔记本上就是 CPU 跑。


`basemap` 动作：有便携包就用，没有再走 docker。注意 docker 档要么换新镜像，要么加 `--method pointcloud`

```diff
         db = work / Path(c["database"]).name
-        cmd = ["docker", "run", ...
+        pkg = Path(c.get("basemap_portable") or ROOT.parent / "runtime" / "fpga-basemap-win64")
+        if (pkg / "python" / "python.exe").exists():
+            cmd = [pkg / "python" / "python.exe", pkg / "fpga_run.py",
+                   ROOT / "ui/geomem/tools/basemap/make_basemap.py",
+                   "--db", db, "--out-dir", work / "basemaps"]
+        else:
+            cmd = ["docker", "run", ...                     # 原命令不变
+                   + (["--gpus", "all"] if c.get("docker_gpus") else []) ...]
```
（`docker_gpus` 可选；`--gpus all` 要插在镜像名之前。）`session.example.json` 可加 `"basemap_portable": ""`。

## 备选：Docker 档

  ＋ open3d 0.19.0，numpy/scipy 不升级）。只把 `session.json` 的 `docker_image` 换成它，原命令一字不改即可跑。
- GPU（可选）：Windows 要 Docker Desktop（WSL 2 后端）＋新版 NVIDIA 驱动，`docker run --rm --gpus all <镜像> nvidia-smi`
  能看到显卡即可；原生 Linux 要装 nvidia-container-toolkit 并 `nvidia-ctk runtime configure --runtime=docker` 后重启 docker。
  容器每次 `--rm`，open3d CUDA 内核首轮 JIT 编译约 30–45 s 每次都要付，所以 GPU 档并不比 CPU 快（下表）。

## 实测（宿主 10-16 库 `20261015_190638.db`，874 118 格，宿主负载 16–20/48 核，8 线程）

| 方式 | 端到端 | cells.bin sha256 |
|---|---|---|
| Py3.12 只装四个包（Linux 轮，模拟便携包；源码只读、`-P`），分支 `41b6953`，CPU | 32.5 / 47.8 s，峰值 1.8 GB | `916fea5a…` |
| 同上 `--gpus device=0` | 75 s（自报 41.5 s，含 CUDA 首轮 JIT） | GPU 与 CPU 不逐字节同（格数同） |

笔记本 CPU 估计 40–90 s（**推断，Windows 上首次需实跑一次并记下耗时**；首次加载 150 MB 的 open3d.pyd 可能被杀毒软件多扫十几秒）。

## 常见失败

- `No module named 'basemap_tsdf'`：没经 `fpga_run.py` 启动。
- `找不到 ...make_basemap.py`：`FPGA_ROOT` 没指到仓库根，或仓库副本不是含 tsdf 的版本。
- `DLL load failed`（open3d）：包不完整或解压到了很深/含中文的路径；重解压。仍不行装一次微软 VC++ 2015–2022 x64 运行库。
  Windows「N」版缺 Media Foundation（`MF.dll`）时也会这样，装「媒体功能包」。
- 内存不足：峰值约 2 GB；关掉大程序或 `--threads 4`。
- 包目录已存在：同一库已生成过，加 `--force` 覆盖或直接用已有的。
- Docker 档 `--method tsdf 需要 open3d`：还在用旧镜像，换新镜像或加 `--method pointcloud`。
