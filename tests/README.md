# tests/ —— 主机侧测试

一条硬规则：**板上不可测的逻辑，必须有主机侧单测。**

板子要到 9 月下旬才发，而且只有一块、三个人抢。凡是不上板也能验证的东西
（算法正确性、数据结构、协议解析、HLS 的 C 仿真），都要能在这台服务器上一条命令跑完。
上板才能验的部分（时序、资源、实测时延功耗），单独在 `runs/` 里留记录。

## 怎么跑

在仓库根目录跑全量：

    PYTHONPATH=. python3 -m unittest discover -s tests

`PYTHONPATH=.` 不能省：`tests/`、`host/`、`tools/` 都没有 `__init__.py`，仓库也不作为包安装。
**点号模块式（`python3 -m unittest tests.test_xxx`）在本机跑不起来**（同名路径被遮蔽），
一律用上面的 discover 形式；单个文件加 `-p`：

    PYTHONPATH=. python3 -m unittest discover -s tests -p test_localization_gate.py

全量约 215 s，纯 Python（numpy/scipy 见 `requirements-replay.txt`），不需要 ROS 2、GPU 或板子。

## 这些测试能证明什么、不能证明什么

- 纯逻辑是**真的被执行**的：门禁状态机与租约、`map→odom` hold、里程计适配器的位姿/twist/协方差
  代数（带已知答案与阴性对照）、map bundle 的哈希与清单校验。
- **ROS 节点的运行时行为没有被执行**——本机没有 ROS 2。`tests/test_odom_base_adapter.py`
  的 `NodeWiringTests` 这类测试用源码检查（`inspect.getsource` + 正则）钉住接线约定
  （每条消息都走被测纯核、只在纯核返回非空时发布、静态外参只从 TRANSIENT_LOCAL 订阅读一次）。
  它们挡得住接线被改坏，但**不等于**节点在真实 DDS 图里跑通：那只能由容器内的端到端排练证明。
- 依赖冻结实验产物的测试（例如 `runs/rtabmap-reload-coordinate-20260912/` 里的 run 根目录与
  map bundle）用 `skipUnless` 守着；旁边没有那些目录时会跳过，跳过数因此随环境变化。

## 容器冒烟：`tools/rtabmap_container_smoke.sh`

主机没有 ROS 2，所以凡是 `import rclpy` / `tf2_ros` 的东西只能在容器里跑。整场排练太贵，
本仓库以**只读**挂到 `/checkout`（`--env PYTHONPATH=/checkout`），`--network none`、`--rm`、

    tools/rtabmap_container_smoke.sh                      # 默认冒烟，真跑
    tools/rtabmap_container_smoke.sh --dry-run            # 只打印 docker argv，不碰 docker
    tools/rtabmap_container_smoke.sh --rw DIR -- CMD ...  # 唯一一个可写目录挂到 /rosresults

不给命令时默认跑两件事：容器里 `import rclpy, tf2_ros, scipy`，以及用容器的 Python
`py_compile` 三个只能在容器里跑的节点源码（门 / 里程计适配器 / TF 录制器）。
`.pyc` 经 `PYTHONPYCACHEPREFIX=/tmp/pycache` 写进容器自己的可写层——只读挂载是有意的，不放松。

它**不是**排练：什么都不记录、什么都不判，绿了只说明那条命令在镜像里退出码为 0。
`tests/test_rtabmap_container_smoke.py` 只检查 `--dry-run` 打出来的 argv（无网络、只读挂载、
