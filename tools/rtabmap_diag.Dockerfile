# rtabmap_diag.Dockerfile —— 在 RTAB-Map 运行镜像上加 heaptrack，用于内存诊断。
#
# 这层只装 heaptrack：先快照包列表与 /opt/ros/humble/lib 的逐文件 sha256，用 `apt-get -s install`
# 预演并断言没有任何已装包被升级/替换（`! grep -E '^Inst [^ ]+ \['`），装完再校验 ROS 库逐字节未变。
# 目的是让诊断镜像与被测镜像在 ROS 侧保持逐字节同一，heaptrack 的读数才归因得了。
#
# 构建命令未留记录。翻遍 docs/、progress/ 与 runs/（含冻结 run 根目录）都没有任何 `docker build`
# 行或对本文件的引用，所以这里不编一条出来。能从产物侧核实的只有：
#   - 本文件的 RUN 层与本机镜像 `fpga-dev1-rtabmap:diagnosis-20260911`
#     （`sha256:690ca7b599a7…`，创建于 2026-09-11T15:00:36Z）的顶层 `CREATED BY` 在归一化续行空白后逐字相同；
#   - 其基础镜像即下面的 FROM：`fpga-dev1-rtabmap:humble-0.23.7-20260910` = `sha256:c96783dbd884…`；
#   - 该 tag 被 `runs/rtabmap-recovery-analysis-20260912/` 的内存诊断记录引用。
# 即：镜像是 2026-09-11 由本文件构建的，但 `docker build` 的具体参数（context / -f / -t）没有记录。
# 要重建请自己写命令并把它记进 run 目录，别把这段注释当作复现依据。
#
FROM fpga-dev1-rtabmap:humble-0.23.7-20260910
RUN dpkg-query -W -f='${Package} ${Version}\n' > /tmp/packages-before.txt \
 && find /opt/ros/humble/lib -type f -exec sha256sum '{}' + > /tmp/ros-libraries-before.sha256 \
 && apt-get update \
 && apt-get -s install --no-install-recommends heaptrack > /tmp/install-plan.txt \
 && ! grep -E '^Inst [^ ]+ \[' /tmp/install-plan.txt \
 && apt-get install -y --no-install-recommends heaptrack \
 && sha256sum --quiet -c /tmp/ros-libraries-before.sha256 \
 && dpkg-query -W -f='${Package} ${Version}\n' > /tmp/packages-after.txt
