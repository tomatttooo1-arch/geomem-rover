# 备选：Docker 档（dev3 控制台 session.py 的 basemap 动作现用 docker）。主路径见 laptop.md 的 Windows 便携包。
#   镜像名：fpga-dev1-basemap:humble-0.23.7-tsdf-20261021
#   = 车栈同版本 RTAB-Map 0.23.7 镜像 fpga-dev1-rtabmap:humble-0.23.7-20260910（c96783dbd884）＋ open3d 0.19.0。
# 构建 context（离线）：wheels/{pip-24.2-py3-none-any.whl, open3d-0.19.0-cp310-cp310-manylinux_2_31_x86_64.whl}
#   ＋ docker_ctx/ 里的 fpga_o3d_stub.py 与 fpga_o3d_stub.pth（make_basemap 41b6953 起自己塞空壳，此钩子仅兼容旧提交）。
#   python3 -m pip download --no-deps --only-binary=:all: --python-version 3.10 --implementation cp \
#     --platform manylinux_2_31_x86_64 -d <ctx>/wheels open3d==0.19.0 pip==24.2
#   docker build --network none -f ui/geomem/tools/basemap/laptop/Dockerfile -t fpga-dev1-basemap:humble-0.23.7-tsdf-20261021 <ctx>
# 设计：--no-deps 不动 apt 的 numpy 1.21.5 / scipy 1.8.0 / PIL（旧 pointcloud 路径结果不变）；open3d 的
# visualization/ml 依赖（plotly/dash/sklearn…）不装，由 .pth 启动钩子放空占位（底图只用 open3d.core/t.geometry）。
# 入口/环境不改：照旧 `--entrypoint bash ... -c "source /opt/ros/humble/setup.bash && python3 ..."`；有 --gpus 用 GPU。
FROM fpga-dev1-rtabmap:humble-0.23.7-20260910
COPY wheels/ /tmp/wheels/
COPY fpga_o3d_stub.py fpga_o3d_stub.pth /usr/local/lib/python3.10/dist-packages/
RUN python3 /tmp/wheels/pip-24.2-py3-none-any.whl/pip install --no-index --no-deps --no-cache-dir \
      --root-user-action=ignore /tmp/wheels/open3d-0.19.0-cp310-cp310-manylinux_2_31_x86_64.whl \
 && rm -rf /tmp/wheels \
 && python3 -c "import numpy, scipy, PIL, open3d, open3d.core as c, open3d.t.geometry as g; print('numpy', numpy.__version__, 'scipy', scipy.__version__, 'PIL', PIL.__version__, 'open3d', open3d.__version__, g.VoxelBlockGrid)"
LABEL org.fpga.basemap="tsdf: open3d 0.19.0 on fpga-dev1-rtabmap:humble-0.23.7-20260910 (c96783dbd884)"
