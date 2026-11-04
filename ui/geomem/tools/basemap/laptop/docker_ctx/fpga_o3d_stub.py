# Docker 档启动钩子（dist-packages/fpga_o3d_stub.pth 里 `import fpga_o3d_stub`，python 启动即执行）。
# open3d 0.19 的 `import open3d` 会无条件导入 open3d.visualization（-> plotly/dash/flask…）与
# open3d.ml（-> scikit-learn/pandas/yaml…）。底图只用 open3d.core / open3d.t.geometry（TSDF），
# 所以缺这些包时放空的占位模块，省掉二十来个依赖；不影响 TSDF 计算。装了完整依赖时本文件什么都不做。
import importlib.util as _u
import os as _os
import sys as _sys
import types as _t

_os.environ.setdefault("OPEN3D_DISABLE_WEB_VISUALIZER", "true")


def _stub(name, need):
    if any(_u.find_spec(n) is None for n in need):
        m = _t.ModuleType(name, "FPGA portable stub: not packaged")
        m.__path__ = []
        _sys.modules.setdefault(name, m)


_stub("open3d.visualization", ("plotly", "dash"))
_stub("open3d.ml", ("sklearn", "pandas", "yaml", "tqdm", "addict"))
