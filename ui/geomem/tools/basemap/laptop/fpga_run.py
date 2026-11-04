"""便携包启动器：python.exe fpga_run.py <脚本.py> [参数...]
嵌入版 Python（python312._pth）不把脚本所在目录放进 sys.path，make_basemap.py 要 import 同目录的
basemap_tsdf / basemap_thin，所以这里补上再按 __main__ 运行。"""
import os
import runpy
import sys

if len(sys.argv) < 2:
    sys.exit("usage: python fpga_run.py <script.py> [args...]")
script = os.path.abspath(sys.argv[1])
sys.path.insert(0, os.path.dirname(script))
sys.argv = [script] + sys.argv[2:]
runpy.run_path(script, run_name="__main__")
