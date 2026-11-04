便携包 fpga-basemap-win64 的组装（宿主 Linux 上做，产物给 Windows 用）：
1. python-3.12.10-embed-amd64.zip（python.org）解到 fpga-basemap-win64/python/
2. pip download --no-deps --only-binary=:all: --python-version 3.12 --implementation cp --platform win_amd64
     open3d==0.19.0 numpy==2.3.5 scipy==1.18.1 pillow==13.0.0   -> 逐个 unzip 到 python/Lib/site-packages/
3. msvc_runtime-14.44.35112-cp312-cp312-win_amd64.whl 里取 vcomp140.dll、msvcp140.dll 放 python/（open3d.pyd 依赖 VCOMP140）
4. python/python312._pth 改为三行：python312.zip / . / Lib\site-packages
5. 本目录 fpga_run.py、make_basemap.bat 放包根；VERSIONS.txt 记各文件 sha256；zip -r -9
