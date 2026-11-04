"""Host dry run only (never deployed: make_package.sh excludes this directory). With FPGA_FAKE_TRT=1 and
this directory first on PYTHONPATH, every python process (board_run, the inference service, the smoke
client) gets tools.nano_detector.nano_detector patched with an onnxruntime stand-in for TensorRT.
The patched module carries FAKE_TRT = True; board_run records it and compare_board refuses to call
such a record a real board result."""
import os

if os.environ.get("FPGA_FAKE_TRT") == "1":
    import fake_trt
    fake_trt.install()
