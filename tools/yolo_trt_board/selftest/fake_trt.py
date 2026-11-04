"""onnxruntime stand-in for nano_detector.TRTModel/Detector (host dry run of the line-3 board scripts).
Same interface as the board classes that YoloSegDetector(backend="trt") and board_run touch:
Detector(engine, W, H, conf, iou, max_det) with .m (.size, .run(), .engine IO listing), .lb,
.out_det/.out_proto, .preprocess(rgb), .postprocess(masks=False). The ONNX comes from FPGA_FAKE_ONNX;
the "engine" path argument is ignored except for being an existing file."""
import os


def install():
    from tools.nano_detector import nano_detector as nd
    from perception.fpga_replay import yolo_detector as yd
    import numpy as np
    import onnxruntime as ort
    import cv2

    onnx_path = os.environ["FPGA_FAKE_ONNX"]

    class FakeEngine:
        def __init__(self, session):
            self.io = [(i.name, i.shape, "DataType.FLOAT", "TensorIOMode.INPUT") for i in session.get_inputs()] + \
                      [(o.name, o.shape, "DataType.FLOAT", "TensorIOMode.OUTPUT") for o in session.get_outputs()]
            self.num_io_tensors = len(self.io)

        def get_tensor_name(self, i):
            return self.io[i][0]

        def _row(self, n):
            return next(r for r in self.io if r[0] == n)

        def get_tensor_shape(self, n):
            return self._row(n)[1]

        def get_tensor_dtype(self, n):
            return self._row(n)[2]

        def get_tensor_mode(self, n):
            return self._row(n)[3]

    class FakeModel:
        def __init__(self, engine_path):
            if not os.path.isfile(engine_path):
                raise FileNotFoundError(engine_path)
            so = ort.SessionOptions(); so.intra_op_num_threads = 2; so.inter_op_num_threads = 1
            self.session = ort.InferenceSession(onnx_path, sess_options=so, providers=["CPUExecutionProvider"])
            self.engine = FakeEngine(self.session)
            self.input_name = self.session.get_inputs()[0].name
            self.outputs = [o.name for o in sorted(self.session.get_outputs(), key=lambda o: len(o.shape))]
            self.size = int(self.session.get_inputs()[0].shape[-1])
            self.x = None
            self.out = None

        def run(self):
            self.out = self.session.run(self.outputs, {self.input_name: self.x})

    class FakeDetector:
        def __init__(self, engine, W=1280, H=720, conf=0.25, iou=0.7, max_det=100):
            self.m = FakeModel(engine)
            self.lb = nd.Letterbox(W, H, self.m.size)
            self.conf, self.iou, self.max_det = conf, iou, max_det
            self.out_det, self.out_proto = self.m.outputs

        def preprocess(self, rgb):
            self.m.x = yd.letterbox_input(rgb, self.lb, cv2)

        def postprocess(self, masks=True):
            return yd.postprocess_raw(self.m.out[0], self.lb, self.conf, self.iou, self.max_det, list(nd.COCO))

    nd.Detector = FakeDetector
    nd.TRTModel = FakeModel
    nd.set_cuda_sync = lambda mode: None
    nd.FAKE_TRT = True
