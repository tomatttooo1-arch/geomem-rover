"""Explicit CPU/CUDA software reference; no automatic backend fallback."""
from pathlib import Path
from .core import Detection


def restore_boxes(boxes, width, height, view):
    """Map a known image augmentation back to the original calibrated pixels."""
    import numpy as np
    result = np.asarray(boxes).copy()
    if view == 'flip':
        result[:, [0, 2]] = width - result[:, [2, 0]]
    elif view == 'left4':
        result[:, [0, 2]] += 4
    elif view == 'right4':
        result[:, [0, 2]] -= 4
    elif view != 'original':
        raise ValueError('unknown augmentation view')
    result[:, [0, 2]] = np.clip(result[:, [0, 2]], 0, width)
    result[:, [1, 3]] = np.clip(result[:, [1, 3]], 0, height)
    return result


class SSDDetector:
    def __init__(self, model_dir, score=0.4, threads=2, device="cpu",
                 memory_format="contiguous", cuda_math="fp32", augmentation="none"):
        import torch
        from torchvision.models.detection import ssdlite320_mobilenet_v3_large, SSDLite320_MobileNet_V3_Large_Weights
        if device not in ("cpu", "cuda"):
            raise ValueError("device must be cpu or cuda")
        if memory_format not in ("contiguous", "channels_last") or cuda_math not in ("fp32", "tf32"):
            raise ValueError("explicit memory format and CUDA math mode required")
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA explicitly requested but unavailable; no CPU fallback")
        self.device = torch.device(device)
        self.execution = {"device": device, "memory_format": memory_format, "cuda_math": cuda_math if device == "cuda" else None}
        if augmentation not in ('none', 'hflip_shift4'):
            raise ValueError('unknown test-time augmentation')
        self.augmentation = augmentation
        if augmentation != 'none':
            self.execution['augmentation'] = augmentation
        if device == "cuda":
            torch.backends.cudnn.allow_tf32 = cuda_math == "tf32"
            torch.backends.cuda.matmul.allow_tf32 = cuda_math == "tf32"
        torch.set_num_threads(threads)
        torch.hub.set_dir(str(Path(model_dir).resolve()))
        self.weights = SSDLite320_MobileNet_V3_Large_Weights.COCO_V1
        self.model = ssdlite320_mobilenet_v3_large(weights=self.weights).to(self.device).eval()
        if memory_format == "channels_last":
            self.model.to(memory_format=torch.channels_last)
        # Calibration found finite input/weights producing NaN at the first ARM
        # CPU convolution. Empty detections alone do not distinguish that failure.
        self.model.backbone.features[0][0][0].register_forward_hook(self._finite_guard)
        self.model.head.register_forward_hook(self._finite_guard)
        self.transform = self.weights.transforms()
        self.score = score
        self.labels = self.weights.meta["categories"]

    @staticmethod
    def _finite_guard(module, inputs, output):
        import torch
        tensors = output.values() if isinstance(output, dict) else (output,)
        if any(not torch.isfinite(value).all() for value in tensors):
            raise RuntimeError("non-finite detector activation; backend calibration failed, not an empty scene")

    def __call__(self, frame):
        import torch
        from torchvision.transforms.functional import to_tensor
        views = [('original', frame.rgb)]
        if self.augmentation == 'hflip_shift4':
            import numpy as np
            rgb = frame.rgb
            views += [('flip', rgb[:, ::-1].copy()),
                      ('left4', np.pad(rgb, ((0, 0), (0, 4), (0, 0)), mode='edge')[:, 4:].copy()),
                      ('right4', np.pad(rgb, ((0, 0), (4, 0), (0, 0)), mode='edge')[:, :-4].copy())]
        with torch.inference_mode():
            predictions = self.model([self.transform(to_tensor(rgb)).to(self.device) for _, rgb in views])
        predictions = [{key: value.detach().cpu() for key, value in pred.items()} for pred in predictions]
        if self.augmentation == 'none':
            pred = predictions[0]
        else:
            from torchvision.ops import batched_nms
            h, w = frame.rgb.shape[:2]
            for (view, _), pred in zip(views, predictions):
                pred['boxes'] = torch.from_numpy(restore_boxes(pred['boxes'].numpy(), w, h, view))
            pred = {key: torch.cat([p[key] for p in predictions]) for key in predictions[0]}
            b = pred['boxes']
            valid = (b[:, 2] > b[:, 0]) & (b[:, 3] > b[:, 1]) & (pred['scores'] >= self.score)
            pred = {key: value[valid] for key, value in pred.items()}
            keep = batched_nms(pred['boxes'], pred['scores'], pred['labels'], .55)
            pred = {key: value[keep] for key, value in pred.items()}
        return [Detection(self.labels[int(label)], float(score), tuple(map(float, box)),
                          "torchvision_ssdlite320_mobilenet_v3_large_COCO_V1" +
                          ('_'+self.augmentation if self.augmentation != 'none' else ''), frame.frame_id, frame.timestamp)
                for box, label, score in zip(pred["boxes"], pred["labels"], pred["scores"])
                if float(score) >= self.score]
