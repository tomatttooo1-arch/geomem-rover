# 用途：在主机上用与 xmodel 同一张浮点图生成「仿真 DPU 输出」；其 decode() 已对 ultralytics 校验（equivalence.json）。
"""Version-neutral raw convolution graph for Vitis AI 3.0 (no Ultralytics dependency).

The descriptor preserves the actual trained module attributes. DFL, sigmoid,
box decoding and NMS deliberately remain outside RawYOLO.forward().
"""
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn


class Conv(nn.Module):
    def forward(self, x):
        return self.act(self.bn(self.conv(x)))


class Bottleneck(nn.Module):
    def forward(self, x):
        y = self.cv2(self.cv1(x))
        return x + y if self.add else y


class C2f(nn.Module):
    def forward(self, x):
        # VAI 3.0 treats split_with_sizes as float; chunk cannot export to XIR.
        # Two single-output channel slices preserve the trained computation.
        stem = self.cv1(x)
        y = [stem[:, :self.c, :, :], stem[:, self.c:2*self.c, :, :]]
        for block in self.m:
            y.append(block(y[-1]))
        return self.cv2(torch.cat(y, dim=1))


class SPPF(nn.Module):
    def forward(self, x):
        x = self.cv1(x)
        y1 = self.m(x)
        y2 = self.m(y1)
        return self.cv2(torch.cat((x, y1, y2, self.m(y2)), 1))


class Concat(nn.Module):
    def forward(self, x):
        return torch.cat(x, self.d)


class RawHead(nn.Module):
    def forward(self, x):
        return tuple(torch.cat((reg(xi), cls(xi)), dim=1)
                     for xi, reg, cls in zip(x, self.cv2, self.cv3))


def describe(module):
    kind = type(module).__name__
    if kind == 'Conv2d':
        attrs = {k: getattr(module, k) for k in ('in_channels', 'out_channels', 'kernel_size',
                 'stride', 'padding', 'dilation', 'groups', 'padding_mode')}
        attrs['bias'] = module.bias is not None
    elif kind == 'BatchNorm2d':
        attrs = {k: getattr(module, k) for k in ('num_features', 'eps', 'momentum', 'affine', 'track_running_stats')}
    elif kind == 'MaxPool2d':
        attrs = {k: getattr(module, k) for k in ('kernel_size', 'stride', 'padding', 'dilation', 'return_indices', 'ceil_mode')}
    elif kind == 'Upsample':
        attrs = {k: getattr(module, k) for k in ('size', 'scale_factor', 'mode', 'align_corners')}
    elif kind in ('LeakyReLU', 'ReLU', 'ReLU6', 'SiLU'):
        attrs = dict(inplace=False)
        if kind == 'LeakyReLU':
            attrs['negative_slope'] = module.negative_slope
    elif kind in ('Sequential', 'ModuleList', 'Identity', 'Conv', 'SPPF', 'Detect'):
        attrs = {}
    elif kind == 'C2f':
        attrs = dict(c=module.c)
    elif kind == 'Bottleneck':
        attrs = dict(add=module.add)
    elif kind == 'Concat':
        attrs = dict(d=module.d)
    else:
        raise ValueError('unsupported module: ' + kind)
    children = {name: describe(child) for name, child in module.named_children()
                if not (kind == 'Detect' and name == 'dfl')}
    return dict(kind=kind, attrs=attrs, children=children)


def construct(spec):
    kind, attrs = spec['kind'], spec['attrs']
    custom = dict(Conv=Conv, Bottleneck=Bottleneck, C2f=C2f, SPPF=SPPF, Concat=Concat, Detect=RawHead)
    if kind in custom:
        module = custom[kind]()
        for key, value in attrs.items():
            setattr(module, key, value)
    elif kind == 'ModuleList':
        module = nn.ModuleList()
    elif kind == 'Sequential':
        module = nn.Sequential()
    else:
        module = getattr(nn, kind)(**attrs)
    for key, child in spec['children'].items():
        module.add_module(key, construct(child))
    return module


class RawYOLO(nn.Module):
    def __init__(self, spec):
        super().__init__()
        self.model = nn.ModuleList([construct(s) for s in spec['layers']])
        self.sources = spec['sources']

    def forward(self, x):
        saved = []
        for source, module in zip(self.sources, self.model):
            if isinstance(source, list):
                value = [x if i == -1 else saved[i] for i in source]
            else:
                value = x if source == -1 else saved[source]
            x = module(value)
            saved.append(x)
        return x


def load_raw(directory):
    directory = Path(directory)
    spec = json.loads((directory / 'architecture.json').read_text())
    model = RawYOLO(spec)
    with np.load(str(directory / 'weights.npz'), allow_pickle=False) as weights:
        model.load_state_dict({key: torch.from_numpy(weights[key].copy()) for key in weights.files}, strict=True)
    return model.float().eval(), spec


def decode(outputs, strides=(8, 16, 32), reg_max=16):
    """CPU floating postprocess, returning [batch, 4+nc, anchors] xywh/scores."""
    decoded = []
    for output, stride in zip(outputs, strides):
        n, channels, h, w = output.shape
        boxes = output[:, :4 * reg_max].reshape(n, 4, reg_max, h * w)
        projection = torch.arange(reg_max, dtype=output.dtype, device=output.device).view(1, 1, -1, 1)
        distance = (boxes.softmax(2) * projection).sum(2)
        y = torch.arange(h, dtype=output.dtype, device=output.device) + .5
        x = torch.arange(w, dtype=output.dtype, device=output.device) + .5
        # Repeat avoids the torch.meshgrid indexing API difference across torch releases.
        centers = torch.stack((x.repeat(h), y.repeat_interleave(w)), dim=0)[None]
        lt, rb = distance[:, :2], distance[:, 2:]
        xy = (centers - lt + centers + rb) * (.5 * stride)
        wh = (lt + rb) * stride
        scores = output[:, 4 * reg_max:].reshape(n, channels - 4 * reg_max, h * w).sigmoid()
        decoded.append(torch.cat((xy, wh, scores), dim=1))
    return torch.cat(decoded, dim=2)


def preprocess(path, size=640):
    import cv2
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError('unreadable image: ' + str(path))
    height, width = image.shape[:2]
    ratio = min(size / height, size / width)
    unpad = (round(width * ratio), round(height * ratio))
    if (width, height) != unpad:
        image = cv2.resize(image, unpad, interpolation=cv2.INTER_LINEAR)
    dw, dh = (size - unpad[0]) / 2, (size - unpad[1]) / 2
    image = cv2.copyMakeBorder(image, round(dh-.1), round(dh+.1), round(dw-.1), round(dw+.1),
                              cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1)[None]).astype(np.float32) / 255.


def export_ultralytics(checkpoint, directory, probes):
    from ultralytics import YOLO
    from ultralytics.data.augment import LetterBox
    import cv2
    model = YOLO(str(checkpoint)).model.float().eval()
    spec = dict(layers=[describe(m) for m in model.model], sources=[m.f for m in model.model],
                names=model.names, strides=model.stride.tolist(), reg_max=model.model[-1].reg_max)
    target = Path(directory)
    target.mkdir(exist_ok=False)
    (target / 'architecture.json').write_text(json.dumps(spec, indent=2))
    state = {key: value.detach().cpu().numpy() for key, value in model.state_dict().items()
             if not key.startswith('model.%d.dfl.' % (len(model.model)-1))}
    np.savez(str(target / 'weights.npz'), **state)
    raw, _ = load_raw(target)
    checks = []
    for path in probes:
        image = cv2.imread(str(path))
        expected = LetterBox((640, 640), auto=False)(image=image)
        expected = np.ascontiguousarray(expected[:, :, ::-1].transpose(2, 0, 1)[None]).astype(np.float32) / 255
        actual = preprocess(path)
        assert np.array_equal(expected, actual), 'letterbox mismatch'
        with torch.inference_mode():
            source = model(torch.from_numpy(actual))[0]
            raw_outputs = raw(torch.from_numpy(actual))
            rebuilt = decode(raw_outputs, spec['strides'], spec['reg_max'])
        box = float((source[:, :4] - rebuilt[:, :4]).abs().max())
        score = float((source[:, 4:] - rebuilt[:, 4:]).abs().max())
        assert box < .001 and score < 1e-5, (box, score)
        checks.append(dict(image=str(path), box_max_abs_error=box, score_max_abs_error=score,
                           shapes=[list(x.shape) for x in raw_outputs]))
    (target / 'equivalence.json').write_text(json.dumps(checks, indent=2))
    return checks


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--probe-list', required=True)
    args = parser.parse_args()
    torch.set_num_threads(4)
    print(json.dumps(export_ultralytics(args.checkpoint, args.output, Path(args.probe_list).read_text().splitlines()[:3]), indent=2))
