"""CPU-only diagnostic: independent processes separate layout, cold state and profiler.

No production detector modifications. One preparation process extracts the real
first-frame input/first-layer weights; each worker's first torch convolution then
uses that same frozen fixture without constructing a detector. Not a speed test.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with Path(path).open("x") as output:
        json.dump(value, output, indent=2, allow_nan=False)


def numpy_conv(x, weights, bias, stride, padding):
    """Literal float64 cross-correlation; no BLAS, scipy or torch convolution."""
    import numpy as np
    x = np.pad(x.astype(np.float64), ((0, 0), (0, 0), (padding, padding), (padding, padding)))
    w = weights.astype(np.float64)
    n, channels, height, width = x.shape
    outputs, _, kh, kw = w.shape
    oh, ow = (height-kh)//stride+1, (width-kw)//stride+1
    result = np.zeros((n, outputs, oh, ow), dtype=np.float64)
    for i in range(kh):
        for j in range(kw):
            for c in range(channels):
                result += x[:, c:c+1, i:i+oh*stride:stride, j:j+ow*stride:stride] * w[None, :, c, i, j, None, None]
    return result + bias[None, :, None, None]


def prepare(args):
    import numpy as np
    import torch
    from torchvision.transforms.functional import to_tensor
    from perception.fpga_replay.detector import SSDDetector
    from perception.fpga_replay.io import load_frame, load_records
    torch.manual_seed(20260908)
    np.random.seed(20260908)
    # Calibrate the independent reference on two distinct known answers.
    np.testing.assert_array_equal(numpy_conv(np.ones((1, 3, 5, 5)), np.ones((2, 3, 3, 3)),
                                             np.zeros(2), 1, 0), np.full((1, 2, 3, 3), 27))
    impulse = np.zeros((1, 1, 5, 5)); impulse[0, 0, 2, 2] = 1
    np.testing.assert_array_equal(numpy_conv(impulse, np.arange(1, 10).reshape(1, 1, 3, 3),
                                             np.zeros(1), 1, 0), np.arange(9, 0, -1).reshape(1, 1, 3, 3))
    frame = load_frame(load_records(Path(args.data)/"input.jsonl")[0], args.data)
    detector = SSDDetector(args.models, threads=1, device="cpu", memory_format="contiguous")
    layer = detector.model.backbone.features[0][0][0]
    if (tuple(layer.kernel_size), tuple(layer.stride), tuple(layer.padding), tuple(layer.dilation), layer.groups) != (
            (3, 3), (2, 2), (1, 1), (1, 1), 1):
        raise ValueError("probe requires the verified first-layer geometry")
    with torch.inference_mode():
        images, _ = detector.model.transform([detector.transform(to_tensor(frame.rgb))], None)
    if not images.tensors.is_contiguous():
        raise ValueError("production transform input layout changed; review this probe")
    x = images.tensors.detach().numpy().copy()
    weights = layer.weight.detach().numpy().copy()
    bias = np.zeros(weights.shape[0], dtype=np.float32) if layer.bias is None else layer.bias.detach().numpy().copy()
    if not all(np.isfinite(a).all() for a in (x, weights, bias)):
        raise ValueError("non-finite frozen input or weights")
    reference = numpy_conv(x, weights, bias, stride=2, padding=1)
    np.savez(Path(args.out)/"fixture.npz", x=x, weights=weights, bias=bias, reference=reference)
    save(Path(args.out)/"fixture.json", {
        "frame_id": frame.frame_id, "timestamp": frame.timestamp,
        "torch": torch.__version__, "seed": 20260908,
        "input_sha256": hashlib.sha256(x.tobytes()).hexdigest(),
        "weights_sha256": hashlib.sha256(weights.tobytes()).hexdigest(),
        "bias_sha256": hashlib.sha256(bias.tobytes()).hexdigest(),
        "fixture_sha256": digest(Path(args.out)/"fixture.npz"),
        "reference_calibration": ["all_ones_27", "impulse_reversed_kernel"],
        "input_shape": list(x.shape), "weight_shape": list(weights.shape),
        "production_input_strides": list(images.tensors.stride()),
        "layer_bias_is_none": layer.bias is None,
        "reference": "float64 literal spatial/channel sum of float32 inputs and weights",
        "scope": "preparation may warm torch; workers are fresh processes and do not construct a model"})


def worker(args):
    import numpy as np
    import torch
    torch.manual_seed(20260908)
    np.random.seed(20260908)
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    with np.load(Path(args.out)/"fixture.npz", allow_pickle=False) as data:
        reference = data["reference"]
        layout = torch.contiguous_format if args.layout == "contiguous" else torch.channels_last
        # Match production: SSDDetector converts MODEL weights; its transform
        # still emits a contiguous NCHW batch for the actual first layer.
        x = torch.from_numpy(data["x"].copy()).contiguous()
        w = torch.from_numpy(data["weights"].copy()).contiguous(memory_format=layout)
        b = torch.from_numpy(data["bias"].copy())
    # Preserve the real first-layer bias=None dispatch, rather than passing zeros.
    fixture = json.loads((Path(args.out)/"fixture.json").read_text())
    if fixture["layer_bias_is_none"]:
        b = None
    rows = []
    with torch.inference_mode():
        for index in range(3):
            profile_this_call = (index == 0 if args.profiler == "cold" else index == 1)
            if profile_this_call:
                with torch.autograd.profiler.profile(use_cuda=False, record_shapes=True) as profile:
                    result = torch.nn.functional.conv2d(x, w, b, stride=2, padding=1)
                operations = sorted({event.name for event in profile.function_events})
            else:
                result = torch.nn.functional.conv2d(x, w, b, stride=2, padding=1)
                operations = None
            values = result.detach().numpy().astype(np.float64)
            finite = np.isfinite(values)
            errors = np.abs(values[finite]-reference[finite])
            rows.append({"call_index": index, "profiled": profile_this_call,
                         "finite": int(finite.sum()), "elements": int(values.size),
                         "max_abs_error_on_finite": float(errors.max()) if errors.size else None,
                         "matches_numpy_at_1e_5": bool(np.allclose(values, reference, rtol=1e-5, atol=1e-5)),
                         "output_sha256": hashlib.sha256(result.detach().contiguous().numpy().tobytes()).hexdigest(),
                         "operators": operations})
    save(Path(args.out)/args.result_file, {
        "pid": os.getpid(), "torch": torch.__version__, "threads": torch.get_num_threads(),
        "interop_threads": torch.get_num_interop_threads(), "layout": args.layout,
        "input_strides": list(x.stride()), "weight_strides": list(w.stride()),
        "profiler_on": args.profiler, "fixture_sha256": digest(Path(args.out)/"fixture.npz"),
        "calls": rows,
        "scope": "first explicit conv in a fresh Python process; contiguous input and configured model-weight layout; no detector construction in worker; CPU only"})


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", required=True); p.add_argument("--models", required=True)
    p.add_argument("--out", required=True); p.add_argument("--repeats", type=int, default=2)
    p.add_argument("--source-commit", required=True)
    p.add_argument("--internal", choices=("prepare", "worker"))
    p.add_argument("--threads", type=int, choices=(1, 2))
    p.add_argument("--layout", choices=("contiguous", "channels_last"))
    p.add_argument("--profiler", choices=("cold", "warm"))
    p.add_argument("--result-file")
    args = p.parse_args()
    if args.internal == "prepare":
        return prepare(args)
    if args.internal == "worker":
        return worker(args)
    if not 1 <= args.repeats <= 3:
        p.error("bounded probe requires 1..3 repeats")
    out = Path(args.out); out.mkdir(parents=True, exist_ok=False)
    script = str(Path(__file__).resolve())
    base = [sys.executable, script, "--data", str(Path(args.data).resolve()), "--models", str(Path(args.models).resolve()),
            "--out", str(out.resolve()), "--source-commit", args.source_commit]
    environment = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1",
                       CUDA_VISIBLE_DEVICES="")
    # Re-executing an absolute script otherwise puts tools/, not the repository,
    # on sys.path. Keep workers on this exact checkout even from another cwd.
    project_path = str(Path(script).parents[1])
    environment["PYTHONPATH"] = project_path + (
        os.pathsep+environment["PYTHONPATH"] if environment.get("PYTHONPATH") else "")
    commands = []
    def run(command, name):
        commands.append(command)
        with (out/(name+".stdout.log")).open("x") as stdout, (out/(name+".stderr.log")).open("x") as stderr:
            subprocess.run(command, env=environment, check=True, timeout=120, stdout=stdout, stderr=stderr)
    run(base+["--internal", "prepare"], "prepare")
    files = []
    # Reverse layout order every repetition; process state is never shared.
    for repeat in range(args.repeats):
        layouts = ("contiguous", "channels_last") if repeat % 2 == 0 else ("channels_last", "contiguous")
        for threads in (1, 2):
            for layout in layouts:
                for profiler in ("cold", "warm"):
                    name = "r{}-t{}-{}-{}".format(repeat, threads, layout, profiler)
                    filename = name+".json"
                    run(base+["--internal", "worker", "--threads", str(threads), "--layout", layout,
                              "--profiler", profiler, "--result-file", filename], name)
                    files.append(filename)
                    row = json.loads((out/filename).read_text())
                    print(json.dumps({"case": name, "finite_by_call": [r["finite"] for r in row["calls"]],
                                      "elements": row["calls"][0]["elements"]}), flush=True)
    save(out/"summary.json", {"source_commit": args.source_commit, "script_sha256": digest(script),
         "python": sys.version, "fixture": json.loads((out/"fixture.json").read_text()), "commands": commands,
         "cases": [json.loads((out/name).read_text()) for name in files],
         "scope": "diagnostic first-layer consistency only; no speed/accuracy claim; no CPU library root-cause closure"})


if __name__ == "__main__":
    main()
