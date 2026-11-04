#!/usr/bin/env python3
"""Evaluate metric TUM trajectories with timestamp association and rigid ATE.

Convention reference: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools
This independent implementation uses closest-first one-to-one timestamp matching
and SVD rigid alignment, without scale fitting, interpolation or clock correction.
Only translation is scored. Both files must describe the same physical frame.
Coverage reports associated samples, not tracking availability between samples.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def read_tum(path):
    """Read timestamp tx ty tz qx qy qz qw; reject malformed input."""
    rows = []
    for line_number, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        try:
            row = [float(value) for value in line.split()]
        except ValueError as exc:
            raise ValueError(f"{path}:{line_number}: nonnumeric TUM row") from exc
        if len(row) != 8 or not np.isfinite(row).all():
            raise ValueError(f"{path}:{line_number}: expected 8 finite TUM values")
        if np.linalg.norm(row[4:]) == 0:
            raise ValueError(f"{path}:{line_number}: zero quaternion")
        rows.append(row)
    if not rows:
        raise ValueError(f"{path}: empty trajectory")
    trajectory = np.asarray(rows, dtype=float)
    trajectory = trajectory[np.argsort(trajectory[:, 0])]
    if np.any(np.diff(trajectory[:, 0]) == 0):
        raise ValueError(f"{path}: duplicate timestamps")
    return trajectory


def associate(estimated_times, ground_truth_times, max_difference=0.02):
    """Greedy nearest-time pairing, never reusing a sample (TUM convention)."""
    if not np.isfinite(max_difference) or max_difference <= 0:
        raise ValueError("max_difference must be finite and positive")
    candidates = []
    for i, timestamp in enumerate(estimated_times):
        start = np.searchsorted(ground_truth_times, timestamp - max_difference)
        end = np.searchsorted(ground_truth_times, timestamp + max_difference, side="right")
        for j in range(start, end):
            difference = abs(timestamp - ground_truth_times[j])
            # Match the strict threshold used by the TUM association script.
            if difference < max_difference:
                candidates.append((difference, i, j))
    used_estimate, used_truth, matches = set(), set(), []
    for _, i, j in sorted(candidates):
        if i not in used_estimate and j not in used_truth:
            used_estimate.add(i)
            used_truth.add(j)
            matches.append((i, j))
    return sorted(matches)


def evaluate(estimated_path, ground_truth_path, max_difference=0.02):
    estimated, truth = read_tum(estimated_path), read_tum(ground_truth_path)
    matches = associate(estimated[:, 0], truth[:, 0], max_difference)
    if len(matches) < 3:
        raise ValueError(f"insufficient timestamp matches: {len(matches)} (need at least 3)")
    indices = np.asarray(matches)
    source, target = estimated[indices[:, 0], 1:4], truth[indices[:, 1], 1:4]
    source_mean, target_mean = source.mean(axis=0), target.mean(axis=0)
    left, singular, right = np.linalg.svd((source - source_mean).T @ (target - target_mean))
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(right.T @ left.T)
    rotation = right.T @ correction @ left.T
    translation = target_mean - rotation @ source_mean
    residuals = source @ rotation.T + translation - target
    errors = np.linalg.norm(residuals, axis=1)
    differences = np.abs(estimated[indices[:, 0], 0] - truth[indices[:, 1], 0])
    result = {
        "protocol": "TUM timestamps; closest-first one-to-one; SE3 SVD; no scale fitting",
        "estimated_path": str(estimated_path),
        "ground_truth_path": str(ground_truth_path),
        "max_time_difference_s": max_difference,
        "estimated_samples": len(estimated),
        "ground_truth_samples": len(truth),
        "matched_samples": len(matches),
        "estimated_sample_coverage": len(matches) / len(estimated),
        "ground_truth_sample_coverage": len(matches) / len(truth),
        "matched_time_difference_max_s": float(differences.max()),
        "ate_translation_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
        "estimated_raw_length_m": float(np.linalg.norm(np.diff(estimated[:, 1:4], axis=0), axis=1).sum()),
        "ground_truth_raw_length_m": float(np.linalg.norm(np.diff(truth[:, 1:4], axis=0), axis=1).sum()),
        "alignment_rotation": rotation.tolist(),
        "alignment_translation_m": translation.tolist(),
        "alignment_covariance_rank": int(np.linalg.matrix_rank((source-source_mean).T @ (target-target_mean))),
        "limitations": "Translation only; sample coverage is not temporal coverage; lengths include gaps; collinear/stationary paths cannot constrain full rotation.",
    }
    if not np.isfinite(errors).all() or not np.isfinite(singular).all():
        raise ValueError("non-finite alignment result")
    # Reject numeric overflow in any derived metric rather than emit Infinity.
    json.dumps(result, allow_nan=False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("estimated", type=Path)
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("--max-difference", type=float, default=0.02)
    args = parser.parse_args()
    try:
        result = evaluate(args.estimated, args.ground_truth, args.max_difference)
    except (ValueError, OSError, np.linalg.LinAlgError) as exc:
        parser.exit(2, f"trajectory evaluation failed: {exc}\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
