# ----------------------------------------------------------------------------
# Ground plane fit kernel of tools/car_run/ground_calib.py (FPGA team, 2026-10-15).
#
# The RANSAC skeleton (pre-generated 3-point samples, plane through 3 points, best model, final inlier selection
# and refit of the plane on them) is a numpy port of Open3D's PointCloud::SegmentPlane,
# cpp/open3d/geometry/PointCloudSegmentation.cpp (github.com/isl-org/Open3D, read 2026-10-15), under the MIT
# license reproduced below.
#
# Changes against Open3D (ours, not Open3D's):
#   - the hypotheses are scored together with numpy (no TBB loop, no adaptive early stop: all `iters` samples are
#     drawn up front from a numpy RandomState with a fixed seed, which Open3D also does for reproducibility);
#   - two hypothesis gates after the PCL constrained plane models (SACMODEL_NORMAL_PARALLEL_PLANE isModelValid,
#     github.com/PointCloudLibrary/pcl, design only, no code): normal within HYP_MAX_DEG of the prior "up" normal and
#     plane distance within HYP_MAX_DH_M of the prior height; the normal is oriented to the prior first;
#   - MSAC scoring (PCL msac.hpp, design only): cost = sum(min(|r|, MSAC_T)) instead of Open3D's inlier ratio + RMSE;
#   - the single least-squares refit is replaced by a Tukey-biweight iteratively reweighted total least squares
#     (IRLS_ROUNDS rounds): residuals divided by the depth noise model sigma(z) = SIGMA_K z^2 + SIGMA0_M, scale
#     from the MAD each round (so the inlier band tightens round by round), weights also 1/sigma^2; the refit is the
#     smallest eigenvector of the weighted scatter (numpy eigh) instead of Open3D's max-determinant closed form;
#   - points whose local surface normal (central differences on the organised samples, 4 px apart) is more than
#     LOCAL_NORMAL_MAX_DEG from the RANSAC plane are excluded from the refit (wall feet, box faces);
#   - the residual bend (residual_bend) is a separate quality measure for the caller.
# Why: the placeholder (RANSAC 2 cm + one least-squares refit, = Open3D's own scheme) adopted a mount 0.44-0.76 deg
# off with a 2 m wide wall 0.7-1.2 m ahead -- the wall foot inside the 2 cm band tilts a one-shot refit
# (controller measurement 10-15; research note ground-calib-research/RESEARCH.md section 3).
# sigma(z) and the 1 mm bend gate are ESTIMATES (stereo formula, synthetic data), not fitted to a Gemini 2:
# re-run the occlusion sweep on real captures before trusting them (docs/design/REAL_CAR_RUNBOOK_20261015.md).
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Open3D: www.open3d.org
# Copyright (c) 2018-2026 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------
"""Robust ground-plane fit (numpy only, Python 3.8 compatible): fit_plane, local_normals, residual_bend."""
import math

import numpy as np

RANSAC_ITERS = 300
MSAC_T = 0.01                 # m, MSAC truncation (hypothesis choice only)
HYP_MAX_DEG = 8.0             # hypothesis normal vs prior up normal
HYP_MAX_DH_M = 0.05           # hypothesis plane distance vs prior height
SIGMA_K = 1.5e-3              # depth noise sigma(z) = SIGMA_K z^2 + SIGMA0_M  (estimate; calibrate on the car)
SIGMA0_M = 5e-4               # floor of sigma: 1 mm depth quantisation
TUKEY_C = 4.685
IRLS_ROUNDS = 5
LOCAL_NORMAL_MAX_DEG = 10.0


def sigma(z):
    z = np.asarray(z, dtype=np.float64)
    return SIGMA_K * z * z + SIGMA0_M


def plane_from_weighted(P, w):
    """weighted total least squares plane n . X + d = 0 (n unit): smallest eigenvector of the weighted scatter."""
    ws = w.sum()
    c = (P * w[:, None]).sum(axis=0) / ws
    Q = (P - c) * np.sqrt(w)[:, None]
    n = np.linalg.eigh(Q.T @ Q)[1][:, 0]
    return n, float(-n @ c)


def local_normals(grid):
    """(h, w, 3) organised optical-frame points (NaN = none) -> (h, w, 3) unit normals from central differences of
    the neighbours one sample apart (NaN at the border and where a neighbour is missing). Sign arbitrary."""
    N = np.full(grid.shape, np.nan)
    a = grid[1:-1, 2:] - grid[1:-1, :-2]
    b = grid[2:, 1:-1] - grid[:-2, 1:-1]
    n = np.cross(a, b)
    with np.errstate(invalid="ignore", divide="ignore"):
        N[1:-1, 1:-1] = n / np.linalg.norm(n, axis=-1, keepdims=True)
    return N


def _orient(n, d, up):
    return (n, d) if n @ up >= 0 else (-n, -d)


def fit_plane(points, prior_normal, prior_d=None, normals=None, iters=RANSAC_ITERS, seed=0,
              max_angle_deg=HYP_MAX_DEG, max_dh_m=HYP_MAX_DH_M):
    """Plane n . X + d = 0 through points (N, 3) (optical frame, z = depth) near the prior.
    prior_normal: the expected up normal (oriented to it: n . prior > 0); prior_d: the expected d (= camera height)
    or None (no height gate); normals: (N, 3) local normals (NaN = unknown) or None (no local-normal exclusion).
    -> (n, d, inliers bool mask) or (None, None, all-False) when no hypothesis passes the gates. Deterministic for a
    given seed."""
    P = np.asarray(points, dtype=np.float64)
    none = (None, None, np.zeros(len(P), dtype=bool))
    if len(P) < 3:
        return none
    up = np.asarray(prior_normal, dtype=np.float64)
    up = up / np.linalg.norm(up)
    rng = np.random.RandomState(seed)
    # --- RANSAC (Open3D SegmentPlane: samples drawn up front; plane through 3 points) -------------------------
    S = rng.randint(0, len(P), size=(iters, 3))
    S = S[(S[:, 0] != S[:, 1]) & (S[:, 0] != S[:, 2]) & (S[:, 1] != S[:, 2])]
    a, b, c = P[S[:, 0]], P[S[:, 1]], P[S[:, 2]]
    n = np.cross(b - a, c - a)
    nn = np.linalg.norm(n, axis=1)
    ok = nn > 1e-12
    n, a = n[ok] / nn[ok, None], a[ok]
    flip = (n @ up) < 0
    n[flip] = -n[flip]
    d = -(n * a).sum(axis=1)
    # --- PCL-style gates: normal near the prior, distance near the prior height ---------------------------------
    keep = (n @ up) >= math.cos(math.radians(max_angle_deg))
    if prior_d is not None:
        keep &= np.abs(d - prior_d) <= max_dh_m
    if not keep.any():
        return none
    n, d = n[keep], d[keep]
    # --- MSAC score (PCL msac.hpp): sum of truncated residuals; first minimum wins ------------------------------
    cost = np.minimum(np.abs(P @ n.T + d), MSAC_T).sum(axis=0)
    k = int(np.argmin(cost))
    n, d = n[k], float(d[k])
    # --- exclusion by local normal ------------------------------------------------------------------------------
    sel = np.ones(len(P), dtype=bool)
    if normals is not None:
        N = np.asarray(normals, dtype=np.float64)
        fin = np.isfinite(N).all(axis=1)
        sel = fin.copy()
        sel[fin] = np.abs(N[fin] @ n) >= math.cos(math.radians(LOCAL_NORMAL_MAX_DEG))
    if sel.sum() < 3:
        return none
    # --- Tukey IRLS refit (residual / sigma(z), MAD scale per round: the band tightens) -------------------------
    sg = sigma(P[:, 2])
    w = None
    for _ in range(IRLS_ROUNDS):
        r = (P @ n + d) / sg
        core = sel & (np.abs(r) < 10.0)
        if core.sum() < 3:
            return none
        s = 1.4826 * float(np.median(np.abs(r[core])))
        cc = TUKEY_C * max(s, 1.0)
        w = np.where(sel & (np.abs(r) < cc), (1.0 - (r / cc) ** 2) ** 2, 0.0) / sg ** 2
        if (w > 0).sum() < 3:
            return none
        n, d = _orient(*plane_from_weighted(P, w), up)
    return n, d, w > 0


def residual_bend(points, n, d, inliers, rows):
    """systematic bend of the residuals along the image rows: the inliers split into three equal-count groups by
    image row (near / middle / far), the median residual of each; -> max - min (m). A plane pulled by a wall foot
    or box edge leaves a bend; a clean floor fitted right leaves noise only."""
    r = (np.asarray(points)[inliers] @ n + d)
    v = np.asarray(rows, dtype=np.float64)[inliers]
    if len(r) < 30:
        return float("inf")
    order = np.argsort(v, kind="stable")
    meds = [float(np.median(r[g])) for g in np.array_split(order, 3)]
    return max(meds) - min(meds)
