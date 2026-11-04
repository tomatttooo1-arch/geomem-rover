#!/usr/bin/env python3
"""FWD_COV: pick the pixel pairs of the sparse ev_pl frames (design aid; the final
judge is the reference simulator g1d_ref plus the C model).

For an ev_dK frame (K = 1, 2, 3), row pair p (rows 2p, 2p+1) holds three in-grid
pixels, everything else has no depth:
  B = (2p,   2b)      any in-grid ray: makes the block "in-grid seen", so the
                       pixel below has in_seen = true;
  A = (2p+1, 2b - K)  first ray to voxel E (kept by the endpoint guarantee);
  J = (2p+1, 2b)      ray to the same voxel E, K pixels after A (pixels in
                       between have no depth). Dlat2 drops J (block seen and E
                       seen). A RAM that has not yet written A's mark when J
                       reads (distance K, no forwarding) keeps J.
A and J differ in direction by K * a rad, so their walks differ; the pair is
accepted only if J's walk marks a free cell that no kept ray of the frame marks
(otherwise the extra ray would not change the output) and every endpoint
coordinate is at least MARGIN q8 units away from a cell boundary (float32 vs
float64 rounding cannot move it). Python float64 replica of the q8 conversion
and of the walk (G1d g1d_ref walk_rec).
usage: (imported by make_frames.py)"""
import math

MARGIN = 6


def q8(world, org, vox):
    return math.floor((world - org) / vox * 256)


def floor_q8(c):
    return c // 256  # python floor division == reference floor for negatives


def walk(O, E, dims):
    c, t, s, delta, nxt = [0] * 3, [0] * 3, [0] * 3, [0] * 3, [0] * 3
    for a in range(3):
        fr, to = O[a], E[a]
        c[a] = int(fr / 256)  # C truncation; fr >= 0 here
        tg = floor_q8(to)
        t[a] = -1 if tg < -1 else (32 if tg > 32 else tg)
        s[a] = 1 if to > fr else (-1 if to < fr else 0)
        delta[a] = abs(to - fr)
        nxt[a] = (c[a] + 1) * 256 - fr if s[a] > 0 else fr - c[a] * 256
    pxy = nxt[0] * delta[1] - nxt[1] * delta[0]
    pxz = nxt[0] * delta[2] - nxt[2] * delta[0]
    pyz = nxt[1] * delta[2] - nxt[2] * delta[1]
    dx, dy, dz = delta[0] << 8, delta[1] << 8, delta[2] << 8
    out, steps = [], 0
    while True:
        ax = s[0] != 0 and c[0] != t[0]
        ay = s[1] != 0 and c[1] != t[1]
        az = s[2] != 0 and c[2] != t[2]
        if not (ax or ay or az):
            break
        stx = ax and (not ay or pxy <= 0) and (not az or pxz <= 0)
        sty = ay and (not ax or pxy >= 0) and (not az or pyz <= 0)
        stz = az and (not ax or pxz >= 0) and (not ay or pyz >= 0)
        if stx:
            pxy += dy; pxz += dz; c[0] += s[0]
        if sty:
            pxy -= dx; pyz += dz; c[1] += s[1]
        if stz:
            pxz -= dx; pyz -= dy; c[2] += s[2]
        steps += 1
        if not all(0 <= c[i] < dims[i] for i in range(3)):
            break
        if c == t:
            break
        out.append(tuple(c))
        if steps == 96:
            break
    return out


def endpoint(u, v, z_mm, ik, org, vox):
    z = z_mm * 0.001
    x = (ik[0] * u + ik[2]) * z
    y = (ik[4] * v + ik[5]) * z
    return [q8(x, org[0], vox), q8(y, org[1], vox), q8(z, org[2], vox)]


def safe(E):
    return all(MARGIN <= (e % 256) <= 255 - MARGIN for e in E)


def design(K, W, H, ik, org, vox, dims, depths):
    """returns {(v, u): depth_mm} and the per-pair report"""
    O = [q8(0.0, org[i], vox) for i in range(3)]
    pix, kept_cells, rep = {}, set(), []
    for p in range(H // 2):
        best = None
        for b in range(2, W // 2):
            uA, uJ, uB = 2 * b - K, 2 * b, 2 * b
            for zB in depths:
                EB = endpoint(uB, 2 * p, zB, ik, org, vox)
                if not safe(EB) or not all(0 <= floor_q8(EB[i]) < dims[i] for i in range(3)):
                    continue
                for zA in depths:
                    EA = endpoint(uA, 2 * p + 1, zA, ik, org, vox)
                    if not safe(EA) or not all(0 <= floor_q8(EA[i]) < dims[i] for i in range(3)):
                        continue
                    vA = tuple(floor_q8(e) for e in EA)
                    if vA == tuple(floor_q8(e) for e in EB):
                        continue
                    for zJ in depths:
                        EJ = endpoint(uJ, 2 * p + 1, zJ, ik, org, vox)
                        if not safe(EJ) or tuple(floor_q8(e) for e in EJ) != vA:
                            continue
                        cA, cB, cJ = walk(O, EA, dims), walk(O, EB, dims), walk(O, EJ, dims)
                        new = set(cJ) - set(cA) - set(cB) - kept_cells - {vA}
                        if best is None or len(new) > best[0]:
                            best = (len(new), b, zA, zJ, zB, cA, cB, new)
                break  # one zB per b is enough (keeps the search small)
        if best is None or best[0] == 0:
            rep.append((p, None))
            continue
        n, b, zA, zJ, zB, cA, cB, new = best
        pix[(2 * p, 2 * b)] = zB
        pix[(2 * p + 1, 2 * b - K)] = zA
        pix[(2 * p + 1, 2 * b)] = zJ
        kept_cells |= set(cA) | set(cB)
        rep.append((p, (b, zA, zJ, zB, n)))
    return pix, rep
