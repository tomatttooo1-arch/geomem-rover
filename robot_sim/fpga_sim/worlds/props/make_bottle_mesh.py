#!/usr/bin/env python3
"""Line D2 (2026-10-14): procedural glass bottle for the car-mount simulation world (our own model, no
download): a lathe (surface of revolution) with smooth normals and UVs, plus an albedo texture with a
paper label and a foil capsule.

  make_bottle_mesh.py OUT_DIR [--style wine_green|beer_brown] [--font TTF]

writes OUT_DIR/bottle_<style>.obj (meters, z up, base at z = 0, axis through x = y = 0) and
OUT_DIR/bottle_<style>.png (u = angle / 2 pi, v = 1 - z / H). Deterministic for a given Pillow and font.
Dimensions are those of an ordinary 0.75 l wine bottle (height 0.30 m, body diameter 0.075 m) or a
0.5 l beer bottle (0.25 m, 0.068 m): modelled values, not measurements of the competition object.
"""
import argparse
import math
from pathlib import Path

STYLES = {
    # profile (z, r) from the base up (m); glass / label / capsule colours (sRGB 0-255); label z band,
    # capsule from z; label text
    'wine_green': {
        'profile': [(0.0, 0.0), (0.0, 0.034), (0.003, 0.0372), (0.010, 0.0375), (0.190, 0.0375),
                    (0.205, 0.0360), (0.218, 0.0320), (0.230, 0.0255), (0.242, 0.0195), (0.252, 0.0160),
                    (0.262, 0.0148), (0.288, 0.0145), (0.291, 0.0160), (0.297, 0.0160), (0.300, 0.0150),
                    (0.300, 0.0)],
        'glass': (24, 72, 34), 'label': (236, 226, 196), 'ink': (110, 20, 30), 'capsule': (120, 18, 28),
        'label_z': (0.055, 0.150), 'capsule_z': 0.262, 'text': ('CHATEAU', 'GRAND VIN', '2019'),
    },
    'beer_brown': {
        'profile': [(0.0, 0.0), (0.0, 0.031), (0.003, 0.0338), (0.010, 0.0340), (0.140, 0.0340),
                    (0.155, 0.0325), (0.170, 0.0270), (0.185, 0.0200), (0.198, 0.0150), (0.205, 0.0135),
                    (0.236, 0.0130), (0.239, 0.0145), (0.246, 0.0145), (0.250, 0.0135), (0.250, 0.0)],
        'glass': (92, 46, 14), 'label': (230, 200, 60), 'ink': (150, 20, 20), 'capsule': (200, 200, 205),
        'label_z': (0.045, 0.120), 'capsule_z': 0.236, 'text': ('LAGER', 'PREMIUM', 'BEER'),
    },
}


def lathe(profile, segments=64):
    """Vertices, normals, uvs and triangle faces of a surface of revolution (smooth normals per ring)."""
    height = max(z for z, _ in profile)
    # per-profile-point 2D normal (outward in r, along z), averaged over the adjacent segments
    normals2 = []
    for i, (z, r) in enumerate(profile):
        acc = [0.0, 0.0]
        for j in (i - 1, i):
            if 0 <= j < len(profile) - 1:
                (z0, r0), (z1, r1) = profile[j], profile[j + 1]
                dz, dr = z1 - z0, r1 - r0
                length = math.hypot(dz, dr) or 1.0
                acc[0] += dz / length          # outward normal of the segment: (dr_n, dz_n) = (dz, -dr)
                acc[1] += -dr / length
        length = math.hypot(*acc) or 1.0
        normals2.append((acc[0] / length, acc[1] / length))
    verts, norms, uvs, faces = [], [], [], []
    for i, (z, r) in enumerate(profile):
        nr, nz = normals2[i]
        for k in range(segments + 1):        # the seam column is duplicated for the texture wrap
            a = 2 * math.pi * k / segments
            verts.append((r * math.cos(a), r * math.sin(a), z))
            norms.append((nr * math.cos(a), nr * math.sin(a), nz))
            uvs.append((k / segments, 1.0 - z / height))
    cols = segments + 1
    for i in range(len(profile) - 1):
        for k in range(segments):
            a, b = i * cols + k, i * cols + k + 1
            c, d = (i + 1) * cols + k, (i + 1) * cols + k + 1
            if profile[i][1] > 0:
                faces.append((a, b, d))
            if profile[i + 1][1] > 0:
                faces.append((a, d, c))
    return verts, norms, uvs, faces, height


def write_obj(path, verts, norms, uvs, faces, name):
    with open(path, 'w') as f:
        f.write('# procedural bottle (robot_sim/fpga_sim/worlds/props/make_bottle_mesh.py), meters, z up\n')
        f.write('o %s\n' % name)
        for v in verts:
            f.write('v %.6f %.6f %.6f\n' % v)
        for t in uvs:
            f.write('vt %.6f %.6f\n' % t)
        for n in norms:
            f.write('vn %.6f %.6f %.6f\n' % n)
        for a, b, c in faces:
            f.write('f %d/%d/%d %d/%d/%d %d/%d/%d\n' % (a + 1, a + 1, a + 1, b + 1, b + 1, b + 1, c + 1, c + 1, c + 1))


def write_texture(path, style, height, font_path, size=(1024, 1024)):
    from PIL import Image, ImageDraw, ImageFont
    w, h = size
    img = Image.new('RGB', size, style['glass'])
    draw = ImageDraw.Draw(img)

    def row(z):                                   # texture row of height z (v = 1 - z / H, row 0 = top)
        return int(round((1.0 - z / height) * (h - 1)))
    # subtle vertical glass shading bands (darker edges of each repeat) so the body is not a flat colour
    for x in range(w):
        shade = 0.85 + 0.15 * math.cos(2 * math.pi * x / w * 4)
        col = tuple(int(c * shade) for c in style['glass'])
        draw.line([(x, row(style['capsule_z'])), (x, h)], fill=col)
    draw.rectangle([0, 0, w, row(style['capsule_z'])], fill=style['capsule'])
    z0, z1 = style['label_z']
    top, bottom = row(z1), row(z0)
    lw = int(w * 0.62)                            # the label wraps 62 % of the circumference
    left = (w - lw) // 2
    draw.rectangle([left, top, left + lw, bottom], fill=style['label'])
    draw.rectangle([left + 6, top + 6, left + lw - 6, bottom - 6], outline=style['ink'], width=4)
    lines = style['text']
    font_px = max(12, (bottom - top) // (len(lines) + 1))
    try:
        font = ImageFont.truetype(font_path, font_px)
    except OSError:
        font = ImageFont.load_default()
    y = top + font_px // 2
    for text in lines:
        box = draw.textbbox((0, 0), text, font=font)
        draw.text((left + (lw - (box[2] - box[0])) // 2, y), text, fill=style['ink'], font=font)
        y += font_px
    # neck label (small band under the capsule)
    nz = style['capsule_z'] - 0.012
    draw.rectangle([0, row(style['capsule_z']), w, row(nz)], fill=style['label'])
    img.save(path, optimize=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', type=Path)
    ap.add_argument('--style', choices=sorted(STYLES), default='wine_green')
    ap.add_argument('--font', default='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')
    ap.add_argument('--segments', type=int, default=64)
    a = ap.parse_args(argv)
    style = STYLES[a.style]
    verts, norms, uvs, faces, height = lathe(style['profile'], a.segments)
    a.out.mkdir(parents=True, exist_ok=True)
    name = 'bottle_' + a.style
    write_obj(a.out / (name + '.obj'), verts, norms, uvs, faces, name)
    write_texture(a.out / (name + '.png'), style, height, a.font)
    print('%s: %d vertices, %d faces, height %.3f m' % (name, len(verts), len(faces), height))


if __name__ == '__main__':
    main()
