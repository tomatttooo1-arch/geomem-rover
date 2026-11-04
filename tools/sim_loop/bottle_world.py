#!/usr/bin/env python3
"""Line D2 (2026-10-14): car-mount world variants with a textured mesh bottle (offline render probe and
the D2 world). Only the bottle, its position and the light change; architecture, robot, camera, the cup
and the book are copied unchanged.

  bottle_world.py BASE.sdf OUT.sdf --bottle wine_green|beer_brown|scanned|primitive
                  [--x 1.25] [--y -0.45] [--yaw 0] [--scale 1] [--light orig|bright]
                  [--props-x X]   (x of the cup and the book too; default: unchanged)

wine_green / beer_brown: robot_sim/fpga_sim/worlds/props/bottle_<style>.obj + .png (make_bottle_mesh.py),
referenced at MESH_ROOT (the installed share path the run containers mount). scanned: the Google Scanned
Objects bottle (CC BY 4.0, runs/20260908-scanned-bottle-asset/model, mounted read-only at /fpga_bottle_asset)
with its texture as the PBR albedo map and a white colour multiplier (sdformat 1.8 defaults ambient/diffuse
to black, 09-08 finding). primitive: the D1 cylinders moved to --x/--y.
Lights: orig = room_carmount.sdf before D1 (sun diffuse 0.9, direction -0.5 0.1 -0.9, attenuation
0.9/0.01/0.001, no <scene> ambient); bright = D1's (ambient 0.8, diffuse 1, direction 0.4 0 -1, none).
The bottle's collision is a cylinder of the mesh's radius and height (static props).
"""
import argparse
import math
import xml.etree.ElementTree as ET

MESH_ROOT = '/opt/fpga_ws/install/share/fpga_sim/worlds/props'
SCANNED_ROOT = '/fpga_bottle_asset'
MESHES = {   # name: (obj, albedo png, radius m, height m) at scale 1
    'wine_green': (MESH_ROOT + '/bottle_wine_green.obj', MESH_ROOT + '/bottle_wine_green.png', 0.0375, 0.300),
    'beer_brown': (MESH_ROOT + '/bottle_beer_brown.obj', MESH_ROOT + '/bottle_beer_brown.png', 0.0340, 0.250),
    'scanned': (SCANNED_ROOT + '/meshes/model.obj', SCANNED_ROOT + '/materials/textures/texture.png', 0.0328, 0.1987),
}
BOTTLE_PARTS = ('bottle_body', 'bottle_neck', 'bottle_cap')


def set_light(world, light):
    scene = world.find('scene')
    sun = world.find("light[@name='sun']")
    att = sun.find('attenuation')
    if light == 'bright':
        if scene is None:
            scene = ET.SubElement(world, 'scene')
        amb = scene.find('ambient')
        if amb is None:
            amb = ET.SubElement(scene, 'ambient')
        amb.text = '0.8 0.8 0.8 1'
        sun.find('diffuse').text = '1 1 1 1'
        sun.find('direction').text = '0.4 0.0 -1.0'
        att.find('constant').text, att.find('linear').text, att.find('quadratic').text = '1.0', '0.0', '0.0'
    elif light == 'orig':
        if scene is not None:
            world.remove(scene)
        sun.find('diffuse').text = '0.9 0.9 0.9 1'
        sun.find('direction').text = '-0.5 0.1 -0.9'
        att.find('constant').text, att.find('linear').text, att.find('quadratic').text = '0.9', '0.01', '0.001'
    else:
        raise ValueError(light)


def el(parent, tag, text=None, **attrib):
    e = ET.SubElement(parent, tag, attrib)
    if text is not None:
        e.text = text
    return e


def mesh_bottle(world, kind, x, y, yaw, scale):
    obj, png, radius, height = MESHES[kind]
    model = el(world, 'model', name='bottle_prop')
    el(model, 'static', 'true')
    el(model, 'pose', '%.4f %.4f 0 0 0 %.4f' % (x, y, yaw))
    link = el(model, 'link', name='bottle')
    col = el(link, 'collision', name='bottle_collision')
    el(col, 'pose', '0 0 %.4f 0 0 0' % (height * scale / 2))
    cyl = el(el(col, 'geometry'), 'cylinder')
    el(cyl, 'radius', '%.4f' % (radius * scale))
    el(cyl, 'length', '%.4f' % (height * scale))
    vis = el(link, 'visual', name='bottle_visual')
    mesh = el(el(vis, 'geometry'), 'mesh')
    el(mesh, 'uri', obj)
    el(mesh, 'scale', '%g %g %g' % (scale, scale, scale))
    mat = el(vis, 'material')
    el(mat, 'ambient', '1 1 1 1')
    el(mat, 'diffuse', '1 1 1 1')
    el(mat, 'specular', '0.6 0.6 0.6 1')
    metal = el(el(mat, 'pbr'), 'metal')
    el(metal, 'albedo_map', png)
    el(metal, 'roughness', '0.25' if kind != 'scanned' else '0.6')
    el(metal, 'metalness', '0.0')


def build(base, out, bottle, x, y, yaw, scale, light, props_x=None):
    tree = ET.parse(base)
    world = tree.find('world')
    props = world.find(".//model[@name='rendered_props']")
    link = props.find('link')
    if props_x is not None:
        for child in link:
            if not child.get('name', '').startswith('bottle_'):
                pose = child.find('pose')
                v = pose.text.split()
                v[0] = '%.4f' % props_x
                pose.text = ' '.join(v)
    for child in list(link):
        name = child.get('name', '')
        if any(name.startswith(p) for p in BOTTLE_PARTS):
            if bottle == 'primitive':
                pose = child.find('pose')
                v = pose.text.split()
                v[0], v[1] = '%.4f' % x, '%.4f' % y
                pose.text = ' '.join(v)
            else:
                link.remove(child)
    if bottle != 'primitive':
        mesh_bottle(world, bottle, x, y, yaw, scale)
    set_light(world, light)
    comment = ET.Comment(' bottle_world.py: bottle=%s x=%.3f y=%.3f yaw=%.3f scale=%g light=%s props_x=%s '
                         % (bottle, x, y, yaw, scale, light, props_x))
    world.insert(0, comment)
    tree.write(out, encoding='unicode', xml_declaration=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('base')
    ap.add_argument('out')
    ap.add_argument('--bottle', choices=('wine_green', 'beer_brown', 'scanned', 'primitive'), required=True)
    ap.add_argument('--x', type=float, default=1.25)
    ap.add_argument('--y', type=float, default=-0.45)
    ap.add_argument('--yaw', type=float, default=0.0, help='rad; the procedural label faces yaw + pi')
    ap.add_argument('--scale', type=float, default=1.0)
    ap.add_argument('--light', choices=('orig', 'bright'), default='orig')
    ap.add_argument('--props-x', type=float)
    a = ap.parse_args(argv)
    if not (0.5 <= a.scale <= 2.0) or not math.isfinite(a.yaw):
        ap.error('scale in [0.5, 2], finite yaw')
    build(a.base, a.out, a.bottle, a.x, a.y, a.yaw, a.scale, a.light, a.props_x)


if __name__ == '__main__':
    main()
