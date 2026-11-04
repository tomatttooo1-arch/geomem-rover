"""Asset consistency checks; these do not replace a running ROS/Gazebo test."""
from pathlib import Path
import ast
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1] / "fpga_sim"


class AssetTests(unittest.TestCase):
    def test_map_matches_collision_footprints(self):
        for name in ("room.sdf", "room_carmount.sdf"):
            with self.subTest(name):
                self.check_map_footprints(ET.parse(ROOT / "worlds" / name))

    def check_map_footprints(self, world):
        architecture = world.find(".//model[@name='architecture']/link")
        footprints = []
        for collision in architecture.findall("collision"):
            if collision.attrib["name"].startswith("floor"):
                continue
            x, y, *_ = map(float, collision.findtext("pose").split())
            sx, sy, _ = map(float, collision.findtext("geometry/box/size").split())
            footprints.append((x-sx/2, x+sx/2, y-sy/2, y+sy/2))
        lines = (ROOT / "maps/room.pgm").read_text().splitlines()
        tokens = " ".join(line for line in lines if not line.startswith("#")).split()
        self.assertEqual(tokens[:4], ["P2", "64", "64", "255"])
        cells = list(map(int, tokens[4:])); self.assertEqual(len(cells), 4096)
        for row in range(64):
            for col in range(64):
                x = -3.2 + (col+.5)*.1; y = 3.2-(row+.5)*.1
                occupied = any(a <= x <= b and c <= y <= d for a,b,c,d in footprints)
                # Outside physical walls is deliberately closed, not free space.
                occupied = occupied or abs(x) > 3.1 or abs(y) > 3.1
                self.assertEqual(cells[row*64+col] == 0, occupied, (x,y))

    def test_bridge_excludes_item_truth_and_wheel_tf(self):
        bridges = yaml.safe_load((ROOT / "config/bridge.yaml").read_text())
        topics = {b["ros_topic_name"] for b in bridges}
        self.assertEqual(topics, {"/clock", "/fpga/rgb", "/fpga/depth", "/fpga/camera_info",
                                  "/fpga/points", "/odom", "/tf", "/fpga/cmd_vel_safe"})
        self.assertEqual(sum(b["direction"] == "ROS_TO_GZ" for b in bridges), 1)
        self.assertFalse(any("wheel_" in b["gz_topic_name"] for b in bridges))
        self.assertTrue(all(b["gz_type_name"].startswith("ignition.msgs.") for b in bridges))

    def test_camera_and_reference_pose_contract(self):
        # Since line D1 (2026-10-14) the launch TF describes the car-mount world, the default of
        # simulation.launch.py; room.sdf (0.95 m mast) is kept as the 09-16 AGX evidence world.
        world = ET.parse(ROOT / "worlds/room_carmount.sdf")
        camera = world.find(".//sensor[@type='rgbd_camera']")
        self.assertEqual(camera.findtext("ignition_frame_id"), "camera_optical_frame")
        self.assertEqual(camera.findtext("camera/optical_frame_id"), "camera_optical_frame")
        self.assertEqual(camera.findtext("pose"), "0.13 0 0.35 0 0.471238898 0")
        self.assertEqual(camera.findtext("camera/horizontal_fov"), "1.58")
        self.assertEqual((camera.findtext("camera/image/width"), camera.findtext("camera/image/height")),
                         ("640", "400"))
        self.assertIn('worlds/room_carmount.sdf', (ROOT / "launch/simulation.launch.py").read_text())
        old = ET.parse(ROOT / "worlds/room.sdf").find(".//sensor[@type='rgbd_camera']")
        self.assertEqual(old.findtext("pose"), "0.18 0 0.95 0 0.25 0")
        base = world.find(".//model[@name='fpga_robot']/link[@name='base_link']")
        for kind in ("visual", "collision"):
            self.assertEqual(base.findtext(f"{kind}[@name='camera_{kind}']/pose"), camera.findtext("pose"))
        launch = ast.parse((ROOT / "launch/navigation.launch.py").read_text())
        call = next(node for node in ast.walk(launch)
                    if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "static_tf"
                    and node.args and isinstance(node.args[0], ast.Constant)
                    and node.args[0].value == "base_to_camera")
        values = {key.arg: ast.literal_eval(key.value) for key in call.keywords}
        sensor_pose = list(map(float, camera.findtext("pose").split()))
        self.assertEqual(list(values["xyz"])+list(values["rpy"]), sensor_pose)
        odom = world.find(".//plugin[@name='ignition::gazebo::systems::OdometryPublisher']")
        self.assertEqual(odom.findtext("dimensions"), "3")
        self.assertEqual(odom.findtext("robot_base_frame"), "base_link")

    def test_carmount_props_are_on_the_floor_in_view_and_not_on_the_table(self):
        world = ET.parse(ROOT / "worlds/room_carmount.sdf")
        props = world.find(".//model[@name='rendered_props']/link")
        poses = {c.attrib["name"]: list(map(float, c.findtext("pose").split()))
                 for c in props.findall("collision")}
        body = poses["bottle_body_collision"]
        self.assertAlmostEqual(body[2], 0.11)            # 0.22 m body standing on z = 0
        for name, (x, y, z, *_r) in poses.items():
            self.assertLess(x, 1.4, name)                 # table front edge is x = 1.4
            self.assertLess(z, 0.3, name)
        # the bottle centre is inside the start view: 1.58 rad horizontal FOV, 640x400, pitch 27 deg
        import math
        fov_v = 2 * math.atan(math.tan(1.58 / 2) * 400 / 640)
        bearing = math.atan2(body[1], body[0] - 0.13)
        elevation = math.atan2(body[2] - 0.35, math.hypot(body[0] - 0.13, body[1]))
        self.assertLess(abs(bearing), 1.58 / 2)
        self.assertLess(abs(elevation + 0.471238898), fov_v / 2)


if __name__ == "__main__":
    unittest.main()
