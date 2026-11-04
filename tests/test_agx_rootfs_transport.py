"""Inspect rootfs launch arguments without creating namespaces or running ROS.

New coverage: malformed/escaping SHM directories cannot cause a /dev/shm mount;
valid opt-in preserves UDP whitelist and PID/IPC isolation. Runtime transport
delivery remains the responsibility of real two-process calibration.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve().parents[1] / "tools/agx_rootfs.sh"


class RootfsTransportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = Path(self.tmp.name)
        self.rootfs = self.base / "rootfs"
        (self.rootfs / "opt/ros/humble").mkdir(parents=True)
        (self.rootfs / "bin").mkdir()
        (self.rootfs / "bin/bash").write_text("#!/bin/bash\n")
        (self.rootfs / "bin/bash").chmod(0o700)
        self.project = self.base / "project"
        for name in ("host", "perception"):
            (self.project / name).mkdir(parents=True)
        self.workspace = self.base / "workspace"
        self.shm = self.workspace / ("ipc/bottle-test-"+self.base.name+"-dds")
        self.shm.mkdir(parents=True, mode=0o700)
        self.bin = self.base / "bin"
        self.bin.mkdir()
        fake = self.bin / "bwrap"
        fake.write_text("#!/usr/bin/python3\nimport json, os, sys\n"
                        "args = sys.argv[1:]\n"
                        "fd = int(args[args.index('--ro-bind-data') + 1])\n"
                        "print(json.dumps({'args': args, 'xml': os.read(fd, 65536).decode()}))\n")
        fake.chmod(0o700)

    def tearDown(self):
        if hasattr(self,"ram") and self.ram.exists(): self.ram.rmdir()
        self.tmp.cleanup()

    def run_script(self, **settings):
        env = {k: v for k, v in os.environ.items() if not k.startswith("FPGA_")}
        env.update(PATH=str(self.bin) + ":" + env["PATH"])
        env.update({k: str(v) for k, v in settings.items()})
        return subprocess.run(["bash", str(SCRIPT), str(self.rootfs), str(self.project), "true"],
                              env=env, text=True, capture_output=True)

    def assert_profile(self, result, kinds):
        self.assertEqual(result.returncode, 0, result.stderr)
        captured = json.loads(result.stdout)
        xml = ET.fromstring(captured["xml"])
        ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        self.assertEqual([e.text for e in xml.findall(".//d:type", ns)], kinds)
        self.assertEqual([e.text for e in xml.findall(".//d:interfaceWhiteList/d:address", ns)],
                         ["127.0.0.1"])
        self.assertEqual(xml.find(".//d:useBuiltinTransports", ns).text, "false")
        self.assertIn("--unshare-pid", captured["args"])
        self.assertIn("--unshare-ipc", captured["args"])
        return captured

    def test_shared_rootfs_and_release_readonly_with_private_home(self):
        captured = self.assert_profile(self.run_script(FPGA_AGX_WORKSPACE=self.workspace), ['UDPv4'])
        args = captured['args']
        for source, target in ((self.rootfs, '/'), (self.project, '/fpga')):
            index = args.index(str(source))
            self.assertEqual(args[index-1:index+2], ['--ro-bind', str(source), target])
        index = args.index(str(self.workspace))
        self.assertEqual(args[index-1:index+2], ['--bind', str(self.workspace), '/workspace'])
        for directory in ('/root', '/tmp'):
            index = args.index(directory)
            self.assertEqual(args[index-1], '--tmpfs')
        self.assertIn('HOME=/root', args)

    def test_default_udp_preserves_private_shm(self):
        captured = self.assert_profile(self.run_script(), ["UDPv4"])
        self.assertNotIn("/dev/shm", captured["args"])

    def test_valid_session_shm_with_udp_loopback(self):
        captured = self.assert_profile(self.run_script(
            FPGA_DDS_TRANSPORT="shm_udp", FPGA_AGX_WORKSPACE=self.workspace,
            FPGA_DDS_SHM_DIR=self.shm), ["UDPv4", "SHM"])
        index = captured["args"].index("/dev/shm")
        self.assertEqual(captured["args"][index-2:index], ["--bind", str(self.shm)])

    def test_owned_session_tmpfs_mount_keeps_exact_project_marker(self):
        self.ram=Path('/dev/shm')/('fpga-'+self.workspace.name+'-'+self.shm.name)
        self.ram.mkdir(mode=0o700)
        captured=self.assert_profile(self.run_script(FPGA_DDS_TRANSPORT='shm_udp',
            FPGA_AGX_WORKSPACE=self.workspace,FPGA_DDS_SHM_DIR=self.shm,
            FPGA_DDS_SHM_BACKING=self.ram),['UDPv4','SHM'])
        args=captured['args']; index=args.index('/dev/shm')
        self.assertEqual(args[index-2:index],['--bind',str(self.ram)])
        index=args.index('/tmp/fpga-session')
        self.assertEqual(args[index-2:index],['--ro-bind',str(self.shm)])

    def test_ram_rejects_host_root_and_other_session(self):
        for bad in ('/dev/shm',self.base):
            result=self.run_script(FPGA_DDS_TRANSPORT='shm_udp',FPGA_AGX_WORKSPACE=self.workspace,
                                  FPGA_DDS_SHM_DIR=self.shm,FPGA_DDS_SHM_BACKING=bad)
            self.assertNotEqual(result.returncode,0)
            self.assertEqual(result.stdout,'')

    def test_shm_rejects_missing_workspace(self):
        self.assertNotEqual(self.run_script(FPGA_DDS_TRANSPORT="shm_udp").returncode, 0)

    def test_shm_rejects_outside_workspace(self):
        outside = self.base / "outside-dds"
        outside.mkdir()
        result = self.run_script(FPGA_DDS_TRANSPORT="shm_udp", FPGA_AGX_WORKSPACE=self.workspace,
                                 FPGA_DDS_SHM_DIR=outside)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")

    def test_shm_rejects_ipc_symlink_escape(self):
        other = self.base / "other"
        other.mkdir()
        (other / "ipc").symlink_to(self.workspace / "ipc", target_is_directory=True)
        self.assertNotEqual(self.run_script(
            FPGA_DDS_TRANSPORT="shm_udp", FPGA_AGX_WORKSPACE=other,
            FPGA_DDS_SHM_DIR=self.shm).returncode, 0)

    def test_shm_rejects_missing_session_directory(self):
        self.assertNotEqual(self.run_script(
            FPGA_DDS_TRANSPORT="shm_udp", FPGA_AGX_WORKSPACE=self.workspace,
            FPGA_DDS_SHM_DIR=self.shm.parent / "missing-dds").returncode, 0)

    def test_bounded_reply_changes_only_service_writer_wait(self):
        captured = self.assert_profile(self.run_script(FPGA_SERVICE_REPLY_PROFILE='bounded'), ['UDPv4'])
        xml = ET.fromstring(captured['xml'])
        ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
        writers = xml.findall('d:publisher', ns)
        self.assertEqual(len(writers), 1)
        self.assertEqual(writers[0].attrib, {'profile_name': 'service'})
        wait = writers[0].find('d:qos/d:reliability/d:max_blocking_time', ns)
        self.assertEqual([e.text for e in wait], ['1', '0'])
        self.assertIn('FPGA_SERVICE_REPLY_PROFILE=bounded', captured['args'])
        self.assertFalse(any(x.startswith('RMW_FASTRTPS_USE_QOS_FROM_XML=') for x in captured['args']))
        legacy = self.assert_profile(self.run_script(FPGA_SERVICE_REPLY_PROFILE='legacy'), ['UDPv4'])
        self.assertEqual(ET.fromstring(legacy['xml']).findall('d:publisher', ns), [])

    def test_rejects_unknown_service_reply_profile(self):
        self.assertNotEqual(self.run_script(FPGA_SERVICE_REPLY_PROFILE='anything').returncode, 0)

    def test_rejects_unknown_transport(self):
        self.assertNotEqual(self.run_script(FPGA_DDS_TRANSPORT="default").returncode, 0)


if __name__ == "__main__":
    unittest.main()
