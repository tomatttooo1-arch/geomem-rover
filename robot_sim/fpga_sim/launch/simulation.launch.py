"""Server process: physics/rendering/ROS bridge only, no object truth bridge."""
from pathlib import Path
import os
import tempfile
import yaml
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("fpga_sim"))
    inputs = runpy.run_path(str(share / "config/nav_input_profile.py"))
    profile = inputs['input_profile'](os.environ)
    bridge_file = share / "config/bridge.yaml"
    generated_bridge = None
    cloud = inputs['cloud_source'](os.environ)
    # FPGA_SIM_CMD_TOPIC (line D2, 2026-10-14; unset = unchanged): the ROS topic bridged to the simulator's
    # wheel command (Gazebo /fpga/cmd_vel) instead of /fpga/cmd_vel_safe -- the MCU stand-in's executed speed
    # (tools/chassis_uart/mcu_sim.py --twist-topic), so the simulator has exactly one speed source.
    cmd_topic = os.environ.get("FPGA_SIM_CMD_TOPIC") or None
    if profile == 'estimated_inspect' or cloud == 'adapter' or cmd_topic:
        entries = inputs['bridge_inputs'](yaml.safe_load(bridge_file.read_text()), profile)
        entries = inputs['cloud_inputs'](entries, cloud)
        if cmd_topic:
            entries = inputs['command_input'](entries, cmd_topic)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='fpga_bridge_', delete=False) as stream:
            yaml.safe_dump(entries, stream)
            generated_bridge = Path(stream.name)
        bridge_file = generated_bridge

    def cleanup(context):
        if generated_bridge is not None:
            generated_bridge.unlink(missing_ok=True)
        return []
    # Car-mount camera world by default since line D1 (2026-10-14); room.sdf is the 0.95 m mast
    # world of the 09-16 AGX runs (navigation.launch.py's camera TF now matches room_carmount.sdf).
    world = os.environ.get("FPGA_SIM_WORLD") or str(share / "worlds/room_carmount.sdf")
    simulator = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r"] +
            (["--headless-rendering"] if os.environ.get("FPGA_SIM_HEADLESS_RENDERING") == "1" else []) + [world],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="fpga_bridge",
        parameters=[{"config_file": str(bridge_file)}],
        output="screen",
    )
    # FPGA_SIM_ESTOP_TOPIC (line D1): the guard's fail-closed e-stop input (scripts/sim_estop_gate.py)
    # FPGA_SIM_GUARD_OUTPUT (2026-10-18, teleop arbitration): unset = /fpga/cmd_vel_safe (unchanged); car_up sets
    # /fpga/nav/cmd_vel_safe with CAR_TELEOP=1 + --nav nav2 (host/ros_teleop_safe.py --nav-topic arbitrates)
    guard_params = {"estop_topic": os.environ.get("FPGA_SIM_ESTOP_TOPIC", "")}
    if os.environ.get("FPGA_SIM_GUARD_OUTPUT"):
        guard_params["output_topic"] = os.environ["FPGA_SIM_GUARD_OUTPUT"]
    guard = Node(package="fpga_sim", executable="velocity_guard.py", output="screen",
                 parameters=[guard_params])
    return LaunchDescription([
        simulator, bridge, guard,
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(
            target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason="Gazebo exited"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=guard,
            on_exit=[EmitEvent(event=Shutdown(reason="motion guard exited"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="sensor bridge exited"))],
        )),
    ])
