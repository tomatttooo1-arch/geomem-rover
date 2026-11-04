# tools/kv_chassis (CHASSIS0, 2026-11-04)

The chassis serial on the KV260: `host/kv_chassis_bridge.py` inside kv_server (`--chassis kv`).
Design, switch, on-car steps and the comparison results: `runs/nationals-prelim-20261102/CHASSIS0/` (DESIGN.md, README.md).

- `compare_nano.py RUN_DIR --out R.json`: the key host comparison -- the Nano's own `host/ros_chassis_driver.py`
  (run with a stub rclpy over a pty) vs the bridge, command frames byte for byte; the bridge's odometry vs the
  Nano odometry node's recorded rows, exact; negative controls for both.
- `params_lift_test.json`: `params_car_bench.json` with the goal 0.3 m ahead (first lifted-wheel test).
