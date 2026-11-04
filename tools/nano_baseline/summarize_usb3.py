"""Summarise probe v3 JSON outputs of a usb3_batch.sh run into a CSV and a markdown table."""
import csv, glob, json, os, re, statistics, sys
d = sys.argv[1]
rows = []
for f in sorted(glob.glob(os.path.join(d, "*_r[0-9].json"))):
    txt = open(f).read()
    js = txt[:txt.index("DEPTH_VALUES")] if "DEPTH_VALUES" in txt else txt
    j = json.loads(js)
    dv = re.search(r"DEPTH_VALUES .*nonzero=(\d+)/(\d+)", txt)
    name = os.path.basename(f)[:-5]
    cfg, rep = name.rsplit("_r", 1)
    dp, cp = j.get("depth", {}), j.get("color", {})
    sd = j.get("settle_depth", {})
    w = j.get("sdk_warnings", {})
    def wsum(prefix, where):
        k = f"{prefix}@{where}"
        return w.get(k, {}).get("lines", 0), w.get(k, {}).get("aggregated_suffix_sum", 0)
    rows.append({
        "config": cfg, "rep": int(rep), "connection": j["connection"], "laser": j.get("laser"), "ldp_triggered": j.get("ldp_triggered"),
        "depth_frames": dp.get("frames"), "depth_missing_by_index": dp.get("missing_by_index"), "depth_wrong_size": dp.get("delivered_wrong_size"),
        "depth_fps_dev": dp.get("fps_device_ts"), "depth_int_host_p50": dp.get("interval_host_ms", {}).get("p50"),
        "depth_int_host_p95": dp.get("interval_host_ms", {}).get("p95"), "depth_int_host_p99": dp.get("interval_host_ms", {}).get("p99"),
        "depth_int_host_max": dp.get("interval_host_ms", {}).get("max"), "depth_int_dev_p50": dp.get("interval_device_ms", {}).get("p50"),
        "depth_int_dev_max": dp.get("interval_device_ms", {}).get("max"), "depth_over_1p5x": dp.get("interval_host_ms", {}).get("over_1p5x_nominal"),
        "color_frames": cp.get("frames", ""), "color_missing_by_index": cp.get("missing_by_index", ""), "color_fps_dev": cp.get("fps_device_ts", ""),
        "color_int_host_max": cp.get("interval_host_ms", {}).get("max", ""),
        "settle_depth_frames": sd.get("frames"), "settle_depth_first_index": sd.get("first_index"),
        "trunc_settle": wsum("truncated_frame_dropped", "settle")[0], "trunc_window_lines": wsum("truncated_frame_dropped", "window")[0],
        "trunc_window_aggsum": wsum("truncated_frame_dropped", "window")[1],
        "oversize_settle": wsum("oversize_frame_exception", "settle")[0], "oversize_window": wsum("oversize_frame_exception", "window")[0],
        "imu_accel": j.get("imu", {}).get("accel_samples", ""), "imu_gyro": j.get("imu", {}).get("gyro_samples", ""),
        "proc_cpu_pct_one_core": j["proc_cpu_pct_of_one_core"], "proc_cpu_ms_per_depth_frame": j["proc_cpu_ms_per_depth_frame"],
        "system_cpu_busy_pct": j["system_cpu_busy_pct"], "wall_s": j["wall_s"],
        "depth_nonzero_last_frame": f"{dv.group(1)}/{dv.group(2)}" if dv else ""})
with open(os.path.join(d, "summary.csv"), "w", newline="") as fp:
    wr = csv.DictWriter(fp, fieldnames=list(rows[0])); wr.writeheader(); wr.writerows(rows)
# per-config aggregate
cfgs = []
for r in rows:
    if r["config"] not in cfgs: cfgs.append(r["config"])
print("| 配置 | 趟 | 深度帧/趟 | 窗口内按索引丢帧 | 深度 fps(设备戳) | 主机帧间隔 p50/p95/p99/max ms | 彩色帧/趟 | 启动 2 s 内截断/超长 | 进程 CPU %单核 (min–max) | ms CPU/深度帧 | 整板忙 % |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for c in cfgs:
    rs = [r for r in rows if r["config"] == c]
    f = lambda k: [r[k] for r in rs]
    cpu = f("proc_cpu_pct_one_core"); msf = f("proc_cpu_ms_per_depth_frame")
    print(f"| {c} | {len(rs)} | {'/'.join(map(str, f('depth_frames')))} | {sum(f('depth_missing_by_index'))} | "
          f"{statistics.median(f('depth_fps_dev')):.3f} | {max(f('depth_int_host_p50')):.2f}/{max(f('depth_int_host_p95')):.2f}/{max(f('depth_int_host_p99')):.2f}/{max(f('depth_int_host_max')):.2f} | "
          f"{'/'.join(map(str, f('color_frames'))) if rs[0]['color_frames'] != '' else '—'} | "
          f"{sum(f('trunc_settle'))}/{sum(f('oversize_settle'))} (窗口内 {sum(f('trunc_window_lines'))}/{sum(f('oversize_window'))}) | "
          f"{statistics.median(cpu):.1f} ({min(cpu):.1f}–{max(cpu):.1f}) | {statistics.median(msf):.2f} | {statistics.median(f('system_cpu_busy_pct')):.1f} |")
