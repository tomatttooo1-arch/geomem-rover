"""Start-up and running health check of the car stack (10-29, owner: report what makes the later run fail BEFORE the
mapping starts -- 10-28 a refused ground fit was never shown, the mapping was wasted and the find stood still).

Warn only: nothing here refuses, stops, or blocks mapping / finding (owner rule 10-26, no new defensive gates).

The hub (host/hub/hub_ros.py) runs ``HealthMonitor.step`` once a second on a thread of its own: inputs are a
read-only snapshot of the hub (``HubCore.health_inputs``: the last composed STATE, per-input receive ages, callback
counters, the newest /fpga/chassis/status and /fpga/wheel_odom/status) plus the run directory (/o: READY,
ground_calib.json, inference.log, camera_rate_check.json, host_facts.json written by car_up.sh on the Nano host).
Output: STATE "health" (bridge snapshot "health" -> the console's check panel under the connection bar), and
<run>/health.jsonl (first report after READY, every level change, a full report every 60 s) + <run>/health.json
(the newest report).

Each check: {code, level, title, detail, hint}; level red (fix before mapping: the run will fail or the demo is
broken), yellow (works, but look at it), green, pending (the stack is still starting). The hint is one Chinese
sentence telling the operator what to do. Before READY the report is pending ("车栈启动中").

Python 3.8, no ROS imports (tests run it with fake inputs).
"""

import collections
import glob
import json
import os
import sys
import time

LEVELS = ("red", "yellow", "green", "pending")
RANK = {"red": 0, "yellow": 1, "pending": 2, "green": 3}

# demo objects that must be in the detector's class set (owner 10-29 night: the six demo items = console chips);
# env FPGA_HEALTH_EXPECT_CLASSES="bottle,backpack,..." replaces the list
DEFAULT_EXPECT_CLASSES = ("backpack", "cup", "bottle", "banana", "laptop")   # owner 10-31: medicine box dropped
CLASS_ZH = {"backpack": "书包", "cup": "水杯", "mouse": "鼠标", "bottle": "瓶子", "remote": "遥控器",
            "medicine bottle": "药瓶", "medicine box": "药盒", "key": "钥匙", "glasses": "眼镜",
            "cell phone": "手机", "drink can": "饮料罐", "earbud case": "耳机充电盒", "keyboard": "键盘",
            "laptop": "笔记本电脑", "book": "书", "banana": "香蕉", "apple": "苹果", "orange": "橙子"}

POWER_MODE_EXPECTED = "MAXN_SUPER"
DISK_RED_GB, DISK_YELLOW_GB = 2.0, 5.0
BATTERY_YELLOW_V, BATTERY_RED_V = 21.6, 20.4      # 6S pack (owner 10-26: 24 V six-cell battery), 3.6 / 3.4 V a cell
VIN_YELLOW_MV = 4750                               # Orin Nano module input (INA3221 in1, ~5.0 V nominal)
CLOCK_YELLOW_S = 2.0
CMD_V_MM_S, CMD_W_MRAD_S = 30, 150                 # well above the dead band (0.005 m/s, 0.04 rad/s)
STILL_COUNTS, STILL_S = 20, 1.5                    # < 20 encoder counts in 1.5 s at >= 3 cm/s = the wheels do not turn
SEQ_FROZEN_S = 1.0                                 # last_cmd_seq unchanged that long while 50 Hz commands go out
STALE = {"chassis_s": 3.0, "mcu_ms": 1000.0, "depth_s": 3.0, "preview_s": 3.0, "detect_s": 5.0, "telem_s": 3.0,
         "permit_s": 3.0, "odom_s": 2.0, "depth_zero_s": 2.0, "heartbeat_s": 2.0}
RATE_WINDOW_S = 5.0
REPORT_EVERY_S = 60.0
SNAPSHOT_EVERY_S = 5.0


def _check(code, level, title, detail, hint=None):
    c = {"code": code, "level": level, "title": title, "detail": detail}
    if hint and level in ("red", "yellow"):
        c["hint"] = hint
    return c


def _num(v):
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _read_json(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _fmt_dur(s):
    s = abs(float(s))
    if s >= 86400:
        return "%.1f 天" % (s / 86400)
    if s >= 3600:
        return "%.1f 小时" % (s / 3600)
    if s >= 60:
        return "%.0f 分钟" % (s / 60)
    return "%.1f 秒" % s


class Rate(object):
    """Events per second of a monotonically growing counter over the last RATE_WINDOW_S."""

    def __init__(self, window_s=RATE_WINDOW_S):
        self.window_s = window_s
        self.rows = collections.deque()

    def feed(self, t, count):
        if count is None:
            return
        if self.rows and count < self.rows[-1][1]:        # counter restarted (hub restart): start over
            self.rows.clear()
        self.rows.append((t, count))
        while len(self.rows) > 2 and t - self.rows[0][0] > self.window_s:
            self.rows.popleft()

    def value(self):
        if len(self.rows) < 2 or self.rows[-1][0] <= self.rows[0][0]:
            return None
        return (self.rows[-1][1] - self.rows[0][1]) / (self.rows[-1][0] - self.rows[0][0])


# ------------------------------------------------------------------------------------------- one-shot run facts
def ground_calib_check(g):
    """ground_calib.json (tools/car_run/ground_calib.py) -> check. None = no file (calibration off)."""
    title = "地面标定"
    hint = "检查相机支架是否被碰歪，扶正拧紧后重新起栈"
    if g is None:
        return _check("ground_calib", "yellow", title, "这次没做地面标定，用的是配置的安装角度",
                      "确认相机支架没动过；不确定就重新起栈让它标定")
    if not isinstance(g, dict):
        return _check("ground_calib", "red", title, "ground_calib.json 读不懂", hint)
    a = g.get("adopted") if isinstance(g.get("adopted"), dict) else {}
    f = g.get("fitted") if isinstance(g.get("fitted"), dict) else {}
    src = a.get("source")
    verdict = g.get("verdict")

    def mount(d):
        h, p, r = _num(d.get("height_m")), _num(d.get("pitch_deg")), _num(d.get("roll_deg"))
        if h is None or p is None:
            return None
        return "高 %.3f m · 下倾 %.1f°%s" % (h, p, "" if r is None else " · 横滚 %.1f°" % r)

    if verdict not in ("adopt", "confirm"):
        why = "；".join(str(x) for x in (g.get("reasons") or [])[:2]) or "没找到可用的地面"
        return _check("ground_calib", "red", title, "标定失败，用的是配置值（%s）" % why[:80],
                      "让车正对空旷地面（前方 0.5–2 m 别有东西），检查相机支架，然后重新起栈")
    if src in ("last_good", "config_fallback") or g.get("alert"):
        used = "上次标定值" if src == "last_good" else "默认安装值"
        fit = mount(f)
        detail = "拟合结果被拒%s，退回%s；地板可能被当成障碍、寻物会原地不动" % ("（%s）" % fit if fit else "", used)
        return _check("ground_calib", "red", title, detail, hint)
    m = mount(a) or "已采用"
    return _check("ground_calib", "green", title, ("拟合采用 · " if src == "fit" else "核对通过 · ") + m)


def inference_ready(path, limit=256 * 1024):
    """The inference service's ready line in inference.log ({"ready": true, ..., "classes": [...]}) or None."""
    try:
        with open(path, "rb") as f:
            data = f.read(limit)
    except OSError:
        return None
    for line in data.decode("utf-8", "replace").splitlines():
        line = line.strip()
        if line.startswith("{") and '"ready"' in line:
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if isinstance(row, dict) and row.get("ready") is True:
                return row
    return None


def read_sysfs_power(root="/sys"):
    """{"vin_mv": module input mV (INA3221 in1) or None, "oc_events": sum of soctherm over-current / under-voltage
    event counters or None}; {} when neither is readable (e.g. a host without the Jetson sensors)."""
    out = {}
    for h in glob.glob(os.path.join(root, "class", "hwmon", "hwmon*")):
        try:
            with open(os.path.join(h, "name")) as f:
                name = f.read().strip()
        except OSError:
            continue
        if name == "ina3221":
            try:
                with open(os.path.join(h, "in1_input")) as f:
                    out["vin_mv"] = int(f.read().strip())
            except (OSError, ValueError):
                pass
        elif name == "soctherm_oc":
            total, seen = 0, False
            for p in glob.glob(os.path.join(h, "oc*_event_cnt")):
                try:
                    with open(p) as f:
                        total += int(f.read().strip())
                    seen = True
                except (OSError, ValueError):
                    pass
            if seen:
                out["oc_events"] = total
    return out


def disk_free_gb(path):
    try:
        st = os.statvfs(path)
    except OSError:
        return None
    return st.f_bavail * st.f_frsize / 1e9


# ------------------------------------------------------------------------------------------------ the monitor
class HealthMonitor(object):
    def __init__(self, run_dir, expect_classes=None, sysfs_root="/sys", mono=time.monotonic, wall=time.time,
                 out=sys.stdout, write_files=True):
        self.run_dir = run_dir
        env = os.environ.get("FPGA_HEALTH_EXPECT_CLASSES")
        if expect_classes is None:
            expect_classes = tuple(x.strip() for x in env.split(",") if x.strip()) if env else DEFAULT_EXPECT_CLASSES
        self.expect_classes = tuple(expect_classes)
        self.sysfs_root = sysfs_root
        self.mono, self.wall, self.out = mono, wall, out
        self.write_files = write_files
        self.rates = collections.defaultdict(Rate)
        self.facts = {}                      # name -> (mtime, value) of run-dir files
        self.chassis_rows = collections.deque(maxlen=64)   # (t, last_cmd_seq, cmd, enc_total)
        self.chassis_key = None
        self.cmd_still_since = None
        self.seq_frozen_since = None
        self.permit_off_since = None
        self.telem_lost_since = None
        self.hb_missing_since = None
        self.preview_seen = False
        self.oc_first = None
        self.power = {}
        self.power_t = None
        self.disk = None
        self.disk_t = None
        self.levels = {}                     # code -> level last reported
        self.last_report_t = None
        self.last_snapshot_t = None
        self.started_logged = False
        self.report = self.pending_report("车栈启动中，体检在就绪后开始")

    # ---------------------------------------------------------------- files
    def _path(self, name):
        return os.path.join(self.run_dir, name)

    def _fact(self, name, loader):
        """Run-dir file through ``loader``, re-read only when its mtime / size changes (None when absent)."""
        p = self._path(name)
        try:
            st = os.stat(p)
            mt = (st.st_mtime_ns, st.st_size)
        except OSError:
            self.facts.pop(name, None)
            return None
        old = self.facts.get(name)
        if old is not None and old[0] == mt:
            return old[1]
        val = loader(p)
        self.facts[name] = (mt, val)
        return val

    def ready(self):
        return os.path.exists(self._path("READY"))

    def pending_report(self, summary):
        return {"v": 1, "t": self.wall(), "overall": "pending", "summary": summary, "checks": []}

    # ---------------------------------------------------------------- one step
    def step(self, inp):
        """``inp`` = HubCore.health_inputs(now). Returns (and keeps in .report) the newest report."""
        t = self.mono()
        counts = inp.get("counts") or {}
        for name, value in counts.items():
            self.rates[name].feed(t, value)
        self._feed_chassis(t, inp)
        if not self.ready():
            self.report = self.pending_report("车栈启动中，体检在就绪后开始")
            return self.report
        if self.power_t is None or t - self.power_t >= 2.0:
            self.power_t = t
            self.power = read_sysfs_power(self.sysfs_root)
            if self.oc_first is None and self.power.get("oc_events") is not None:
                self.oc_first = self.power["oc_events"]
        if self.disk_t is None or t - self.disk_t >= 10.0:
            self.disk_t = t
            self.disk = disk_free_gb(self.run_dir)
        checks = [c for c in self.evaluate(t, inp) if c is not None]
        self.report = self.compose(checks)
        if self.write_files:
            self._log(t)
        return self.report

    def compose(self, checks):
        checks = sorted(checks, key=lambda c: RANK.get(c["level"], 9))
        red = [c for c in checks if c["level"] == "red"]
        yellow = [c for c in checks if c["level"] == "yellow"]
        if red:
            overall = "red"
            summary = "有 %d 项要先处理再开始：%s" % (len(red), "、".join(c["title"] for c in red))
        elif yellow:
            overall = "yellow"
            summary = "可以开始建图；%d 项提醒：%s" % (len(yellow), "、".join(c["title"] for c in yellow))
        else:
            overall = "green"
            summary = "体检通过，可以开始建图"
        return {"v": 1, "t": self.wall(), "overall": overall, "summary": summary, "checks": checks}

    # ---------------------------------------------------------------- chassis history
    def _feed_chassis(self, t, inp):
        ch = inp.get("chassis")
        if not isinstance(ch, dict):
            return
        mcu = ch.get("mcu") if isinstance(ch.get("mcu"), dict) else None
        key = (inp.get("chassis_recv_ns"), None if mcu is None else (mcu.get("seq"), mcu.get("mcu_ms")))
        if key == self.chassis_key:
            return
        self.chassis_key = key
        if mcu is None:
            return
        enc = [mcu.get(k) for k in ("enc_a", "enc_b", "enc_c", "enc_d")]
        if any(v is None for v in enc):
            enc = [mcu.get("left_count"), mcu.get("right_count")]
        enc = None if any(_num(v) is None for v in enc) else tuple(enc)
        cmd = ch.get("cmd") if isinstance(ch.get("cmd"), (list, tuple)) and len(ch.get("cmd")) >= 3 else None
        self.chassis_rows.append((t, mcu.get("last_cmd_seq"), cmd, enc))

    def _chassis_motion_verdicts(self, t):
        """(seq_frozen_s, still_s): how long the MCU has not acknowledged a new command / has been commanded to move
        without the wheels turning (None = not the case)."""
        rows = list(self.chassis_rows)
        seq_frozen = still = None
        if len(rows) >= 3:
            last_seq = rows[-1][1]
            k = len(rows) - 1
            while k > 0 and rows[k - 1][1] == last_seq:
                k -= 1
            span = rows[-1][0] - rows[k][0]
            if last_seq is not None and len(rows) - k >= 3 and span >= SEQ_FROZEN_S:
                seq_frozen = span
        # commanded-but-still: the newest unbroken run of rows that all command a clear motion
        k = len(rows)
        while k > 0:
            cmd = rows[k - 1][2]
            if cmd is None or not cmd[2] or (abs(cmd[0]) < CMD_V_MM_S and abs(cmd[1]) < CMD_W_MRAD_S):
                break
            k -= 1
        run = rows[k:]
        if len(run) >= 2 and run[-1][0] - run[0][0] >= STILL_S and run[0][3] is not None and run[-1][3] is not None:
            moved = sum(abs(a - b) for a, b in zip(run[-1][3], run[0][3]))
            if moved < STILL_COUNTS * len(run[0][3]) / 2:
                still = run[-1][0] - run[0][0]
        return seq_frozen, still

    # ---------------------------------------------------------------- the checks
    def evaluate(self, t, inp):
        st = inp.get("state") if isinstance(inp.get("state"), dict) else {}
        ages = inp.get("ages_s") or {}
        car = self._fact("car.json", _read_json)
        car = car if isinstance(car, dict) else {}
        out = [self.check_ground_calib(car), self.check_chassis(t, inp, st, car), self.check_camera(t, inp, ages),
               self.check_detector(t, inp, ages, car), self.check_kv(t, inp, st), self.check_ar(st),
               self.check_clock(st), self.check_gyro(inp, ages), self.check_odom(ages, car),
               self.check_power_mode(st), self.check_disk(), self.check_power(inp)]
        return out

    def check_ground_calib(self, car):
        if car and car.get("camera") not in (None, "live"):
            return None                       # replay / simulation: no camera mount to calibrate
        g = self._fact("ground_calib.json", _read_json)
        if g is not None:
            return ground_calib_check(g)
        gc = (car.get("camera_mount") or {}).get("ground_calib") if isinstance(car.get("camera_mount"), dict) else None
        mode = gc.get("mode") if isinstance(gc, dict) else None
        if mode in (None, "off"):
            return ground_calib_check(None)
        return _check("ground_calib", "red", "地面标定", "标定模式 %s，但没有标定结果" % mode,
                      "检查相机支架，然后重新起栈")

    def check_chassis(self, t, inp, st, car):
        title = "底盘串口"
        ch = inp.get("chassis")
        age = inp.get("chassis_age_s")
        if car.get("chassis") in ("none", "sim"):
            return None                       # bench / simulation without the MCU
        if not isinstance(ch, dict):
            return _check("chassis", "red", title, "收不到底盘驱动的状态",
                          "给单片机/底盘上电、检查 Nano 与单片机的串口线，然后重新起栈")
        if age is not None and age > STALE["chassis_s"]:
            return _check("chassis", "red", title, "底盘驱动 %s 没发状态（驱动可能退出了）" % _fmt_dur(age),
                          "重新起栈；还不行检查串口线")
        sage = _num(ch.get("status_age_ms"))
        if not ch.get("link_alive") or (sage is not None and sage > STALE["mcu_ms"]):
            return _check("chassis", "red", title,
                          "单片机 %s 没发状态帧" % ("很久" if sage is None else _fmt_dur(sage / 1000.0)),
                          "检查单片机电源和串口线；遥控器开着就关掉；恢复后这里自动变绿")
        mcu = ch.get("mcu") if isinstance(ch.get("mcu"), dict) else {}
        if mcu.get("driver_fault"):
            return _check("chassis", "red", title, "单片机报电机驱动故障", "断电检查电机和驱动板，再上电重新起栈")
        seq_frozen, still = self._chassis_motion_verdicts(t)
        if still is not None:
            if mcu.get("stop_latched") or mcu.get("heartbeat_seen") is False:
                return _check("chassis", "red", title, "下发行驶命令 %s 了，单片机停机锁存（没收到 KV 行驶心跳），轮子不转"
                              % _fmt_dur(still), "解除急停；看「KV260 链路」项是否放行，再检查 KV 到单片机的心跳线")
            return _check("chassis", "red", title, "下发行驶命令 %s 了，轮子没转" % _fmt_dur(still),
                          "关掉遥控器（遥控器开着时单片机不听 Nano 的命令）；再检查电机电源")
        if seq_frozen is not None:
            return _check("chassis", "red", title, "单片机 %s 没接收 Nano 的串口命令" % _fmt_dur(seq_frozen),
                          "关掉遥控器（遥控器开着时单片机不听串口命令）；仍不行检查串口线 TX")
        kvp = ((st.get("kv") or {}).get("permit") or {}) if isinstance(st.get("kv"), dict) else {}
        hb = mcu.get("heartbeat_seen")
        if hb is False and kvp.get("permit") is True:
            if self.hb_missing_since is None:
                self.hb_missing_since = t
        else:
            self.hb_missing_since = None
        if self.hb_missing_since is not None and t - self.hb_missing_since >= STALE["heartbeat_s"]:
            return _check("chassis", "red", title, "KV 已放行，但单片机收不到 KV 的行驶心跳，车不会动",
                          "检查 KV 到单片机的心跳线是否插好")
        frame = ch.get("mcu_frame") or ""
        detail = "单片机在线%s · 行驶时自动检查遥控器" % (" · 0x82 帧" if frame == "0x82" else "")
        if ch.get("warning"):
            return _check("chassis", "yellow", title, detail + "；单片机还在发旧帧（陀螺不可用）",
                          "单片机固件可能是旧版，航向会只用轮子")
        return _check("chassis", "green", title, detail)

    def check_camera(self, t, inp, ages):
        title = "相机"
        cr = self._fact("camera_rate_check.json", _read_json)
        start_note = ""
        start_yellow = None
        if isinstance(cr, dict) and isinstance(cr.get("rate"), dict):
            r = cr["rate"]
            dm, cm = _num(r.get("depth_median_ms")), _num(r.get("colour_median_ms"))
            if dm and cm:
                start_note = " · 起栈测 彩色 %.0f / 深度 %.0f 帧/秒" % (1000.0 / cm, 1000.0 / dm)
            if r.get("verdict") not in (None, "normal"):
                start_yellow = "起栈时相机帧率不正常（%s）" % r.get("verdict")
        dz = _num(inp.get("depth_invalid_s"))
        if dz is not None and dz >= STALE["depth_zero_s"]:
            return _check("camera", "red", title, "深度图全是 0 已 %s（深度流没起来）" % _fmt_dur(dz),
                          "重新起栈；还不行拔插相机 USB 再起栈")
        fs_age = ages.get("frame_stats")
        if fs_age is None:
            return _check("camera", "red", title, "收不到深度帧", "检查相机 USB 线（插 USB3 口），然后重新起栈")
        if fs_age > STALE["depth_s"]:
            return _check("camera", "red", title, "深度帧断了 %s" % _fmt_dur(fs_age),
                          "检查相机 USB 线，然后重新起栈")
        pv_age = ages.get("preview")
        if pv_age is not None:
            self.preview_seen = True
        if self.preview_seen and (pv_age is None or pv_age > STALE["preview_s"]):
            return _check("camera", "red", title, "彩色画面断了 %s" % _fmt_dur(pv_age or 0),
                          "检查相机 USB 线，然后重新起栈")
        depth_fps = self.rates["cb:/fpga/kv_link/frame_stats"].value()
        pv_fps = self.rates["cb:/fpga/preview/jpeg"].value()
        parts = []
        if pv_fps is not None and self.preview_seen:
            parts.append("画面 %.0f 帧/秒" % pv_fps)
        if depth_fps is not None:
            parts.append("深度 %.1f 帧/秒" % depth_fps)
        detail = " · ".join(parts) or "彩色、深度在发"
        if start_yellow:
            return _check("camera", "yellow", title, start_yellow + "；" + detail,
                          "重新起栈一次；仍不正常拔插相机 USB")
        return _check("camera", "green", title, detail + start_note)

    def check_detector(self, t, inp, ages, car):
        title = "识别"
        if car.get("pose") == "map":
            return None                       # mapping-only run: no detector, no runner
        ready = self._fact("inference.log", inference_ready)
        classes = ready.get("classes") if isinstance(ready, dict) and isinstance(ready.get("classes"), list) else None
        if ages.get("runtime_info") is None and ages.get("status_frame") is None:
            return _check("detector", "red", title, "识别程序没有起来（没收到它的启动信息）", "重新起栈")
        fp_age = ages.get("status_frame")
        if fp_age is None or fp_age > STALE["detect_s"]:
            return _check("detector", "red", title,
                          "识别 %s 没有出结果" % ("一直" if fp_age is None else _fmt_dur(fp_age)),
                          "看相机项是否正常；然后重新起栈")
        fps = self.rates["frames_processed_seen"].value()
        parts = []
        if classes is not None:
            parts.append("%d 类" % len(classes))
        if fps is not None:
            parts.append("%.1f 帧/秒" % fps)
        detail = "识别 " + " · ".join(parts) if parts else "识别在出结果"
        if classes is not None:
            missing = [c for c in self.expect_classes if c not in classes]
            if missing:
                return _check("detector", "yellow", title,
                              detail + "；不认识：" + "、".join(CLASS_ZH.get(c, c) for c in missing),
                              "演示别用这些物品，或换回含这些类别的模型")
        if fps is not None and fps < 1.0:
            return _check("detector", "yellow", title, detail + "（偏慢）",
                          "看功率档是不是 MAXN_SUPER；关掉 Nano 上其他程序")
        return _check("detector", "green", title, detail)

    def check_kv(self, t, inp, st):
        title = "KV260 链路"
        kv = st.get("kv") if isinstance(st.get("kv"), dict) else None
        if kv is None:
            return _check("kv", "pending", title, "等待 KV 遥测")
        if not kv.get("link_ok"):
            if self.telem_lost_since is None:
                self.telem_lost_since = t
        else:
            self.telem_lost_since = None
        if self.telem_lost_since is not None and t - self.telem_lost_since >= STALE["telem_s"]:
            return _check("kv", "red", title, "KV260 遥测断了 %s" % _fmt_dur(t - self.telem_lost_since),
                          "检查 Nano–KV 网线和 KV 电源；恢复后自动变绿，不行就重新起栈")
        permit = kv.get("permit") if isinstance(kv.get("permit"), dict) else {}
        estop = bool((st.get("estop") or {}).get("latched")) or bool(permit.get("estop"))
        if permit.get("avail") and permit.get("permit") is False and not estop:
            if self.permit_off_since is None:
                self.permit_off_since = t
        else:
            self.permit_off_since = None
        if estop:
            return _check("kv", "yellow", title, "急停锁存中，车不会动", "确认安全后点「解除急停」")
        if self.permit_off_since is not None and t - self.permit_off_since >= STALE["permit_s"]:
            return _check("kv", "red", title, "KV 没有放行（行驶许可关了 %s），车不会动" % _fmt_dur(t - self.permit_off_since),
                          "多半是深度帧没送到 KV：先看相机项；仍不行重新起栈")
        fps = _num((st.get("perf") or {}).get("frames_per_s")) if isinstance(st.get("perf"), dict) else None
        parts = ["遥测正常"]
        if permit.get("avail"):
            parts.append("许可开 · 心跳在发" if permit.get("permit") else "许可关")
        if fps is not None:
            parts.append("几何 %.1f 帧/秒" % fps)
        return _check("kv", "green", title, " · ".join(parts))

    def check_ar(self, st):
        title = "车屏"
        facts = self._fact("host_facts.json", _read_json) or {}
        kr = facts.get("kv_ready") if isinstance(facts, dict) and isinstance(facts.get("kv_ready"), dict) else None
        ar_tel = ((st.get("kv") or {}).get("ar") or {}) if isinstance(st.get("kv"), dict) else {}
        if kr is not None:
            scr = str(kr.get("ar_screen") or "")
            if kr.get("ar") != "drm":
                return _check("ar", "red", title, "车屏没开（%s）" % str(kr.get("ar_reason") or "原因不明")[:60],
                              "给屏幕上电、在 KV 端重插 HDMI，然后重新起栈")
            if "sink_count=0" in scr or "edid_bytes=0" in scr:
                return _check("ar", "red", title, "KV 看不到屏幕（%s），车屏多半是黑的" % scr,
                              "给屏幕上电、在 KV 端重插 HDMI，然后重新起栈")
            return _check("ar", "green", title, "车屏已开%s" % ("（%s）" % scr if scr else ""))
        if ar_tel.get("avail") and ar_tel.get("mode") is not None:
            return _check("ar", "green", title, "KV 车屏模式 %s" % ar_tel.get("mode"))
        return None

    def check_clock(self, st):
        title = "KV 时钟"
        facts = self._fact("host_facts.json", _read_json) or {}
        skew = _num(facts.get("kv_wall_minus_nano_s")) if isinstance(facts, dict) else None
        clk = st.get("clock") if isinstance(st.get("clock"), dict) else {}
        if skew is not None and abs(skew) > CLOCK_YELLOW_S:
            return _check("clock", "yellow", title,
                          "KV 的时间比 Nano %s %s（KV 没对上网络时间）" % ("快" if skew > 0 else "慢", _fmt_dur(skew)),
                          "在 Nano 上执行：ssh -F ~/.ssh/geomem-kv.conf geomem-kv260 sudo date -s @$(date +%s)")
        if clk.get("error"):
            return _check("clock", "yellow", title, "Nano–KV 对时出错（%s）" % str(clk.get("error"))[:60],
                          "检查 Nano–KV 网线；重新起栈")
        if skew is None:
            return None if not clk else _check("clock", "green", title, "对时正常")
        return _check("clock", "green", title, "相差 %.1f 秒" % abs(skew))

    def check_gyro(self, inp, ages):
        title = "陀螺航向"
        wo = inp.get("wheel_odom")
        if not isinstance(wo, dict):
            return None
        if wo.get("heading") != "gyro":       # wheel mode (or a v0.1 status without the heading keys)
            return _check("gyro", "green", title, "按配置用轮子算航向")
        if wo.get("heading_source") == "gyro":
            g = wo.get("gyro") if isinstance(wo.get("gyro"), dict) else {}
            if g.get("ready") is False:
                return _check("gyro", "yellow", title, "陀螺在用，零偏还没估好",
                              "起步前让车静止几秒")
            return _check("gyro", "green", title, "航向用陀螺")
        return _check("gyro", "yellow", title,
                      "配置用陀螺，实际退回轮子算航向%s" % ("（单片机发旧帧）" if wo.get("mcu_frame") == "0x81" else ""),
                      "转弯后位置会偏；检查单片机固件/陀螺，重新起栈")

    def check_odom(self, ages, car):
        title = "里程与 TF"
        age = ages.get("odom")
        if age is None:
            return _check("odom", "red", title, "收不到 /odom（里程没在发）", "看底盘项；然后重新起栈")
        if age > STALE["odom_s"]:
            return _check("odom", "red", title, "/odom 停了 %s" % _fmt_dur(age), "看底盘项；然后重新起栈")
        fps = self.rates["cb:/odom"].value()
        return _check("odom", "green", title, "里程在发" + ("（%.0f 次/秒）" % fps if fps else ""))

    def check_power_mode(self, st):
        title = "功率档"
        facts = self._fact("host_facts.json", _read_json) or {}
        mode = facts.get("power_mode") if isinstance(facts, dict) else None
        if not mode:
            mode = ((st.get("nano") or {}).get("power_mode")) if isinstance(st.get("nano"), dict) else None
        if not mode:
            return None
        if mode == POWER_MODE_EXPECTED:
            return _check("power_mode", "green", title, mode)
        return _check("power_mode", "yellow", title, "现在是 %s，应为 %s（识别会变慢）" % (mode, POWER_MODE_EXPECTED),
                      "在 Nano 上执行 sudo nvpmodel -m 2，然后重新起栈")

    def check_disk(self):
        title = "磁盘余量"
        gb = self.disk
        if gb is None:
            return None
        if gb < DISK_RED_GB:
            return _check("disk", "red", title, "只剩 %.1f GB，建图存盘会失败" % gb, "删掉 /data/fpga/car/runs 里的旧记录再起栈")
        if gb < DISK_YELLOW_GB:
            return _check("disk", "yellow", title, "只剩 %.1f GB" % gb, "有空删掉 /data/fpga/car/runs 里的旧记录")
        return _check("disk", "green", title, "剩 %.0f GB" % gb)

    def check_power(self, inp):
        title = "供电"
        ch = inp.get("chassis") if isinstance(inp.get("chassis"), dict) else {}
        mcu = ch.get("mcu") if isinstance(ch.get("mcu"), dict) else {}
        bat = _num(mcu.get("battery_mv"))
        bat_v = bat / 1000.0 if bat else None
        vin = _num(self.power.get("vin_mv"))
        oc = _num(self.power.get("oc_events"))
        oc_new = None if oc is None or self.oc_first is None else oc - self.oc_first
        if bat_v is None and vin is None and oc is None:
            return None
        parts = []
        if bat_v is not None:
            parts.append("电池 %.1f V" % bat_v)
        if vin is not None:
            parts.append("Nano 输入 %.2f V" % (vin / 1000.0))
        detail = " · ".join(parts) or "Nano 供电"
        if bat_v is not None and bat_v < BATTERY_RED_V:
            return _check("power", "red", title, detail + "（电池快没电了）", "换上充满的电池")
        if oc_new:
            return _check("power", "yellow", title, detail + "；Nano 记到 %d 次欠压/过流" % oc_new,
                          "检查降压模块和电源线接头；电池电量低就换电池")
        if vin is not None and vin < VIN_YELLOW_MV:
            return _check("power", "yellow", title, detail + "（偏低）", "检查降压模块输出和电源线接头")
        if bat_v is not None and bat_v < BATTERY_YELLOW_V:
            return _check("power", "yellow", title, detail + "（电池偏低）", "准备换电池")
        return _check("power", "green", title, detail)

    # ---------------------------------------------------------------- log files
    def _log(self, t):
        r = self.report
        lines = []
        stamp = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(r["t"]))
        if not self.started_logged:
            self.started_logged = True
            lines.append({"t": stamp, "event": "first_report", "report": r})
            self._say("体检：%s" % r["summary"])
            for c in r["checks"]:
                if c["level"] in ("red", "yellow"):
                    self._say("  [%s] %s：%s；%s" % (c["level"], c["title"], c["detail"], c.get("hint") or ""))
        else:
            for c in r["checks"]:
                prev = self.levels.get(c["code"])
                if prev is not None and prev != c["level"]:
                    lines.append({"t": stamp, "event": "change", "code": c["code"], "title": c["title"],
                                  "from": prev, "to": c["level"], "detail": c["detail"], "hint": c.get("hint")})
                    self._say("体检变化：%s %s -> %s：%s" % (c["title"], prev, c["level"], c["detail"]))
        self.levels = dict((c["code"], c["level"]) for c in r["checks"])
        if self.last_report_t is None or t - self.last_report_t >= REPORT_EVERY_S:
            self.last_report_t = t
            if lines and lines[0]["event"] == "first_report":
                pass
            else:
                lines.append({"t": stamp, "event": "report", "overall": r["overall"], "summary": r["summary"],
                              "levels": self.levels})
        if lines:
            try:
                with open(self._path("health.jsonl"), "a", encoding="utf-8") as f:
                    for row in lines:
                        f.write(json.dumps(row, ensure_ascii=False) + "\n")
            except OSError:
                pass
        if lines or self.last_snapshot_t is None or t - self.last_snapshot_t >= SNAPSHOT_EVERY_S:
            self.last_snapshot_t = t
            tmp = self._path("health.json.tmp")
            try:
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(r, f, ensure_ascii=False, indent=1)
                os.replace(tmp, self._path("health.json"))
            except OSError:
                pass

    def _say(self, text):
        if self.out is not None:
            try:
                self.out.write(text + "\n")
                self.out.flush()
            except (OSError, ValueError):
                pass


def start_health_thread(server, monitor, period_s=1.0):
    """Run ``monitor.step`` every ``period_s`` on a daemon thread; the report goes to ``server.core.health`` (STATE
    "health"). Inputs are taken under the hub lock, evaluated and written outside it."""
    import threading

    def loop():
        while True:
            try:
                with server.lock:
                    inp = server.core.health_inputs(server.clock())
                report = monitor.step(inp)
                with server.lock:
                    server.core.health = report
            except Exception as exc:                  # visible, never silent; the hub keeps running
                server.core.counts["health_error:" + type(exc).__name__] += 1
            time.sleep(period_s)

    th = threading.Thread(target=loop, name="health", daemon=True)
    th.start()
    return th


def summary_line(report):
    """One plain line of a report (board_preflight.sh / logs)."""
    if not isinstance(report, dict):
        return "体检：没有结果"
    bad = ["%s（%s）" % (c.get("title"), c.get("detail")) for c in report.get("checks") or []
           if c.get("level") in ("red", "yellow")]
    return "体检 %s：%s%s" % (report.get("overall"), report.get("summary"), ("；" + "；".join(bad)) if bad else "")


if __name__ == "__main__":     # python3 -m host.hub.health <run_dir>/health.json  -> one line
    print(summary_line(_read_json(sys.argv[1]) if len(sys.argv) > 1 else None))
