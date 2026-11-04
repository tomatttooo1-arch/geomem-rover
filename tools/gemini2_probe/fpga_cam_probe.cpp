// fpga_cam_probe: list Gemini 2 stream profiles, or stream depth+color for N seconds
// and report received frames, device-index gaps (drops), rates from device and
// host timestamps, frame-interval distribution, SDK drop/oversize warnings and
// CPU cost (this process and whole system). No decoding, no processing: the CPU
// figure is SDK transport plus (if --record) raw file writes.
//
//   fpga_cam_probe --list
//   fpga_cam_probe --depth 640x400@30 --color 1280x720@30:MJPG --seconds 20
//   options: --noise-removal 0|1   set the SDK depth noise-removal (soft) filter
//            --sdk-d2d 0|1         set host-side disparity-to-depth conversion
//            --laser 0|1           set the IR laser projector
//            --dump <csv>          per-frame stream,index,device_us,host_us,window
//            --settle <s>          seconds counted separately before the window (default 2)
//            --imu                 also run a separate IMU pipeline (accel+gyro)
//            --color-first         enable colour before depth in the config (the ROS driver's order; default depth first)
//            --record <dir>        write raw depth/color/IMU + camera.json into <dir>
// The device JSON always reports these switches as read back after any set.
// v3 (2026-09-26): settle-window stats, interval percentiles, SDK warning
// counters via the logger callback, IMU pipeline and raw recording.
#include <libobsensor/ObSensor.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <tuple>
#include <unistd.h>
#include <vector>

struct Spec { bool on = false; uint32_t w = 0, h = 0, fps = 0; OBFormat fmt = OB_FORMAT_ANY; std::string text; };

static OBFormat parse_format(const std::string &s) {
    static const std::map<std::string, OBFormat> m = {{"Y16", OB_FORMAT_Y16}, {"MJPG", OB_FORMAT_MJPG}, {"RGB", OB_FORMAT_RGB},
                                                       {"YUYV", OB_FORMAT_YUYV}, {"Y8", OB_FORMAT_Y8}, {"Y14", OB_FORMAT_Y14},
                                                       {"RLE", OB_FORMAT_RLE}, {"ANY", OB_FORMAT_ANY}};
    auto it = m.find(s);
    if(it == m.end()) { fprintf(stderr, "unknown format %s\n", s.c_str()); exit(2); }
    return it->second;
}

static Spec parse_spec(const char *arg) {  // WxH@F[:FMT]
    Spec s; s.on = true; s.text = arg;
    char fmt[16] = "ANY";
    if(sscanf(arg, "%ux%u@%u:%15s", &s.w, &s.h, &s.fps, fmt) < 3) { fprintf(stderr, "bad spec %s\n", arg); exit(2); }
    s.fmt = parse_format(fmt);
    return s;
}

static double proc_cpu_seconds() {  // utime+stime of this process
    std::ifstream f("/proc/self/stat"); std::string all; std::getline(f, all);
    std::istringstream is(all.substr(all.rfind(')') + 2));
    std::string tok; unsigned long ut = 0, st = 0;
    for(int i = 3; i <= 15 && is >> tok; ++i) { if(i == 14) ut = std::stoul(tok); if(i == 15) st = std::stoul(tok); }
    return double(ut + st) / sysconf(_SC_CLK_TCK);
}

static void sys_cpu(unsigned long long &busy, unsigned long long &total) {
    std::ifstream f("/proc/stat"); std::string cpu; unsigned long long v[10] = {0};
    f >> cpu; for(auto &x : v) f >> x;
    total = 0; for(auto x : v) total += x;
    busy = total - v[3] - v[4];  // minus idle, iowait
}

// ---- SDK warning counters (logger callback). The SDK rate-limits repeats as
// "[**N logs in Tms**]"; that suffix is added to the aggregated count.
static std::mutex g_log_mu;
static std::map<std::string, unsigned long> g_log_lines, g_log_aggregated;
static std::atomic<int> g_window{0};  // 0 settle, 1 measurement, 2 after
static void classify_log(const char *msg) {
    std::string m(msg), key = "other_warning";
    if(m.find("does not match the expectation") != std::string::npos) key = "truncated_frame_dropped";
    else if(m.find("> data buffer size") != std::string::npos)
        key = m.find("Execute failure") != std::string::npos ? "oversize_frame_exception_echo" : "oversize_frame_exception";
    else if(m.find("Wait for frame timeout") != std::string::npos) key = "wait_timeout";
    else if(m.find("not found in OBPropertyBaseInfoMap") != std::string::npos) key = "property_map_notice";
    else if(m.find("no matching stream profile") != std::string::npos) key = "default_profile_notice";
    key += g_window.load() == 0 ? "@settle" : (g_window.load() == 1 ? "@window" : "@after");
    unsigned long agg = 0;
    auto p = m.find("[**");
    if(p != std::string::npos) agg = strtoul(m.c_str() + p + 3, nullptr, 10);
    std::lock_guard<std::mutex> lk(g_log_mu);
    g_log_lines[key]++;
    g_log_aggregated[key] += agg;
    fputs(msg, stderr);
}

struct Row { char stream; uint64_t idx, dev, sys; int window; };
static std::vector<Row> g_dump;
static std::shared_ptr<ob::Frame> g_last_depth;

static double pct(std::vector<double> v, double q) {
    if(v.empty()) return 0; std::sort(v.begin(), v.end());
    size_t i = std::min(v.size() - 1, (size_t)(q * (v.size() - 1) + 0.5)); return v[i];
}

struct Stat {
    uint64_t n = 0, gaps = 0, missing = 0, last_idx = 0, first_dev = 0, last_dev = 0, first_sys = 0, last_sys = 0, bytes = 0,
             first_idx = 0, bad_size = 0, nonmono_dev = 0;
    uint32_t w = 0, h = 0; std::string fmt; uint64_t expect = 0;
    std::vector<double> ddev, dsys;  // intervals, ms
    void add(const std::shared_ptr<ob::Frame> &f, int window) {
        uint64_t idx = f->getIndex(), dev = f->getTimeStampUs(), sys = f->getSystemTimeStampUs();
        g_dump.push_back({f->getType() == OB_FRAME_DEPTH ? 'D' : 'C', idx, dev, sys, window});
        if(n && idx > last_idx + 1) { gaps++; missing += idx - last_idx - 1; }
        if(n) { if(dev <= last_dev) nonmono_dev++; ddev.push_back((double(dev) - double(last_dev)) / 1e3); dsys.push_back((double(sys) - double(last_sys)) / 1e3); }
        if(!n) { first_dev = dev; first_sys = sys; first_idx = idx; }
        last_idx = idx; last_dev = dev; last_sys = sys; bytes += f->getDataSize(); n++;
        if(!w) { auto v = f->as<ob::VideoFrame>(); w = v->getWidth(); h = v->getHeight(); fmt = ob::TypeHelper::convertOBFormatTypeToString(f->getFormat());
                 expect = (f->getFormat() == OB_FORMAT_Y16 || f->getFormat() == OB_FORMAT_Y14) ? uint64_t(w) * h * 2 : 0; }
        if(expect && f->getDataSize() != expect) bad_size++;
    }
    void print(const char *name, double nominal_ms) const {
        double dspan = (last_dev - first_dev) / 1e6, sspan = (last_sys - first_sys) / 1e6;
        auto over = [&](const std::vector<double> &v) { size_t c = 0; for(double x : v) if(x > 1.5 * nominal_ms) c++; return c; };
        printf("  \"%s\": {\"frames\": %llu, \"size\": \"%ux%u\", \"format\": \"%s\", \"first_index\": %llu, \"index_gaps\": %llu, \"missing_by_index\": %llu, "
               "\"delivered_wrong_size\": %llu, \"nonmonotonic_device_ts\": %llu, "
               "\"fps_device_ts\": %.3f, \"fps_host_ts\": %.3f, \"mbytes\": %.1f, "
               "\"interval_device_ms\": {\"min\": %.3f, \"p50\": %.3f, \"p95\": %.3f, \"p99\": %.3f, \"max\": %.3f, \"over_1p5x_nominal\": %zu}, "
               "\"interval_host_ms\": {\"min\": %.3f, \"p50\": %.3f, \"p95\": %.3f, \"p99\": %.3f, \"max\": %.3f, \"over_1p5x_nominal\": %zu}}",
               name, (unsigned long long)n, w, h, fmt.c_str(), (unsigned long long)first_idx, (unsigned long long)gaps, (unsigned long long)missing,
               (unsigned long long)bad_size, (unsigned long long)nonmono_dev,
               n > 1 && dspan > 0 ? (n - 1) / dspan : 0.0, n > 1 && sspan > 0 ? (n - 1) / sspan : 0.0, bytes / 1e6,
               pct(ddev, 0), pct(ddev, .5), pct(ddev, .95), pct(ddev, .99), pct(ddev, 1), over(ddev),
               pct(dsys, 0), pct(dsys, .5), pct(dsys, .95), pct(dsys, .99), pct(dsys, 1), over(dsys));
    }
};

// ---- raw recorder: one concatenated binary per stream + index CSV.
struct Recorder {
    FILE *bin = nullptr, *idx = nullptr; uint64_t off = 0;
    void open(const std::string &dir, const char *name) {
        bin = fopen((dir + "/" + name + ".bin").c_str(), "wb");
        idx = fopen((dir + "/" + name + "_index.csv").c_str(), "w");
        if(!bin || !idx) { fprintf(stderr, "RECORD_OPEN_FAILED %s\n", name); exit(6); }
        setvbuf(bin, nullptr, _IOFBF, 8 << 20);
        fprintf(idx, "seq,index,device_us,host_us,global_us,offset,bytes,width,height,format\n");
    }
    void put(const std::shared_ptr<ob::Frame> &f, uint64_t seq) {
        if(!bin) return;
        auto v = f->as<ob::VideoFrame>();
        size_t nb = f->getDataSize();
        if(fwrite(f->getData(), 1, nb, bin) != nb) { fprintf(stderr, "RECORD_WRITE_FAILED\n"); exit(6); }
        fprintf(idx, "%llu,%llu,%llu,%llu,%llu,%llu,%zu,%u,%u,%s\n", (unsigned long long)seq, (unsigned long long)f->getIndex(),
                (unsigned long long)f->getTimeStampUs(), (unsigned long long)f->getSystemTimeStampUs(),
                (unsigned long long)f->getGlobalTimeStampUs(), (unsigned long long)off, nb, v->getWidth(), v->getHeight(),
                ob::TypeHelper::convertOBFormatTypeToString(f->getFormat()).c_str());
        off += nb;
    }
    void close() { if(bin) fclose(bin); if(idx) fclose(idx); bin = idx = nullptr; }
};

static void write_camera_json(const std::string &dir, std::shared_ptr<ob::Frame> depth, std::shared_ptr<ob::Frame> color) {
    FILE *fp = fopen((dir + "/camera.json").c_str(), "w");
    auto intr = [&](const char *name, std::shared_ptr<ob::Frame> f, bool last) {
        auto p = f->getStreamProfile()->as<ob::VideoStreamProfile>();
        auto k = p->getIntrinsic(); auto d = p->getDistortion();
        fprintf(fp, "  \"%s\": {\"width\": %d, \"height\": %d, \"fx\": %.9g, \"fy\": %.9g, \"cx\": %.9g, \"cy\": %.9g, "
                    "\"distortion\": {\"model\": %d, \"k1\": %.9g, \"k2\": %.9g, \"k3\": %.9g, \"k4\": %.9g, \"k5\": %.9g, \"k6\": %.9g, \"p1\": %.9g, \"p2\": %.9g}, "
                    "\"format\": \"%s\", \"fps\": %u}%s\n",
                name, k.width, k.height, k.fx, k.fy, k.cx, k.cy, (int)d.model, d.k1, d.k2, d.k3, d.k4, d.k5, d.k6, d.p1, d.p2,
                ob::TypeHelper::convertOBFormatTypeToString(f->getFormat()).c_str(), p->getFps(), last ? "" : ",");
    };
    fprintf(fp, "{\n");
    if(depth) {
        fprintf(fp, "  \"depth_value_scale_mm_per_unit\": %.9g,\n", depth->as<ob::DepthFrame>()->getValueScale());
        intr("depth", depth, !color);
    }
    if(color) intr("color", color, true);
    if(depth && color) {
        auto e = depth->getStreamProfile()->getExtrinsicTo(color->getStreamProfile());
        fprintf(fp, "  ,\"extrinsic_depth_to_color\": {\"rot_row_major\": [");
        for(int i = 0; i < 9; ++i) fprintf(fp, "%s%.9g", i ? ", " : "", e.rot[i]);
        fprintf(fp, "], \"trans_mm\": [%.9g, %.9g, %.9g]}\n", e.trans[0], e.trans[1], e.trans[2]);
    }
    fprintf(fp, "}\n");
    fclose(fp);
}

int main(int argc, char **argv) try {
    Spec depth, color; double seconds = 20, settle = 2; bool list = false, imu = false, color_first = false; int noise = -1, d2d = -1, laser = -1;
    std::string dump, record;
    for(int i = 1; i < argc; ++i) {
        if(!strcmp(argv[i], "--list")) list = true;
        else if(!strcmp(argv[i], "--depth") && i + 1 < argc) depth = parse_spec(argv[++i]);
        else if(!strcmp(argv[i], "--color") && i + 1 < argc) color = parse_spec(argv[++i]);
        else if(!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atof(argv[++i]);
        else if(!strcmp(argv[i], "--settle") && i + 1 < argc) settle = atof(argv[++i]);
        else if(!strcmp(argv[i], "--noise-removal") && i + 1 < argc) noise = atoi(argv[++i]);
        else if(!strcmp(argv[i], "--sdk-d2d") && i + 1 < argc) d2d = atoi(argv[++i]);
        else if(!strcmp(argv[i], "--laser") && i + 1 < argc) laser = atoi(argv[++i]);
        else if(!strcmp(argv[i], "--dump") && i + 1 < argc) dump = argv[++i];
        else if(!strcmp(argv[i], "--record") && i + 1 < argc) record = argv[++i];
        else if(!strcmp(argv[i], "--imu")) imu = true;
        else if(!strcmp(argv[i], "--color-first")) color_first = true;   // 2026-10-15: enable colour before depth (the ROS driver's order)
        else { fprintf(stderr, "usage: see header\n"); return 2; }
    }
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);
    ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_OFF);  // keep stdout pure JSON; callback echoes to stderr
    ob::Context::setLoggerToCallback(OB_LOG_SEVERITY_WARN, [](OBLogSeverity, const char *msg) { classify_log(msg); });
    ob::Context ctx;
    auto devs = ctx.queryDeviceList();
    if(devs->getCount() == 0) { fprintf(stderr, "NO_DEVICE\n"); return 3; }
    auto dev = devs->getDevice(0);
    auto info = dev->getDeviceInfo();
    printf("{\"probe\": \"v3\", \"device\": \"%s\", \"serial\": \"%s\", \"firmware\": \"%s\", \"hardware\": \"%s\", \"connection\": \"%s\"",
           info->getName(), info->getSerialNumber(), info->getFirmwareVersion(), info->getHardwareVersion(), info->getConnectionType());
    auto setp = [&](OBPropertyID id, int v) {
        if(v < 0) return;
        if(!dev->isPropertySupported(id, OB_PERMISSION_WRITE)) { fprintf(stderr, "PROPERTY_NOT_WRITABLE %d\n", (int)id); exit(5); }
        dev->setBoolProperty(id, v != 0);
    };
    setp(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, noise);
    setp(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, d2d);
    setp(OB_PROP_LASER_BOOL, laser);
    auto getp = [&](OBPropertyID id) -> int {
        return dev->isPropertySupported(id, OB_PERMISSION_READ) ? (int)dev->getBoolProperty(id) : -1;
    };
    printf(", \"noise_removal\": %d, \"sdk_d2d\": %d, \"hw_d2d\": %d", getp(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL),
           getp(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL), getp(OB_PROP_DISPARITY_TO_DEPTH_BOOL));
    auto geti = [&](OBPropertyID id) -> int {
        return dev->isPropertySupported(id, OB_PERMISSION_READ) ? dev->getIntProperty(id) : -1;
    };
    printf(", \"laser\": %d, \"laser_control\": %d, \"laser_level\": %d, \"ldp\": %d, \"ldp_triggered\": %d, \"depth_mode\": \"%s\"",
           getp(OB_PROP_LASER_BOOL), geti(OB_PROP_LASER_CONTROL_INT), geti(OB_PROP_LASER_POWER_LEVEL_CONTROL_INT),
           getp(OB_PROP_LDP_BOOL), getp(OB_PROP_LDP_STATUS_BOOL), dev->getCurrentDepthModeName());
    if(list) {
        printf(",\n \"sensors\": [\n");
        auto sensors = dev->getSensorList();
        for(uint32_t i = 0; i < sensors->getCount(); ++i) {
            auto s = sensors->getSensor(i);
            printf("%s  {\"type\": \"%s\"", i ? ",\n" : "", ob::TypeHelper::convertOBSensorTypeToString(s->getType()).c_str());
            auto t = s->getType();
            if(t == OB_SENSOR_DEPTH || t == OB_SENSOR_COLOR || t == OB_SENSOR_IR || t == OB_SENSOR_IR_LEFT || t == OB_SENSOR_IR_RIGHT) {
                auto pl = s->getStreamProfileList(); printf(", \"profiles\": [");
                for(uint32_t j = 0; j < pl->getCount(); ++j) {
                    auto p = pl->getProfile(j)->as<ob::VideoStreamProfile>();
                    printf("%s\"%ux%u@%u:%s\"", j ? ", " : "", p->getWidth(), p->getHeight(), p->getFps(),
                           ob::TypeHelper::convertOBFormatTypeToString(p->getFormat()).c_str());
                }
                printf("]");
            }
            printf("}");
        }
        printf("\n ]}\n");
        return 0;
    }
    Recorder rd, rc; FILE *imu_fp = nullptr;
    if(!record.empty()) {
        if(depth.on) rd.open(record, "depth");
        if(color.on) rc.open(record, "color");
        if(imu) { imu_fp = fopen((record + "/imu.csv").c_str(), "w"); fprintf(imu_fp, "type,index,device_us,host_us,x,y,z,temp_c\n"); }
    }
    auto cfg = std::make_shared<ob::Config>();
    if(color_first && color.on) cfg->enableVideoStream(OB_STREAM_COLOR, color.w, color.h, color.fps, color.fmt);
    if(depth.on) cfg->enableVideoStream(OB_STREAM_DEPTH, depth.w, depth.h, depth.fps, depth.fmt);
    if(!color_first && color.on) cfg->enableVideoStream(OB_STREAM_COLOR, color.w, color.h, color.fps, color.fmt);
    cfg->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
    ob::Pipeline pipe(dev);
    std::shared_ptr<ob::Pipeline> imu_pipe;
    std::mutex imu_mu; uint64_t n_acc = 0, n_gyr = 0;
    if(imu) {
        imu_pipe = std::make_shared<ob::Pipeline>(dev);
        auto icfg = std::make_shared<ob::Config>();
        icfg->enableAccelStream(); icfg->enableGyroStream();
        icfg->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
        imu_pipe->start(icfg, [&](std::shared_ptr<ob::FrameSet> fs) {
            std::lock_guard<std::mutex> lk(imu_mu);
            if(g_window.load() != 1) return;
            if(auto a = fs->getFrame(OB_FRAME_ACCEL)) {
                auto af = a->as<ob::AccelFrame>(); auto v = af->getValue(); n_acc++;
                if(imu_fp) fprintf(imu_fp, "A,%llu,%llu,%llu,%.7g,%.7g,%.7g,%.4g\n", (unsigned long long)a->getIndex(), (unsigned long long)a->getTimeStampUs(),
                                   (unsigned long long)a->getSystemTimeStampUs(), v.x, v.y, v.z, af->getTemperature());
            }
            if(auto g = fs->getFrame(OB_FRAME_GYRO)) {
                auto gf = g->as<ob::GyroFrame>(); auto v = gf->getValue(); n_gyr++;
                if(imu_fp) fprintf(imu_fp, "G,%llu,%llu,%llu,%.7g,%.7g,%.7g,%.4g\n", (unsigned long long)g->getIndex(), (unsigned long long)g->getTimeStampUs(),
                                   (unsigned long long)g->getSystemTimeStampUs(), v.x, v.y, v.z, gf->getTemperature());
            }
        });
    }
    pipe.start(cfg);
    // settle window: counted separately (start-up truncations, auto exposure)
    Stat ssd, ssc, sd, sc; uint64_t empty_waits = 0, settle_empty = 0;
    auto t0 = std::chrono::steady_clock::now();
    while(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < settle) {
        auto fs = pipe.waitForFrameset(100);
        if(!fs) { settle_empty++; continue; }
        if(auto d = fs->getFrame(OB_FRAME_DEPTH)) ssd.add(d, 0);
        if(auto c = fs->getFrame(OB_FRAME_COLOR)) ssc.add(c, 0);
    }
    g_window = 1;
    double c0 = proc_cpu_seconds(); unsigned long long b0, T0, b1, T1; sys_cpu(b0, T0);
    auto start = std::chrono::steady_clock::now();
    std::shared_ptr<ob::Frame> first_depth, first_color; uint64_t seq_d = 0, seq_c = 0;
    while(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds) {
        auto fs = pipe.waitForFrameset(200);
        if(!fs) { empty_waits++; continue; }
        if(auto d = fs->getFrame(OB_FRAME_DEPTH)) { sd.add(d, 1); g_last_depth = d; if(!first_depth) first_depth = d; rd.put(d, seq_d++); }
        if(auto c = fs->getFrame(OB_FRAME_COLOR)) { sc.add(c, 1); if(!first_color) first_color = c; rc.put(c, seq_c++); }
    }
    double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    double c1 = proc_cpu_seconds(); sys_cpu(b1, T1);
    g_window = 2;
    pipe.stop();
    if(imu_pipe) imu_pipe->stop();
    rd.close(); rc.close(); if(imu_fp) fclose(imu_fp);
    if(!record.empty()) write_camera_json(record, first_depth, first_color);
    printf(",\n \"request\": {\"depth\": \"%s\", \"color\": \"%s\", \"seconds\": %.1f, \"settle_s\": %.1f, \"imu\": %d, \"record\": %d},\n",
           depth.on ? depth.text.c_str() : "off", color.on ? color.text.c_str() : "off", seconds, settle, imu ? 1 : 0, record.empty() ? 0 : 1);
    if(depth.on) { ssd.print("settle_depth", 1000.0 / depth.fps); printf(",\n"); sd.print("depth", 1000.0 / depth.fps); printf(",\n"); }
    if(color.on) { ssc.print("settle_color", 1000.0 / color.fps); printf(",\n"); sc.print("color", 1000.0 / color.fps); printf(",\n"); }
    if(imu) printf("  \"imu\": {\"accel_samples\": %llu, \"gyro_samples\": %llu},\n", (unsigned long long)n_acc, (unsigned long long)n_gyr);
    printf("  \"sdk_warnings\": {");
    { std::lock_guard<std::mutex> lk(g_log_mu); bool first = true;
      for(auto &kv : g_log_lines) { printf("%s\"%s\": {\"lines\": %lu, \"aggregated_suffix_sum\": %lu}", first ? "" : ", ", kv.first.c_str(), kv.second, g_log_aggregated[kv.first]); first = false; } }
    printf("},\n");
    printf("  \"empty_waits_200ms\": %llu, \"settle_empty_waits_100ms\": %llu, \"wall_s\": %.2f, \"proc_cpu_pct_of_one_core\": %.1f, "
           "\"proc_cpu_ms_per_depth_frame\": %.3f, \"system_cpu_busy_pct\": %.1f, \"cores\": %ld}\n",
           (unsigned long long)empty_waits, (unsigned long long)settle_empty, wall, 100.0 * (c1 - c0) / wall,
           sd.n ? 1000.0 * (c1 - c0) / sd.n : 0.0, 100.0 * double(b1 - b0) / double(T1 - T0), sysconf(_SC_NPROCESSORS_ONLN));
    if(g_last_depth && g_last_depth->getFormat() == OB_FORMAT_Y16) {  // value semantics of the last depth frame
        auto df = g_last_depth->as<ob::DepthFrame>();
        const uint16_t *px = reinterpret_cast<const uint16_t *>(df->getData());
        uint32_t n = df->getWidth() * df->getHeight(), cw = df->getWidth() / 2, ch = df->getHeight() / 2;
        std::vector<uint16_t> nz; for(uint32_t i = 0; i < n; ++i) if(px[i]) nz.push_back(px[i]);
        std::sort(nz.begin(), nz.end());
        printf("DEPTH_VALUES scale=%g nonzero=%zu/%u p5=%u p50=%u p95=%u center=%u\n", df->getValueScale(), nz.size(), n,
               nz.empty() ? 0 : nz[nz.size() * 5 / 100], nz.empty() ? 0 : nz[nz.size() / 2], nz.empty() ? 0 : nz[nz.size() * 95 / 100],
               px[ch * df->getWidth() + cw]);
    }
    if(!dump.empty()) {
        FILE *fp = fopen(dump.c_str(), "w");
        fprintf(fp, "stream,index,device_us,host_us,window\n");
        for(auto &r : g_dump) fprintf(fp, "%c,%llu,%llu,%llu,%d\n", r.stream, (unsigned long long)r.idx,
                                    (unsigned long long)r.dev, (unsigned long long)r.sys, r.window);
        fclose(fp);
    }
    return 0;
} catch(ob::Error &e) {
    fprintf(stderr, "OB_ERROR %s: %s\n", e.getFunction(), e.what());
    return 4;
}
