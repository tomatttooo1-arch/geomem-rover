// kv_cam OrbbecSDK source (NN-CAM1, 2026-11-04). Built with -DKVCAM_ORBBEC against OrbbecSDK v2.9.3 (the SDK
// CAMKV0 measured on the KV260); without it this file only provides stubs, so the host build runs replay/synth.
// Stream setup follows CAMKV0 camkv_probe.cpp (SDK examples hw_d2c_align / stream.callback); differences: colour
// YUYV by default (no host MJPG decode), the frame data goes into the ring, the Context is re-created after
// repeated failures (USB re-enumeration).
#include "kv_cam_common.h"

#ifndef KVCAM_ORBBEC

namespace kvcam {
std::unique_ptr<Source> make_orbbec_source(const SourceOptions&) { return nullptr; }
bool orbbec_available() { return false; }
int orbbec_list(const SourceOptions&) {
  std::fprintf(stderr, "kv_cam: built without OrbbecSDK (host build); --list needs the board build\n");
  return 2;
}
}  // namespace kvcam

#else

#include <libobsensor/ObSensor.hpp>

#include <mutex>

namespace kvcam {
namespace {

const char* fmtname(OBFormat f) {
  switch (f) {
    case OB_FORMAT_MJPG: return "MJPG";
    case OB_FORMAT_RGB: return "RGB";
    case OB_FORMAT_Y16: return "Y16";
    case OB_FORMAT_YUYV: return "YUYV";
    default: return "other";
  }
}

std::once_flag g_sdk_once;
void sdk_setup(const SourceOptions& o) {
  std::call_once(g_sdk_once, [&] {
    ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_WARN);
    if (!o.sdk_log_dir.empty()) ob::Context::setLoggerToFile(OB_LOG_SEVERITY_INFO, o.sdk_log_dir.c_str());
    // deploy layout <root>/bin/kv_cam, <root>/lib/extensions (same as camkv0)
    char exe[4096] = {0};
    ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n > 0) {
      std::string d(exe, size_t(n));
      d = d.substr(0, d.rfind('/'));
      d = d.substr(0, d.rfind('/')) + "/lib/extensions";
      ob::Context::setExtensionsDirectory(d.c_str());
    }
  });
}

OBFormat colour_format(const SourceOptions& o) { return o.colour_format == "rgb" ? OB_FORMAT_RGB : OB_FORMAT_YUYV; }

// depth-processing properties that are candidate causes of the depth-path CPU (NONANO0 DESIGN section 7)
void dump_props(const std::shared_ptr<ob::Device>& dev) {
  const std::pair<int, const char*> ids[] = {
      {OB_PROP_DEPTH_POSTFILTER_BOOL, "DEPTH_POSTFILTER"},
      {OB_PROP_DEPTH_HOLEFILTER_BOOL, "DEPTH_HOLEFILTER"},
      {OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, "DEPTH_NOISE_REMOVAL_FILTER (soft filter)"},
      {OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_MAX_DIFF_INT, "NOISE_REMOVAL_MAX_DIFF"},
      {OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_MAX_SPECKLE_SIZE_INT, "NOISE_REMOVAL_MAX_SPECKLE_SIZE"},
      {OB_PROP_DISPARITY_TO_DEPTH_BOOL, "DISPARITY_TO_DEPTH (hardware)"},
      {OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, "SDK_DISPARITY_TO_DEPTH (host software)"},
      {OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL, "HW_NOISE_REMOVE_FILTER"},
  };
  for (auto& p : ids) {
    auto id = OBPropertyID(p.first);
    try {
      if (!dev->isPropertySupported(id, OB_PERMISSION_READ)) {
        std::printf("kv_cam: prop %d %s: not supported\n", p.first, p.second);
        continue;
      }
      int v;
      try { v = dev->getBoolProperty(id) ? 1 : 0; } catch (ob::Error&) { v = dev->getIntProperty(id); }
      std::printf("kv_cam: prop %d %s = %d\n", p.first, p.second, v);
    } catch (ob::Error& e) {
      std::printf("kv_cam: prop %d %s: error %s\n", p.first, p.second, e.what());
    }
  }
  std::fflush(stdout);
}

class OrbbecSource : public Source {
 public:
  explicit OrbbecSource(const SourceOptions& o) : o_(o) {}
  ~OrbbecSource() override { stop(); }

  bool start(const FrameSink& sink, CamInfo* info, std::string* err) override {
    stop();
    try {
      sdk_setup(o_);
      // every third failed start: drop the Context too, so a re-enumerated camera is found again
      if (!ctx_ || fails_ % 3 == 2) {
        pipe_.reset();
        ctx_ = std::make_shared<ob::Context>();
      }
      auto devs = ctx_->queryDeviceList();
      if (devs->getCount() == 0) {
        ++fails_;
        if (err) *err = "no Orbbec device found";
        return false;
      }
      auto dev = devs->getDevice(0);
      pipe_ = std::make_shared<ob::Pipeline>(dev);
      auto di = dev->getDeviceInfo();
      std::string conn = di->getConnectionType();
      info->text = std::string(di->getName()) + " sn=" + di->getSerialNumber() + " fw=" + di->getFirmwareVersion() +
                   " conn=" + conn;
      info->usb3 = conn.find("USB3") != std::string::npos;
      auto cpl = pipe_->getStreamProfileList(OB_SENSOR_COLOR);
      auto dpl = pipe_->getStreamProfileList(OB_SENSOR_DEPTH);
      auto cp = cpl->getVideoStreamProfile(640, 480, colour_format(o_), o_.fps);
      std::shared_ptr<ob::VideoStreamProfile> dp;
      bool hw = false;
      if (o_.align == "hw") {
        auto d2c = pipe_->getD2CDepthProfileList(cp, ALIGN_D2C_HW_MODE);
        for (uint32_t i = 0; i < d2c->getCount(); i++) {
          auto v = d2c->getProfile(i)->as<ob::VideoStreamProfile>();
          if (v->getWidth() == 640 && v->getHeight() == 400 && v->getFormat() == OB_FORMAT_Y16 &&
              int(v->getFps()) == o_.fps) {
            dp = v;
            hw = true;
            break;
          }
        }
      }
      if (!dp) dp = dpl->getVideoStreamProfile(640, 400, OB_FORMAT_Y16, o_.fps);  // unaligned fallback (alarmed)
      info->colour_fmt = fmtname(cp->getFormat());
      info->depth_fmt = std::string(fmtname(dp->getFormat())) + (hw ? " HW-D2C" : " unaligned");
      auto in = cp->getIntrinsic();
      auto ds = cp->getDistortion();
      info->intr_valid = in.fx > 0;
      info->fx = in.fx;
      info->fy = in.fy;
      info->cx = in.cx;
      info->cy = in.cy;
      info->k[0] = ds.k1; info->k[1] = ds.k2; info->k[2] = ds.k3; info->k[3] = ds.k4; info->k[4] = ds.k5; info->k[5] = ds.k6;
      info->p1 = ds.p1;
      info->p2 = ds.p2;
      info->dist_model = uint32_t(ds.model);

      // diagnosis knobs (defaults: nothing changed): device / SDK depth-processing properties
      for (auto& kv : o_.set_props) {
        auto id = OBPropertyID(kv.first);
        if (!dev->isPropertySupported(id, OB_PERMISSION_WRITE)) {
          std::fprintf(stderr, "kv_cam: property %d not writable on this device\n", kv.first);
          continue;
        }
        try { dev->setBoolProperty(id, kv.second != 0); } catch (ob::Error&) { dev->setIntProperty(id, kv.second); }
        std::printf("kv_cam: set property %d = %d\n", kv.first, kv.second);
      }
      if (o_.dump_props) dump_props(dev);
      bool want_d = o_.streams != "colour", want_c = o_.streams != "depth";
      if (!want_c) hw = false;  // HW D2C needs the colour stream
      info->hw_align = hw || !want_d;
      auto cfg = std::make_shared<ob::Config>();
      if (want_d) cfg->enableStream(dp);
      if (want_c) cfg->enableStream(cp);
      if (hw) cfg->setAlignMode(ALIGN_D2C_HW_MODE);
      cfg->setFrameAggregateOutputMode(o_.aggregate == "full" ? OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE
                                                              : OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
      if (o_.sync && want_c && want_d) pipe_->enableFrameSync();
      bool unaligned = !hw;
      pipe_->start(cfg, [sink, unaligned](std::shared_ptr<ob::FrameSet> fs) {
        FrameView f;
        f.arrive_ns = mono_ns();
        std::shared_ptr<ob::Frame> c, d;
        try { c = fs->getFrame(OB_FRAME_COLOR); } catch (...) {}
        try { d = fs->getFrame(OB_FRAME_DEPTH); } catch (...) {}
        if (c) {
          f.colour = static_cast<const uint8_t*>(c->getData());
          f.colour_bytes = c->getDataSize();
          f.colour_index = c->getIndex();
          f.colour_dev_us = c->getTimeStampUs();
          f.colour_sys_us = c->getSystemTimeStampUs();
        }
        if (d) {
          auto vd = d->as<ob::VideoFrame>();
          f.depth = static_cast<const uint8_t*>(d->getData());
          f.depth_bytes = d->getDataSize();
          f.depth_w = vd->getWidth();
          f.depth_h = vd->getHeight();
          f.depth_index = d->getIndex();
          f.depth_dev_us = d->getTimeStampUs();
          f.depth_sys_us = d->getSystemTimeStampUs();
          try { f.depth_scale_mm = d->as<ob::DepthFrame>()->getValueScale(); } catch (...) {}
          f.depth_unaligned = unaligned;
        }
        if (c || d) sink(f);
      });
      started_ = true;
      fails_ = 0;
      return true;
    } catch (ob::Error& e) {
      ++fails_;
      if (err) *err = std::string("OB ") + e.getFunction() + ": " + e.what();
      pipe_.reset();
      return false;
    } catch (std::exception& e) {
      ++fails_;
      if (err) *err = e.what();
      pipe_.reset();
      return false;
    }
  }

  void stop() override {
    if (pipe_ && started_) {
      try { pipe_->stop(); } catch (...) {}
    }
    started_ = false;
    pipe_.reset();
  }

 private:
  SourceOptions o_;
  std::shared_ptr<ob::Context> ctx_;
  std::shared_ptr<ob::Pipeline> pipe_;
  bool started_ = false;
  int fails_ = 0;
};

}  // namespace

std::unique_ptr<Source> make_orbbec_source(const SourceOptions& o) { return std::unique_ptr<Source>(new OrbbecSource(o)); }
bool orbbec_available() { return true; }

int orbbec_list(const SourceOptions& o) try {
  sdk_setup(o);
  ob::Context ctx;
  auto devs = ctx.queryDeviceList();
  if (devs->getCount() == 0) {
    std::fprintf(stderr, "no Orbbec device found\n");
    return 3;
  }
  auto dev = devs->getDevice(0);
  auto pipe = std::make_shared<ob::Pipeline>(dev);
  auto di = dev->getDeviceInfo();
  std::printf("device: %s sn=%s fw=%s conn=%s\n", di->getName(), di->getSerialNumber(), di->getFirmwareVersion(),
              di->getConnectionType());
  auto cpl = pipe->getStreamProfileList(OB_SENSOR_COLOR);
  for (uint32_t i = 0; i < cpl->getCount(); i++) {
    auto v = cpl->getProfile(i)->as<ob::VideoStreamProfile>();
    std::printf("colour %ux%u %s %u fps\n", v->getWidth(), v->getHeight(), fmtname(v->getFormat()), v->getFps());
  }
  // the question the board run must answer first: HW D2C depth profiles for the YUYV 640x480 colour profile
  for (OBFormat cf : {OB_FORMAT_YUYV, OB_FORMAT_RGB}) {
    for (int fps : {30, 15}) {
      try {
        auto cp = cpl->getVideoStreamProfile(640, 480, cf, fps);
        auto d2c = pipe->getD2CDepthProfileList(cp, ALIGN_D2C_HW_MODE);
        for (uint32_t i = 0; i < d2c->getCount(); i++) {
          auto v = d2c->getProfile(i)->as<ob::VideoStreamProfile>();
          if (v->getWidth() == 640)
            std::printf("hwd2c for colour 640x480 %s@%d: depth %ux%u %s %u fps\n", fmtname(cf), fps, v->getWidth(),
                        v->getHeight(), fmtname(v->getFormat()), v->getFps());
        }
      } catch (ob::Error& e) {
        std::printf("hwd2c for colour 640x480 %s@%d: none (%s)\n", fmtname(cf), fps, e.what());
      }
    }
  }
  return 0;
} catch (ob::Error& e) {
  std::fprintf(stderr, "OB ERROR %s: %s\n", e.getFunction(), e.what());
  return 5;
}

}  // namespace kvcam

#endif
