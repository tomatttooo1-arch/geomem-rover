// Native rmw_fastrtps 6.2.10 service response discovery-wait control.
// Discovery events are injected; this is not a network loss/replay test.
#include <rcl/client.h>
#include <rcl/publisher.h>
#include <rcl/service.h>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw_fastrtps_shared_cpp/custom_client_info.hpp>
#include <rmw_fastrtps_shared_cpp/custom_publisher_info.hpp>
#include <rmw_fastrtps_shared_cpp/custom_service_info.hpp>
#include <rmw_fastrtps_shared_cpp/guid_utils.hpp>
#include <fastdds/dds/core/status/PublicationMatchedStatus.hpp>
#include <fastdds/rtps/common/Guid.h>
#include <fastdds/rtps/common/InstanceHandle.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using GUID = eprosima::fastrtps::rtps::GUID_t;
using Handle = eprosima::fastrtps::rtps::InstanceHandle_t;
using lifecycle_msgs::srv::GetState;

std::string escape(const std::string &s) {
  std::ostringstream o;
  for (char c : s) { if (c == '\\') o << "\\\\"; else if (c == '"') o << "\\\""; else if (c == '\n') o << "\\n"; else o << c; }
  return o.str();
}
double seconds(const eprosima::fastrtps::Duration_t &d) { return d.seconds + d.nanosec / 1e9; }
GUID guid(uint8_t n, bool reader) {
  GUID g;
  for (size_t i = 0; i < 12; ++i) g.guidPrefix.value[i] = static_cast<uint8_t>(0x40 + i);
  g.entityId.value[0] = 0x21; g.entityId.value[1] = 0x32; g.entityId.value[2] = n;
  g.entityId.value[3] = reader ? 0x04 : 0x03;  // reader bit tested by rmw_response.cpp
  return g;
}
Handle handle(const GUID &g) {
  Handle h;
  for (size_t i = 0; i < 12; ++i) h.value[i] = g.guidPrefix.value[i];
  for (size_t i = 0; i < 4; ++i) h.value[12+i] = g.entityId.value[i];
  return h;
}
struct Join { std::thread &t; ~Join() { if (t.joinable()) t.join(); } };
struct Result { rmw_ret_t ret; double elapsed; std::string error; };

Result trial(const rmw_service_t *service, CustomServiceInfo *info,
             const std::string &condition, int repeat) {
  uint8_t n = repeat + (condition == "immediate" ? 0x10 : condition == "delayed_0.3s" ? 0x20 : 0x30);
  GUID reader = guid(n, true), writer = guid(n + 0x40, false);
  info->pub_listener_->endpoint_add_reader_and_writer(reader, writer);
  auto match = [info, reader]() {
    eprosima::fastdds::dds::PublicationMatchedStatus s;
    s.current_count_change = 1; s.last_subscription_handle = handle(reader);
    info->pub_listener_->on_publication_matched(info->response_writer_, s);
  };
  std::thread delayed; Join join{delayed};
  if (condition == "immediate") match();
  else if (condition == "delayed_0.3s") delayed = std::thread([match]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); match();
  });
  rmw_request_id_t id{};
  rmw_fastrtps_shared_cpp::copy_from_fastrtps_guid_to_byte_array(reader, id.writer_guid);
  id.sequence_number = 1000 + repeat;
  GetState::Response response;
  rmw_reset_error(); auto start = Clock::now();
  rmw_ret_t ret = rmw_send_response(service, &id, &response);
  double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
  std::string error = ret == RMW_RET_OK ? "" : rmw_get_error_string().str;
  rmw_reset_error(); return {ret, elapsed, error};
}
bool range(double v, double low, double high) { return v >= low && v <= high; }
}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) { std::cerr << "usage: service_response_match_control EXPECTED_MS\n"; return 2; }
  long expected_ms = std::strtol(argv[1], nullptr, 10);
  if (expected_ms != 100 && expected_ms != 1000) { std::cerr << "EXPECTED_MS must be 100 or 1000\n"; return 2; }
  bool initialized = false;
  try {
    rclcpp::init(argc, argv); initialized = true;
    auto node = std::make_shared<rclcpp::Node>("service_response_match_control");
    auto service = node->create_service<GetState>("/service_response_match_control/get_state",
      [](std::shared_ptr<GetState::Request>, std::shared_ptr<GetState::Response>) {});
    auto publisher = node->create_publisher<std_msgs::msg::String>("/service_response_match_control/topic", rclcpp::QoS(1).reliable());
    auto client = node->create_client<GetState>("/service_response_match_control/client_control");
    auto *rc = rcl_client_get_rmw_handle(client->get_client_handle().get());
    auto *rs = rcl_service_get_rmw_handle(service->get_service_handle().get());
    auto *rp = rcl_publisher_get_rmw_handle(publisher->get_publisher_handle().get());
    std::string implementation = rmw_get_implementation_identifier();
    if (!rc || !rs || !rp || implementation != "rmw_fastrtps_cpp") throw std::runtime_error("requires rmw_fastrtps_cpp handles");
    auto *si = static_cast<CustomServiceInfo *>(rs->data);
    auto *pi = static_cast<CustomPublisherInfo *>(rp->data);
    auto *ci = static_cast<CustomClientInfo *>(rc->data);
    if (!ci || !ci->request_writer_ || !si || !si->response_writer_ || !si->pub_listener_ || !pi || !pi->data_writer_) throw std::runtime_error("null Fast DDS internal");
    double service_qos = seconds(si->response_writer_->get_qos().reliability().max_blocking_time);
    double topic_qos = seconds(pi->data_writer_->get_qos().reliability().max_blocking_time);
    double client_qos = seconds(ci->request_writer_->get_qos().reliability().max_blocking_time);
    double expected = expected_ms / 1000.0;
    bool passed = std::fabs(service_qos - expected) < 1e-6 && std::fabs(topic_qos - 0.1) < 1e-6 && std::fabs(client_qos - 0.1) < 1e-6;
    std::cout << std::fixed << std::setprecision(6)
      << "{\"type\":\"metadata\",\"rmw\":\"" << escape(implementation)
      << "\",\"source\":\"DataWriter::get_qos\",\"service_max_blocking_s\":" << service_qos
      << ",\"client_max_blocking_s\":" << client_qos
      << ",\"topic_max_blocking_s\":" << topic_qos << ",\"expected_service_max_blocking_s\":" << expected
      << ",\"mechanism\":\"ServicePubListener discovery event injection\"}\n";
    for (const std::string condition : {"immediate", "delayed_0.3s", "never"}) {
      for (int repeat = 1; repeat <= 2; ++repeat) {
        Result r = trial(rs, si, condition, repeat); bool ok;
        if (condition == "immediate") ok = r.ret == RMW_RET_OK && r.elapsed < 0.08;
        else if (condition == "delayed_0.3s" && expected_ms == 100) ok = r.ret == RMW_RET_TIMEOUT && range(r.elapsed, 0.07, 0.25);
        else if (condition == "delayed_0.3s") ok = r.ret == RMW_RET_OK && range(r.elapsed, 0.25, 0.70);
        else ok = r.ret == RMW_RET_TIMEOUT && range(r.elapsed, expected * 0.70, expected + 0.50);
        passed = passed && ok;
        std::cout << "{\"type\":\"trial\",\"condition\":\"" << condition << "\",\"repeat\":" << repeat
          << ",\"retcode\":" << static_cast<int>(r.ret) << ",\"elapsed_s\":" << r.elapsed
          << ",\"service_max_blocking_s\":" << service_qos << ",\"source\":\"rmw_send_response\",\"error\":\""
          << escape(r.error) << "\",\"pass\":" << (ok ? "true" : "false") << "}\n";
      }
    }
    std::cout << "{\"type\":\"summary\",\"pass\":" << (passed ? "true" : "false") << "}\n";
    client.reset(); service.reset(); publisher.reset(); node.reset(); rclcpp::shutdown(); initialized = false;
    return passed ? 0 : 1;
  } catch (const std::exception &e) {
    std::cerr << "fatal: " << e.what() << '\n'; if (initialized) rclcpp::shutdown(); return 1;
  }
}
