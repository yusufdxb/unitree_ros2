// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
/* unitree_sdk2 software peer. It uses Unitree's own generated C++ types and the
 * Cyclone DDS libraries bundled with unitree_sdk2, not this repository's IDL.
 * Every topic is under rt/<namespace>/, so no robot endpoint is addressed.
 *
 * For each type it echoes rt/<ns>/in/<key> to rt/<ns>/out/<key>, prints the
 * named fields it received (RECV), and publishes a sample it filled itself on
 * rt/<ns>/origin/<key> (ORIGIN lists those fields). Exits on SIGINT/SIGTERM. */
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "dds/dds.hpp"
#include "unitree/idl/go2/LidarState_.hpp"
#include "unitree/idl/go2/LowCmd_.hpp"
#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/idl/go2/MotorCmds_.hpp"
#include "unitree/idl/go2/SportModeState_.hpp"
#include "unitree/idl/go2/WirelessController_.hpp"
#include "unitree/idl/hg/BmsState_.hpp"
#include "unitree/idl/hg/HandCmd_.hpp"
#include "unitree/idl/hg/HandState_.hpp"
#include "unitree/idl/hg/IMUState_.hpp"
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/hg/MainBoardState_.hpp"
#include "unitree/idl/hg/MotorCmd_.hpp"
#include "unitree/idl/hg/MotorState_.hpp"
#include "unitree/idl/hg/PressSensorState_.hpp"
#include "unitree/idl/ros2/Point32_.hpp"

namespace go = unitree_go::msg::dds_;
namespace hg = unitree_hg::msg::dds_;

namespace {

std::atomic<bool> running{true};

template <typename T> const char *Kind() {
  if (std::is_same<T, uint8_t>::value) return "u8";
  if (std::is_same<T, int8_t>::value) return "i8";
  if (std::is_same<T, uint16_t>::value) return "u16";
  if (std::is_same<T, int16_t>::value) return "i16";
  if (std::is_same<T, uint32_t>::value) return "u32";
  if (std::is_same<T, int32_t>::value) return "i32";
  if (std::is_same<T, float>::value) return "f32";
  if (std::is_same<T, double>::value) return "f64";
  return "?";
}

// Writes deterministic, distinct values: k-th field gets k+2 (negated if signed).
class Filler {
 public:
  template <typename T>
  typename std::enable_if<std::is_integral<T>::value>::type operator()(const char *path, T &v) {
    v = std::is_signed<T>::value ? static_cast<T>(-(next_ + 2)) : static_cast<T>(next_ + 2);
    Record(path, std::to_string(static_cast<long long>(v)), Kind<T>());
  }
  template <typename T>
  typename std::enable_if<std::is_floating_point<T>::value>::type operator()(const char *path, T &v) {
    v = static_cast<T>((next_ + 1) * 0.5);
    char text[64];
    std::snprintf(text, sizeof(text), "%.9g", static_cast<double>(v));
    Record(path, text, Kind<T>());
  }
  void operator()(const char *path, std::string &v) {
    v = "sdk2-" + std::to_string(next_);
    Record(path, v, "str");
  }
  template <typename S> void size(const char *path, S &sequence) {
    sequence.resize(2);
    out_ << ' ' << path << "@size=2:u32";
  }
  std::string text() const { return out_.str(); }

 private:
  void Record(const char *path, const std::string &value, const char *kind) {
    out_ << ' ' << path << '=' << value << ':' << kind;
    ++next_;
  }
  int next_ = 0;
  std::ostringstream out_;
};

class Printer {
 public:
  template <typename T>
  typename std::enable_if<std::is_integral<T>::value>::type operator()(const char *path, const T &v) {
    out_ << ' ' << path << '=' << static_cast<long long>(v) << ':' << Kind<T>();
  }
  template <typename T>
  typename std::enable_if<std::is_floating_point<T>::value>::type operator()(const char *path, const T &v) {
    char text[64];
    std::snprintf(text, sizeof(text), "%.9g", static_cast<double>(v));
    out_ << ' ' << path << '=' << text << ':' << Kind<T>();
  }
  void operator()(const char *path, const std::string &v) { out_ << ' ' << path << '=' << v << ":str"; }
  template <typename S> void size(const char *path, const S &sequence) {
    out_ << ' ' << path << "@size=" << sequence.size() << ":u32";
  }
  std::string text() const { return out_.str(); }

 private:
  std::ostringstream out_;
};

// Visitors name fields exactly as the official definitions do. Arrays are
// sampled at both ends; sequences are visited after their size.
template <typename M, typename F> void VisitMotorStateGo(M &m, F &f, const std::string &p) {
  f((p + ".mode").c_str(), m.mode());
  f((p + ".q").c_str(), m.q());
  f((p + ".dq").c_str(), m.dq());
  f((p + ".ddq").c_str(), m.ddq());
  f((p + ".tau_est").c_str(), m.tau_est());
  f((p + ".temperature").c_str(), m.temperature());
  f((p + ".lost").c_str(), m.lost());
  f((p + ".reserve[1]").c_str(), m.reserve()[1]);
}

template <typename M, typename F> void VisitImu(M &m, F &f, const std::string &p) {
  f((p + ".quaternion[0]").c_str(), m.quaternion()[0]);
  f((p + ".quaternion[3]").c_str(), m.quaternion()[3]);
  f((p + ".gyroscope[2]").c_str(), m.gyroscope()[2]);
  f((p + ".accelerometer[0]").c_str(), m.accelerometer()[0]);
  f((p + ".rpy[2]").c_str(), m.rpy()[2]);
  f((p + ".temperature").c_str(), m.temperature());
}

template <typename M, typename F> void VisitMotorStateHg(M &m, F &f, const std::string &p) {
  f((p + ".mode").c_str(), m.mode());
  f((p + ".q").c_str(), m.q());
  f((p + ".dq").c_str(), m.dq());
  f((p + ".ddq").c_str(), m.ddq());
  f((p + ".tau_est").c_str(), m.tau_est());
  f((p + ".temperature[0]").c_str(), m.temperature()[0]);
  f((p + ".temperature[1]").c_str(), m.temperature()[1]);
  f((p + ".vol").c_str(), m.vol());
  f((p + ".sensor[1]").c_str(), m.sensor()[1]);
  f((p + ".motorstate").c_str(), m.motorstate());
  f((p + ".reserve[3]").c_str(), m.reserve()[3]);
}

template <typename M, typename F> void VisitMotorCmdHg(M &m, F &f, const std::string &p) {
  f((p + ".mode").c_str(), m.mode());
  f((p + ".q").c_str(), m.q());
  f((p + ".dq").c_str(), m.dq());
  f((p + ".tau").c_str(), m.tau());
  f((p + ".kp").c_str(), m.kp());
  f((p + ".kd").c_str(), m.kd());
  f((p + ".reserve").c_str(), m.reserve());
}

template <typename M, typename F> void VisitPress(M &m, F &f, const std::string &p) {
  f((p + ".pressure[0]").c_str(), m.pressure()[0]);
  f((p + ".pressure[11]").c_str(), m.pressure()[11]);
  f((p + ".temperature[11]").c_str(), m.temperature()[11]);
  f((p + ".lost").c_str(), m.lost());
  f((p + ".reserve").c_str(), m.reserve());
}

template <typename M, typename F> void Visit(go::LowState_ *, M &m, F &f) {
  f("head[0]", m.head()[0]);
  f("head[1]", m.head()[1]);
  f("level_flag", m.level_flag());
  f("frame_reserve", m.frame_reserve());
  f("sn[1]", m.sn()[1]);
  f("version[1]", m.version()[1]);
  f("bandwidth", m.bandwidth());
  VisitImu(m.imu_state(), f, "imu_state");
  VisitMotorStateGo(m.motor_state()[0], f, "motor_state[0]");
  VisitMotorStateGo(m.motor_state()[19], f, "motor_state[19]");
  f("bms_state.status", m.bms_state().status());
  f("bms_state.soc", m.bms_state().soc());
  f("bms_state.current", m.bms_state().current());
  f("bms_state.cycle", m.bms_state().cycle());
  f("bms_state.bq_ntc[1]", m.bms_state().bq_ntc()[1]);
  f("bms_state.mcu_ntc[0]", m.bms_state().mcu_ntc()[0]);
  f("bms_state.cell_vol[14]", m.bms_state().cell_vol()[14]);
  f("foot_force[3]", m.foot_force()[3]);
  f("foot_force_est[0]", m.foot_force_est()[0]);
  f("tick", m.tick());
  f("wireless_remote[39]", m.wireless_remote()[39]);
  f("bit_flag", m.bit_flag());
  f("adc_reel", m.adc_reel());
  f("temperature_ntc1", m.temperature_ntc1());
  f("temperature_ntc2", m.temperature_ntc2());
  f("power_v", m.power_v());
  f("power_a", m.power_a());
  f("fan_frequency[3]", m.fan_frequency()[3]);
  f("reserve", m.reserve());
  f("crc", m.crc());
}

template <typename M, typename F> void Visit(go::SportModeState_ *, M &m, F &f) {
  f("stamp.sec", m.stamp().sec());
  f("stamp.nanosec", m.stamp().nanosec());
  f("error_code", m.error_code());
  VisitImu(m.imu_state(), f, "imu_state");
  f("mode", m.mode());
  f("progress", m.progress());
  f("gait_type", m.gait_type());
  f("foot_raise_height", m.foot_raise_height());
  f("position[2]", m.position()[2]);
  f("body_height", m.body_height());
  f("velocity[0]", m.velocity()[0]);
  f("yaw_speed", m.yaw_speed());
  f("range_obstacle[3]", m.range_obstacle()[3]);
  f("foot_force[3]", m.foot_force()[3]);
  f("foot_position_body[11]", m.foot_position_body()[11]);
  f("foot_speed_body[0]", m.foot_speed_body()[0]);
  f("path_point[0].t_from_start", m.path_point()[0].t_from_start());
  f("path_point[9].vyaw", m.path_point()[9].vyaw());
}

template <typename M, typename F> void Visit(go::LidarState_ *, M &m, F &f) {
  f("stamp", m.stamp());
  f("firmware_version", m.firmware_version());
  f("software_version", m.software_version());
  f("sdk_version", m.sdk_version());
  f("sys_rotation_speed", m.sys_rotation_speed());
  f("com_rotation_speed", m.com_rotation_speed());
  f("error_state", m.error_state());
  f("dirty_percentage", m.dirty_percentage());
  f("cloud_frequency", m.cloud_frequency());
  f("cloud_packet_loss_rate", m.cloud_packet_loss_rate());
  f("cloud_size", m.cloud_size());
  f("cloud_scan_num", m.cloud_scan_num());
  f("imu_frequency", m.imu_frequency());
  f("imu_packet_loss_rate", m.imu_packet_loss_rate());
  f("imu_rpy[2]", m.imu_rpy()[2]);
  f("serial_recv_stamp", m.serial_recv_stamp());
  f("serial_buffer_size", m.serial_buffer_size());
  f("serial_buffer_read", m.serial_buffer_read());
}

template <typename M, typename F> void Visit(go::WirelessController_ *, M &m, F &f) {
  f("lx", m.lx());
  f("ly", m.ly());
  f("rx", m.rx());
  f("ry", m.ry());
  f("keys", m.keys());
}

template <typename M, typename F> void Visit(go::LowCmd_ *, M &m, F &f) {
  f("head[1]", m.head()[1]);
  f("level_flag", m.level_flag());
  f("frame_reserve", m.frame_reserve());
  f("sn[0]", m.sn()[0]);
  f("version[1]", m.version()[1]);
  f("bandwidth", m.bandwidth());
  f("motor_cmd[0].mode", m.motor_cmd()[0].mode());
  f("motor_cmd[19].q", m.motor_cmd()[19].q());
  f("motor_cmd[19].dq", m.motor_cmd()[19].dq());
  f("motor_cmd[19].tau", m.motor_cmd()[19].tau());
  f("motor_cmd[19].kp", m.motor_cmd()[19].kp());
  f("motor_cmd[19].kd", m.motor_cmd()[19].kd());
  f("motor_cmd[19].reserve[2]", m.motor_cmd()[19].reserve()[2]);
  f("bms_cmd.off", m.bms_cmd().off());
  f("bms_cmd.reserve[2]", m.bms_cmd().reserve()[2]);
  f("wireless_remote[39]", m.wireless_remote()[39]);
  f("led[11]", m.led()[11]);
  f("fan[1]", m.fan()[1]);
  f("gpio", m.gpio());
  f("reserve", m.reserve());
  f("crc", m.crc());
}

template <typename M, typename F> void Visit(go::MotorCmds_ *, M &m, F &f) {
  f.size("cmds", m.cmds());
  if (m.cmds().size() < 2) return;
  f("cmds[0].mode", m.cmds()[0].mode());
  f("cmds[1].q", m.cmds()[1].q());
  f("cmds[1].kd", m.cmds()[1].kd());
  f("cmds[1].reserve[2]", m.cmds()[1].reserve()[2]);
}

template <typename M, typename F> void Visit(hg::LowState_ *, M &m, F &f) {
  f("version[1]", m.version()[1]);
  f("mode_pr", m.mode_pr());
  f("mode_machine", m.mode_machine());
  f("tick", m.tick());
  VisitImu(m.imu_state(), f, "imu_state");
  VisitMotorStateHg(m.motor_state()[0], f, "motor_state[0]");
  VisitMotorStateHg(m.motor_state()[34], f, "motor_state[34]");
  f("wireless_remote[0]", m.wireless_remote()[0]);
  f("wireless_remote[39]", m.wireless_remote()[39]);
  f("reserve[3]", m.reserve()[3]);
  f("crc", m.crc());
}

template <typename M, typename F> void Visit(hg::LowCmd_ *, M &m, F &f) {
  f("mode_pr", m.mode_pr());
  f("mode_machine", m.mode_machine());
  VisitMotorCmdHg(m.motor_cmd()[0], f, "motor_cmd[0]");
  VisitMotorCmdHg(m.motor_cmd()[34], f, "motor_cmd[34]");
  f("reserve[3]", m.reserve()[3]);
  f("crc", m.crc());
}

template <typename M, typename F> void Visit(hg::IMUState_ *, M &m, F &f) {
  VisitImu(m, f, "");
}

template <typename M, typename F> void Visit(hg::MotorState_ *, M &m, F &f) {
  VisitMotorStateHg(m, f, "");
}

template <typename M, typename F> void Visit(hg::MotorCmd_ *, M &m, F &f) {
  VisitMotorCmdHg(m, f, "");
}

template <typename M, typename F> void Visit(hg::HandState_ *, M &m, F &f) {
  f.size("motor_state", m.motor_state());
  f.size("press_sensor_state", m.press_sensor_state());
  if (m.motor_state().size() >= 2) VisitMotorStateHg(m.motor_state()[1], f, "motor_state[1]");
  if (m.press_sensor_state().size() >= 2) VisitPress(m.press_sensor_state()[1], f, "press_sensor_state[1]");
  VisitImu(m.imu_state(), f, "imu_state");
  f("power_v", m.power_v());
  f("power_a", m.power_a());
  f("system_v", m.system_v());
  f("device_v", m.device_v());
  f("error[1]", m.error()[1]);
  f("reserve[1]", m.reserve()[1]);
}

template <typename M, typename F> void Visit(hg::HandCmd_ *, M &m, F &f) {
  f.size("motor_cmd", m.motor_cmd());
  if (m.motor_cmd().size() >= 2) VisitMotorCmdHg(m.motor_cmd()[1], f, "motor_cmd[1]");
  f("reserve[3]", m.reserve()[3]);
}

template <typename M, typename F> void Visit(hg::BmsState_ *, M &m, F &f) {
  f("version_high", m.version_high());
  f("version_low", m.version_low());
  f("fn", m.fn());
  f("cell_vol[39]", m.cell_vol()[39]);
  f("bmsvoltage[2]", m.bmsvoltage()[2]);
  f("current", m.current());
  f("soc", m.soc());
  f("soh", m.soh());
  f("temperature[11]", m.temperature()[11]);
  f("cycle", m.cycle());
  f("manufacturer_date", m.manufacturer_date());
  f("bmsstate[4]", m.bmsstate()[4]);
  f("reserve[2]", m.reserve()[2]);
}

template <typename M, typename F> void Visit(hg::MainBoardState_ *, M &m, F &f) {
  f("fan_state[5]", m.fan_state()[5]);
  f("temperature[5]", m.temperature()[5]);
  f("value[5]", m.value()[5]);
  f("state[5]", m.state()[5]);
}

template <typename M, typename F> void Visit(hg::PressSensorState_ *, M &m, F &f) {
  VisitPress(m, f, "");
}

template <typename M, typename F> void Visit(geometry_msgs::msg::dds_::Point32_ *, M &m, F &f) {
  f("x", m.x());
  f("y", m.y());
  f("z", m.z());
}

// Strip the leading '.' that nested visitors add to top-level paths.
std::string Clean(const std::string &text) {
  std::string out;
  for (size_t i = 0; i < text.size(); ++i) {
    if (text[i] == '.' && i > 0 && text[i - 1] == ' ') continue;
    out += text[i];
  }
  return out;
}

class Channel {
 public:
  virtual ~Channel() = default;
  virtual void Poll(bool publish_origin) = 0;
};

template <typename T> class TypedChannel : public Channel {
 public:
  TypedChannel(dds::domain::DomainParticipant &participant, const std::string &ns, const std::string &key)
      : key_(key),
        in_topic_(participant, "rt/" + ns + "/in/" + key),
        out_topic_(participant, "rt/" + ns + "/out/" + key),
        origin_topic_(participant, "rt/" + ns + "/origin/" + key),
        subscriber_(participant),
        publisher_(participant),
        reader_(subscriber_, in_topic_, ReaderQos()),
        echo_(publisher_, out_topic_, WriterQos()),
        origin_(publisher_, origin_topic_, WriterQos()) {
    Filler filler;
    Visit(static_cast<T *>(nullptr), origin_sample_, filler);
    std::cout << "ORIGIN " << key_ << Clean(filler.text()) << std::endl;
  }

  void Poll(bool publish_origin) override {
    auto samples = reader_.take();
    for (auto it = samples.begin(); it != samples.end(); ++it) {
      if (!it->info().valid()) continue;
      Printer printer;
      Visit(static_cast<T *>(nullptr), it->data(), printer);
      std::cout << "RECV " << key_ << Clean(printer.text()) << std::endl;
      echo_.write(it->data());
    }
    if (publish_origin && origin_.publication_matched_status().current_count() > 0) {
      origin_.write(origin_sample_);
    }
  }

 private:
  dds::sub::qos::DataReaderQos ReaderQos() {
    dds::sub::qos::DataReaderQos qos = subscriber_.default_datareader_qos();
    qos << dds::core::policy::Reliability::Reliable(dds::core::Duration::from_secs(2))
        << dds::core::policy::History::KeepLast(10);
    return qos;
  }
  dds::pub::qos::DataWriterQos WriterQos() {
    dds::pub::qos::DataWriterQos qos = publisher_.default_datawriter_qos();
    qos << dds::core::policy::Reliability::Reliable(dds::core::Duration::from_secs(2))
        << dds::core::policy::History::KeepLast(10);
    return qos;
  }

  std::string key_;
  dds::topic::Topic<T> in_topic_, out_topic_, origin_topic_;
  dds::sub::Subscriber subscriber_;
  dds::pub::Publisher publisher_;
  dds::sub::DataReader<T> reader_;
  dds::pub::DataWriter<T> echo_, origin_;
  T origin_sample_;
};

}  // namespace

int main() {
  const char *domain = std::getenv("ROS_DOMAIN_ID");
  const char *ns = std::getenv("UNITREE_DDS_TEST_NAMESPACE");
  std::signal(SIGINT, [](int) { running = false; });
  std::signal(SIGTERM, [](int) { running = false; });
  try {
    dds::domain::DomainParticipant participant(domain ? std::strtoul(domain, nullptr, 10) : 173);
    const std::string prefix = ns ? ns : "unitree_test/sdk2";
    std::vector<std::unique_ptr<Channel>> channels;
    channels.emplace_back(new TypedChannel<go::LowState_>(participant, prefix, "go_LowState"));
    channels.emplace_back(new TypedChannel<go::SportModeState_>(participant, prefix, "go_SportModeState"));
    channels.emplace_back(new TypedChannel<go::LidarState_>(participant, prefix, "go_LidarState"));
    channels.emplace_back(new TypedChannel<go::WirelessController_>(participant, prefix, "go_WirelessController"));
    channels.emplace_back(new TypedChannel<go::LowCmd_>(participant, prefix, "go_LowCmd"));
    channels.emplace_back(new TypedChannel<go::MotorCmds_>(participant, prefix, "go_MotorCmds"));
    channels.emplace_back(new TypedChannel<hg::LowState_>(participant, prefix, "hg_LowState"));
    channels.emplace_back(new TypedChannel<hg::LowCmd_>(participant, prefix, "hg_LowCmd"));
    channels.emplace_back(new TypedChannel<hg::IMUState_>(participant, prefix, "hg_IMUState"));
    channels.emplace_back(new TypedChannel<hg::MotorState_>(participant, prefix, "hg_MotorState"));
    channels.emplace_back(new TypedChannel<hg::MotorCmd_>(participant, prefix, "hg_MotorCmd"));
    channels.emplace_back(new TypedChannel<hg::HandState_>(participant, prefix, "hg_HandState"));
    channels.emplace_back(new TypedChannel<hg::HandCmd_>(participant, prefix, "hg_HandCmd"));
    channels.emplace_back(new TypedChannel<hg::BmsState_>(participant, prefix, "hg_BmsState"));
    channels.emplace_back(new TypedChannel<hg::MainBoardState_>(participant, prefix, "hg_MainBoardState"));
    channels.emplace_back(new TypedChannel<hg::PressSensorState_>(participant, prefix, "hg_PressSensorState"));
    channels.emplace_back(
        new TypedChannel<geometry_msgs::msg::dds_::Point32_>(participant, prefix, "geometry_Point32"));
    std::cout << "READY " << channels.size() << std::endl;
    for (unsigned tick = 0; running; ++tick) {
      for (auto &channel : channels) channel->Poll(tick % 10 == 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } catch (const dds::core::Exception &error) {
    std::cerr << "DDS: " << error.what() << std::endl;
    return 2;
  }
  return 0;
}
