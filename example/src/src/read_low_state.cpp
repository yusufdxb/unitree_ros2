/**
 * This example demonstrates how to use ROS2 to receive low states of unitree
 * robots that publish unitree_go/msg/LowState: go2 (default), go2w, b2, b2w
 * and h1. Select the model with --ros-args -p robot:=NAME. It only subscribes.
 **/
#include <stdexcept>
#include <string>

#include "common/robot_models.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/motor_state.hpp"

constexpr bool INFO_IMU = true;         // Set 1 to info IMU states
constexpr bool INFO_MOTOR = true;       // Set 1 to info motor states
constexpr bool INFO_FOOT_FORCE = true;  // Set 1 to info foot force states
constexpr bool INFO_BATTERY = true;     // Set 1 to info battery states
constexpr bool HIGH_FREQ = true;
// Set 1 to subscribe to low states with high frequencies (500Hz)

class LowStateSuber : public rclcpp::Node {
 public:
  LowStateSuber() : Node("low_state_suber") {
    using unitree_ros2_example::MessageFamily;
    const auto robot = declare_parameter<std::string>("robot", "go2");
    const auto *model = unitree_ros2_example::FindRobotModel(robot);
    if (model == nullptr || model->family != MessageFamily::kGo) {
      throw std::invalid_argument(
          "robot '" + robot + "' does not publish unitree_go/msg/LowState; use " +
          unitree_ros2_example::RobotModelNames(MessageFamily::kGo) +
          " (read_low_state_hg handles " +
          unitree_ros2_example::RobotModelNames(MessageFamily::kHg) + ")");
    }
    motor_count_ = model->motor_count;
    if (motor_count_ > static_cast<int>(sizeof(motor_) / sizeof(motor_[0]))) {
      throw std::logic_error("robot model exceeds the LowState motor slots");
    }
    // suber is set to subscribe "/lowcmd" or  "lf/lowstate" (low frequencies)
    // topic
    const auto *topic_name = "hf/lowstate";
    if (HIGH_FREQ) {
      topic_name = "lowstate";
    }
    // The suber  callback function is bind to low_state_suber::topic_callback
    suber_ = this->create_subscription<unitree_go::msg::LowState>(
        topic_name, rclcpp::SensorDataQoS(),
        [this](const unitree_go::msg::LowState::SharedPtr data) {
          topic_callback(data);
        });
  }

 private:
  void topic_callback(const unitree_go::msg::LowState::SharedPtr &data) {
    if (INFO_IMU) {
      // Info IMU states
      // RPY euler angle(ZYX order respected to body frame)
      // Quaternion
      // Gyroscope (raw data)
      // Accelerometer (raw data)
      imu_ = data->imu_state;

      RCLCPP_INFO(this->get_logger(),
                  "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0],
                  imu_.rpy[1], imu_.rpy[2]);
      RCLCPP_INFO(this->get_logger(),
                  "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                  imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2],
                  imu_.quaternion[3]);
      RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f",
                  imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
      RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                  imu_.accelerometer[0], imu_.accelerometer[1],
                  imu_.accelerometer[2]);
    }

    if (INFO_MOTOR) {
      // Info motor states
      // q: angluar (rad)
      // dq: angluar velocity (rad/s)
      // ddq: angluar acceleration (rad/(s^2))
      // tau_est: Estimated external torque

      for (int i = 0; i < motor_count_; i++) {
        motor_[i] = data->motor_state[i];
        RCLCPP_INFO(this->get_logger(),
                    "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                    i, motor_[i].q, motor_[i].dq, motor_[i].ddq,
                    motor_[i].tau_est);
      }
    }

    if (INFO_FOOT_FORCE) {
      // Info foot force value (int not true value)
      for (int i = 0; i < 4; i++) {
        foot_force_[i] = data->foot_force[i];
        foot_force_est_[i] = data->foot_force_est[i];
      }

      RCLCPP_INFO(this->get_logger(),
                  "Foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
                  foot_force_[0], foot_force_[1], foot_force_[2],
                  foot_force_[3]);
      RCLCPP_INFO(
          this->get_logger(),
          "Estimated foot force -- foot0: %d; foot1: %d; foot2: %d; foot3: %d",
          foot_force_est_[0], foot_force_est_[1], foot_force_est_[2],
          foot_force_est_[3]);
    }

    if (INFO_BATTERY) {
      // Info battery states
      // battery current
      // battery voltage
      battery_current_ = data->power_a;
      battery_voltage_ = data->power_v;

      RCLCPP_INFO(this->get_logger(),
                  "Battery state -- current: %f; voltage: %f", battery_current_,
                  battery_voltage_);
    }
  }

  // Create the suber  to receive low state of robot
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr suber_;

  unitree_go::msg::IMUState imu_;          // Unitree go2 IMU message
  unitree_go::msg::MotorState motor_[20];  // Unitree go2 motor state message
  int motor_count_{12};                    // Motor slots used by the robot
  int16_t foot_force_[4]{};                // External contact force value (int)
  int16_t
      foot_force_est_[4]{};  // Estimated  external contact force value (int)
  float battery_voltage_{};  // Battery voltage
  float battery_current_{};  // Battery current
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  int status = 0;
  try {
    rclcpp::spin(
        std::make_shared<LowStateSuber>());  // Run ROS2 node which is make
                                             // share with low_state_suber class
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("read_low_state"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}