/**
 * This example demonstrates how to use ROS2 to receive low states of unitree
 * robots that publish unitree_hg/msg/LowState: g1 (default), h1_2, h2 and r1.
 * Select the model with --ros-args -p robot:=NAME; -p secondary_imu:=true also
 * reads the torso IMU on "secondary_imu". It only subscribes.
 **/
#include <stdexcept>
#include <string>

#include "common/robot_models.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_state.hpp"

constexpr bool INFO_IMU = true;    // Set 1 to info IMU states
constexpr bool INFO_MOTOR = true;  // Set 1 to info motor states
constexpr bool HIGH_FREQ = true;
// Set 1 to subscribe to low states with high frequencies (500Hz)

class LowStateSuber : public rclcpp::Node {
 public:
  LowStateSuber() : Node("low_state_suber") {
    using unitree_ros2_example::MessageFamily;
    const auto robot = declare_parameter<std::string>("robot", "g1");
    const auto *model = unitree_ros2_example::FindRobotModel(robot);
    if (model == nullptr || model->family != MessageFamily::kHg) {
      throw std::invalid_argument(
          "robot '" + robot + "' does not publish unitree_hg/msg/LowState; use " +
          unitree_ros2_example::RobotModelNames(MessageFamily::kHg) +
          " (read_low_state handles " +
          unitree_ros2_example::RobotModelNames(MessageFamily::kGo) + ")");
    }
    motor_count_ = model->motor_count;
    if (motor_count_ > static_cast<int>(sizeof(motor_) / sizeof(motor_[0]))) {
      throw std::logic_error("robot model exceeds the LowState motor slots");
    }
    if (declare_parameter<bool>("secondary_imu", false)) {
      imu_suber_ = this->create_subscription<unitree_hg::msg::IMUState>(
          "secondary_imu", rclcpp::SensorDataQoS(),
          [this](const unitree_hg::msg::IMUState::SharedPtr imu) {
            RCLCPP_INFO(this->get_logger(),
                        "Secondary IMU -- roll: %f; pitch: %f; yaw: %f",
                        imu->rpy[0], imu->rpy[1], imu->rpy[2]);
          });
    }
    // suber is set to subscribe "/lowcmd" or  "lf/lowstate" (low frequencies)
    // topic
    const auto *topic_name = "lf/lowstate";
    if (HIGH_FREQ) {
      topic_name = "lowstate";
    }
    // The suber  callback function is bind to low_state_suber::topic_callback
    suber_ = this->create_subscription<unitree_hg::msg::LowState>(
        topic_name, rclcpp::SensorDataQoS(),
        [this](const unitree_hg::msg::LowState::SharedPtr data) {
          topic_callback(data);
        });
  }

 private:
  void topic_callback(const unitree_hg::msg::LowState::SharedPtr &data) {
    // mode_machine identifies the robot configuration (for G1, see the
    // mode_machine table of unitree_ros g1_description).
    RCLCPP_INFO(this->get_logger(), "Mode -- mode_pr: %u; mode_machine: %u",
                static_cast<unsigned>(data->mode_pr),
                static_cast<unsigned>(data->mode_machine));
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
  }

  // Create the suber  to receive low state of robot
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr suber_;
  rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr imu_suber_;

  unitree_hg::msg::IMUState imu_;          // Unitree hg IMU message
  unitree_hg::msg::MotorState motor_[35];  // Unitree hg motor state message
  int motor_count_{29};                    // Motor slots used by the robot
  float battery_voltage_{};                // Battery voltage
  float battery_current_{};                // Battery current
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  int status = 0;
  try {
    rclcpp::spin(
        std::make_shared<LowStateSuber>());  // Run ROS2 node which is make
                                             // share with low_state_suber class
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("read_low_state_hg"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}