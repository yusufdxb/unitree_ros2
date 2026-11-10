/**
 * Robot models whose LowState layout is documented by official Unitree
 * sources. motor_count is the number of motor_state slots the official
 * unitree_sdk2 / unitree_sdk2_python low-level examples use for that model;
 * it is not evidence that a particular firmware release populates them.
 * See docs/robots.md for sources and validation status.
 **/
#ifndef UNITREE_ROS2_EXAMPLE_ROBOT_MODELS_HPP_
#define UNITREE_ROS2_EXAMPLE_ROBOT_MODELS_HPP_

#include <cstddef>
#include <string>

namespace unitree_ros2_example {

// unitree_go: unitree_go/msg/LowState, 20 motor slots.
// unitree_hg: unitree_hg/msg/LowState, 35 motor slots.
enum class MessageFamily { kGo, kHg };

struct RobotModel {
  const char *name;
  MessageFamily family;
  int motor_count;
};

inline const RobotModel *RobotModels(std::size_t *count) {
  static const RobotModel models[] = {
      {"go2", MessageFamily::kGo, 12},   // 4 legs x 3
      {"go2w", MessageFamily::kGo, 16},  // 12 leg + 4 wheel
      {"b2", MessageFamily::kGo, 12},
      {"b2w", MessageFamily::kGo, 16},  // 12 leg + 4 wheel
      {"h1", MessageFamily::kGo, 20},   // 19 joints; slot 9 is unused
      {"g1", MessageFamily::kHg, 29},   // 23-DOF variants leave 6 slots invalid
      {"h1_2", MessageFamily::kHg, 27},
      {"h2", MessageFamily::kHg, 31},
      {"r1", MessageFamily::kHg, 26},
  };
  *count = sizeof(models) / sizeof(models[0]);
  return models;
}

inline const RobotModel *FindRobotModel(const std::string &name) {
  std::size_t count = 0;
  const RobotModel *models = RobotModels(&count);
  for (std::size_t i = 0; i < count; ++i) {
    if (name == models[i].name) {
      return &models[i];
    }
  }
  return nullptr;
}

inline std::string RobotModelNames(MessageFamily family) {
  std::size_t count = 0;
  const RobotModel *models = RobotModels(&count);
  std::string names;
  for (std::size_t i = 0; i < count; ++i) {
    if (models[i].family == family) {
      names += names.empty() ? "" : ", ";
      names += models[i].name;
    }
  }
  return names;
}

}  // namespace unitree_ros2_example

#endif  // UNITREE_ROS2_EXAMPLE_ROBOT_MODELS_HPP_
