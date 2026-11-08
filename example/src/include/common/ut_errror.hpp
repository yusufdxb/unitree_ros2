#pragma once
#include <cstdint>
#define UT_DECL_ERR(name, code, desc) \
  const int32_t name = code;          \
  const constexpr char* name##_DESC = desc;

UT_DECL_ERR(UT_ROBOT_SUCCESS, 0, "success.")

UT_DECL_ERR(UT_ROBOT_TASK_TIMEOUT, -1, "task timeout.")

UT_DECL_ERR(UT_ROBOT_TASK_UNKNOWN_ERROR, -2, "task unknown error.")

UT_DECL_ERR(UT_ROBOT_TASK_NOT_PUBLISHED, -3, "task not published before its deadline.")

#define UT_PRINT_ERR(code, error)                         \
  if ((code) == (error)) {                                \
    RCLCPP_ERROR(this->get_logger(), "%s", error##_DESC); \
  }
