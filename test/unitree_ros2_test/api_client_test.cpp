// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// Exercise the existing client against a local mock on private topics.
#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include "common/base_client.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "success";
  auto node = std::make_shared<rclcpp::Node>("unitree_api_client_test");
  auto server = std::make_shared<rclcpp::Node>("unitree_api_mock");
  auto publisher = server->create_publisher<unitree_api::msg::Response>(
    "/unitree_test/client/response", 10);
  auto subscription = server->create_subscription<unitree_api::msg::Request>(
    "/unitree_test/client/request", 10,
    [publisher, mode](unitree_api::msg::Request::ConstSharedPtr request) {
      if (mode == "timeout") {return;}
      unitree_api::msg::Response response;
      response.header.identity = request->header.identity;
      if (mode == "success") {
        response.header.identity.id++;
        response.data = "{\"probe\":\"wrong-id\"}";
        publisher->publish(response);
        response.header.identity = request->header.identity;
        response.header.identity.api_id++;
        response.data = "{\"probe\":\"wrong-api\"}";
        publisher->publish(response);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        response.header.identity = request->header.identity;
      }
      response.header.status.code = mode == "status" ? -23 : 0;
      response.data = "{\"probe\":\"ok\"}";
      publisher->publish(response);
      publisher->publish(response);    // Duplicate delivery must be harmless.
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.add_node(server);
  std::atomic<bool> spin_failed{false};
  std::thread spin([&]() {
      try {
        executor.spin();
      } catch (const std::exception & error) {
        std::cerr << error.what() << std::endl;
        spin_failed.store(true);
      }
    });
  BaseClient client(node.get(), "/unitree_test/client/request",
    "/unitree_test/client/response");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  unitree_api::msg::Request request;
  request.header.identity.api_id = 1001;
  request.parameter = "{}";
  nlohmann::json data;
  const int result = client.Call(request, data);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor.cancel();
  spin.join();
  const int expected = mode == "status" ? -23 :
    mode == "timeout" ? UT_ROBOT_TASK_TIMEOUT : UT_ROBOT_SUCCESS;
  bool passed = result == expected && !spin_failed.load();
  if (mode == "success") {passed = passed && data.value("probe", "") == "ok";}
  std::cout << mode << ": result=" << result << " data=" << data.dump() << std::endl;
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
