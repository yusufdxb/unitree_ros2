// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// Check the internally spun B2 client against an independent software peer.
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "common/b2_base_client.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "success";
  auto node = std::make_shared<rclcpp::Node>("unitree_b2_client_test");
  auto server = std::make_shared<rclcpp::Node>("unitree_b2_client_peer");
  auto publisher = server->create_publisher<unitree_api::msg::Response>(
    "/unitree_test/b2/response", 10);
  std::atomic<bool> request_valid{false};
  auto subscription = server->create_subscription<unitree_api::msg::Request>(
    "/unitree_test/b2/request", 10,
    [publisher, mode, &request_valid](unitree_api::msg::Request::ConstSharedPtr request) {
      request_valid.store(
        request->header.identity.id > 0 && request->header.identity.api_id == 1001 &&
        request->header.lease.id == 123456789 && request->header.policy.priority == 7 &&
        !request->header.policy.noreply && request->parameter == "{\"probe\":true}" &&
        request->binary == std::vector<uint8_t>({0, 127, 255}));
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
      if (mode == "malformed") {response.data = "{invalid-json";}
      if (mode == "empty" || mode == "status-only") {response.data.clear();}
      if (mode == "escaped-null") {response.data = R"({"probe":"\u0000"})";}
      if (mode == "embedded-null") {
        response.data.push_back('\0');
        response.data += "invalid-trailing-data";
      }
      publisher->publish(response);
      publisher->publish(response);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server);
  std::thread spin([&executor]() {executor.spin();});
  unitree_api::msg::Request request;
  request.header.identity.api_id = 1001;
  request.header.lease.id = 123456789;
  request.header.policy.priority = 7;
  request.header.policy.noreply = false;
  request.parameter = "{\"probe\":true}";
  request.binary = {0, 127, 255};
  nlohmann::json data = {{"stale", true}};
  int result;
  {
    BaseClient client(node.get(), "/unitree_test/b2/request", "/unitree_test/b2/response");
    result = mode == "status-only" ? client.Call(request) : client.Call(request, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  executor.cancel();
  spin.join();
  const int expected = mode == "status" ? -23 : mode == "timeout" ? UT_ROBOT_TASK_TIMEOUT :
    (mode == "malformed" || mode == "embedded-null") ?
    UT_ROBOT_TASK_UNKNOWN_ERROR : UT_ROBOT_SUCCESS;
  bool passed = result == expected && request_valid.load();
  if (mode == "success") {passed = passed && data.value("probe", "") == "ok";}
  if (mode == "empty") {passed = passed && data == nlohmann::json::object();}
  if (mode == "escaped-null") {
    passed = passed && data.at("probe").get<std::string>() == std::string(1, '\0');
  }
  if (expected != UT_ROBOT_SUCCESS || mode == "status-only") {
    passed = passed && data == nlohmann::json({{"stale", true}});
  }
  std::cout << mode << ": result=" << result << " data=" << data.dump() << std::endl;
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
