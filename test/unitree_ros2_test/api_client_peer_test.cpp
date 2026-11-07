// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// Drive BaseClient against the separate plain DDS server in dds_api_server.c.
// The first call is made immediately after construction, with no settle time.
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include "common/base_client.hpp"

namespace
{
bool g_passed = true;

void expect(bool condition, const std::string & what)
{
  std::cout << (condition ? "ok   " : "FAIL ") << what << std::endl;
  g_passed = g_passed && condition;
}

bool genuine(const nlohmann::json & js, int64_t api)
{
  return js.is_object() && js.value("api", int64_t{0}) == api && !js.value("decoy", true);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const char * ns = std::getenv("UNITREE_DDS_TEST_NAMESPACE");
  if (ns == nullptr) {
    std::cerr << "UNITREE_DDS_TEST_NAMESPACE is required" << std::endl;
    return 2;
  }
  const std::string prefix = std::string("/") + ns;
  auto node = std::make_shared<rclcpp::Node>("unitree_api_peer_client");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::atomic<bool> spin_failed{false};
  std::thread spin([&]() {
      try {
        executor.spin();
      } catch (const std::exception & error) {
        std::cerr << error.what() << std::endl;
        spin_failed.store(true);
      }
    });
  {
    BaseClient client(node.get(), prefix + "/request", prefix + "/response");
    auto call = [&client](int64_t api, nlohmann::json & js) {
        unitree_api::msg::Request request;
        request.header.identity.api_id = api;
        return client.Call(request, js);
      };

    // 1. No settle time: the request and response endpoints are still being
    //    discovered. Decoys with a wrong id or api and a duplicate follow.
    nlohmann::json first;
    expect(call(2001, first) == UT_ROBOT_SUCCESS, "first call right after construction");
    expect(genuine(first, 2001), "first call ignores decoys: " + first.dump());

    // 2. Two concurrent calls answered in reverse order.
    nlohmann::json held, second;
    int32_t held_result = -1, second_result = -1;
    std::thread held_call([&]() {held_result = call(2002, held);});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::thread second_call([&]() {second_result = call(2003, second);});
    held_call.join();
    second_call.join();
    expect(held_result == UT_ROBOT_SUCCESS && genuine(held, 2002),
      "held call gets its own out-of-order response: " + held.dump());
    expect(second_result == UT_ROBOT_SUCCESS && genuine(second, 2003),
      "later call answered first: " + second.dump());

    // 3. A response delivered after its call timed out must not complete or
    //    corrupt the next call, which is pending when the late response arrives.
    nlohmann::json late = {{"untouched", true}};
    client.SetTimeout(std::chrono::milliseconds(300));
    expect(call(2004, late) == UT_ROBOT_TASK_TIMEOUT, "short timeout expires");
    expect(late == nlohmann::json({{"untouched", true}}), "timed-out call leaves output");
    client.SetTimeout(std::chrono::seconds(5));
    nlohmann::json next;
    expect(call(2005, next) == UT_ROBOT_SUCCESS && genuine(next, 2005),
      "next call ignores the late response and a cross-api decoy: " + next.dump());
  }
  executor.cancel();
  spin.join();
  expect(!spin_failed.load(), "executor did not throw");
  rclcpp::shutdown();
  return g_passed ? 0 : 1;
}
