// Copyright 2026 Yusuf Guenena
// SPDX-License-Identifier: BSD-3-Clause
// The request writer an API client advertises: reliable, volatile, and a
// five-second lifespan so a request held up by an outage is not retransmitted
// long after its call timed out. Read from the ROS graph, as a server sees it.
// Built for the generic client and, with UNITREE_B2_CLIENT, the B2 client.
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#ifdef UNITREE_ROBOT_STATE
#include "common/ros2_robot_state_client.h"
#elif defined(UNITREE_LEGACY_SPORT)
#ifdef UNITREE_B2_SPORT
#include "common/ros2_b2_sport_client.h"
#else
#include "common/ros2_sport_client.h"
#endif
#elif defined(UNITREE_B2_CLIENT)
#include "common/b2_base_client.hpp"
#else
#include "common/base_client.hpp"
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string prefix = "/unitree_test/request_qos_" + std::to_string(getpid());
  auto node = std::make_shared<rclcpp::Node>("unitree_request_qos_client");
  bool passed = false;
  {
#ifdef UNITREE_ROBOT_STATE
    RobotStateClient client(node.get());
    const std::string request_topic =
      node->get_node_topics_interface()->resolve_topic_name("/api/robot_state/request");
    const size_t expected_writers = 2;
#elif defined(UNITREE_LEGACY_SPORT)
    SportClient client(node.get());
    const std::string request_topic =
      node->get_node_topics_interface()->resolve_topic_name("/api/sport/request");
    const size_t expected_writers = 2;
#else
    BaseClient client(node.get(), prefix + "/request", prefix + "/response");
    const std::string request_topic = prefix + "/request";
    const size_t expected_writers = 1;
#endif
    std::vector<rclcpp::TopicEndpointInfo> writers;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (writers.size() != expected_writers && std::chrono::steady_clock::now() < deadline) {
      writers = node->get_publishers_info_by_topic(request_topic);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    passed = writers.size() == expected_writers;
    for (const auto & writer : writers) {
      const rclcpp::QoS & qos = writer.qos_profile();
      const auto lifespan = qos.lifespan().nanoseconds();
      std::cout << "reliability=" << static_cast<int>(qos.reliability()) <<
        " durability=" << static_cast<int>(qos.durability()) <<
        " lifespan_ns=" << lifespan << std::endl;
      passed = passed && qos.reliability() == rclcpp::ReliabilityPolicy::Reliable &&
        qos.durability() == rclcpp::DurabilityPolicy::Volatile &&
        qos.get_rmw_qos_profile().depth == 1 &&
        lifespan == std::chrono::nanoseconds(std::chrono::seconds(5)).count();
    }
    std::cout << "request writers found: " << writers.size() << std::endl;
  }
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
