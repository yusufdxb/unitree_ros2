#pragma once
#include <cstdint>
#include <utility>
#include <atomic>
#include <mutex>
#include <iostream>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request  = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

  rclcpp::Node* node_{nullptr};
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;
  rclcpp::Subscription<Response>::SharedPtr persistent_sub_;

  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::SubscriptionOptions sub_opts_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::atomic<uint64_t> current_request_id_{0};
  std::atomic<int64_t> current_api_id_{0};
  std::mutex call_mutex_;
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  std::shared_ptr<const Response> received_response_;
  bool response_ready_{false};

public:
  BaseClient(rclcpp::Node* node,
             const std::string& topic_name_request,
             std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)) {
    InitRosComm();
  }

  ~BaseClient() {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  int32_t Call(Request req, nlohmann::json& js) {
    return CallImpl(std::move(req), &js);
  }

  int32_t Call(Request req) {
    return CallImpl(std::move(req), nullptr);
  }

private:
  int32_t CallImpl(Request req, nlohmann::json* js) {
    std::lock_guard<std::mutex> call_lock(call_mutex_);
    ResetState();

    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    current_api_id_.store(req.header.identity.api_id);
    current_request_id_.store(req.header.identity.id);

        req_puber_->publish(req);
    std::unique_lock<std::mutex> response_lock(response_mutex_);
    auto start_time = std::chrono::steady_clock::now();
    const std::chrono::seconds timeout(5);

    if (response_cv_.wait_for(response_lock, timeout, [this]() { 
        return response_ready_; 
    })) {

        if (!received_response_) {
            return UT_ROBOT_TASK_UNKNOWN_ERROR;
        }

        if (received_response_->header.status.code != 0) {
            return received_response_->header.status.code;
        }
        if (js == nullptr) {
            return UT_ROBOT_SUCCESS;
        }
        if (received_response_->data.empty()) {
            *js = nlohmann::json::object();
            return UT_ROBOT_SUCCESS;
        }

        try {
            if (received_response_->data.find('\0') != std::string::npos) {
                return UT_ROBOT_TASK_UNKNOWN_ERROR;
            }
            *js = nlohmann::json::parse(received_response_->data);
            auto end_time = std::chrono::steady_clock::now();
            auto wait_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            return UT_ROBOT_SUCCESS;
        } catch (const nlohmann::detail::exception& e) {
            return UT_ROBOT_TASK_UNKNOWN_ERROR;
        }
    } else {
        return UT_ROBOT_TASK_TIMEOUT;
    }
  }

  void InitRosComm() {
    req_puber_ = node_->create_publisher<Request>(topic_name_request_, rclcpp::QoS(10));
    // Only the response callback belongs to the private executor. The caller
    // may already spin this node, or add other B2 clients to the same node.
    sub_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
    sub_opts_.callback_group = sub_group_;

    persistent_sub_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(20),
        [this](const std::shared_ptr<const Response> data) {
          this->OnResponseReceived(data);
        },
        sub_opts_);

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_callback_group(sub_group_, node_->get_node_base_interface());
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    WaitForConnection();
  }

  void WaitForConnection() {
    const int max_attempts = 10;

    for (int i = 0; i < max_attempts; ++i) {
      auto request_subscribers = node_->count_subscribers(topic_name_request_);
      auto response_publishers = node_->count_publishers(topic_name_response_);

      if (request_subscribers > 0 && response_publishers > 0) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void OnResponseReceived(const std::shared_ptr<const Response> data) {
    std::lock_guard<std::mutex> lk(response_mutex_);
    const uint64_t expected_id = current_request_id_.load();
    const uint64_t resp_id = data->header.identity.id;

    if (expected_id != 0 && resp_id == expected_id &&
        data->header.identity.api_id == current_api_id_.load() && !response_ready_) {
            received_response_ = data;
            response_ready_ = true;
        response_cv_.notify_one();
    }
  }

  void ResetState() {
    current_request_id_.store(0);
    current_api_id_.store(0);
    {
        std::lock_guard<std::mutex> lk(response_mutex_);
        received_response_.reset();
        response_ready_ = false;
    }
  }
};
