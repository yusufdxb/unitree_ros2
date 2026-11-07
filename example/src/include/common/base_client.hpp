#pragma once
#include <atomic>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;

  rclcpp::Node* node_;
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;

 public:
  BaseClient(rclcpp::Node* node, const std::string& topic_name_request,
             std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)),
        req_puber_(node_->create_publisher<Request>(topic_name_request,
                                                    rclcpp::QoS(1))) {}

  int32_t Call(Request req, nlohmann::json& js) {
    return CallImpl(std::move(req), &js);
  }

  int32_t Call(Request req) {
    return CallImpl(std::move(req), nullptr);
  }

 private:
  int32_t CallImpl(Request req, nlohmann::json* js) {
    // Callbacks can outlive a timeout and duplicate responses can be queued.
    // Own completion state independently of this call's stack frame.
    auto response_promise =
        std::make_shared<std::promise<Response::ConstSharedPtr>>();
    auto completed = std::make_shared<std::atomic<bool>>(false);
    auto response_future = response_promise->get_future();
    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    const auto identity_id = req.header.identity.id;
    const auto api_id = req.header.identity.api_id;

    auto req_suber_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(1),
        [response_promise, completed, identity_id, api_id](
            const Response::ConstSharedPtr data) {
          if (data->header.identity.id == identity_id &&
              data->header.identity.api_id == api_id &&
              !completed->exchange(true)) {
            response_promise->set_value(data);
          }
        });

    req_puber_->publish(req);
    auto status = response_future.wait_for(std::chrono::seconds(5));

    Response response;
    if (status == std::future_status::ready) {
      response = *response_future.get();
      if (response.header.status.code != 0) {
        std::cout << "error code: " << response.header.status.code << std::endl;
        return response.header.status.code;
      }
      if (js != nullptr) {
        // JSON cannot contain literal NUL bytes. A C-string parser would hide
        // any bytes following one and could accept a corrupt response.
        if (response.data.find('\0') != std::string::npos) {
          return UT_ROBOT_TASK_UNKNOWN_ERROR;
        }
        try {
          *js = nlohmann::json::parse(response.data);
        } catch (const nlohmann::json::exception&) {
          return UT_ROBOT_TASK_UNKNOWN_ERROR;
        }
      }
      return UT_ROBOT_SUCCESS;
    }
    if (status == std::future_status::timeout) {
      return UT_ROBOT_TASK_TIMEOUT;
    }
    return UT_ROBOT_TASK_UNKNOWN_ERROR;
  }

};
