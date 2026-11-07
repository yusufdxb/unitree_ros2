#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

// Request/response client for the Unitree API topics.
//
// One response subscription lives for the whole client lifetime, so responses
// are never lost to a reader created at call time. Each call registers its
// (identity.id, api_id) before publishing; the subscription callback completes
// only the matching pending call, once. Late responses for calls that already
// timed out, duplicates and responses for other clients are dropped. Calls may
// run concurrently and their responses may arrive in any order.
//
// As before, the caller must spin the node passed to the constructor while
// Call() waits, from a thread other than the one calling Call().
class BaseClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;
  using Key = std::pair<int64_t, int64_t>;  // identity.id, identity.api_id

  struct PendingCalls {
    std::mutex mutex;
    std::map<Key, std::promise<Response::ConstSharedPtr>> calls;
  };

  rclcpp::Node* node_;
  std::string topic_name_request_;
  std::string topic_name_response_;
  rclcpp::Publisher<Request>::SharedPtr req_puber_;
  // Shared with the subscription callback so it never touches this object.
  std::shared_ptr<PendingCalls> pending_;
  rclcpp::Subscription<Response>::SharedPtr resp_suber_;
  std::atomic<int64_t> timeout_ns_{std::chrono::nanoseconds(std::chrono::seconds(5)).count()};

  // Request ids must be unique among all clients in this process: clients on
  // the same response topic each see every response.
  static int64_t NextRequestId() {
    static std::atomic<int64_t> last{0};
    int64_t previous = last.load();
    int64_t next = 0;
    do {
      next = std::max(unitree::common::GetSystemUptimeInNanoseconds(), previous + 1);
    } while (!last.compare_exchange_weak(previous, next));
    return next;
  }

  // A request published before the server's reader is matched is lost, and a
  // response published before our reader is matched to the server's writer is
  // lost. Both are volatile topics, so wait for discovery within the deadline.
  bool WaitForMatch(std::chrono::steady_clock::time_point deadline) const {
    while (req_puber_->get_subscription_count() == 0 ||
           resp_suber_->get_publisher_count() == 0) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
  }

 public:
  BaseClient(rclcpp::Node* node, const std::string& topic_name_request,
             std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)),
        req_puber_(node_->create_publisher<Request>(topic_name_request,
                                                    rclcpp::QoS(1))),
        pending_(std::make_shared<PendingCalls>()) {
    // Depth > 1 so bursts of duplicate or out-of-order responses are not
    // overwritten before the callback runs. Reliability is unchanged.
    std::weak_ptr<PendingCalls> weak_pending = pending_;
    resp_suber_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(10),
        [weak_pending](const Response::ConstSharedPtr data) {
          auto pending = weak_pending.lock();
          if (!pending) {
            return;
          }
          std::promise<Response::ConstSharedPtr> promise;
          {
            std::lock_guard<std::mutex> lock(pending->mutex);
            auto it = pending->calls.find(
                Key(data->header.identity.id, data->header.identity.api_id));
            if (it == pending->calls.end()) {
              return;  // Not ours, duplicate, or arrived after a timeout.
            }
            promise = std::move(it->second);
            pending->calls.erase(it);
          }
          promise.set_value(data);
        });
  }

  // Per-call timeout, including DDS discovery; the default is five seconds.
  void SetTimeout(std::chrono::nanoseconds timeout) {
    timeout_ns_.store(timeout.count());
  }

  int32_t Call(Request req, nlohmann::json& js) {
    return CallImpl(std::move(req), &js);
  }

  int32_t Call(Request req) {
    return CallImpl(std::move(req), nullptr);
  }

 private:
  int32_t CallImpl(Request req, nlohmann::json* js) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::nanoseconds(timeout_ns_.load());
    req.header.identity.id = NextRequestId();
    const Key key(req.header.identity.id, req.header.identity.api_id);

    std::future<Response::ConstSharedPtr> response_future;
    {
      std::lock_guard<std::mutex> lock(pending_->mutex);
      response_future = pending_->calls[key].get_future();
    }

    WaitForMatch(deadline);
    req_puber_->publish(req);
    auto status = response_future.wait_until(deadline);
    if (status != std::future_status::ready) {
      std::lock_guard<std::mutex> lock(pending_->mutex);
      // The callback may have completed the call just before this erase.
      if (pending_->calls.erase(key) == 1) {
        return UT_ROBOT_TASK_TIMEOUT;
      }
    }

    // Keep the message alive for the whole parse; get() returns a temporary.
    const Response::ConstSharedPtr response_ptr = response_future.get();
    const Response& response = *response_ptr;
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
};
