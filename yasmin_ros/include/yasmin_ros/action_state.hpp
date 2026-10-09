// Copyright (C) 2023 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YASMIN_ROS__ACTION_STATE_HPP_
#define YASMIN_ROS__ACTION_STATE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "yasmin/blackboard.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace yasmin_ros {

/**
 * @brief A state class for handling ROS 2 action client operations.
 *
 * This class encapsulates the behavior of a ROS 2 action client within a YASMIN
 * state. It allows the creation and management of goals, feedback, and results
 * associated with an action server.
 *
 * @tparam ActionT The type of the action this state will interface with.
 */
template <typename ActionT> class ActionState : public yasmin::State {
  /// @brief Alias for the action goal type.
  using Goal = typename ActionT::Goal;
  /// @brief Alias for the action result type.
  using Result = typename ActionT::Result::SharedPtr;

  /// @brief Alias for the action feedback type.
  using Feedback = typename ActionT::Feedback;
  /// @brief Options for sending goals.
  using SendGoalOptions =
      typename rclcpp_action::Client<ActionT>::SendGoalOptions;
  /// @brief Shared pointer type for the action client.
  using ActionClient = typename rclcpp_action::Client<ActionT>::SharedPtr;
  /// @brief Handle for the action goal.
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  /// @brief Function type for creating a goal.
  using CreateGoalHandler = std::function<Goal(yasmin::Blackboard::SharedPtr)>;
  /// @brief Function type for handling results.
  using ResultHandler =
      std::function<std::string(yasmin::Blackboard::SharedPtr, Result)>;
  /// @brief Function type for handling feedback.
  using FeedbackHandler = std::function<void(yasmin::Blackboard::SharedPtr,
                                             std::shared_ptr<const Feedback>)>;

public:
  /**
   * @brief Shared pointer type for ActionState.
   */
  YASMIN_PTR_ALIASES(ActionState)

  /**
   * @brief Construct an ActionState with a specific action name and goal
   * handler.
   *
   * This constructor initializes the action state with a specified action name,
   * goal handler, and optional timeout.
   *
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param outcomes A set of possible outcomes for this action state.
   * @param wait_timeout (Optional) The maximum time to wait for the action
   * server. Default is -1 (no timeout).
   * @param response_timeout (Optional) The maximum time to wait for the action
   * response. Default is -1 (no timeout).
   * @param maximum_retry (Optional) Maximum retries of the action if it
   * returns timeout. Default is 3.
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(const std::string &action_name,
              CreateGoalHandler create_goal_handler,
              const yasmin::Outcomes &outcomes, int wait_timeout = -1,
              int response_timeout = -1, int maximum_retry = 3)
      : ActionState(nullptr, action_name, create_goal_handler, outcomes,
                    nullptr, nullptr, nullptr, wait_timeout, response_timeout,
                    maximum_retry) {}

  /**
   * @brief Construct an ActionState with a specific action name and goal
   * handler.
   *
   * This constructor initializes the action state with a specified action name,
   * goal handler, and optional timeout.
   *
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param outcomes A set of possible outcomes for this action state.
   * @param callback_group (Optional) The callback group for the action client.
   * @param wait_timeout (Optional) The maximum time to wait for the action
   * server. Default is -1 (no timeout).
   * @param response_timeout (Optional) The maximum time to wait for the action
   * response. Default is -1 (no timeout).
   * @param maximum_retry (Optional) Maximum retries of the action if it
   * returns timeout. Default is 3.
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(const std::string &action_name,
              CreateGoalHandler create_goal_handler,
              const yasmin::Outcomes &outcomes,
              rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
              int wait_timeout = -1, int response_timeout = -1,
              int maximum_retry = 3)
      : ActionState(nullptr, action_name, create_goal_handler, outcomes,
                    nullptr, nullptr, callback_group, wait_timeout,
                    response_timeout, maximum_retry) {}

  /**
   * @brief Construct an ActionState with result and feedback handlers.
   *
   * This constructor allows the specification of result and feedback handlers.
   *
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param result_handler (Optional) A function to handle the result of the
   * action.
   * @param feedback_handler (Optional) A function to handle feedback from the
   * action.
   * @param wait_timeout (Optional) The maximum time to wait for the action
   * server. Default is -1 (no timeout).
   * @param response_timeout (Optional) The maximum time to wait for the action
   * response. Default is -1 (no timeout).
   * @param maximum_retry (Optional) Maximum retries of the action if it returns
   * timeout. Default is 3.
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(const std::string &action_name,
              CreateGoalHandler create_goal_handler,
              ResultHandler result_handler = nullptr,
              FeedbackHandler feedback_handler = nullptr, int wait_timeout = -1,
              int response_timeout = -1, int maximum_retry = 3)
      : ActionState(nullptr, action_name, create_goal_handler, {},
                    result_handler, feedback_handler, nullptr, wait_timeout,
                    response_timeout, maximum_retry) {}

  /**
   * @brief Construct an ActionState with outcomes and handlers.
   *
   * This constructor allows specifying outcomes along with handlers for results
   * and feedback.
   *
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param outcomes A set of possible outcomes for this action state.
   * @param result_handler (Optional) A function to handle the result of the
   * action.
   * @param feedback_handler (Optional) A function to handle feedback from the
   * action.
   * @param wait_timeout (Optional) The maximum time to wait for the action
   * server. Default is -1 (no timeout).
   * @param response_timeout (Optional) The maximum time to wait for the action
   * response. Default is -1 (no timeout).
   * @param maximum_retry (Optional) Maximum retries of the action if it returns
   * timeout. Default is 3.
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(const std::string &action_name,
              CreateGoalHandler create_goal_handler,
              const yasmin::Outcomes &outcomes,
              ResultHandler result_handler = nullptr,
              FeedbackHandler feedback_handler = nullptr, int wait_timeout = -1,
              int response_timeout = -1, int maximum_retry = 3)
      : ActionState(nullptr, action_name, create_goal_handler, outcomes,
                    result_handler, feedback_handler, nullptr, wait_timeout,
                    response_timeout, maximum_retry) {}

  /**
   * @brief Construct an ActionState with a specified node and action name.
   *
   * This constructor allows specifying a ROS 2 node in addition to the action
   * name and goal handler.
   *
   * @param node A shared pointer to the ROS 2 node.
   * @param action_name The name of the action to communicate with.
   * @param create_goal_handler A function that creates a goal for the action.
   * @param outcomes A set of possible outcomes for this action state.
   * @param result_handler (Optional) A function to handle the result of the
   * action.
   * @param feedback_handler (Optional) A function to handle feedback from the
   * action.
   * @param callback_group (Optional) The callback group for the action client.
   * @param wait_timeout (Optional) The maximum time to wait for the action
   * server. Default is -1 (no timeout).
   * @param response_timeout (Optional) The maximum time to wait for the action
   * response. Default is -1 (no timeout).
   * @param maximum_retry (Optional) Maximum retries of the action if it returns
   * timeout. Default is 3.
   *
   * @throws std::invalid_argument if create_goal_handler is nullptr.
   */
  ActionState(const rclcpp::Node::SharedPtr &node,
              const std::string &action_name,
              CreateGoalHandler create_goal_handler,
              const yasmin::Outcomes &outcomes,
              ResultHandler result_handler = nullptr,
              FeedbackHandler feedback_handler = nullptr,
              rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
              int wait_timeout = -1, int response_timeout = -1,
              int maximum_retry = 3)
      : State({basic_outcomes::SUCCEED, basic_outcomes::ABORT,
               basic_outcomes::CANCEL}),
        action_name(action_name),
        create_goal_handler(std::move(create_goal_handler)),
        result_handler(std::move(result_handler)),
        feedback_handler(std::move(feedback_handler)),
        wait_timeout(wait_timeout), response_timeout(response_timeout),
        maximum_retry(maximum_retry) {

    this->set_outcome_description(basic_outcomes::SUCCEED,
                                  "The action succeeded");
    this->set_outcome_description(basic_outcomes::ABORT,
                                  "The action was aborted");
    this->set_outcome_description(basic_outcomes::CANCEL,
                                  "The action was canceled");

    if (this->wait_timeout != -1 || this->response_timeout != -1) {
      this->outcomes.insert(basic_outcomes::TIMEOUT);
      this->set_outcome_description(
          basic_outcomes::TIMEOUT,
          "The action server was not available in time");
    }

    if (outcomes.size() > 0) {
      for (const std::string &outcome : outcomes) {
        this->outcomes.insert(outcome);
      }
    }

    if (node == nullptr) {
      this->node_ = YasminNode::get_instance();
    } else {
      this->node_ = node;
    }

    this->action_client = ROSClientsCache::get_or_create_action_client<ActionT>(
        this->node_, action_name, callback_group);

    if (this->create_goal_handler == nullptr) {
      throw std::invalid_argument("create_goal_handler is needed");
    }
  }

  /**
   * @brief Destroy the action state.
   *
   * Disables late callbacks so they cannot access the destroyed state.
   */
  ~ActionState() override {
    {
      std::lock_guard<std::mutex> lock(this->callback_guard->mutex);
      this->callback_guard->alive = false;
    }
    cancel_execution(this->execution_, this->action_client);
  }

  /**
   * @brief Wake the local waiter and asynchronously request goal cancellation.
   *
   * A request made before acceptance is retained by the goal response callback.
   */
  void cancel_state() override {
    std::shared_ptr<GoalExecution> execution;
    {
      std::lock_guard<std::mutex> lock(this->action_done_mutex);
      yasmin::State::cancel_state();
      execution = this->execution_;
    }
    this->action_done_cond.notify_all();
    cancel_execution(execution, this->action_client);
  }

  /**
   * @brief Set a handler for results of goals that the server aborted.
   *
   * Action servers such as Nav2 report why a goal failed in the result of an
   * aborted goal (e.g. `error_code` and `error_msg`). The handler receives
   * that result and returns the outcome, which must be one of the state's
   * outcomes. Without a handler, an aborted goal returns ABORT. A goal
   * rejected by the server has no result and always returns ABORT.
   *
   * @param abort_handler Handler mapping an aborted result to an outcome.
   */
  void set_abort_handler(ResultHandler abort_handler) {
    this->abort_handler = std::move(abort_handler);
  }

  /**
   * @brief Bound the wait for the server to accept or reject the goal.
   *
   * Like the server timeout of Nav2's behavior tree action nodes, this does
   * not limit how long an accepted goal may run (see response_timeout). If the
   * server does not respond in time, the state requests cancellation, so a
   * goal accepted later is canceled, and returns TIMEOUT. This adds the
   * TIMEOUT outcome, so call it before adding the state to a state machine.
   *
   * @param timeout Maximum wait for the goal response; zero disables it.
   */
  void set_goal_response_timeout(std::chrono::nanoseconds timeout) {
    this->goal_response_timeout_ = timeout;
    if (timeout > std::chrono::nanoseconds::zero()) {
      this->outcomes.insert(basic_outcomes::TIMEOUT);
      this->set_outcome_description(
          basic_outcomes::TIMEOUT, "The action server did not respond in time");
    }
  }

  /**
   * @brief Notify that the action cancellation has completed.
   *
   * This function is called to notify that the action cancellation process
   * has finished.
   */
  void cancel_done() {
    {
      std::lock_guard<std::mutex> lock(this->action_cancel_mutex);
      this->cancel_done_ = true;
    }
    this->action_cancel_cond.notify_all();
  }

  /**
   * @brief Execute the action and return the outcome.
   *
   * This function creates a goal using the provided goal handler, sends the
   * goal to the action server, and waits for the result or feedback.
   *
   * @param blackboard A shared pointer to the blackboard used for
   * communication.
   * @return A string representing the outcome of the action execution.
   *
   * Possible outcomes include:
   * - `basic_outcomes::SUCCEED`: The action succeeded.
   * - `basic_outcomes::ABORT`: The action was aborted.
   * - `basic_outcomes::CANCEL`: The action was canceled.
   * - `basic_outcomes::TIMEOUT`: The action server was not available in time.
   */
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {

    auto execution = std::make_shared<GoalExecution>();
    uint64_t epoch;
    {
      // Finish any in-flight feedback handler before starting a new run.
      // Callback lock order is guard -> action_done_mutex throughout.
      std::lock_guard<std::mutex> guard_lock(this->callback_guard->mutex);
      std::lock_guard<std::mutex> lock(this->action_done_mutex);
      this->execution_ = execution;
      epoch = ++this->action_epoch_;
      this->action_done_.store(false);
      this->action_result.reset();
      this->action_status = rclcpp_action::ResultCode::UNKNOWN;
    }
    int retry_count = 0;

    // Wait for the action server to be available
    YASMIN_LOG_INFO("Waiting for action '%s'", this->action_name.c_str());

    const auto action_wait_timeout =
        std::chrono::duration<int64_t, std::ratio<1>>(this->wait_timeout);
    const auto wait_slice = std::chrono::milliseconds(100);
    auto waited = std::chrono::milliseconds::zero();

    while (!this->action_client->wait_for_action_server(wait_slice)) {

      if (this->is_canceled() ||
          !rclcpp::ok(this->node_->get_node_base_interface()->get_context())) {
        return basic_outcomes::CANCEL;
      }

      if (this->wait_timeout < 0) {
        continue;
      }

      waited += wait_slice;

      if (waited < action_wait_timeout) {
        continue;
      }

      waited = std::chrono::milliseconds::zero();
      YASMIN_LOG_WARN("Timeout reached, action '%s' is not available",
                      this->action_name.c_str());
      if (retry_count < this->maximum_retry) {
        retry_count++;
        YASMIN_LOG_WARN("Retrying to connect to action '%s' "
                        "(%d/%d)",
                        this->action_name.c_str(), retry_count,
                        this->maximum_retry);
      } else {
        return basic_outcomes::TIMEOUT;
      }
    }

    if (this->is_canceled()) {
      return basic_outcomes::CANCEL;
    }

    Goal goal = this->create_goal_handler(blackboard);

    if (this->is_canceled()) {
      return basic_outcomes::CANCEL;
    }

    auto callback_guard = this->callback_guard;
    std::weak_ptr<rclcpp_action::Client<ActionT>> weak_client =
        this->action_client;
    std::weak_ptr<GoalExecution> weak_execution = execution;
    SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback = [this, callback_guard, epoch,
                                                execution, weak_client](
                                                   GoalResponseArg response) {
      const auto handle = resolve_goal_response(response);
      {
        std::lock_guard<std::mutex> lock(execution->mutex);
        execution->goal_handle = handle;
        if (!handle) {
          execution->completed = true;
        }
      }
      execution->responded.store(true);
      bool cancel = false;
      {
        std::lock_guard<std::mutex> guard_lock(callback_guard->mutex);
        if (callback_guard->alive) {
          std::lock_guard<std::mutex> lock(this->action_done_mutex);
          cancel = this->action_epoch_.load() != epoch || this->is_canceled();
          if (this->action_epoch_.load() == epoch) {
            if (!handle) {
              this->action_status = rclcpp_action::ResultCode::ABORTED;
              this->action_done_.store(true);
            }
            // Also wakes a wait bounded by the goal response timeout.
            this->action_done_cond.notify_all();
          }
        } else {
          cancel = true;
        }
      }
      {
        std::lock_guard<std::mutex> lock(execution->mutex);
        cancel = cancel || execution->cancel_requested;
      }
      if (cancel) {
        cancel_execution(execution, weak_client.lock());
      }
    };

    send_goal_options.result_callback =
        [this, callback_guard, epoch,
         weak_execution](const typename GoalHandle::WrappedResult &result) {
          if (auto execution = weak_execution.lock()) {
            {
              std::lock_guard<std::mutex> lock(execution->mutex);
              execution->completed = true;
            }
            std::lock_guard<std::mutex> guard_lock(callback_guard->mutex);
            if (callback_guard->alive) {
              this->result_callback(result, epoch);
            }
          }
        };

    if (this->feedback_handler) {
      send_goal_options.feedback_callback =
          [this, callback_guard, epoch, weak_execution,
           blackboard](typename GoalHandle::SharedPtr,
                       std::shared_ptr<const Feedback> feedback) {
            std::lock_guard<std::mutex> guard_lock(callback_guard->mutex);
            auto execution = weak_execution.lock();
            if (!callback_guard->alive || !execution) {
              return;
            }
            {
              std::lock_guard<std::mutex> lock(execution->mutex);
              if (execution->cancel_requested || execution->completed) {
                return;
              }
            }
            if (this->action_epoch_.load() == epoch && !this->is_canceled()) {
              this->feedback_handler(blackboard, feedback);
            }
          };
    }

    // Send the goal to the action server
    YASMIN_LOG_INFO("Sending goal to action '%s'", this->action_name.c_str());
    const auto sent = std::chrono::steady_clock::now();
    this->action_client->async_send_goal(goal, send_goal_options);
    std::unique_lock<std::mutex> lock(this->action_done_mutex);

    if (this->goal_response_timeout_ > std::chrono::nanoseconds::zero() &&
        !this->action_done_cond.wait_until(
            lock, sent + this->goal_response_timeout_, [this, &execution]() {
              return execution->responded.load() || this->action_done_.load() ||
                     this->is_canceled();
            })) {
      lock.unlock();
      YASMIN_LOG_WARN("Action '%s' did not respond to the goal in time",
                      this->action_name.c_str());
      // A goal accepted after this point is canceled by its response callback.
      cancel_execution(execution, this->action_client);
      return basic_outcomes::TIMEOUT;
    }

    if (this->response_timeout > 0) {
      // Use a single total timeout instead of a retry loop that doesn't
      // re-send the request on each iteration
      const auto total_timeout =
          std::chrono::seconds(static_cast<int64_t>(this->response_timeout) *
                               (static_cast<int64_t>(this->maximum_retry) + 1));
      if (!this->action_done_cond.wait_until(
              lock, sent + total_timeout, [this]() {
                return this->action_done_.load() || this->is_canceled();
              })) {
        lock.unlock();
        cancel_execution(execution, this->action_client);
        return basic_outcomes::TIMEOUT;
      }

    } else {
      this->action_done_cond.wait(lock, [this]() {
        return this->action_done_.load() || this->is_canceled();
      });
    }

    if (this->is_canceled()) {
      return basic_outcomes::CANCEL;
    }

    const auto status = this->action_status;
    const auto result = this->action_result;
    lock.unlock();
    switch (status) {
    case rclcpp_action::ResultCode::CANCELED:
      return basic_outcomes::CANCEL;

    case rclcpp_action::ResultCode::ABORTED:
      if (this->abort_handler && result) {
        return this->abort_handler(blackboard, result);
      }
      return basic_outcomes::ABORT;

    case rclcpp_action::ResultCode::SUCCEEDED:
      if (this->result_handler) {
        return this->result_handler(blackboard, result);
      }
      return basic_outcomes::SUCCEED;

    default:
      return basic_outcomes::ABORT;
    }
  }

protected:
  /// @brief Shared pointer to the ROS 2 node.
  rclcpp::Node::SharedPtr node_;

private:
  /// @brief Guard disabling callbacks after the state is destroyed.
  struct CallbackGuard {
    std::mutex mutex;
    bool alive{true};
  };

  /// @brief Name of the action to communicate with.
  std::string action_name;
  /// @brief Shared pointer to the action client.
  ActionClient action_client;

  /// @brief Condition variable for action completion.
  std::condition_variable action_done_cond;
  /// @brief Mutex for protecting action completion.
  std::mutex action_done_mutex;
  /// @brief Condition variable for action cancellation.
  std::condition_variable action_cancel_cond;
  /// @brief Mutex for protecting action cancellation.
  std::mutex action_cancel_mutex;

  /// @brief Flag set when result is received.
  std::atomic<bool> action_done_{false};
  /// @brief Flag set when cancellation is confirmed.
  bool cancel_done_{false};
  /// @brief Shared pointer to the action result.
  Result action_result;
  /// @brief Status of the action execution.
  rclcpp_action::ResultCode action_status{};

  // This record outlives an invocation while its goal response is pending.
  // Result/feedback callbacks hold weak references to avoid a goal-handle
  // cycle.
  struct GoalExecution {
    std::mutex mutex;
    typename GoalHandle::SharedPtr goal_handle;
    bool cancel_requested{false};
    bool cancel_sent{false};
    bool completed{false};
    std::atomic_bool responded{false};
  };
  std::shared_ptr<GoalExecution> execution_;

  static void cancel_execution(const std::shared_ptr<GoalExecution> &execution,
                               const ActionClient &client) {
    if (!execution) {
      return;
    }
    typename GoalHandle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(execution->mutex);
      execution->cancel_requested = true;
      if (!client || execution->completed || execution->cancel_sent ||
          !execution->goal_handle) {
        return;
      }
      execution->cancel_sent = true;
      handle = execution->goal_handle;
    }
    try {
      client->async_cancel_goal(handle);
    } catch (const std::exception &error) {
      YASMIN_LOG_WARN("Failed to cancel action goal: %s", error.what());
    }
  }

  /// @brief Handler function for creating goals.
  CreateGoalHandler create_goal_handler;
  /// @brief Handler function for processing results.
  ResultHandler result_handler;
  /// @brief Handler function for processing feedback.
  FeedbackHandler feedback_handler;
  /// @brief Handler function for processing aborted results.
  ResultHandler abort_handler;
  /// @brief Maximum wait for the goal response; zero disables it.
  std::chrono::nanoseconds goal_response_timeout_{0};

  /// @brief Maximum time to wait for the action server.
  int wait_timeout;
  /// @brief Timeout for the action response.
  int response_timeout;
  /// @brief Maximum number of retries.
  int maximum_retry;

  /// @brief Guard shared with callbacks stored on the cached action client.
  std::shared_ptr<CallbackGuard> callback_guard =
      std::make_shared<CallbackGuard>();
  /// @brief Execution counter used to discard callbacks from previous runs.
  std::atomic<uint64_t> action_epoch_{0};

#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(2, 4, 3) // Greater or equal to latest Foxy
  using GoalResponseArg = typename GoalHandle::SharedPtr;
#else
  using GoalResponseArg = std::shared_future<typename GoalHandle::SharedPtr>;
#endif
#else
  using GoalResponseArg = std::shared_future<typename GoalHandle::SharedPtr>;
#endif

  static typename GoalHandle::SharedPtr
  resolve_goal_response(const typename GoalHandle::SharedPtr &handle) {
    return handle;
  }

  static typename GoalHandle::SharedPtr resolve_goal_response(
      std::shared_future<typename GoalHandle::SharedPtr> future) {
    return future.get();
  }

  /**
   * @brief Callback for handling the result of the action.
   *
   * This function is called when the action result is available.
   *
   * @param result The wrapped result of the action.
   */
  void result_callback(const typename GoalHandle::WrappedResult &result,
                       uint64_t epoch) {
    std::lock_guard<std::mutex> lock(this->action_done_mutex);
    if (this->action_epoch_.load() != epoch) {
      return;
    }
    this->action_result = result.result;
    this->action_status = result.code;
    this->action_done_.store(true);
    this->action_done_cond.notify_one();
  }
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__ACTION_STATE_HPP_
