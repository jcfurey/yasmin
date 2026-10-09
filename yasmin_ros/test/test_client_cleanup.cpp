// Copyright (C) 2026 YASMIN contributors
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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <std_msgs/msg/string.hpp>

#include "yasmin_ros/action_state.hpp"
#include "yasmin_ros/monitor_state.hpp"
#include "yasmin_ros/service_state.hpp"

using namespace std::chrono_literals;
using namespace yasmin_ros;
using namespace yasmin_ros::basic_outcomes;

namespace {

// Foxy's service client has no public cleanup API. Keep the common behavioral
// tests available there and enable request-count assertions where supported.
template <typename Client, typename = void>
struct HasPendingCleanup : std::false_type {};
template <typename Client>
struct HasPendingCleanup<
    Client,
    std::void_t<decltype(std::declval<Client &>().prune_pending_requests())>>
    : std::true_type {};

template <typename Client>
void expect_no_pending_requests(const std::shared_ptr<Client> &client) {
  if constexpr (HasPendingCleanup<Client>::value) {
    EXPECT_EQ(client->prune_pending_requests(), 0u);
  }
}

template <typename Client, typename Pending>
void expect_request_preserved(const std::shared_ptr<Client> &client,
                              const Pending &pending) {
  if constexpr (HasPendingCleanup<Client>::value) {
    EXPECT_TRUE(client->remove_pending_request(pending));
    EXPECT_EQ(client->prune_pending_requests(), 0u);
  }
}

class TestClientCleanup : public ::testing::Test {
protected:
  using Action = example_interfaces::action::Fibonacci;
  using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;
  using ActionStateT = ActionState<Action>;
  using Service = example_interfaces::srv::AddTwoInts;
  using ServiceStateT = ServiceState<Service>;

  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    static unsigned int next_id = 0;
    const auto suffix = std::to_string(++next_id);
    server_node = std::make_shared<rclcpp::Node>("cleanup_server_" + suffix);
    client_node = std::make_shared<rclcpp::Node>("cleanup_client_" + suffix);
    action_name = "/cleanup_action_" + suffix;
    service_name = "/cleanup_service_" + suffix;
    action_server = rclcpp_action::create_server<Action>(
        server_node, action_name,
        [](const rclcpp_action::GoalUUID &,
           std::shared_ptr<const Action::Goal> goal) {
          return goal->order < 0
                     ? rclcpp_action::GoalResponse::REJECT
                     : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<ServerGoal> goal) {
          std::lock_guard<std::mutex> lock(goals_mutex);
          canceled_orders.insert(goal->get_goal()->order);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<ServerGoal> goal) {
          std::lock_guard<std::mutex> lock(goals_mutex);
          goals.emplace(goal->get_goal()->order, goal);
        });
    feedback_timer = server_node->create_wall_timer(10ms, [this]() {
      std::lock_guard<std::mutex> lock(goals_mutex);
      for (const auto &entry : goals) {
        if (entry.second->is_active() && !entry.second->is_canceling()) {
          entry.second->publish_feedback(std::make_shared<Action::Feedback>());
        }
      }
    });
    service_server = server_node->create_service<Service>(
        service_name, [this](Service::Request::SharedPtr request,
                             Service::Response::SharedPtr response) {
          response->sum = request->a + request->b;
          ++service_requests;
        });
    server_executor.add_node(server_node);
    client_executor.add_node(client_node);
    server_thread = std::thread([this]() { server_executor.spin(); });
  }

  void TearDown() override {
    server_executor.cancel();
    server_thread.join();
    for (const auto &entry : goals) {
      if (entry.second->is_canceling()) {
        entry.second->canceled(std::make_shared<Action::Result>());
      } else if (entry.second->is_active()) {
        entry.second->abort(std::make_shared<Action::Result>());
      }
    }
    client_executor.remove_node(client_node);
    server_executor.remove_node(server_node);
    ROSClientsCache::clear_for_node(client_node);
  }

  template <typename Predicate>
  bool wait_until(Predicate predicate, bool spin_client = true,
                  std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      if (spin_client) {
        client_executor.spin_some();
      }
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }

  bool accepted(int order) {
    std::lock_guard<std::mutex> lock(goals_mutex);
    return goals.count(order) != 0;
  }

  bool canceled(int order) {
    std::lock_guard<std::mutex> lock(goals_mutex);
    return canceled_orders.count(order) != 0 && goals.at(order)->is_canceling();
  }

  void finish_goal(int order) {
    std::lock_guard<std::mutex> lock(goals_mutex);
    auto goal = goals.at(order);
    auto result = std::make_shared<Action::Result>();
    result->sequence = {order};
    if (goal->is_canceling()) {
      goal->canceled(result);
    } else {
      goal->succeed(result);
    }
  }

  static Action::Goal goal_for(int order) {
    Action::Goal goal;
    goal.order = order;
    return goal;
  }

  std::shared_ptr<ActionStateT> make_action(int order) {
    return std::make_shared<ActionStateT>(
        client_node, action_name,
        [order](yasmin::Blackboard::SharedPtr) { return goal_for(order); },
        yasmin::Outcomes{}, nullptr, nullptr, nullptr, 1, 1, 0);
  }

  static Service::Request::SharedPtr
  make_request(yasmin::Blackboard::SharedPtr) {
    auto request = std::make_shared<Service::Request>();
    request->a = 2;
    request->b = 3;
    return request;
  }

  std::shared_ptr<ServiceStateT> make_service() {
    return std::make_shared<ServiceStateT>(client_node, service_name,
                                           make_request, yasmin::Outcomes{},
                                           nullptr, nullptr, 1, 1, 0);
  }

  template <typename StateT>
  static std::future<std::string> run(const std::shared_ptr<StateT> &state) {
    return std::async(std::launch::async, [state]() {
      return (*state)(yasmin::Blackboard::make_shared());
    });
  }

  static bool ready(std::future<std::string> &future) {
    return future.wait_for(0ms) == std::future_status::ready;
  }

  rclcpp::Node::SharedPtr server_node, client_node;
  rclcpp::executors::SingleThreadedExecutor server_executor, client_executor;
  std::thread server_thread;
  rclcpp_action::Server<Action>::SharedPtr action_server;
  rclcpp::Service<Service>::SharedPtr service_server;
  rclcpp::TimerBase::SharedPtr feedback_timer;
  std::string action_name, service_name;
  std::mutex goals_mutex;
  std::map<int, std::shared_ptr<ServerGoal>> goals;
  std::set<int> canceled_orders;
  std::atomic<unsigned int> service_requests{0};
};

TEST_F(TestClientCleanup, ActionTimeoutCancelsAcceptedGoal) {
  std::atomic<bool> received_feedback{false};
  auto state = std::make_shared<ActionStateT>(
      client_node, action_name,
      [](yasmin::Blackboard::SharedPtr) { return goal_for(1); },
      yasmin::Outcomes{}, nullptr,
      [&received_feedback](yasmin::Blackboard::SharedPtr,
                           std::shared_ptr<const Action::Feedback>) {
        received_feedback = true;
      },
      nullptr, 1, 1, 0);
  auto future = run(state);
  ASSERT_TRUE(wait_until([&]() { return received_feedback.load(); }));
  ASSERT_TRUE(wait_until([&]() { return ready(future); }, false));
  EXPECT_EQ(future.get(), TIMEOUT);
  EXPECT_TRUE(wait_until([&]() { return canceled(1); }, false));
}

TEST_F(TestClientCleanup,
       AcceptedActionCancellationDoesNotWaitForAcknowledgement) {
  std::atomic<bool> received_feedback{false};
  auto state = std::make_shared<ActionStateT>(
      client_node, action_name,
      [](yasmin::Blackboard::SharedPtr) { return goal_for(1); },
      yasmin::Outcomes{}, nullptr,
      [&received_feedback](yasmin::Blackboard::SharedPtr,
                           std::shared_ptr<const Action::Feedback>) {
        received_feedback = true;
      },
      nullptr, 1, 1, 0);
  auto future = run(state);
  ASSERT_TRUE(wait_until([&]() { return received_feedback.load(); }));
  // Stop spinning the client so its cancel acknowledgement cannot be delivered.
  const auto start = std::chrono::steady_clock::now();
  state->cancel_state();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
  EXPECT_EQ(future.wait_for(250ms), std::future_status::ready);
  EXPECT_EQ(future.get(), CANCEL);
  EXPECT_TRUE(wait_until([&]() { return canceled(1); }, false));
}

TEST_F(TestClientCleanup, ActionTimeoutCancelsLateAcceptance) {
  auto state = make_action(1);
  auto future = run(state);
  ASSERT_TRUE(wait_until([&]() { return accepted(1); }, false));
  ASSERT_TRUE(wait_until([&]() { return ready(future); }, false));
  EXPECT_EQ(future.get(), TIMEOUT);
  EXPECT_TRUE(wait_until([&]() { return canceled(1); }));
}

TEST_F(TestClientCleanup,
       LateAcceptanceAndOldResultCannotCompleteReusedAction) {
  int next_order = 0;
  auto state = std::make_shared<ActionStateT>(
      client_node, action_name,
      [&next_order](yasmin::Blackboard::SharedPtr) {
        return goal_for(++next_order);
      },
      yasmin::Outcomes{},
      [](yasmin::Blackboard::SharedPtr blackboard,
         Action::Result::SharedPtr result) {
        blackboard->set<int>("order", result->sequence.at(0));
        return SUCCEED;
      },
      nullptr, nullptr, 1, 1, 0);
  auto first = run(state);
  ASSERT_TRUE(wait_until([&]() { return accepted(1); }, false));
  state->cancel_state();
  EXPECT_EQ(first.wait_for(250ms), std::future_status::ready);
  EXPECT_EQ(first.get(), CANCEL);

  auto blackboard = yasmin::Blackboard::make_shared();
  auto second = std::async(std::launch::async, [state, blackboard]() {
    return (*state)(blackboard);
  });
  ASSERT_TRUE(wait_until([&]() { return accepted(2); }, false));
  ASSERT_TRUE(wait_until([&]() { return canceled(1); }));
  EXPECT_FALSE(canceled(2));
  finish_goal(1);
  // Drain the old result while the new goal is still running.
  wait_until([&]() { return ready(second); }, true, 50ms);
  EXPECT_FALSE(ready(second));
  finish_goal(2);
  ASSERT_TRUE(wait_until([&]() { return ready(second); }));
  EXPECT_EQ(second.get(), SUCCEED);
  EXPECT_EQ(blackboard->get<int>("order"), 2);
}

TEST_F(TestClientCleanup, ReuseWaitsForInFlightFeedbackHandler) {
  std::promise<void> release_feedback;
  auto released = release_feedback.get_future().share();
  std::atomic<bool> feedback_entered{false}, feedback_finished{false};
  std::atomic<int> next_order{0};
  auto state = std::make_shared<ActionStateT>(
      client_node, action_name,
      [&](yasmin::Blackboard::SharedPtr) {
        const int order = ++next_order;
        if (order == 2) {
          EXPECT_TRUE(feedback_finished.load());
        }
        return goal_for(order);
      },
      yasmin::Outcomes{}, nullptr,
      [&](yasmin::Blackboard::SharedPtr,
          std::shared_ptr<const Action::Feedback>) {
        if (!feedback_entered.exchange(true)) {
          released.wait();
          feedback_finished = true;
        }
      },
      nullptr, 1, 1, 0);
  auto first = run(state);
  auto callback = std::async(std::launch::async, [&]() {
    return wait_until([&]() { return feedback_entered.load(); });
  });
  auto cancellation = std::async(std::launch::async, [&]() {
    wait_until([&]() { return feedback_entered.load(); }, false);
    state->cancel_state();
  });
  // Release the handler before joining either callback/cancellation worker on
  // every exit. rclcpp may hold its client mutex while invoking feedback.
  struct FeedbackRelease {
    std::promise<void> &promise;
    bool opened{false};
    void open() {
      if (!opened) {
        opened = true;
        promise.set_value();
      }
    }
    ~FeedbackRelease() { open(); }
  } release{release_feedback};

  ASSERT_TRUE(wait_until([&]() { return feedback_entered.load(); }, false));
  ASSERT_EQ(first.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(first.get(), CANCEL);
  auto second = run(state);
  EXPECT_TRUE(wait_until([&]() { return state->is_running(); }, false));
  EXPECT_FALSE(wait_until([&]() { return next_order == 2; }, false, 50ms));
  release.open();
  EXPECT_TRUE(callback.get());
  cancellation.get();
  ASSERT_TRUE(wait_until([&]() { return accepted(2); }));
  finish_goal(2);
  ASSERT_TRUE(wait_until([&]() { return ready(second); }));
  EXPECT_EQ(second.get(), SUCCEED);
}

TEST_F(TestClientCleanup, DestroyedActionStillCancelsLateAcceptance) {
  auto state = make_action(1);
  auto future = run(state);
  ASSERT_TRUE(wait_until([&]() { return accepted(1); }, false));
  state->cancel_state();
  EXPECT_EQ(future.get(), CANCEL);
  state.reset();
  EXPECT_TRUE(wait_until([&]() { return canceled(1); }));
}

TEST_F(TestClientCleanup, RejectedActionWakesAndCanBeReused) {
  auto state = make_action(-1);
  for (int i = 0; i < 20; ++i) {
    auto future = run(state);
    ASSERT_TRUE(wait_until([&]() { return ready(future); }));
    EXPECT_EQ(future.get(), ABORT);
  }
}

TEST_F(TestClientCleanup, ActionGoalHandlerCanCancelBeforeSending) {
  std::shared_ptr<ActionStateT> state;
  state = std::make_shared<ActionStateT>(
      client_node, action_name,
      [&state](yasmin::Blackboard::SharedPtr) {
        state->cancel_state();
        return goal_for(1);
      },
      yasmin::Outcomes{}, nullptr, nullptr, nullptr, 1, 1, 0);
  auto future = run(state);
  EXPECT_EQ(future.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(future.get(), CANCEL);
  EXPECT_FALSE(accepted(1));
}

TEST_F(TestClientCleanup, ActionResultHandlerCanCancel) {
  std::shared_ptr<ActionStateT> state;
  state = std::make_shared<ActionStateT>(
      client_node, action_name,
      [](yasmin::Blackboard::SharedPtr) { return goal_for(1); },
      yasmin::Outcomes{},
      [&state](yasmin::Blackboard::SharedPtr,
               Action::Result::SharedPtr result) {
        EXPECT_EQ(result->sequence.at(0), 1);
        state->cancel_state();
        return CANCEL;
      },
      nullptr, nullptr, 1, 1, 0);
  auto future = run(state);
  ASSERT_TRUE(wait_until([&]() { return accepted(1); }));
  finish_goal(1);
  ASSERT_TRUE(wait_until([&]() { return ready(future); }));
  EXPECT_EQ(future.get(), CANCEL);
}

TEST_F(TestClientCleanup, ServiceTimeoutRemovesOnlyItsPendingRequest) {
  auto client = ROSClientsCache::get_or_create_service_client<Service>(
      client_node, service_name, nullptr);
  ASSERT_TRUE(client->wait_for_service(2s));
  auto unrelated = client->async_send_request(make_request(nullptr));
  auto state = make_service();
  auto future = run(state);
  ASSERT_TRUE(wait_until([&]() { return ready(future); }, false));
  EXPECT_EQ(future.get(), TIMEOUT);
  expect_request_preserved(client, unrelated);
}

TEST_F(TestClientCleanup,
       ServiceCancellationRemovesPendingRequestsAcrossReuse) {
  auto state = make_service();
  auto client = ROSClientsCache::get_or_create_service_client<Service>(
      client_node, service_name, nullptr);
  for (unsigned int i = 0; i < 20; ++i) {
    auto future = run(state);
    ASSERT_TRUE(wait_until([&]() { return service_requests > i; }, false));
    state->cancel_state();
    EXPECT_EQ(future.wait_for(250ms), std::future_status::ready);
    EXPECT_EQ(future.get(), CANCEL);
    expect_no_pending_requests(client);
  }
}

TEST_F(TestClientCleanup, ServiceFastResponsesWakeAcrossReuse) {
  auto state = make_service();
  for (int i = 0; i < 20; ++i) {
    auto future = run(state);
    ASSERT_TRUE(wait_until([&]() { return ready(future); }));
    EXPECT_EQ(future.get(), SUCCEED);
  }
}

TEST_F(TestClientCleanup, ServiceRequestHandlerCanCancelBeforeSending) {
  std::shared_ptr<ServiceStateT> state;
  state = std::make_shared<ServiceStateT>(
      client_node, service_name,
      [&state](yasmin::Blackboard::SharedPtr blackboard) {
        state->cancel_state();
        return make_request(blackboard);
      },
      yasmin::Outcomes{}, nullptr, nullptr, 1, 1, 0);
  auto future = run(state);
  EXPECT_EQ(future.wait_for(500ms), std::future_status::ready);
  EXPECT_EQ(future.get(), CANCEL);
  EXPECT_EQ(service_requests.load(), 0u);
}

TEST_F(TestClientCleanup, ServiceResponseHandlerCanCancel) {
  std::shared_ptr<ServiceStateT> state;
  state = std::make_shared<ServiceStateT>(
      client_node, service_name, make_request, yasmin::Outcomes{},
      [&state](yasmin::Blackboard::SharedPtr,
               Service::Response::SharedPtr response) {
        EXPECT_EQ(response->sum, 5);
        state->cancel_state();
        return CANCEL;
      },
      nullptr, 1, 1, 0);
  auto future = run(state);
  ASSERT_TRUE(wait_until([&]() { return ready(future); }));
  EXPECT_EQ(future.get(), CANCEL);
}

TEST_F(TestClientCleanup, CancellationInterruptsEndpointDiscovery) {
  auto action = std::make_shared<ActionStateT>(
      client_node, action_name + "_missing",
      [](yasmin::Blackboard::SharedPtr) { return goal_for(1); },
      yasmin::Outcomes{}, nullptr, nullptr, nullptr, 1, 1, 0);
  auto service = std::make_shared<ServiceStateT>(
      client_node, service_name + "_missing", make_request, yasmin::Outcomes{},
      nullptr, nullptr, 1, 1, 0);
  for (const auto &state : {std::static_pointer_cast<yasmin::State>(action),
                            std::static_pointer_cast<yasmin::State>(service)}) {
    auto future = run(state);
    ASSERT_TRUE(wait_until([&]() { return state->is_running(); }, false));
    const auto start = std::chrono::steady_clock::now();
    state->cancel_state();
    EXPECT_EQ(future.wait_for(250ms), std::future_status::ready);
    EXPECT_EQ(future.get(), CANCEL);
    EXPECT_LT(std::chrono::steady_clock::now() - start, 300ms);
  }
}

TEST_F(TestClientCleanup, PreCanceledMonitorSkipsTimedWait) {
  auto state = std::make_shared<MonitorState<std_msgs::msg::String>>(
      client_node, "/cleanup_monitor", yasmin::Outcomes{SUCCEED},
      [](yasmin::Blackboard::SharedPtr, std_msgs::msg::String::SharedPtr) {
        return SUCCEED;
      },
      rclcpp::QoS(10), nullptr, 10, 1, 0);
  state->cancel_state();
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(state->execute(yasmin::Blackboard::make_shared()), CANCEL);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
}

TEST_F(TestClientCleanup, MonitorCancellationWakesAcrossReuse) {
  auto state = std::make_shared<MonitorState<std_msgs::msg::String>>(
      client_node, "/cleanup_monitor", yasmin::Outcomes{SUCCEED},
      [](yasmin::Blackboard::SharedPtr, std_msgs::msg::String::SharedPtr) {
        return SUCCEED;
      },
      rclcpp::QoS(10), nullptr, 10, 1, 0);
  for (int i = 0; i < 20; ++i) {
    auto future = run(state);
    ASSERT_TRUE(wait_until([&]() { return state->is_running(); }, false));
    state->cancel_state();
    EXPECT_EQ(future.wait_for(250ms), std::future_status::ready);
    EXPECT_EQ(future.get(), CANCEL);
  }
}

} // namespace
