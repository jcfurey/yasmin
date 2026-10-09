// Copyright (C) 2026
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
#include <csignal>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <unistd.h>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin/state_machine_cancel_exception.hpp"

using namespace yasmin;

namespace {

std::atomic_int previous_handler_calls{0};
std::atomic_int top_handler_calls{0};
struct sigaction top_handler_previous {};

extern "C" void counting_handler(int) { ++previous_handler_calls; }

extern "C" void top_handler(int signum, siginfo_t *info, void *context) {
  ++top_handler_calls;
  if (top_handler_previous.sa_flags & SA_SIGINFO) {
    top_handler_previous.sa_sigaction(signum, info, context);
  }
}

bool wait_for(const std::function<bool()> &condition,
              std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return condition();
}

bool sigint_handler_is(void (*handler)(int)) {
  struct sigaction current {};
  sigaction(SIGINT, nullptr, &current);
  return !(current.sa_flags & SA_SIGINFO) && current.sa_handler == handler;
}

// Blocks until released; optionally ignores cancellation to model a state
// that does not honor the cooperative contract.
class BlockingState : public State {
public:
  explicit BlockingState(bool honor_cancel)
      : State({"done"}), honor_cancel_(honor_cancel) {}

  std::string execute(Blackboard::SharedPtr) override {
    entered = true;
    while (!released && !(honor_cancel_ && this->is_canceled())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return "done";
  }

  void cancel_state() override {
    cancel_thread = std::this_thread::get_id();
    ++cancel_calls;
    State::cancel_state();
  }

  std::atomic_bool entered{false};
  std::atomic_bool released{false};
  std::atomic_int cancel_calls{0};
  std::thread::id cancel_thread;

private:
  bool honor_cancel_;
};

class TestSigintHandler : public ::testing::Test {
protected:
  void SetUp() override {
    previous_handler_calls = 0;
    top_handler_calls = 0;
    struct sigaction action {};
    action.sa_handler = counting_handler;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, &original_action_);
  }

  void TearDown() override { sigaction(SIGINT, &original_action_, nullptr); }

  static StateMachine::SharedPtr
  make_sm(const std::shared_ptr<BlockingState> &state) {
    auto sm = StateMachine::make_shared(Outcomes{"done"}, true);
    sm->add_state("BLOCK", state, {{"done", "done"}});
    return sm;
  }

  struct sigaction original_action_ {};
};

} // namespace

TEST_F(TestSigintHandler, FirstSigintCancelsOutsideSignalContext) {
  auto state = std::make_shared<BlockingState>(true);
  auto sm = make_sm(state);
  std::thread::id executor_thread;
  auto result = std::async(std::launch::async, [&]() {
    executor_thread = std::this_thread::get_id();
    return (*sm)();
  });
  ASSERT_TRUE(wait_for([&]() { return state->entered.load(); }));
  EXPECT_FALSE(sigint_handler_is(counting_handler));

  ASSERT_EQ(kill(getpid(), SIGINT), 0);

  EXPECT_THROW(result.get(), StateMachineCancelException);
  EXPECT_EQ(state->cancel_calls.load(), 1);
  // Cancellation ran on the dispatcher, not inside the signal handler of the
  // interrupted test or executor thread.
  EXPECT_NE(state->cancel_thread, executor_thread);
  EXPECT_NE(state->cancel_thread, std::this_thread::get_id());
  EXPECT_EQ(previous_handler_calls.load(), 0);
  EXPECT_TRUE(sigint_handler_is(counting_handler));
}

TEST_F(TestSigintHandler, RepeatedSigintEscalatesToPreviousHandler) {
  auto state = std::make_shared<BlockingState>(false);
  auto sm = make_sm(state);
  auto result = std::async(std::launch::async, [&]() { return (*sm)(); });
  ASSERT_TRUE(wait_for([&]() { return state->entered.load(); }));

  ASSERT_EQ(kill(getpid(), SIGINT), 0);
  ASSERT_TRUE(wait_for([&]() { return state->cancel_calls.load() == 1; }));
  EXPECT_EQ(previous_handler_calls.load(), 0);

  ASSERT_EQ(kill(getpid(), SIGINT), 0);
  ASSERT_TRUE(wait_for([&]() { return previous_handler_calls.load() == 1; }));
  EXPECT_EQ(state->cancel_calls.load(), 1);

  state->released = true;
  EXPECT_THROW(result.get(), StateMachineCancelException);
  EXPECT_TRUE(sigint_handler_is(counting_handler));
}

TEST_F(TestSigintHandler, SigtermCancelsAndEscalatesLikeSigint) {
  std::atomic_int previous_sigterm_calls{0};
  static std::atomic_int *sigterm_counter = nullptr;
  sigterm_counter = &previous_sigterm_calls;
  struct sigaction counting {};
  counting.sa_handler = [](int) { ++*sigterm_counter; };
  sigemptyset(&counting.sa_mask);
  struct sigaction original {};
  sigaction(SIGTERM, &counting, &original);

  auto state = std::make_shared<BlockingState>(false);
  auto sm = make_sm(state);
  auto result = std::async(std::launch::async, [&]() { return (*sm)(); });
  ASSERT_TRUE(wait_for([&]() { return state->entered.load(); }));

  ASSERT_EQ(kill(getpid(), SIGTERM), 0);
  ASSERT_TRUE(wait_for([&]() { return state->cancel_calls.load() == 1; }));
  EXPECT_EQ(previous_sigterm_calls.load(), 0);

  ASSERT_EQ(kill(getpid(), SIGTERM), 0);
  ASSERT_TRUE(wait_for([&]() { return previous_sigterm_calls.load() == 1; }));

  state->released = true;
  EXPECT_THROW(result.get(), StateMachineCancelException);
  struct sigaction current {};
  sigaction(SIGTERM, nullptr, &current);
  EXPECT_EQ(current.sa_handler, counting.sa_handler);
  sigaction(SIGTERM, &original, nullptr);
}

TEST_F(TestSigintHandler, LaterExecutionGetsFreshFirstSigint) {
  for (int run = 0; run < 2; ++run) {
    auto state = std::make_shared<BlockingState>(true);
    auto sm = make_sm(state);
    auto result = std::async(std::launch::async, [&]() { return (*sm)(); });
    ASSERT_TRUE(wait_for([&]() { return state->entered.load(); }));
    ASSERT_EQ(kill(getpid(), SIGINT), 0);
    EXPECT_THROW(result.get(), StateMachineCancelException);
    EXPECT_EQ(state->cancel_calls.load(), 1);
  }
  EXPECT_EQ(previous_handler_calls.load(), 0);
}

TEST_F(TestSigintHandler, DisabledHandlerLeavesSigintAlone) {
  auto state = std::make_shared<BlockingState>(true);
  auto sm = make_sm(state);
  sm->set_sigint_handler(false);
  auto result = std::async(std::launch::async, [&]() { return (*sm)(); });
  ASSERT_TRUE(wait_for([&]() { return state->entered.load(); }));
  EXPECT_TRUE(sigint_handler_is(counting_handler));

  state->released = true;
  EXPECT_EQ(result.get(), "done");
}

TEST_F(TestSigintHandler, HandlerInstalledLaterKeepsItsChain) {
  // Models rclcpp::init() installing its handler during execution: it chains
  // to the YASMIN handler, which must then pass signals through.
  auto state = std::make_shared<BlockingState>(true);
  auto sm = make_sm(state);
  auto result = std::async(std::launch::async, [&]() { return (*sm)(); });
  ASSERT_TRUE(wait_for([&]() { return state->entered.load(); }));

  struct sigaction top {};
  top.sa_sigaction = top_handler;
  top.sa_flags = SA_SIGINFO;
  sigemptyset(&top.sa_mask);
  sigaction(SIGINT, &top, &top_handler_previous);

  sm->cancel_state_machine();
  EXPECT_THROW(result.get(), StateMachineCancelException);
  const int direct_cancels = state->cancel_calls.load();

  struct sigaction current {};
  sigaction(SIGINT, nullptr, &current);
  EXPECT_EQ(current.sa_sigaction, top_handler);

  ASSERT_EQ(kill(getpid(), SIGINT), 0);
  ASSERT_TRUE(wait_for([&]() { return top_handler_calls.load() == 1; }));
  ASSERT_TRUE(wait_for([&]() { return previous_handler_calls.load() == 1; }));
  EXPECT_EQ(state->cancel_calls.load(), direct_cancels);

  // Unwind the top handler, then let a short run restore the original one.
  sigaction(SIGINT, &top_handler_previous, nullptr);
  auto quick = std::make_shared<BlockingState>(true);
  quick->released = true;
  EXPECT_EQ((*make_sm(quick))(), "done");
  EXPECT_TRUE(sigint_handler_is(counting_handler));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
