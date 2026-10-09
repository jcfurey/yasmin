// Copyright (C) 2026
// Licensed under the Apache License, Version 2.0.

#ifndef YASMIN_TEST_PARALLEL_UTILS_HPP_
#define YASMIN_TEST_PARALLEL_UTILS_HPP_

#include "yasmin/state.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

class CancellationWaitState : public yasmin::State {
public:
  CancellationWaitState() : State({"done"}) {}
  std::atomic<bool> entered{false};
  std::atomic<bool> finish_normally{false};
  std::atomic<bool> timed_out{false};
  std::string execute(yasmin::Blackboard::SharedPtr) override {
    std::unique_lock<std::mutex> lock(mutex);
    entered.store(true);
    timed_out.store(!changed.wait_for(lock, std::chrono::seconds(2), [this] {
      return is_canceled() || finish_normally.load();
    }));
    return "done";
  }
  void cancel_state() override {
    std::lock_guard<std::mutex> lock(mutex);
    State::cancel_state();
    changed.notify_all();
  }

private:
  std::mutex mutex;
  std::condition_variable changed;
};

class ParallelFailureState : public yasmin::State {
public:
  explicit ParallelFailureState(
      std::shared_ptr<CancellationWaitState> sibling = nullptr)
      : State({"done"}), sibling(std::move(sibling)) {}
  std::string execute(yasmin::Blackboard::SharedPtr) override {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (sibling && !sibling->entered.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("original worker failure");
  }

private:
  std::shared_ptr<CancellationWaitState> sibling;
};

#endif
