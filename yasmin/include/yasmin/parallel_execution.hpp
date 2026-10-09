// Copyright (C) 2026
// Licensed under the Apache License, Version 2.0.

#ifndef YASMIN__PARALLEL_EXECUTION_HPP_
#define YASMIN__PARALLEL_EXECUTION_HPP_

#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace yasmin::detail {

// Cancel cooperatively until every launched worker has stopped. Repeating the
// request also covers workers that start after the first cancellation request.
// The caller must release the Python GIL before entering this function.
template <class Work, class Cancel, class IsCanceled>
void run_parallel(std::size_t count, Work work, Cancel cancel,
                  IsCanceled is_canceled) {
  std::mutex mutex;
  std::condition_variable changed;
  std::size_t completed = 0;
  std::exception_ptr first_exception;
  std::vector<std::thread> workers;
  workers.reserve(count);
  try {
    for (std::size_t i = 0; i < count; ++i) {
      workers.emplace_back([&, i]() {
        try {
          work(i);
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex);
          if (!first_exception) {
            first_exception = std::current_exception();
          }
        }
        {
          std::lock_guard<std::mutex> lock(mutex);
          ++completed;
        }
        changed.notify_one();
      });
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!first_exception) {
      first_exception = std::current_exception();
    }
  }

  std::unique_lock<std::mutex> lock(mutex);
  while (completed != workers.size()) {
    const bool stopping = first_exception || is_canceled();
    lock.unlock();
    if (stopping) {
      try {
        cancel();
      } catch (...) {
        std::lock_guard<std::mutex> guard(mutex);
        if (!first_exception) {
          first_exception = std::current_exception();
        }
      }
    }
    lock.lock();
    if (completed != workers.size()) {
      changed.wait_for(lock, std::chrono::milliseconds(10));
    }
  }
  lock.unlock();
  for (auto &worker : workers) {
    worker.join();
  }
  if (first_exception) {
    std::rethrow_exception(first_exception);
  }
}

} // namespace yasmin::detail

#endif // YASMIN__PARALLEL_EXECUTION_HPP_
