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

#include "yasmin/state_machine.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <exception>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#include "yasmin/blackboard.hpp"
#include "yasmin/concurrence.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/orthogonal_state.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine_cancel_exception.hpp"
#include "yasmin/state_utils.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

namespace {

// Termination signals (SIGINT and SIGTERM, which rclcpp also treats alike)
// are handled in two parts. The handler only performs async-signal-safe work
// (atomics and a pipe write); a dispatcher thread runs the registered
// cancellation callbacks in an ordinary thread context, where they may lock
// mutexes, log, or acquire the Python GIL.
//
// The first signal while a state machine is registered cancels it and keeps
// the ROS context alive, so remote goals can still be canceled. A repeated
// signal before the registrations end is forwarded to that signal's
// previously installed handler (rclcpp, rclpy or the default action), so a
// cancellation that does not finish can still be escalated.
constexpr int kHandledSignals[] = {SIGINT, SIGTERM};
constexpr std::size_t kSignalCount = 2;

std::size_t signal_index(int signum) { return signum == SIGTERM ? 1 : 0; }

struct SigintRegistry {
  std::mutex mutex;
  std::unordered_map<int, std::function<void()>> callbacks;
  int next_id = 0;
  bool handler_installed[kSignalCount] = {false, false};
  bool dispatcher_started = false;
  unsigned char generation = 0;
};

// Intentionally leaked: the detached dispatcher must never observe a
// registry destroyed during static destruction.
SigintRegistry &sigint_registry() {
  static auto *registry = new SigintRegistry();
  return *registry;
}

// State read by the signal handler: lock-free atomics, plus the previous
// actions, each only written while this handler is not installed for it.
std::atomic<int> sigint_pipe_write_fd{-1};
std::atomic<bool> sigint_active{false};
std::atomic<bool> sigint_pending{false};
std::atomic<unsigned char> sigint_generation{0};
struct sigaction previous_actions[kSignalCount]{};

static_assert(std::atomic<int>::is_always_lock_free &&
                  std::atomic<bool>::is_always_lock_free &&
                  std::atomic<unsigned char>::is_always_lock_free,
              "SIGINT handler state must be lock-free");

void forward_sigint(int signum, siginfo_t *info, void *context,
                    bool allow_default_action) {
  const struct sigaction &previous = previous_actions[signal_index(signum)];
  if (previous.sa_flags & SA_SIGINFO) {
    if (previous.sa_sigaction != nullptr) {
      previous.sa_sigaction(signum, info, context);
    }
  } else if (previous.sa_handler == SIG_DFL) {
    if (allow_default_action) {
      // The signal is blocked while this handler runs, so the default
      // action is taken as soon as the handler returns.
      signal(signum, SIG_DFL);
      raise(signum);
    }
  } else if (previous.sa_handler != SIG_IGN && previous.sa_handler != nullptr) {
    previous.sa_handler(signum);
  }
}

extern "C" void sigint_handler(int signum, siginfo_t *info, void *context) {
  const int saved_errno = errno;
  if (!sigint_active.load()) {
    // Another handler installed on top of this one still chains here.
    forward_sigint(signum, info, context, false);
  } else if (sigint_pending.exchange(true)) {
    forward_sigint(signum, info, context, true);
  } else {
    const int fd = sigint_pipe_write_fd.load();
    if (fd >= 0) {
      const auto wake = static_cast<char>(sigint_generation.load());
      // A full pipe already holds a pending wake-up.
      [[maybe_unused]] const auto written = write(fd, &wake, 1);
    }
  }
  errno = saved_errno;
}

void sigint_dispatch_loop(int read_fd) {
  // Let other threads receive the signals while callbacks run here.
  sigset_t mask;
  sigemptyset(&mask);
  for (const int signum : kHandledSignals) {
    sigaddset(&mask, signum);
  }
  pthread_sigmask(SIG_BLOCK, &mask, nullptr);

  auto &registry = sigint_registry();
  while (true) {
    char wake = 0;
    const auto count = read(read_fd, &wake, 1);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      return;
    }

    // Callbacks run under the registry lock, so unregistering waits for an
    // in-flight cancellation and the captured state machine stays alive.
    std::lock_guard<std::mutex> lock(registry.mutex);
    if (static_cast<unsigned char>(wake) != registry.generation) {
      continue; // wake-up from registrations that have since ended
    }
    for (const auto &[id, callback] : registry.callbacks) {
      (void)id;
      try {
        callback();
      } catch (const std::exception &error) {
        YASMIN_LOG_ERROR("SIGINT cancellation failed: %s", error.what());
      } catch (...) {
        YASMIN_LOG_ERROR("SIGINT cancellation failed with an unknown error");
      }
    }
  }
}

void start_sigint_dispatcher(SigintRegistry &registry) {
  int fds[2];
  if (pipe(fds) != 0) {
    throw std::system_error(errno, std::generic_category(),
                            "Failed to create the SIGINT pipe");
  }
  for (const int fd : fds) {
    fcntl(fd, F_SETFD, FD_CLOEXEC);
  }
  fcntl(fds[1], F_SETFL, fcntl(fds[1], F_GETFL) | O_NONBLOCK);

  try {
    std::thread(sigint_dispatch_loop, fds[0]).detach();
  } catch (...) {
    close(fds[0]);
    close(fds[1]);
    throw;
  }
  sigint_pipe_write_fd.store(fds[1]);
  registry.dispatcher_started = true;
}

int register_sigint_callback(std::function<void()> cb) {
  auto &registry = sigint_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  if (!registry.dispatcher_started) {
    start_sigint_dispatcher(registry);
  }
  for (std::size_t i = 0; i < kSignalCount; ++i) {
    if (registry.handler_installed[i]) {
      continue;
    }
    struct sigaction action {};
    action.sa_sigaction = sigint_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO;
    if (sigaction(kHandledSignals[i], &action, &previous_actions[i]) != 0) {
      throw std::system_error(errno, std::generic_category(),
                              "Failed to install a termination signal handler");
    }
    registry.handler_installed[i] = true;
  }
  if (registry.callbacks.empty()) {
    sigint_generation.store(++registry.generation);
    sigint_pending.store(false);
    sigint_active.store(true);
  }
  int id = registry.next_id++;
  registry.callbacks[id] = std::move(cb);
  return id;
}

void unregister_sigint_callback(int id) {
  auto &registry = sigint_registry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.callbacks.erase(id);
  if (!registry.callbacks.empty()) {
    return;
  }
  sigint_active.store(false);

  // Restore a previous action only if this handler is still the installed
  // one. Otherwise a handler installed later chains here and would lose its
  // own predecessor; stay in place as a pass-through instead.
  for (std::size_t i = 0; i < kSignalCount; ++i) {
    struct sigaction current {};
    if (registry.handler_installed[i] &&
        sigaction(kHandledSignals[i], nullptr, &current) == 0 &&
        (current.sa_flags & SA_SIGINFO) &&
        current.sa_sigaction == sigint_handler) {
      sigaction(kHandledSignals[i], &previous_actions[i], nullptr);
      registry.handler_installed[i] = false;
    }
  }
}
} // namespace

StateMachine::StateMachine(const Outcomes &outcomes, bool handle_sigint)
    : StateMachine("", outcomes, handle_sigint) {}

StateMachine::StateMachine(const std::string &name, const Outcomes &outcomes,
                           bool handle_sigint)
    : State(outcomes), name(name) {
  this->set_sigint_handler(handle_sigint);
}

StateMachine::~StateMachine() {
  this->states.clear();
  this->transitions.clear();
  this->remappings.clear();
  this->parameter_mappings.clear();
}

void StateMachine::add_state(const std::string &name, State::SharedPtr state,
                             const Transitions &transitions,
                             const Remappings &remappings,
                             const ParameterMappings &parameter_mappings) {

  if (!state) {
    throw std::invalid_argument("State '" + name + "' cannot be null");
  }

  if (this->states.find(name) != this->states.end()) {
    throw std::logic_error("State '" + name +
                           "' already registered in the state machine");
  }

  if (this->outcomes.find(name) != this->outcomes.end()) {
    throw std::logic_error("State name '" + name +
                           "' is already registered as an outcome");
  }

  for (auto it = transitions.begin(); it != transitions.end(); ++it) {
    const std::string &key = it->first;
    const std::string &value = it->second;

    if (key.empty()) {
      throw std::invalid_argument("Transitions with empty source in state '" +
                                  name + "'");
    }

    if (value.empty()) {
      throw std::invalid_argument("Transitions with empty target in state '" +
                                  name + "'");
    }

    const auto &state_outcomes = state->get_outcomes();
    if (state_outcomes.find(key) == state_outcomes.end()) {
      std::ostringstream oss;
      oss << "State '" << name << "' references unregistered outcomes '" << key
          << "', available outcomes are ["
          << yasmin::join(state_outcomes, ", ",
                          [](const std::string &o) { return "'" + o + "'"; })
          << "]";
      throw std::invalid_argument(oss.str());
    }
  }

  std::ostringstream transitions_oss;

  for (auto const &t : transitions) {
    transitions_oss << "\n\t" << t.first << " --> " << t.second;
  }

  YASMIN_LOG_DEBUG("Adding state '%s' of type '%s' with transitions: %s",
                   name.c_str(), state->to_string().c_str(),
                   transitions_oss.str().c_str());

  this->states.insert({name, std::move(state)});
  this->transitions.insert({name, transitions});
  this->remappings.insert({name, remappings});
  this->parameter_mappings.insert({name, parameter_mappings});

  if (this->start_state.empty()) {
    this->set_start_state(name);
  }

  // Mark state machine as no validated
  this->validated.store(false);
  this->configured.store(false);
}

void StateMachine::set_start_state(const std::string &state_name) {

  if (state_name.empty()) {
    throw std::invalid_argument("Initial state cannot be empty");

  } else if (this->states.find(state_name) == this->states.end()) {
    throw std::invalid_argument("Initial state '" + state_name +
                                "' is not in the state machine");
  }

  YASMIN_LOG_DEBUG("Setting start state to '%s'", state_name.c_str());

  this->start_state = state_name;

  // Mark state machine as no validated
  this->validated.store(false);
  this->configured.store(false);
}

void StateMachine::set_parameter_mappings(
    const std::string &state_name,
    const ParameterMappings &parameter_mappings) {

  if (this->states.find(state_name) == this->states.end()) {
    throw std::invalid_argument("State '" + state_name +
                                "' is not in the state machine");
  }

  this->parameter_mappings[state_name] = parameter_mappings;
  this->configured.store(false);
}

const ParameterMappingsMap &
StateMachine::get_parameter_mappings() const noexcept {
  return this->parameter_mappings;
}

void StateMachine::apply_parameter_mappings(const std::string &state_name,
                                            const State::SharedPtr &state) {
  yasmin::apply_parameter_mappings("State machine", this->parameter_mappings,
                                   state_name, *this, state);
}

void StateMachine::configure() {
  if (yasmin::check_already_configured(this->configured, "State machine",
                                       this->to_string().c_str()))
    return;

  for (const auto &[state_name, state] : this->states) {
    this->apply_parameter_mappings(state_name, state);
    state->configure();
  }

  this->configured.store(true);
}

std::string StateMachine::wait_for_current_state() {
  std::unique_lock<std::mutex> lock(this->current_state_mutex);

  this->current_state_cond.wait(lock, [this]() {
    return !this->current_state.empty() || !this->execution_active.load() ||
           this->cancel_state_machine_requested.load();
  });

  return this->current_state;
}

void StateMachine::throw_if_cancel_state_machine_requested() {
  if (!this->cancel_state_machine_requested.load()) {
    return;
  }

  this->execution_active.store(false);
  this->set_current_state("");
  this->current_state_cond.notify_all();
  throw StateMachineCancelException(this->to_string());
}

std::string const &StateMachine::get_start_state() const noexcept {
  return this->start_state;
}

StateMap const &StateMachine::get_states() const noexcept {
  return this->states;
}

TransitionsMap const &StateMachine::get_transitions() const noexcept {
  return this->transitions;
}

std::string StateMachine::get_current_state() const {
  const std::lock_guard<std::mutex> lock(this->current_state_mutex);
  return this->current_state;
}

void StateMachine::set_current_state(const std::string &state_name) {
  const std::lock_guard<std::mutex> lock(this->current_state_mutex);
  this->current_state = state_name;
  this->current_state_cond.notify_all();
}

void StateMachine::add_start_cb(StartCallbackType cb) {
  const std::lock_guard<std::mutex> lock(this->cbs_mutex_);
  this->start_cbs.emplace_back(std::move(cb));
}

void StateMachine::add_transition_cb(TransitionCallbackType cb) {
  const std::lock_guard<std::mutex> lock(this->cbs_mutex_);
  this->transition_cbs.emplace_back(std::move(cb));
}

void StateMachine::add_end_cb(EndCallbackType cb) {
  const std::lock_guard<std::mutex> lock(this->cbs_mutex_);
  this->end_cbs.emplace_back(std::move(cb));
}

void StateMachine::call_start_cbs(Blackboard::SharedPtr blackboard,
                                  const std::string &start_state) {

  std::vector<StartCallbackType> callbacks;
  {
    const std::lock_guard<std::mutex> lock(this->cbs_mutex_);
    callbacks = this->start_cbs;
  }

  try {
    for (const auto &callback : callbacks) {
      callback(blackboard, start_state);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute start callback: %s",
                     std::string(e.what()).c_str());
  }
}

void StateMachine::call_transition_cbs(Blackboard::SharedPtr blackboard,
                                       const std::string &from_state,
                                       const std::string &to_state,
                                       const std::string &outcome) {

  std::vector<TransitionCallbackType> callbacks;
  {
    const std::lock_guard<std::mutex> lock(this->cbs_mutex_);
    callbacks = this->transition_cbs;
  }

  try {
    for (const auto &callback : callbacks) {
      callback(blackboard, from_state, to_state, outcome);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute transition callback: %s",
                     std::string(e.what()).c_str());
  }
}

void StateMachine::call_end_cbs(Blackboard::SharedPtr blackboard,
                                const std::string &outcome) {

  std::vector<EndCallbackType> callbacks;
  {
    const std::lock_guard<std::mutex> lock(this->cbs_mutex_);
    callbacks = this->end_cbs;
  }

  try {
    for (const auto &callback : callbacks) {
      callback(blackboard, outcome);
    }

  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("Could not execute end callback: %s",
                     std::string(e.what()).c_str());
  }
}

Remappings
StateMachine::compose_remappings(const Remappings &parent_remappings,
                                 const Remappings &state_remappings) {
  Remappings composed_remappings = parent_remappings;

  for (const auto &[state_key, state_target] : state_remappings) {
    auto parent_it = parent_remappings.find(state_target);
    if (parent_it != parent_remappings.end()) {
      composed_remappings[state_key] = parent_it->second;
    } else {
      composed_remappings[state_key] = state_target;
    }
  }

  return composed_remappings;
}

void StateMachine::validate(bool strict_mode) {

  YASMIN_LOG_DEBUG("Validating state machine '%s'", this->to_string().c_str());

  if (this->validated.load() && !strict_mode) {
    YASMIN_LOG_DEBUG("State machine '%s' has already been validated",
                     this->to_string().c_str());
    return;
  }

  // Check initial state
  if (this->start_state.empty()) {
    throw std::runtime_error("No initial state set");
  }

  // Terminal outcomes from all transitions
  std::set<std::string> terminal_outcomes;

  // Check all states
  for (auto it = this->states.begin(); it != this->states.end(); ++it) {

    const std::string &state_name = it->first;
    const State::SharedPtr &state = it->second;
    const Transitions &transitions = this->transitions.at(state_name);

    const Outcomes &outcomes = state->get_outcomes();

    if (strict_mode) {
      // Check if all outcomes of the state are in transitions
      const auto &sm_outcomes = this->get_outcomes();
      for (const std::string &o : outcomes) {

        if (transitions.find(o) == transitions.end() &&
            sm_outcomes.find(o) == sm_outcomes.end()) {

          throw std::runtime_error("State '" + state_name + "' outcome '" + o +
                                   "' not registered in transitions");

          // Outcomes of the state that are in outcomes of the state machine
          // do not need transitions
        } else if (sm_outcomes.find(o) != sm_outcomes.end()) {
          terminal_outcomes.insert(o);
        }
      }
    }

    // If state is a state machine, concurrence, or orthogonal state, validate
    // it. Use get_inner_state() to unwrap proxy wrappers (e.g.
    // PythonStateHolder) so that Python-loaded states are correctly identified.
    auto *inner = state->get_inner_state();
    if (auto *sm = dynamic_cast<StateMachine *>(inner)) {
      sm->validate(strict_mode);
    } else if (auto *c = dynamic_cast<Concurrence *>(inner)) {
      c->validate(strict_mode);
    } else if (auto *o = dynamic_cast<OrthogonalState *>(inner)) {
      o->validate(strict_mode);
    }

    // Add terminal outcomes
    for (auto trans_it = transitions.begin(); trans_it != transitions.end();
         ++trans_it) {
      const std::string &value = trans_it->second;
      terminal_outcomes.insert(value);
    }
  }

  // Check terminal outcomes for the state machine
  const auto &sm_outcomes = this->get_outcomes();

  if (strict_mode) {
    // Check if all outcomes from the state machine are in the terminal outcomes
    for (const std::string &o : sm_outcomes) {
      if (terminal_outcomes.find(o) == terminal_outcomes.end()) {
        throw std::runtime_error("Target outcome '" + o +
                                 "' not registered in transitions");
      }
    }
  }

  // Check if all terminal outcomes are states or outcomes of the state machine
  for (const std::string &o : terminal_outcomes) {
    if (this->states.find(o) == this->states.end() &&
        sm_outcomes.find(o) == sm_outcomes.end()) {
      throw std::runtime_error("State machine outcome '" + o +
                               "' not registered as outcome neither state");
    }
  }

  // Check for unreachable states from start_state via BFS
  {
    std::unordered_set<std::string> reachable;
    std::queue<std::string> to_visit;
    to_visit.push(this->start_state);
    reachable.insert(this->start_state);

    while (!to_visit.empty()) {
      std::string current = to_visit.front();
      to_visit.pop();

      auto trans_map_it = this->transitions.find(current);
      if (trans_map_it != this->transitions.end()) {
        for (const auto &[outcome, target] : trans_map_it->second) {
          if (this->outcomes.find(target) != this->outcomes.end()) {
            continue;
          }
          if (reachable.find(target) == reachable.end()) {
            reachable.insert(target);
            to_visit.push(target);
          }
        }
      }
    }

    for (const auto &[state_name, _] : this->states) {
      if (reachable.find(state_name) == reachable.end()) {
        throw std::runtime_error("State '" + state_name +
                                 "' is unreachable from start state '" +
                                 this->start_state + "'");
      }
    }
  }

  // State machine has been validated
  this->validated.store(true);
}

std::string StateMachine::execute(Blackboard::SharedPtr blackboard) {

  if (!this->validated.load()) {
    this->validate();
  }
  if (!this->configured.load()) {
    this->configure();
  }

  YASMIN_LOG_INFO("Executing state machine with initial state '%s'",
                  this->start_state.c_str());

  this->cancel_state_machine_requested.store(this->is_canceled());
  this->execution_active.store(true);
  this->set_current_state(this->start_state);

  // Start callbacks run after flags are set so cancel_state_machine() during a
  // callback is properly tracked rather than silently cleared.
  this->call_start_cbs(blackboard, this->start_state);

  int sigint_reg_id = -1;
  if (this->handle_sigint_) {
    sigint_reg_id =
        register_sigint_callback([this]() { this->cancel_state_machine(); });
  }

  bool state_machine_ends = false;
  std::string outcome;

  try {
    while (!state_machine_ends) {
      this->throw_if_cancel_state_machine_requested();

      std::string current_state = this->get_current_state();
      outcome =
          this->execute_step(blackboard, current_state, state_machine_ends);
    }
  } catch (...) {
    if (sigint_reg_id >= 0) {
      unregister_sigint_callback(sigint_reg_id);
    }
    this->call_end_cbs(blackboard, "");
    this->execution_active.store(false);
    this->set_current_state("");
    this->current_state_cond.notify_all();
    throw;
  }

  if (sigint_reg_id >= 0) {
    unregister_sigint_callback(sigint_reg_id);
  }
  this->execution_active.store(false);
  this->current_state_cond.notify_all();
  return outcome;
}

std::string StateMachine::execute() {
  Blackboard::SharedPtr blackboard = yasmin::Blackboard::make_shared();

  return this->execute(blackboard);
}

std::string StateMachine::operator()() {
  Blackboard::SharedPtr blackboard = yasmin::Blackboard::make_shared();

  return this->operator()(blackboard);
}

void StateMachine::cancel_state() {

  if (this->is_running()) {
    const auto current_state = this->wait_for_current_state();

    if (!current_state.empty()) {
      this->states.at(current_state)->cancel_state();
    }
  }
}

void StateMachine::cancel_state_machine() {

  if (this->is_running() || this->execution_active.load()) {
    if (!this->cancel_state_machine_requested.exchange(true)) {
      YASMIN_LOG_INFO("Canceling state machine '%s'",
                      this->to_string().c_str());
    }
    this->current_state_cond.notify_all();

    const auto current_state = this->wait_for_current_state();

    if (!current_state.empty()) {
      const auto &state = this->states.at(current_state);
      if (auto *child_state_machine =
              dynamic_cast<StateMachine *>(state->get_inner_state())) {
        child_state_machine->cancel_state_machine();
      } else {
        state->cancel_state();
      }
    }

    State::cancel_state();
  }
}

void StateMachine::set_sigint_handler(bool handle) {
  this->handle_sigint_ = handle;
}

std::string StateMachine::execute_step(Blackboard::SharedPtr blackboard,
                                       const std::string &current_state,
                                       bool &state_machine_ends) {

  const auto &states = this->states;
  const auto &local_outcomes = this->outcomes;

  auto state_it = states.find(current_state);
  if (state_it == states.end()) {
    throw std::logic_error("Active state '" + current_state +
                           "' not found in state machine");
  }
  const auto &state = state_it->second;
  const auto &local_transitions = this->transitions.at(current_state);
  const auto &local_remappings = this->remappings.at(current_state);

  auto parent_remappings = blackboard->get_remappings();
  auto composed_remappings =
      StateMachine::compose_remappings(parent_remappings, local_remappings);
  blackboard->set_remappings(composed_remappings);

  std::string outcome;
  std::string old_outcome;

  try {
    outcome = (*state.get())(blackboard);
  } catch (...) {
    blackboard->set_remappings(parent_remappings);
    throw;
  }
  old_outcome = outcome;

  blackboard->set_remappings(parent_remappings);

  this->throw_if_cancel_state_machine_requested();

  if (local_transitions.find(outcome) != local_transitions.end()) {
    outcome = local_transitions.at(outcome);
  }

  YASMIN_LOG_INFO("State machine transitioning '%s' : '%s' --> '%s'",
                  current_state.c_str(), old_outcome.c_str(), outcome.c_str());

  if (local_outcomes.find(outcome) != local_outcomes.end()) {
    this->set_current_state("");
    YASMIN_LOG_INFO("State machine ends with outcome '%s'", outcome.c_str());
    this->call_end_cbs(blackboard, outcome);
    state_machine_ends = true;
  } else if (states.find(outcome) != states.end()) {
    this->call_transition_cbs(blackboard, current_state, outcome, old_outcome);
    this->set_current_state(outcome);
  } else {
    throw std::logic_error("Outcome '" + outcome +
                           "' is not a state nor a state machine outcome");
  }

  return outcome;
}

std::string StateMachine::to_string() const {
  return "State Machine [" +
         yasmin::join(this->get_states(), ", ",
                      [](const auto &p) {
                        return p.first + " (" + p.second->to_string() + ")";
                      }) +
         "]";
}
