// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include "yasmin/orthogonal_state.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <thread>
#include <unordered_set>

#include "yasmin/logs.hpp"
#include "yasmin/parallel_execution.hpp"
#include "yasmin/state_utils.hpp"

namespace yasmin {

std::shared_ptr<OrthogonalState::GilHook> OrthogonalState::before_fork_hook_;
std::shared_ptr<OrthogonalState::GilHook> OrthogonalState::after_join_hook_;
std::mutex OrthogonalState::hooks_mutex_;

void OrthogonalState::set_thread_hooks(GilHook before_fork,
                                       GilHook after_join) {
  std::lock_guard<std::mutex> lock(hooks_mutex_);
  before_fork_hook_ = std::make_shared<GilHook>(std::move(before_fork));
  after_join_hook_ = std::make_shared<GilHook>(std::move(after_join));
}

OrthogonalState::OrthogonalState(const std::string &default_outcome,
                                 const OutcomeMap &outcome_map)
    : State(yasmin::generate_possible_outcomes(outcome_map, default_outcome)),
      outcome_map_(outcome_map), default_outcome_(default_outcome) {}

void OrthogonalState::add_region(const std::string &name,
                                 StateMachine::SharedPtr sm) {
  if (!sm) {
    throw std::invalid_argument("Region state machine cannot be null");
  }
  for (const auto &r : this->regions_) {
    if (r.name == name) {
      throw std::invalid_argument("Region '" + name + "' already exists");
    }
    if (r.sm == sm) {
      throw std::invalid_argument(
          "Regions cannot share a state machine instance");
    }
  }
  this->regions_.push_back({name, std::move(sm)});
  this->configured_.store(false);
}

const std::vector<OrthogonalState::RegionDescriptor> &
OrthogonalState::get_regions() const noexcept {
  return this->regions_;
}

const OutcomeMap &OrthogonalState::get_outcome_map() const noexcept {
  return this->outcome_map_;
}

const std::string &OrthogonalState::get_default_outcome() const noexcept {
  return this->default_outcome_;
}

void OrthogonalState::configure() {
  if (yasmin::check_already_configured(this->configured_, "OrthogonalState",
                                       this->to_string().c_str()))
    return;

  // Configure all regions
  for (auto &region : this->regions_) {
    region.sm->configure();
  }

  // Collect all JoinStates grouped by sync_id, including those in nested
  // state machines. Nested orthogonal states and concurrences own their
  // joins.
  struct Participant {
    std::size_t region;
    JoinState *join_state;
  };
  std::unordered_map<std::string, std::vector<Participant>> join_groups;

  for (std::size_t i = 0; i < this->regions_.size(); ++i) {
    const auto &region = this->regions_[i];
    std::unordered_set<std::string> region_sync_ids;
    std::unordered_set<StateMachine *> visited{region.sm.get()};
    std::vector<StateMachine *> pending{region.sm.get()};
    while (!pending.empty()) {
      StateMachine *sm = pending.back();
      pending.pop_back();
      for (const auto &[state_name, state] : sm->get_states()) {
        (void)state_name;
        State *inner = state->get_inner_state();
        if (auto *nested = dynamic_cast<StateMachine *>(inner)) {
          if (visited.insert(nested).second) {
            pending.push_back(nested);
          }
          continue;
        }
        JoinState *js = dynamic_cast<JoinState *>(inner);
        if (js) {
          if (!region_sync_ids.insert(js->get_sync_id()).second) {
            throw std::invalid_argument(
                "Region '" + region.name +
                "' contains multiple JoinStates for '" + js->get_sync_id() +
                "'");
          }
          join_groups[js->get_sync_id()].push_back({i, js});
        }
      }
    }
  }

  // Create barriers for each group
  this->barriers_.clear();
  this->region_barriers_.assign(this->regions_.size(), {});
  for (auto &[sync_id, participants] : join_groups) {
    if (participants.size() < 2) {
      throw std::runtime_error("JoinState sync_id '" + sync_id +
                               "' has < 2 participants; "
                               "each sync point needs at least 2 regions");
    }
    auto barrier =
        std::make_shared<RegionBarrier>(static_cast<int>(participants.size()));
    for (const auto &participant : participants) {
      participant.join_state->set_barrier(barrier);
      this->region_barriers_[participant.region].push_back(barrier);
    }
    this->barriers_[sync_id] = barrier;
    YASMIN_LOG_DEBUG("Created barrier '%s' with %zu participants",
                     sync_id.c_str(), participants.size());
  }

  // Build region name -> index map for O(1) lookups
  this->region_name_to_index_.clear();
  this->region_name_to_index_.reserve(this->regions_.size());
  for (size_t i = 0; i < this->regions_.size(); i++) {
    this->region_name_to_index_[this->regions_[i].name] = i;
  }

  this->configured_.store(true);
}

std::string OrthogonalState::execute(Blackboard::SharedPtr blackboard) {
  this->configure();

  for (auto &[sync_id, barrier] : this->barriers_) {
    (void)sync_id;
    barrier->reset();
  }

  if (this->regions_.empty()) {
    return this->default_outcome_;
  }

  std::vector<std::string> region_outcomes(this->regions_.size());

  // Snapshot hooks under lock for thread-safe access during concurrent
  // execution
  std::shared_ptr<GilHook> before_hook;
  std::shared_ptr<GilHook> after_hook;
  {
    std::lock_guard<std::mutex> lock(hooks_mutex_);
    before_hook = before_fork_hook_;
    after_hook = after_join_hook_;
  }

  // Invoke before-fork hook (e.g., GIL release when Python states are used)
  if (before_hook) {
    (*before_hook)();
  }

  try {
    detail::run_parallel(
        this->regions_.size(),
        [this, &blackboard, &region_outcomes](std::size_t i) {
          // A finished region stops holding back its barriers, including
          // when it ends through a path that skips its JoinState.
          struct DropOnExit {
            const std::vector<RegionBarrier::SharedPtr> &barriers;
            ~DropOnExit() {
              for (const auto &barrier : barriers) {
                barrier->drop();
              }
            }
          } drop_on_exit{this->region_barriers_[i]};
          auto bb_copy = std::make_shared<Blackboard>(*blackboard);
          region_outcomes[i] = (*this->regions_[i].sm)(bb_copy);
        },
        [this]() {
          for (auto &[id, barrier] : this->barriers_) {
            (void)id;
            barrier->cancel();
          }
          for (auto &region : this->regions_) {
            region.sm->cancel_state_machine();
          }
        },
        [this]() { return this->is_canceled(); });
  } catch (const StateMachineCancelException &) {
    if (after_hook) {
      (*after_hook)();
    }
    if (this->is_canceled()) {
      return this->default_outcome_;
    }
    throw;
  } catch (...) {
    if (after_hook) {
      (*after_hook)();
    }
    throw;
  }
  if (after_hook) {
    (*after_hook)();
  }

  // Handle cancel
  if (this->is_canceled()) {
    return this->default_outcome_;
  }

  // Evaluate outcome map
  return this->evaluate_outcomes(region_outcomes);
}

void OrthogonalState::cancel_state() {
  State::cancel_state();
  // Unblock all barriers first
  // configure() publishes the barrier map only when it is complete.
  if (this->configured_.load()) {
    for (auto &[id, barrier] : this->barriers_) {
      (void)id;
      barrier->cancel();
    }
  }
  // Cancel each region's state machine
  for (auto &region : this->regions_) {
    if (region.sm->is_running()) {
      region.sm->cancel_state_machine();
    }
  }
}

std::string OrthogonalState::evaluate_outcomes(
    const std::vector<std::string> &region_outcomes) const {
  Outcomes satisfied_outcomes = yasmin::evaluate_satisfied_outcomes(
      this->outcome_map_,
      [this, &region_outcomes](const std::string &state_name) -> std::string {
        auto it = this->region_name_to_index_.find(state_name);
        if (it != this->region_name_to_index_.end()) {
          return region_outcomes[it->second];
        }
        throw std::runtime_error("Unknown region name '" + state_name +
                                 "' in orthogonal state outcome map");
      });

  return yasmin::resolve_outcome(satisfied_outcomes, this->default_outcome_,
                                 "orthogonal state execution (" +
                                     this->to_string() + ")");
}

void OrthogonalState::validate(bool strict_mode) {
  for (auto &region : this->regions_) {
    region.sm->validate(strict_mode);
  }
}

std::string OrthogonalState::to_string() const {
  return "OrthogonalState [" +
         yasmin::join(this->regions_, ", ",
                      [](const auto &r) {
                        return r.name + " (" + r.sm->get_name() + ")";
                      }) +
         "]";
}

} // namespace yasmin
