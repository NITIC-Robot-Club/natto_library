
// Copyright 2025 Kazusa Hashimoto
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

#include "natto_state_machine/state_machine.hpp"

namespace state_machine {

state_machine::state_machine (const rclcpp::NodeOptions &node_options) : Node ("state_machine", node_options) {
    state_action_publisher_     = this->create_publisher<natto_msgs::msg::StateAction> ("state_action", 10);
    cancel_sequence_publisher_  = this->create_publisher<std_msgs::msg::Empty> ("cancel_sequence", 10);
    state_graph_subscriber_     = this->create_subscription<natto_msgs::msg::StateGraph> ("state_graph", rclcpp::QoS (1).transient_local ().reliable (), std::bind (&state_machine::state_graph_callback, this, std::placeholders::_1));
    state_result_subscriber_    = this->create_subscription<natto_msgs::msg::StateResult> ("state_result", 10, std::bind (&state_machine::state_result_callback, this, std::placeholders::_1));
    run_sequence_subscriber_    = this->create_subscription<std_msgs::msg::String> ("run_sequence", 10, std::bind (&state_machine::run_sequence_callback, this, std::placeholders::_1));
    cancel_sequence_subscriber_ = this->create_subscription<std_msgs::msg::Empty> ("cancel_sequence", 10, std::bind (&state_machine::cancel_sequence_callback, this, std::placeholders::_1));
    force_set_state_subscriber_ = this->create_subscription<std_msgs::msg::UInt64> ("force_set_state", 10, std::bind (&state_machine::force_set_state_callback, this, std::placeholders::_1));

    double frequency   = this->declare_parameter<double> ("frequency", 10.0);
    double timeout_sec = this->declare_parameter<double> ("state_timeout_sec", 1.0);
    retry_action_on_timeout_ = this->declare_parameter<bool> ("retry_action_on_timeout", false);
    const auto sequence_timeout_names = this->declare_parameter<std::vector<std::string>> ("sequence_timeout_names", std::vector<std::string>{});
    const auto sequence_timeout_secs  = this->declare_parameter<std::vector<double>> ("sequence_timeout_secs", std::vector<double>{});
    if (sequence_timeout_names.size () != sequence_timeout_secs.size ()) {
        RCLCPP_WARN (this->get_logger (), "sequence_timeout_names and sequence_timeout_secs must have the same size. Sequence timeouts are disabled for mismatched entries.");
    }
    const auto timeout_size = std::min (sequence_timeout_names.size (), sequence_timeout_secs.size ());
    for (size_t i = 0; i < timeout_size; ++i) {
        if (sequence_timeout_secs[i] <= 0.0) {
            RCLCPP_WARN (this->get_logger (), "Ignoring non-positive timeout %.3f for sequence '%s'.", sequence_timeout_secs[i], sequence_timeout_names[i].c_str ());
            continue;
        }
        sequence_timeout_sec_[sequence_timeout_names[i]] = sequence_timeout_secs[i];
        RCLCPP_INFO (this->get_logger (), "sequence timeout: %s = %.3f sec", sequence_timeout_names[i].c_str (), sequence_timeout_secs[i]);
    }
    timeout_count_     = static_cast<uint64_t> (frequency * timeout_sec);
    timer_             = this->create_wall_timer (std::chrono::duration (std::chrono::duration<double> (1.0 / frequency)), std::bind (&state_machine::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "state_machine node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    RCLCPP_INFO (this->get_logger (), "state_timeout_sec: %.2f sec -> %lu counts", timeout_sec, timeout_count_);
    RCLCPP_INFO (this->get_logger (), "retry_action_on_timeout: %s", retry_action_on_timeout_ ? "true" : "false");
}

uint64_t state_machine::get_state_id_by_name (const std::string &state_name) {
    for (const auto &state : state_graph_.states) {
        if (state.state_name == state_name) {
            return state.state_id;
        }
    }
    RCLCPP_ERROR (this->get_logger (), "State name '%s' not found in state graph.", state_name.c_str ());
    return 0;
}

bool state_machine::has_running_state () const {
    const std::string exit_suffix = "/_exit";
    for (const auto state_id : current_state_ids_) {
        const auto state_it = std::find_if (state_graph_.states.begin (), state_graph_.states.end (), [state_id] (const auto &state) { return state.state_id == state_id; });
        if (state_it == state_graph_.states.end ()) {
            continue;
        }

        const bool is_entry_or_exit = state_it->state_name == "/_entry" || state_it->state_name == "/_exit" ||
                                      (state_it->state_name.size () >= exit_suffix.size () && state_it->state_name.compare (state_it->state_name.size () - exit_suffix.size (), exit_suffix.size (), exit_suffix) == 0);
        if (is_entry_or_exit) {
            const bool has_outgoing_transition = std::any_of (state_graph_.transitions.begin (), state_graph_.transitions.end (), [state_id] (const auto &transition) { return transition.from_state_id == state_id; });
            if (!has_outgoing_transition) {
                continue;
            }
        }
        return true;
    }
    return false;
}

void state_machine::state_graph_callback (const natto_msgs::msg::StateGraph::SharedPtr msg) {
    state_graph_ = *msg;

    uint64_t max_state_id = 0;
    for (const auto &state : state_graph_.states) {
        max_state_id = std::max (max_state_id, state.state_id);
    }

    const auto required_size = state_graph_.states.empty () ? 0 : static_cast<size_t> (max_state_id + 1);
    current_state_results_.assign (required_size, false);
    action_started_.assign (required_size, false);
    action_timeout_warned_.assign (required_size, false);
    action_timeout_counts_.assign (required_size, 0);

    current_state_ids_.erase (std::remove_if (current_state_ids_.begin (), current_state_ids_.end (), [required_size] (uint64_t id) { return id >= required_size; }), current_state_ids_.end ());

    if (current_state_ids_.empty () && !state_graph_.states.empty ()) {
        const auto entry_it = std::find_if (state_graph_.states.begin (), state_graph_.states.end (), [] (const auto &state) { return state.state_name == "/_entry"; });
        if (entry_it != state_graph_.states.end ()) {
            current_state_ids_.push_back (entry_it->state_id);
        } else {
            RCLCPP_WARN (this->get_logger (), "Entry state '/_entry' not found in state graph");
        }
    }
    RCLCPP_INFO (this->get_logger (), "State graph updated. Total states: %zu", state_graph_.states.size ());
}

void state_machine::state_result_callback (const natto_msgs::msg::StateResult::SharedPtr msg) {
    if (!msg->success) {
        return;
    }

    if (msg->state_id >= current_state_results_.size ()) {
        RCLCPP_WARN (this->get_logger (), "State result for unknown state id %lu", msg->state_id);
        return;
    }

    current_state_results_[msg->state_id] = true;
}

void state_machine::force_set_state_callback (const std_msgs::msg::UInt64::SharedPtr msg) {
    if (msg->data >= current_state_results_.size ()) {
        RCLCPP_WARN (this->get_logger (), "force_set_state requested unknown state id %lu", msg->data);
        return;
    }

    current_state_ids_.clear ();
    current_state_ids_.push_back (msg->data);
}

void state_machine::cancel_sequence_callback (const std_msgs::msg::Empty::SharedPtr) {
    if (!sequence_running_ && !has_running_state ()) {
        return;
    }

    cancel_active_sequence ("manual", false);
}

void state_machine::cancel_active_sequence (const std::string &reason, bool publish_cancel) {
    RCLCPP_WARN (this->get_logger (), "Cancelling sequence '%s' (%s).", active_sequence_name_.c_str (), reason.c_str ());

    if (publish_cancel) {
        cancel_sequence_publisher_->publish (std_msgs::msg::Empty {});
    }

    sequence_running_ = false;
    active_sequence_name_.clear ();
    current_state_results_.assign (current_state_results_.size (), false);
    action_started_.assign (action_started_.size (), false);
    action_timeout_warned_.assign (action_timeout_warned_.size (), false);
    action_timeout_counts_.assign (action_timeout_counts_.size (), 0);
    current_state_ids_.clear ();

    const auto entry_it = std::find_if (state_graph_.states.begin (), state_graph_.states.end (), [] (const auto &state) { return state.state_name == "/_entry"; });
    if (entry_it != state_graph_.states.end ()) {
        current_state_ids_.push_back (entry_it->state_id);
    }
}

void state_machine::run_sequence_callback (const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Received an empty sequence name.");
        return;
    }

    if (sequence_running_) {
        RCLCPP_WARN (this->get_logger (), "Sequence '%s' is already running. Ignored '%s'.", active_sequence_name_.c_str (), msg->data.c_str ());
        return;
    }
    if (has_running_state ()) {
        RCLCPP_WARN (this->get_logger (), "State machine is already running another state graph. Ignored sequence '%s'.", msg->data.c_str ());
        return;
    }

    const std::string entry_name = "/" + msg->data + "/_entry";
    const uint64_t    entry_id   = get_state_id_by_name (entry_name);
    if (entry_id == 0) {
        RCLCPP_WARN (this->get_logger (), "Sequence '%s' was not found. Expected state '%s'.", msg->data.c_str (), entry_name.c_str ());
        return;
    }

    current_state_ids_.clear ();
    current_state_ids_.push_back (entry_id);
    current_state_results_[entry_id] = false;
    action_started_[entry_id]         = false;
    action_timeout_warned_[entry_id]  = false;
    action_timeout_counts_[entry_id] = 0;
    active_sequence_name_            = msg->data;
    sequence_started_at_             = this->now ();
    sequence_running_                = true;

    RCLCPP_INFO (this->get_logger (), "Starting sequence '%s'.", active_sequence_name_.c_str ());
}

void state_machine::timer_callback () {
    if (state_graph_.states.empty ()) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "State graph is empty. Waiting for state graph...");
        return;
    }
    if (current_state_ids_.empty ()) {
        const auto entry_it = std::find_if (state_graph_.states.begin (), state_graph_.states.end (), [] (const auto &state) { return state.state_name == "/_entry"; });
        if (entry_it != state_graph_.states.end ()) {
            current_state_ids_.push_back (entry_it->state_id);
        } else {
            RCLCPP_ERROR_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "Entry state '/_entry' not found in state graph");
            return;
        }
    }

    if (sequence_running_) {
        const auto timeout_it = sequence_timeout_sec_.find (active_sequence_name_);
        if (timeout_it != sequence_timeout_sec_.end () && (this->now () - sequence_started_at_).seconds () >= timeout_it->second) {
            cancel_active_sequence ("timeout", true);
            return;
        }
    }

    std::vector<uint64_t> next_state_ids = current_state_ids_;

    for (auto state_id : current_state_ids_) {
        if (state_id >= action_timeout_counts_.size ()) {
            RCLCPP_ERROR_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "State id %lu out of bounds for action timeout storage", state_id);
            continue;
        }
        if (action_timeout_counts_[state_id] > 0) {
            action_timeout_counts_[state_id]--;
        }
    }

    for (auto state_id : current_state_ids_) {
        if (state_id >= current_state_results_.size () || state_id >= action_timeout_counts_.size ()) {
            RCLCPP_ERROR_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "State id %lu out of bounds for current graph size %zu", state_id, current_state_results_.size ());
            continue;
        }

        bool parents_done = true;
        for (const auto &t : state_graph_.transitions) {
            if (t.to_state_id == state_id && std::find (current_state_ids_.begin (), current_state_ids_.end (), t.from_state_id) != current_state_ids_.end ()) {
                parents_done = false;
                break;
            }
        }
        if (!parents_done) continue;

        for (const auto &transition : state_graph_.transitions) {
            if (transition.from_state_id != state_id || transition.condition.empty ()) continue;

            if (action_timeout_counts_[state_id] == 0) {
                if (action_started_[state_id] && !retry_action_on_timeout_) {
                    if (!action_timeout_warned_[state_id]) {
                        RCLCPP_WARN (
                            this->get_logger (),
                            "Action for state ID %lu timed out. "
                            "Retry is disabled; action will not be republished.",
                            state_id);
                        action_timeout_warned_[state_id] = true;
                    }
                } else {
                    natto_msgs::msg::StateAction action;
                    action.state_id = state_id;

                    std::regex  action_regex (R"(^\s*([a-zA-Z0-9_]+)\((.*)\)\s*$)");
                    std::smatch match;
                    if (std::regex_match (transition.condition, match, action_regex)) {
                        action.action_name   = match[1];
                        std::string args_str = match[2];

                        std::regex        arg_regex (R"(\s*([^=]+)\s*=\s*(.+)\s*)");
                        std::stringstream ss (args_str);
                        std::string       token;
                        while (std::getline (ss, token, ',')) {
                            std::smatch arg_match;
                            if (std::regex_match (token, arg_match, arg_regex)) {
                                std::string name  = arg_match[1];
                                std::string value = arg_match[2];
                                name.erase (0, name.find_first_not_of (" \t"));
                                name.erase (name.find_last_not_of (" \t") + 1);
                                if (!value.empty () && value[0] != '\'' && value[0] != '"') {
                                    value.erase (remove_if (value.begin (), value.end (), ::isspace), value.end ());
                                }
                                action.arguments_names.push_back (name);
                                action.arguments_values.push_back (value);
                            }
                        }
                    }

                    state_action_publisher_->publish (action);
                    action_started_[state_id] = true;
                    action_timeout_counts_[state_id] = timeout_count_;

                    RCLCPP_INFO (this->get_logger (), "Publishing action '%s' for state ID %lu", action.action_name.c_str (), state_id);
                }
            }
        }

        for (const auto &transition : state_graph_.transitions) {
            if (transition.from_state_id == state_id) {
                bool can_transition = transition.condition.empty () || current_state_results_[state_id];
                if (can_transition) {
                    for (size_t i = 0; i < next_state_ids.size (); ++i) {
                        if (next_state_ids[i] == state_id) {
                            next_state_ids[i] = transition.to_state_id;
                        }
                    }
                    std::string current_state_name, next_state_name;
                    for (const auto &state : state_graph_.states) {
                        if (state.state_id == state_id) {
                            current_state_name = state.state_name;
                        }
                        if (state.state_id == transition.to_state_id) {
                            next_state_name = state.state_name;
                        }
                    }

                    RCLCPP_INFO (this->get_logger (), "Transitioning from state %s (%lu) to state %s (%lu)", current_state_name.c_str (), state_id, next_state_name.c_str (), transition.to_state_id);

                    if (transition.to_state_id < current_state_results_.size () && transition.to_state_id < action_timeout_counts_.size ()) {
                        current_state_results_[transition.to_state_id] = false;
                        action_started_[transition.to_state_id] = false;
                        action_timeout_warned_[transition.to_state_id] = false;
                        action_timeout_counts_[transition.to_state_id] = 0;
                    } else {
                        RCLCPP_ERROR_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "Transition target state id %lu out of bounds", transition.to_state_id);
                    }
                }
            }
        }
    }
    current_state_ids_ = next_state_ids;

    if (sequence_running_) {
        const std::string exit_name = "/" + active_sequence_name_ + "/_exit";
        const auto        exit_it   = std::find_if (state_graph_.states.begin (), state_graph_.states.end (), [&exit_name] (const auto &state) { return state.state_name == exit_name; });
        if (exit_it != state_graph_.states.end () && current_state_ids_.size () == 1 && current_state_ids_.front () == exit_it->state_id) {
            RCLCPP_INFO (this->get_logger (), "Sequence '%s' finished.", active_sequence_name_.c_str ());
            sequence_running_ = false;
            active_sequence_name_.clear ();
        }
    }
}

}  // namespace state_machine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (state_machine::state_machine)
