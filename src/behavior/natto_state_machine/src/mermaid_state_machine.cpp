
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

#include "natto_state_machine/mermaid_state_machine.hpp"

namespace mermaid_state_machine {

mermaid_state_machine::mermaid_state_machine (const rclcpp::NodeOptions &node_options) : Node ("mermaid_state_machine", node_options) {
    state_graph_publisher_      = this->create_publisher<natto_msgs::msg::StateGraph> ("state_graph", rclcpp::QoS (1).transient_local ().reliable ());
    state_action_publisher_     = this->create_publisher<natto_msgs::msg::StateAction> ("state_action", 10);
    state_result_subscriber_    = this->create_subscription<natto_msgs::msg::StateResult> ("state_result", 10, std::bind (&mermaid_state_machine::state_result_callback, this, std::placeholders::_1));
    force_set_state_subscriber_ = this->create_subscription<std_msgs::msg::UInt64> ("force_set_state", 10, std::bind (&mermaid_state_machine::force_set_state_callback, this, std::placeholders::_1));

    std::string mermaid_path = this->declare_parameter<std::string> ("mermaid_path", "");
    parse_state_graph (mermaid_path);
    state_graph_publisher_->publish (state_graph_);

    double frequency   = this->declare_parameter<double> ("frequency", 10.0);
    double timeout_sec = this->declare_parameter<double> ("state_timeout_sec", 1.0);
    timeout_count_     = static_cast<uint64_t> (frequency * timeout_sec);
    timer_             = this->create_wall_timer (std::chrono::duration (std::chrono::duration<double> (1.0 / frequency)), std::bind (&mermaid_state_machine::timer_callback, this));

    RCLCPP_INFO (this->get_logger (), "mermaid_state_machine node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "mermaid_path: %s", mermaid_path.c_str ());
    RCLCPP_INFO (this->get_logger (), "frequency: %.2f Hz", frequency);
    RCLCPP_INFO (this->get_logger (), "state_timeout_sec: %.2f sec -> %lu counts", timeout_sec, timeout_count_);
}

uint64_t mermaid_state_machine::get_or_create_state_id (const std::string &state_name) {
    auto it = state_name_to_id_.find (state_name);
    if (it != state_name_to_id_.end ()) {
        return it->second;
    }
    uint64_t id = next_state_id_++;

    state_name_to_id_[state_name] = id;

    natto_msgs::msg::State s;
    s.state_id   = id;
    s.state_name = state_name;
    state_graph_.states.push_back (s);

    return id;
}

std::string mermaid_state_machine::get_current_scope_name () {
    std::string str;
    for (const auto &scope : scope_stack_) {
        if (!scope.empty ()) {
            str += "/" + scope;
        }
    }
    return str.empty () ? "/" : str;
}

std::string mermaid_state_machine::join_scope (const std::string &scope, const std::string &name) {
    if (scope == "/" || scope.empty ()) {
        return "/" + name;
    }
    return scope + "/" + name;
}

void mermaid_state_machine::parse_state_graph (const std::string &path) {
    state_graph_.states.clear ();
    state_graph_.transitions.clear ();
    state_name_to_id_.clear ();
    scope_stack_.clear ();
    scope_stack_.push_back ("");

    state_graph_.graph_name = path;
    std::ifstream ifs (path);
    if (!ifs.is_open ()) {
        RCLCPP_ERROR (get_logger (), "Failed to open %s", path.c_str ());
        return;
    }

    auto trim = [] (std::string &str) {
        str.erase (0, str.find_first_not_of (" \t"));
        str.erase (str.find_last_not_of (" \t") + 1);
    };

    get_or_create_state_id ("/_entry");
    get_or_create_state_id ("/_exit");

    std::vector<std::string> lines;
    std::string              temp_line;

    while (std::getline (ifs, temp_line)) {
        trim (temp_line);
        if (temp_line.empty () || temp_line == "stateDiagram-v2") continue;
        lines.push_back (temp_line);
    }

    std::unordered_set<std::string> defined_states;
    for (const auto &line : lines) {
        if (line.rfind ("state ", 0) == 0 && line.back () == '{') {
            std::string name = line.substr (6, line.size () - 7);
            trim (name);
            std::string parent_scope = get_current_scope_name ();
            scope_stack_.push_back (name);
            std::string scope = get_current_scope_name ();
            defined_states.insert (scope);
            get_or_create_state_id (join_scope (scope, "_entry"));
            get_or_create_state_id (join_scope (scope, "_exit"));
        } else if (line == "}") {
            scope_stack_.pop_back ();
        }
    }

    scope_stack_.clear ();
    scope_stack_.push_back ("");
    for (const auto &line : lines) {
        if (line.rfind ("state ", 0) == 0 && line.back () == '{') {
            std::string name = line.substr (6, line.size () - 7);
            trim (name);
            scope_stack_.push_back (name);
            continue;
        }
        if (line == "}") {
            scope_stack_.pop_back ();
            continue;
        }

        auto arrow = line.find ("-->");
        if (arrow != std::string::npos) {
            std::string left  = line.substr (0, arrow);
            std::string right = line.substr (arrow + 3);
            trim (left);
            trim (right);

            std::string condition;
            auto        colon = right.find (":");
            if (colon != std::string::npos) {
                condition = right.substr (colon + 1);
                right     = right.substr (0, colon);
                trim (condition);
                trim (right);
            }

            std::string scope   = get_current_scope_name ();
            auto        resolve = [&] (const std::string &s, bool is_to) {
                if (s == "[*]") return is_to ? join_scope (scope, "_exit") : join_scope (scope, "_entry");
                std::string candidate = (s[0] == '/') ? s : join_scope (scope, s);
                if (is_to && defined_states.count (candidate)) candidate += "/_entry";
                return candidate;
            };

            std::string from = resolve (left, false);
            std::string to   = resolve (right, true);

            if (defined_states.count (from) && !defined_states.count (to)) {
                from += "/_exit";
            }

            uint64_t from_id = get_or_create_state_id (from);
            uint64_t to_id   = get_or_create_state_id (to);

            natto_msgs::msg::StateTransition transition;
            transition.from_state_id = from_id;
            transition.to_state_id   = to_id;
            transition.condition     = condition;
            state_graph_.transitions.push_back (transition);
        }
    }

    current_state_results_.resize (next_state_id_, false);
    action_timeout_counts_.resize (next_state_id_, 0);
}

void mermaid_state_machine::state_result_callback (const natto_msgs::msg::StateResult::SharedPtr msg) {
    if (!msg->success) {
        return;
    }

    current_state_results_[msg->state_id] = true;
}

void mermaid_state_machine::force_set_state_callback (const std_msgs::msg::UInt64::SharedPtr msg) {
    current_state_ids_.clear ();
    current_state_ids_.push_back (msg->data);
}

void mermaid_state_machine::timer_callback () {
    if (current_state_ids_.empty ()) {
        current_state_ids_.push_back (get_or_create_state_id ("/_entry"));
    }

    std::vector<uint64_t> next_state_ids = current_state_ids_;

    for (auto state_id : current_state_ids_) {
        if (action_timeout_counts_[state_id] > 0) {
            action_timeout_counts_[state_id]--;
        }
    }

    for (auto state_id : current_state_ids_) {
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

            bool send_action = (!current_state_results_[state_id] || action_timeout_counts_[state_id] == 0);
            if (send_action) {
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
                action_timeout_counts_[state_id] = timeout_count_;
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
                    current_state_results_[transition.to_state_id] = false;
                    action_timeout_counts_[transition.to_state_id] = 0;
                }
            }
        }
    }

    current_state_ids_ = next_state_ids;
}

}  // namespace mermaid_state_machine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (mermaid_state_machine::mermaid_state_machine)