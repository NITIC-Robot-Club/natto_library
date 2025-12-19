
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

    double frequency = this->declare_parameter<double> ("frequency", 10.0);
    timer_           = this->create_wall_timer (std::chrono::duration (std::chrono::duration<double> (1.0 / frequency)), std::bind (&mermaid_state_machine::timer_callback, this));
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
    std::string s;
    for (const auto &p : scope_stack_) {
        if (!p.empty ()) {
            s += "/" + p;
        }
    }
    return s.empty () ? "/" : s;
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

    std::ifstream ifs (path);
    if (!ifs.is_open ()) {
        RCLCPP_ERROR (get_logger (), "Failed to open %s", path.c_str ());
        return;
    }
    auto trim = [] (std::string &s) {
        s.erase (0, s.find_first_not_of (" \t"));
        s.erase (s.find_last_not_of (" \t") + 1);
    };

    get_or_create_state_id ("/_entry");
    get_or_create_state_id ("/_exit");

    std::vector<std::string> lines;
    std::string              line;

    while (std::getline (ifs, line)) {
        trim (line);
        if (line.empty () || line == "stateDiagram-v2") continue;
        lines.push_back (line);
    }

    std::unordered_set<std::string> defined_states;
    for (const auto &l : lines) {
        if (l.rfind ("state ", 0) == 0 && l.back () == '{') {
            std::string name = l.substr (6, l.size () - 7);
            trim (name);
            std::string parent_scope = get_current_scope_name ();
            scope_stack_.push_back (name);
            std::string scope = get_current_scope_name ();
            defined_states.insert (scope);
            get_or_create_state_id (join_scope (scope, "_entry"));
            get_or_create_state_id (join_scope (scope, "_exit"));
        } else if (l == "}") {
            scope_stack_.pop_back ();
        }
    }

    scope_stack_.clear ();
    scope_stack_.push_back ("");
    for (const auto &l : lines) {
        if (l.rfind ("state ", 0) == 0 && l.back () == '{') {
            std::string name = l.substr (6, l.size () - 7);
            trim (name);
            scope_stack_.push_back (name);
            continue;
        }
        if (l == "}") {
            scope_stack_.pop_back ();
            continue;
        }

        auto arrow = l.find ("-->");
        if (arrow != std::string::npos) {
            std::string left  = l.substr (0, arrow);
            std::string right = l.substr (arrow + 3);
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

            natto_msgs::msg::StateTransition t;
            t.from_state_id = from_id;
            t.to_state_id   = to_id;
            t.condition     = condition;
            state_graph_.transitions.push_back (t);

            RCLCPP_INFO (get_logger (), "Parsed transition: %s --> %s%s", from.c_str (), to.c_str (), condition.empty () ? "" : (" [condition: " + condition + "]").c_str ());
        }
    }

    current_state_results_.resize (next_state_id_, false);
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

    std::vector<uint64_t> next_states;

    for (auto state_id : current_state_ids_) {
        bool parents_done = true;
        for (const auto &t : state_graph_.transitions) {
            if (t.to_state_id == state_id) {
                if (std::find (current_state_ids_.begin (), current_state_ids_.end (), t.from_state_id) != current_state_ids_.end ()) {
                    parents_done = false;
                    break;
                }
            }
        }
        if (!parents_done) continue;

        for (const auto &t : state_graph_.transitions) {
            if (t.from_state_id == state_id && !t.condition.empty ()) {
                if (!current_state_results_[state_id]) {
                    natto_msgs::msg::StateAction action;
                    action.state_id = state_id;

                    std::regex  action_regex (R"(^\s*([a-zA-Z0-9_]+)\((.*)\)\s*$)");
                    std::smatch match;
                    if (std::regex_match (t.condition, match, action_regex)) {
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
                    } else {
                        RCLCPP_WARN (get_logger (), "Invalid action format: '%s'", t.condition.c_str ());
                    }

                    state_action_publisher_->publish (action);

                    next_states.push_back (state_id);
                }
            }
        }
        for (const auto &t : state_graph_.transitions) {
            if (t.from_state_id == state_id) {
                bool can_transition = t.condition.empty () || current_state_results_[state_id];
                if (can_transition) {
                    next_states.push_back (t.to_state_id);
                }
            }
        }
    }

    current_state_ids_ = next_states;
}

}  // namespace mermaid_state_machine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (mermaid_state_machine::mermaid_state_machine)