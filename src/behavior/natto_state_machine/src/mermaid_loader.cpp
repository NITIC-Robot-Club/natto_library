
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

#include "natto_state_machine/mermaid_loader.hpp"

namespace mermaid_loader {

mermaid_loader::mermaid_loader (const rclcpp::NodeOptions &node_options) : Node ("mermaid_loader", node_options) {
    state_graph_publisher_ = this->create_publisher<natto_msgs::msg::StateGraph> ("state_graph", rclcpp::QoS (1).transient_local ().reliable ());

    std::string mermaid_path = this->declare_parameter<std::string> ("mermaid_path", "");
    parse_state_graph (mermaid_path);
    state_graph_publisher_->publish (state_graph_);

    RCLCPP_INFO (this->get_logger (), "mermaid_loader node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "mermaid_path: %s", mermaid_path.c_str ());
}

uint64_t mermaid_loader::get_or_create_state_id (const std::string &state_name) {
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

std::string mermaid_loader::get_current_scope_name () {
    std::string str;
    for (const auto &scope : scope_stack_) {
        if (!scope.empty ()) {
            str += "/" + scope;
        }
    }
    return str.empty () ? "/" : str;
}

std::string mermaid_loader::join_scope (const std::string &scope, const std::string &name) {
    if (scope == "/" || scope.empty ()) {
        return "/" + name;
    }
    return scope + "/" + name;
}

void mermaid_loader::parse_state_graph (const std::string &path) {
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
}

}  // namespace mermaid_loader

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (mermaid_loader::mermaid_loader)