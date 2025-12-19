
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

#ifndef __MERMAID_STATE_MACHINE_HPP__
#define __MERMAID_STATE_MACHINE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/state_action.hpp"
#include "natto_msgs/msg/state_graph.hpp"
#include "natto_msgs/msg/state_result.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace mermaid_state_machine {
class mermaid_state_machine : public rclcpp::Node {
   public:
    mermaid_state_machine (const rclcpp::NodeOptions &node_options);

   private:
    natto_msgs::msg::StateGraph state_graph_;

    std::vector<bool>                         current_state_results_;
    std::vector<uint64_t>                     action_timeout_counts_;
    std::vector<uint64_t>                     current_state_ids_;
    std::vector<std::string>                  scope_stack_;
    std::unordered_map<std::string, uint64_t> state_name_to_id_;

    uint64_t timeout_count_;
    uint64_t next_state_id_ = 1;

    void        parse_state_graph (const std::string &mermaid_path);
    uint64_t    get_or_create_state_id (const std::string &state_name);
    std::string get_current_scope_name ();
    std::string join_scope (const std::string &scope, const std::string &name);

    void state_result_callback (const natto_msgs::msg::StateResult::SharedPtr msg);
    void force_set_state_callback (const std_msgs::msg::UInt64::SharedPtr msg);

    void timer_callback ();

    rclcpp::Publisher<natto_msgs::msg::StateGraph>::SharedPtr     state_graph_publisher_;
    rclcpp::Publisher<natto_msgs::msg::StateAction>::SharedPtr    state_action_publisher_;
    rclcpp::Subscription<natto_msgs::msg::StateResult>::SharedPtr state_result_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr        force_set_state_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
};
}  // namespace mermaid_state_machine

#endif  // __MERMAID_STATE_MACHINE_HPP__