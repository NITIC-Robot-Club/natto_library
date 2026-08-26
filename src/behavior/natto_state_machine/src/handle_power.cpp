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

#include "natto_state_machine/default_action.hpp"

namespace default_action {

void default_action::handle_power (const natto_msgs::msg::StateAction::SharedPtr msg) {
    std_msgs::msg::Bool power_msg;
    bool power            = false;
    bool power_specified = false;

    for (size_t i = 0; i < msg->arguments_names.size () && i < msg->arguments_values.size (); ++i) {
        if (msg->arguments_names[i] != "power") {
            continue;
        }
        if (msg->arguments_values[i] == "true" || msg->arguments_values[i] == "1") {
            power            = true;
            power_specified  = true;
        } else if (msg->arguments_values[i] == "false" || msg->arguments_values[i] == "0") {
            power            = false;
            power_specified  = true;
        }
    }

    // Keep poweroff() compatible while migrating state graphs to set_power(power=false).
    if (msg->action_name == "poweroff") {
        power_specified = true;
        power           = false;
    }

    if (!power_specified) {
        RCLCPP_ERROR (this->get_logger (), "set_power requires a boolean 'power' argument.");
        natto_msgs::msg::StateResult result;
        result.state_id    = msg->state_id;
        result.success     = false;
        result.action_name = "set_power";
        state_result_publisher_->publish (result);
        return;
    }

    power_msg.data = power;
    power_publisher_->publish (power_msg);

    natto_msgs::msg::StateResult result;
    result.state_id    = msg->state_id;
    result.success     = true;
    result.action_name = msg->action_name == "poweroff" ? "poweroff" : "set_power";
    state_result_publisher_->publish (result);
}

}  // namespace default_action
