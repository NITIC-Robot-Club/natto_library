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

void default_action::handle_poweroff (const natto_msgs::msg::StateAction::SharedPtr msg) {
    std_msgs::msg::Bool power_msg;
    power_msg.data = false;
    power_publisher_->publish (power_msg);

    natto_msgs::msg::StateResult result;
    result.state_id    = msg->state_id;
    result.success     = true;
    result.action_name = "poweroff";
    state_result_publisher_->publish (result);
}

}  // namespace default_action
