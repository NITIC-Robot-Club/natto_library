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

#include <cmath>

namespace default_action {

void default_action::handle_set_joint_position (const natto_msgs::msg::StateAction::SharedPtr msg) {
    joint_state_id_   = msg->state_id;
    joint_state_sent_ = true;
    command_joint_state_.name.clear ();
    command_joint_state_.position.clear ();
    command_joint_state_.velocity.clear ();
    for (size_t i = 0; i < msg->arguments_names.size () && i < msg->arguments_values.size (); i++) {
        size_t j;
        for (j = 0; j < command_joint_state_.name.size (); j++) {
            if (msg->arguments_names[i] == command_joint_state_.name[j]) {
                command_joint_state_.position[j] = std::stod (msg->arguments_values[i]);
            }
        }
        if (j == command_joint_state_.name.size ()) {
            command_joint_state_.name.push_back (msg->arguments_names[i]);
            command_joint_state_.position.push_back (std::stod (msg->arguments_values[i]));
            command_joint_state_.velocity.push_back (0.0);
        }
    }
}

void default_action::handle_set_joint_velocity (const natto_msgs::msg::StateAction::SharedPtr msg) {
    joint_state_id_   = msg->state_id;
    joint_state_sent_ = false;
    command_joint_state_.name.clear ();
    command_joint_state_.position.clear ();
    command_joint_state_.velocity.clear ();
    for (size_t i = 0; i < msg->arguments_names.size () && i < msg->arguments_values.size (); i++) {
        size_t j;
        for (j = 0; j < command_joint_state_.name.size (); j++) {
            if (msg->arguments_names[i] == command_joint_state_.name[j]) {
                command_joint_state_.velocity[j] = std::stod (msg->arguments_values[i]);
            }
        }
        if (j == command_joint_state_.name.size ()) {
            command_joint_state_.name.push_back (msg->arguments_names[i]);
            command_joint_state_.position.push_back (0.0);
            command_joint_state_.velocity.push_back (std::stod (msg->arguments_values[i]));
        }
    }

    natto_msgs::msg::StateResult result;
    result.state_id    = joint_state_id_;
    result.success     = true;
    result.action_name = "set_joint_velocity";
    state_result_publisher_->publish (result);
}

void default_action::joint_state_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!joint_state_sent_) {
        return;
    }
    bool reached = true;
    for (size_t i = 0; i < msg->name.size (); i++) {
        for (size_t j = 0; j < command_joint_state_.name.size (); j++) {
            if (msg->name[i] == command_joint_state_.name[j]) {
                auto   it_tol = joint_tolerances_.find (msg->name[i]);
                double tol    = 0.01;
                if (it_tol != joint_tolerances_.end ()) {
                    tol = it_tol->second;
                }
                if (fabs (msg->position[i] - command_joint_state_.position[j]) > tol) {
                    reached = false;
                    RCLCPP_INFO (this->get_logger (), "Joint %s not reached: current=%.4f, command=%.4f, tol=%.4f", msg->name[i].c_str (), msg->position[i], command_joint_state_.position[j], tol);
                }
            }
        }
    }
    natto_msgs::msg::StateResult result;
    result.state_id    = joint_state_id_;
    result.success     = reached;
    result.action_name = "set_joint_position";
    state_result_publisher_->publish (result);
}

}  // namespace default_action
