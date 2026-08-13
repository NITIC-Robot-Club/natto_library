// Copyright 2026 Kazusa Hashimoto
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

#include "natto_joint_state/joint_state_merger.hpp"

namespace joint_state_merger {

joint_state_merger::joint_state_merger (const rclcpp::NodeOptions &node_options) : Node ("joint_state_merger", node_options) {
    merged_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState> ("merged_joint_states", rclcpp::SensorDataQoS ());

    std::vector<std::string> joint_state_topics = this->declare_parameter<std::vector<std::string>> ("joint_state_topics", {""});
    for (const auto &topic : joint_state_topics) {
        if (topic.empty ()) {
            RCLCPP_WARN (this->get_logger (), "Ignored empty joint_state topic entry.");
            continue;
        }
        joint_state_subscribers_.push_back (this->create_subscription<sensor_msgs::msg::JointState> (topic, rclcpp::SensorDataQoS (), std::bind (&joint_state_merger::joint_states_callback, this, std::placeholders::_1)));
    }
    RCLCPP_INFO (this->get_logger (), "joint_state_merger node has been initialized.");
    RCLCPP_INFO (this->get_logger (), "Subscribed topics:");
    for (const auto &topic : joint_state_topics) {
        if (!topic.empty ()) {
            RCLCPP_INFO (this->get_logger (), "  %s", topic.c_str ());
        }
    }
}

void joint_state_merger::joint_states_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->name.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Received empty JointState name list. Ignored.");
        return;
    }
    if (!msg->position.empty () && msg->position.size () != msg->name.size ()) {
        RCLCPP_WARN (this->get_logger (), "Received JointState with invalid position entries: name=%zu position=%zu. Ignored.", msg->name.size (), msg->position.size ());
        return;
    }
    if (!msg->velocity.empty () && msg->velocity.size () != msg->name.size ()) {
        RCLCPP_WARN (this->get_logger (), "Received JointState with invalid velocity entries: name=%zu velocity=%zu. Ignored.", msg->name.size (), msg->velocity.size ());
        return;
    }
    if (!msg->effort.empty () && msg->effort.size () != msg->name.size ()) {
        RCLCPP_WARN (this->get_logger (), "Received JointState with invalid effort entries: name=%zu effort=%zu. Ignored.", msg->name.size (), msg->effort.size ());
        return;
    }

    merged_joint_states_.header.stamp = this->now ();

    for (size_t i = 0; i < msg->name.size (); i++) {
        auto it = std::find (merged_joint_states_.name.begin (), merged_joint_states_.name.end (), msg->name[i]);
        if (it != merged_joint_states_.name.end ()) {
            size_t index = static_cast<size_t> (std::distance (merged_joint_states_.name.begin (), it));
            if (index >= merged_joint_states_.position.size ()) {
                merged_joint_states_.position.resize (merged_joint_states_.name.size (), 0.0);
            }
            if (index >= merged_joint_states_.velocity.size ()) {
                merged_joint_states_.velocity.resize (merged_joint_states_.name.size (), 0.0);
            }
            if (index >= merged_joint_states_.effort.size ()) {
                merged_joint_states_.effort.resize (merged_joint_states_.name.size (), 0.0);
            }
            if (!msg->position.empty ()) {
                merged_joint_states_.position[index] = msg->position[i];
            }
            if (!msg->velocity.empty ()) {
                merged_joint_states_.velocity[index] = msg->velocity[i];
            }
            if (!msg->effort.empty ()) {
                merged_joint_states_.effort[index] = msg->effort[i];
            }
        } else {
            merged_joint_states_.name.push_back (msg->name[i]);
            if (!msg->position.empty ()) {
                merged_joint_states_.position.push_back (msg->position[i]);
            } else {
                merged_joint_states_.position.push_back (0.0);
            }
            if (!msg->velocity.empty ()) {
                merged_joint_states_.velocity.push_back (msg->velocity[i]);
            } else {
                merged_joint_states_.velocity.push_back (0.0);
            }
            if (!msg->effort.empty ()) {
                merged_joint_states_.effort.push_back (msg->effort[i]);
            } else {
                merged_joint_states_.effort.push_back (0.0);
            }
        }
    }
    merged_joint_state_publisher_->publish (merged_joint_states_);
}

}  // namespace joint_state_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (joint_state_merger::joint_state_merger)
