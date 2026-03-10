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

#include "natto_joy/button_manager.hpp"

namespace button_manager {

button_manager::button_manager (const rclcpp::NodeOptions &node_options) : Node ("button_manager", node_options) {
    power_publisher_            = this->create_publisher<std_msgs::msg::Bool> ("power", 10);
    joint_state_publisher_      = this->create_publisher<sensor_msgs::msg::JointState> ("joint_states", rclcpp::SensorDataQoS ());
    allow_auto_drive_publisher_ = this->create_publisher<std_msgs::msg::Bool> ("allow_auto_drive", 10);
    origin_get_publisher_       = this->create_publisher<std_msgs::msg::String> ("get_origin_joint_name", 100);
    joy_subscriber_             = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 10, std::bind (&button_manager::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO (this->get_logger (), "button_manager node has been initialized.");
    num_button_ = static_cast<size_t> (this->declare_parameter<int> ("num_button", 0));
    RCLCPP_INFO (this->get_logger (), "num_button: %zu", num_button_);

    last_button_state_.resize (num_button_, 0);
    button_joint_state_index_.resize (num_button_, -1);
    button_joint_enabled_.resize (num_button_, true);

    std::vector<int>  joint_priority_by_index;
    std::vector<int>  joint_owner_by_index;
    std::vector<bool> joint_publish_always_by_index;

    for (size_t i = 0; i < num_button_; i++) {
        button_mode_.push_back (this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".mode", "none"));
        button_function_.push_back (this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".function", "none"));
        RCLCPP_INFO (this->get_logger (), "button_%zu.mode: %s", i, button_mode_[i].c_str ());

        joint_name_.push_back (this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".joint_name", ""));
        joint_names_.push_back (this->declare_parameter<std::vector<std::string>> ("button_" + std::to_string (i) + ".joint_names", {""}));
        position_on_.push_back (this->declare_parameter<double> ("button_" + std::to_string (i) + ".position_on", 1.0));
        position_off_.push_back (this->declare_parameter<double> ("button_" + std::to_string (i) + ".position_off", 0.0));
        speed_on_.push_back (this->declare_parameter<double> ("button_" + std::to_string (i) + ".speed_on", 1.0));
        speed_off_.push_back (this->declare_parameter<double> ("button_" + std::to_string (i) + ".speed_off", 0.0));
        publish_always_.push_back (this->declare_parameter<bool> ("button_" + std::to_string (i) + ".publish_always", false));
        priority_.push_back (this->declare_parameter<int> ("button_" + std::to_string (i) + ".priority", 0));

        if (button_mode_[i] != "toggle" && button_mode_[i] != "toggle_on" && button_mode_[i] != "toggle_off" && button_mode_[i] != "hold" && button_mode_[i] != "none" && button_mode_[i] != "positive_edge") {
            RCLCPP_WARN (this->get_logger (), "button_%zu.mode: '%s' is invalid. selected 'none'.", i, button_mode_[i].c_str ());
            RCLCPP_INFO (this->get_logger (), "Please set 'toggle' or 'toggle_on' or 'toggle_off' or 'hold' or 'none' or 'positive_edge'.");
            button_mode_[i] = "none";
        }

        if (button_function_[i] == "none") {
        } else if (button_function_[i] == "power") {
        } else if (button_function_[i] == "allow_auto_drive") {
        } else if (button_function_[i] == "joint_position" || button_function_[i] == "joint_speed") {
            if (joint_name_[i] == "") {
                RCLCPP_WARN (this->get_logger (), "button_%zu.joint_name is empty. please set joint_name.", i);
                throw std::runtime_error ("invalid parameter");
            }
            RCLCPP_INFO (this->get_logger (), "button_%zu.joint_name: %s", i, joint_name_[i].c_str ());
            RCLCPP_INFO (this->get_logger (), "button_%zu.position_on: %f", i, position_on_[i]);
            RCLCPP_INFO (this->get_logger (), "button_%zu.position_off: %f", i, position_off_[i]);
            RCLCPP_INFO (this->get_logger (), "button_%zu.speed_on: %f", i, speed_on_[i]);
            RCLCPP_INFO (this->get_logger (), "button_%zu.speed_off: %f", i, speed_off_[i]);

            auto   it_existing  = std::find (command_joint_state_msg_.name.begin (), command_joint_state_msg_.name.end (), joint_name_[i]);
            bool   has_existing = (it_existing != command_joint_state_msg_.name.end ());
            size_t joint_index  = 0;

            if (!has_existing) {
                joint_index = command_joint_state_msg_.name.size ();
                command_joint_state_msg_.name.push_back (joint_name_[i]);
                command_joint_state_msg_.position.push_back (position_off_[i]);
                command_joint_state_msg_.velocity.push_back (speed_off_[i]);
                joint_priority_by_index.push_back (priority_[i]);
                joint_owner_by_index.push_back (static_cast<int> (i));
                joint_publish_always_by_index.push_back (publish_always_[i]);
                button_joint_state_index_[i] = static_cast<int> (joint_index);
                button_joint_enabled_[i]     = true;
            } else {
                joint_index                                = std::distance (command_joint_state_msg_.name.begin (), it_existing);
                joint_priority_by_index[joint_index]       = std::max (joint_priority_by_index[joint_index], priority_[i]);
                joint_publish_always_by_index[joint_index] = joint_publish_always_by_index[joint_index] || publish_always_[i];
                button_joint_state_index_[i]               = static_cast<int> (joint_index);
                button_joint_enabled_[i]                   = true;

                RCLCPP_INFO (this->get_logger (), "joint_name '%s' is duplicated. button_%zu shares same command slot and runtime priority arbitration is applied.", joint_name_[i].c_str (), i);
            }
        } else if (button_function_[i] == "get_origin") {
            if (joint_names_[i].empty ()) {
                RCLCPP_WARN (this->get_logger (), "button_%zu.joint_names is empty. please set joint_names.", i);
                throw std::runtime_error ("invalid parameter");
            }
            RCLCPP_INFO (this->get_logger (), "button_%zu.joint_names:", i);
            for (const auto &jn : joint_names_[i]) {
                RCLCPP_INFO (this->get_logger (), "  - %s", jn.c_str ());
            }
        } else {
            button_function_[i] = "none";
            RCLCPP_WARN (this->get_logger (), "button_%zu.function: %s is invalid. selected 'none'.", i, button_function_[i].c_str ());
            RCLCPP_INFO (this->get_logger (), "Please set 'none', 'power', 'allow_auto_drive', 'joint_position' or 'joint_speed'.");
        }
        last_toggle_state_.push_back (false);
    }

    zr_mode_           = this->declare_parameter<std::string> ("zr.mode", "none");
    zr_function_       = this->declare_parameter<std::string> ("zr.function", "none");
    zl_mode_           = this->declare_parameter<std::string> ("zl.mode", "none");
    zl_function_       = this->declare_parameter<std::string> ("zl.function", "none");
    zr_joint_name_     = this->declare_parameter<std::string> ("zr.joint_name", "");
    zl_joint_name_     = this->declare_parameter<std::string> ("zl.joint_name", "");
    zr_position_on_    = this->declare_parameter<double> ("zr.position_on", 1.0);
    zr_position_off_   = this->declare_parameter<double> ("zr.position_off", 0.0);
    zl_position_on_    = this->declare_parameter<double> ("zl.position_on", 1.0);
    zl_position_off_   = this->declare_parameter<double> ("zl.position_off", 0.0);
    zr_speed_on_       = this->declare_parameter<double> ("zr.speed_on", 1.0);
    zr_speed_off_      = this->declare_parameter<double> ("zr.speed_off", 0.0);
    zl_speed_on_       = this->declare_parameter<double> ("zl.speed_on", 1.0);
    zl_speed_off_      = this->declare_parameter<double> ("zl.speed_off", 0.0);
    zr_publish_always_ = this->declare_parameter<bool> ("zr.publish_always", false);
    zl_publish_always_ = this->declare_parameter<bool> ("zl.publish_always", false);

    RCLCPP_INFO (this->get_logger (), "zr.mode: %s", zr_mode_.c_str ());
    RCLCPP_INFO (this->get_logger (), "zr.function: %s", zr_function_.c_str ());
    if (zr_function_ == "joint_position" || zr_function_ == "joint_speed") {
        if (zr_joint_name_ == "") {
            RCLCPP_WARN (this->get_logger (), "zr.joint_name is empty. please set joint_name.");
            throw std::runtime_error ("invalid parameter");
        }
        RCLCPP_INFO (this->get_logger (), "zr.joint_name: %s", zr_joint_name_.c_str ());
        RCLCPP_INFO (this->get_logger (), "zr.position_on: %f", zr_position_on_);
        RCLCPP_INFO (this->get_logger (), "zr.position_off: %f", zr_position_off_);
        RCLCPP_INFO (this->get_logger (), "zr.speed_on: %f", zr_speed_on_);
        RCLCPP_INFO (this->get_logger (), "zr.speed_off: %f", zr_speed_off_);
        auto it_existing = std::find (command_joint_state_msg_.name.begin (), command_joint_state_msg_.name.end (), zr_joint_name_);
        if (it_existing == command_joint_state_msg_.name.end ()) {
            zr_joint_state_index_ = static_cast<int> (command_joint_state_msg_.name.size ());
            command_joint_state_msg_.name.push_back (zr_joint_name_);
            command_joint_state_msg_.position.push_back (zr_position_off_);
            command_joint_state_msg_.velocity.push_back (zr_speed_off_);
            joint_priority_by_index.push_back (std::numeric_limits<int>::min ());
            joint_owner_by_index.push_back (-1);
            joint_publish_always_by_index.push_back (zr_publish_always_);
        } else {
            zr_joint_state_index_ = static_cast<int> (std::distance (command_joint_state_msg_.name.begin (), it_existing));
            if (zr_publish_always_) {
                joint_publish_always_by_index[zr_joint_state_index_] = true;
            }
        }
    }
    RCLCPP_INFO (this->get_logger (), "zl.mode: %s", zl_mode_.c_str ());
    RCLCPP_INFO (this->get_logger (), "zl.function: %s", zl_function_.c_str ());
    if (zl_function_ == "joint_position" || zl_function_ == "joint_speed") {
        if (zl_joint_name_ == "") {
            RCLCPP_WARN (this->get_logger (), "zl.joint_name is empty. please set joint_name.");
            throw std::runtime_error ("invalid parameter");
        }
        RCLCPP_INFO (this->get_logger (), "zl.joint_name: %s", zl_joint_name_.c_str ());
        RCLCPP_INFO (this->get_logger (), "zl.position_on: %f", zl_position_on_);
        RCLCPP_INFO (this->get_logger (), "zl.position_off: %f", zl_position_off_);
        RCLCPP_INFO (this->get_logger (), "zl.speed_on: %f", zl_speed_on_);
        RCLCPP_INFO (this->get_logger (), "zl.speed_off: %f", zl_speed_off_);
        auto it_existing = std::find (command_joint_state_msg_.name.begin (), command_joint_state_msg_.name.end (), zl_joint_name_);
        if (it_existing == command_joint_state_msg_.name.end ()) {
            zl_joint_state_index_ = static_cast<int> (command_joint_state_msg_.name.size ());
            command_joint_state_msg_.name.push_back (zl_joint_name_);
            command_joint_state_msg_.position.push_back (zl_position_off_);
            command_joint_state_msg_.velocity.push_back (zl_speed_off_);
            joint_priority_by_index.push_back (std::numeric_limits<int>::min ());
            joint_owner_by_index.push_back (-1);
            joint_publish_always_by_index.push_back (zl_publish_always_);
        } else {
            zl_joint_state_index_ = static_cast<int> (std::distance (command_joint_state_msg_.name.begin (), it_existing));
            if (zl_publish_always_) {
                joint_publish_always_by_index[zl_joint_state_index_] = true;
            }
        }
    }

    for (size_t i = 0; i < command_joint_state_msg_.name.size (); i++) {
        if (joint_publish_always_by_index[i]) {
            command_joint_state_always_msg_.name.push_back (command_joint_state_msg_.name[i]);
            command_joint_state_always_msg_.position.push_back (command_joint_state_msg_.position[i]);
            command_joint_state_always_msg_.velocity.push_back (command_joint_state_msg_.velocity[i]);
        }
    }
}

void button_manager::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
    const auto has_higher_priority_pressed = [this, &msg] (size_t button_index) {
        for (size_t k = 0; k < num_button_; k++) {
            if (k == button_index) {
                continue;
            }
            if (button_function_[k] != button_function_[button_index]) {
                continue;
            }
            if (button_mode_[k] != "hold") {
                continue;
            }
            if (joint_name_[k] != joint_name_[button_index]) {
                continue;
            }
            if (priority_[k] <= priority_[button_index]) {
                continue;
            }
            if (k < msg->buttons.size () && msg->buttons[k] == 1) {
                return true;
            }
        }
        return false;
    };

    const auto has_any_other_pressed_same_joint = [this, &msg] (size_t button_index) {
        for (size_t k = 0; k < num_button_; k++) {
            if (k == button_index) {
                continue;
            }
            if (button_function_[k] != button_function_[button_index]) {
                continue;
            }
            if (button_mode_[k] != "hold") {
                continue;
            }
            if (joint_name_[k] != joint_name_[button_index]) {
                continue;
            }
            if (k < msg->buttons.size () && msg->buttons[k] == 1) {
                return true;
            }
        }
        return false;
    };

    for (size_t i = 0; i < num_button_; i++) {
        if (i >= msg->buttons.size ()) {
            RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "Received joy message has no button_%zu", i);
            continue;
        }
        if (button_mode_[i] == "none") {
            continue;
        }
        if (button_function_[i] == "none") {
            continue;
        }

        if (button_function_[i] == "power") {
            if (button_mode_[i] == "toggle") {
                if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                    last_toggle_state_[i] = !last_toggle_state_[i];
                }
                power_msg_.data = last_toggle_state_[i];
            } else if (button_mode_[i] == "toggle_on") {
                if (msg->buttons[i] == 1) {
                    power_msg_.data = true;
                }
            } else if (button_mode_[i] == "toggle_off") {
                if (msg->buttons[i] == 1) {
                    power_msg_.data = false;
                }
            } else if (button_mode_[i] == "hold") {
                power_msg_.data = msg->buttons[i];
            }
        } else if (button_function_[i] == "allow_auto_drive") {
            if (button_mode_[i] == "toggle") {
                if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                    last_toggle_state_[i] = !last_toggle_state_[i];
                }
                allow_auto_drive_msg_.data = last_toggle_state_[i];
            } else if (button_mode_[i] == "toggle_on") {
                if (msg->buttons[i] == 1) {
                    allow_auto_drive_msg_.data = true;
                }
            } else if (button_mode_[i] == "toggle_off") {
                if (msg->buttons[i] == 1) {
                    allow_auto_drive_msg_.data = false;
                }
            } else if (button_mode_[i] == "hold") {
                allow_auto_drive_msg_.data = (msg->buttons[i] == 1);
            }
        } else if (button_function_[i] == "joint_position") {
            if (!button_joint_enabled_[i] || button_joint_state_index_[i] < 0) {
                continue;
            }
            command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
            size_t index                          = static_cast<size_t> (button_joint_state_index_[i]);
            if (button_mode_[i] == "toggle") {
                if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                    last_toggle_state_[i] = !last_toggle_state_[i];
                }
                if (last_toggle_state_[i]) {
                    command_joint_state_msg_.position[index] = position_on_[i];
                } else {
                    command_joint_state_msg_.position[index] = position_off_[i];
                }
            } else if (button_mode_[i] == "toggle_on") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.position[index] = position_on_[i];
                }
            } else if (button_mode_[i] == "toggle_off") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.position[index] = position_off_[i];
                }
            } else if (button_mode_[i] == "hold") {
                if (msg->buttons[i] == 1) {
                    if (has_higher_priority_pressed (i)) {
                        last_button_state_[i] = msg->buttons[i];
                        continue;
                    }
                    command_joint_state_msg_.position[index] = position_on_[i];
                } else {
                    if (has_any_other_pressed_same_joint (i)) {
                        last_button_state_[i] = msg->buttons[i];
                        continue;
                    }
                    command_joint_state_msg_.position[index] = position_off_[i];
                }
            }
        } else if (button_function_[i] == "joint_speed") {
            if (!button_joint_enabled_[i] || button_joint_state_index_[i] < 0) {
                continue;
            }
            command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
            size_t index                          = static_cast<size_t> (button_joint_state_index_[i]);
            if (button_mode_[i] == "toggle") {
                if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                    last_toggle_state_[i] = !last_toggle_state_[i];
                }
                if (last_toggle_state_[i]) {
                    command_joint_state_msg_.velocity[index] = speed_on_[i];
                } else {
                    command_joint_state_msg_.velocity[index] = speed_off_[i];
                }
            } else if (button_mode_[i] == "toggle_on") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.velocity[index] = speed_on_[i];
                }
            } else if (button_mode_[i] == "toggle_off") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.velocity[index] = speed_off_[i];
                }
            } else if (button_mode_[i] == "hold") {
                if (msg->buttons[i] == 1) {
                    if (has_higher_priority_pressed (i)) {
                        last_button_state_[i] = msg->buttons[i];
                        continue;
                    }
                    command_joint_state_msg_.velocity[index] = speed_on_[i];
                } else {
                    if (has_any_other_pressed_same_joint (i)) {
                        last_button_state_[i] = msg->buttons[i];
                        continue;
                    }
                    command_joint_state_msg_.velocity[index] = speed_off_[i];
                }
            }
        } else if (button_function_[i] == "get_origin") {
            if (button_mode_[i] == "positive_edge") {
                if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                    for (const auto &joint_name : joint_names_[i]) {
                        std_msgs::msg::String origin_get_msg;
                        origin_get_msg.data = joint_name;
                        origin_get_publisher_->publish (origin_get_msg);
                    }
                }
            }
        }
        last_button_state_[i] = msg->buttons[i];
    }
    if (zl_function_ != "none" && zl_mode_ != "none") {
        if (msg->axes.size () > 4 && zl_joint_state_index_ >= 0) {
            if (zl_function_ == "joint_position") {
                command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
                size_t index                          = static_cast<size_t> (zl_joint_state_index_);
                if (zl_mode_ == "toggle_on") {
                    if (msg->axes[4] < 0.0) {
                        command_joint_state_msg_.position[index] = zl_position_on_;
                    }
                } else if (zl_mode_ == "toggle_off") {
                    if (msg->axes[4] < 0.0) {
                        command_joint_state_msg_.position[index] = zl_position_off_;
                    }
                } else if (zl_mode_ == "hold") {
                    command_joint_state_msg_.position[index] = -msg->axes[4] * (zl_position_on_ - zl_position_off_) + zl_position_off_;
                }
            } else if (zl_function_ == "joint_speed") {
                command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
                size_t index                          = static_cast<size_t> (zl_joint_state_index_);
                if (zl_mode_ == "toggle_on") {
                    if (msg->axes[4] < 0.0) {
                        command_joint_state_msg_.velocity[index] = zl_speed_on_;
                    }
                } else if (zl_mode_ == "toggle_off") {
                    if (msg->axes[4] < 0.0) {
                        command_joint_state_msg_.velocity[index] = zl_speed_off_;
                    }
                } else if (zl_mode_ == "hold") {
                    command_joint_state_msg_.velocity[index] = -msg->axes[4] * (zl_speed_on_ - zl_speed_off_) + zl_speed_off_;
                }
            }
        }
    }
    if (zr_function_ != "none" && zr_mode_ != "none") {
        if (msg->axes.size () > 5 && zr_joint_state_index_ >= 0) {
            if (zr_function_ == "joint_position") {
                command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
                size_t index                          = static_cast<size_t> (zr_joint_state_index_);
                if (zr_mode_ == "toggle_on") {
                    if (msg->axes[5] < 0.0) {
                        command_joint_state_msg_.position[index] = zr_position_on_;
                    }
                } else if (zr_mode_ == "toggle_off") {
                    if (msg->axes[5] < 0.0) {
                        command_joint_state_msg_.position[index] = zr_position_off_;
                    }
                } else if (zr_mode_ == "hold") {
                    command_joint_state_msg_.position[index] = -msg->axes[5] * (zr_position_on_ - zr_position_off_) + zr_position_off_;
                }
            } else if (zr_function_ == "joint_speed") {
                command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
                size_t index                          = static_cast<size_t> (zr_joint_state_index_);
                if (zr_mode_ == "toggle_on") {
                    if (msg->axes[5] < 0.0) {
                        command_joint_state_msg_.velocity[index] = zr_speed_on_;
                    }
                } else if (zr_mode_ == "toggle_off") {
                    if (msg->axes[5] < 0.0) {
                        command_joint_state_msg_.velocity[index] = zr_speed_off_;
                    }
                } else if (zr_mode_ == "hold") {
                    command_joint_state_msg_.velocity[index] = -msg->axes[5] * (zr_speed_on_ - zr_speed_off_) + zr_speed_off_;
                }
            }
        }
    }
    power_publisher_->publish (power_msg_);
    allow_auto_drive_publisher_->publish (allow_auto_drive_msg_);
    if (!allow_auto_drive_msg_.data) {
        joint_state_publisher_->publish (command_joint_state_msg_);
    } else {
        for (size_t i = 0; i < command_joint_state_always_msg_.name.size (); i++) {
            size_t index = 0;
            for (size_t j = 0; j < command_joint_state_msg_.name.size (); j++) {
                if (command_joint_state_msg_.name[j] == command_joint_state_always_msg_.name[i]) {
                    index = j;
                    break;
                }
            }
            command_joint_state_always_msg_.position[i] = command_joint_state_msg_.position[index];
            command_joint_state_always_msg_.velocity[i] = command_joint_state_msg_.velocity[index];
        }
        joint_state_publisher_->publish (command_joint_state_always_msg_);
    }
}

}  // namespace button_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (button_manager::button_manager)