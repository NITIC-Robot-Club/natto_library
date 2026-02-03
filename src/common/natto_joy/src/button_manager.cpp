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
    origin_get_publisher_       = this->create_publisher<std_msgs::msg::String> ("get_origin_joint_name", 10);
    joy_subscriber_             = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 10, std::bind (&button_manager::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO (this->get_logger (), "button_manager node has been initialized.");
    num_button_ = static_cast<size_t> (this->declare_parameter<int> ("num_button", 0));
    RCLCPP_INFO (this->get_logger (), "num_button: %zu", num_button_);

    last_button_state_.resize (num_button_, 0);

    for (size_t i = 0; i < num_button_; i++) {
        button_mode_.push_back (this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".mode", "toggle"));
        button_function_.push_back (this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".function", "none"));
        RCLCPP_INFO (this->get_logger (), "button_%zu.mode: %s", i, button_mode_[i].c_str ());

        joint_name_.push_back (this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".joint_name", ""));
        position_on_.push_back (this->declare_parameter<double> ("button_" + std::to_string (i) + ".position_on", 1.0));
        position_off_.push_back (this->declare_parameter<double> ("button_" + std::to_string (i) + ".position_off", 0.0));
        speed_on_.push_back (this->declare_parameter<double> ("button_" + std::to_string (i) + ".speed_on", 1.0));
        speed_off_.push_back (this->declare_parameter<double> ("button_" + std::to_string (i) + ".speed_off", 0.0));
        publish_always_.push_back (this->declare_parameter<bool> ("button_" + std::to_string (i) + ".publish_always", false));

        if (button_mode_[i] != "toggle_on" && button_mode_[i] != "toggle_off" && button_mode_[i] != "hold" && button_mode_[i] != "none" && button_mode_[i] != "click") {
            button_mode_[i] = "none";
            RCLCPP_ERROR (this->get_logger (), "button_%zu.mode: %s is invalid. selected 'none'.", i, button_mode_[i].c_str ());
            RCLCPP_INFO (this->get_logger (), "Please set 'toggle_on' or 'toggle_off' or 'hold' or 'none' or 'click'.");
        }

        if (button_function_[i] == "none") {
        } else if (button_function_[i] == "power") {
        } else if (button_function_[i] == "allow_auto_drive") {
        } else if (button_function_[i] == "joint_position" || button_function_[i] == "joint_speed") {
            if (joint_name_[i] == "") {
                RCLCPP_ERROR (this->get_logger (), "button_%zu.joint_name is empty. please set joint_name.", i);
                throw std::runtime_error ("invalid parameter");
            }
            RCLCPP_INFO (this->get_logger (), "button_%zu.joint_name: %s", i, joint_name_[i].c_str ());
            RCLCPP_INFO (this->get_logger (), "button_%zu.position_on: %f", i, position_on_[i]);
            RCLCPP_INFO (this->get_logger (), "button_%zu.position_off: %f", i, position_off_[i]);
            RCLCPP_INFO (this->get_logger (), "button_%zu.speed_on: %f", i, speed_on_[i]);
            RCLCPP_INFO (this->get_logger (), "button_%zu.speed_off: %f", i, speed_off_[i]);
            command_joint_state_msg_.name.push_back (joint_name_[i]);
            command_joint_state_msg_.position.push_back (position_off_[i]);
            command_joint_state_msg_.velocity.push_back (speed_off_[i]);
            if (publish_always_[i]) {
                command_joint_state_always_msg_.name.push_back (joint_name_[i]);
                command_joint_state_always_msg_.position.push_back (position_off_[i]);
                command_joint_state_always_msg_.velocity.push_back (speed_off_[i]);
            }
        } else if (button_function_[i] == "get_origin") {
        } else {
            button_function_[i] = "none";
            RCLCPP_ERROR (this->get_logger (), "button_%zu.function: %s is invalid. selected 'none'.", i, button_function_[i].c_str ());
            RCLCPP_INFO (this->get_logger (), "Please set 'none', 'power', 'allow_auto_drive', 'joint_position' or 'joint_speed'.");
        }
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
            RCLCPP_ERROR (this->get_logger (), "zr.joint_name is empty. please set joint_name.");
            throw std::runtime_error ("invalid parameter");
        }
        RCLCPP_INFO (this->get_logger (), "zr.joint_name: %s", zr_joint_name_.c_str ());
        RCLCPP_INFO (this->get_logger (), "zr.position_on: %f", zr_position_on_);
        RCLCPP_INFO (this->get_logger (), "zr.position_off: %f", zr_position_off_);
        RCLCPP_INFO (this->get_logger (), "zr.speed_on: %f", zr_speed_on_);
        RCLCPP_INFO (this->get_logger (), "zr.speed_off: %f", zr_speed_off_);
        command_joint_state_msg_.name.push_back (zr_joint_name_);
        command_joint_state_msg_.position.push_back (zr_position_off_);
        command_joint_state_msg_.velocity.push_back (zr_speed_off_);
        if (zr_publish_always_) {
            command_joint_state_always_msg_.name.push_back (zr_joint_name_);
            command_joint_state_always_msg_.position.push_back (zr_position_off_);
            command_joint_state_always_msg_.velocity.push_back (zr_speed_off_);
        }
    }
    RCLCPP_INFO (this->get_logger (), "zl.mode: %s", zl_mode_.c_str ());
    RCLCPP_INFO (this->get_logger (), "zl.function: %s", zl_function_.c_str ());
    if (zl_function_ == "joint_position" || zl_function_ == "joint_speed") {
        if (zl_joint_name_ == "") {
            RCLCPP_ERROR (this->get_logger (), "zl.joint_name is empty. please set joint_name.");
            throw std::runtime_error ("invalid parameter");
        }
        RCLCPP_INFO (this->get_logger (), "zl.joint_name: %s", zl_joint_name_.c_str ());
        RCLCPP_INFO (this->get_logger (), "zl.position_on: %f", zl_position_on_);
        RCLCPP_INFO (this->get_logger (), "zl.position_off: %f", zl_position_off_);
        RCLCPP_INFO (this->get_logger (), "zl.speed_on: %f", zl_speed_on_);
        RCLCPP_INFO (this->get_logger (), "zl.speed_off: %f", zl_speed_off_);
        command_joint_state_msg_.name.push_back (zl_joint_name_);
        command_joint_state_msg_.position.push_back (zl_position_off_);
        command_joint_state_msg_.velocity.push_back (zl_speed_off_);
        if (zl_publish_always_) {
            command_joint_state_always_msg_.name.push_back (zl_joint_name_);
            command_joint_state_always_msg_.position.push_back (zl_position_off_);
            command_joint_state_always_msg_.velocity.push_back (zl_speed_off_);
        }
    }
}

void button_manager::joy_callback (const sensor_msgs::msg::Joy::SharedPtr msg) {
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
            if (button_mode_[i] == "toggle_on") {
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
            if (button_mode_[i] == "toggle_on") {
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
            command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
            size_t index                          = 0;
            for (size_t j = 0; j < i; j++) {
                if (command_joint_state_msg_.name[j] == joint_name_[i]) {
                    index = j;
                    break;
                }
            }
            if (button_mode_[i] == "toggle_on") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.position[index] = position_on_[i];
                }
            } else if (button_mode_[i] == "toggle_off") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.position[index] = position_off_[i];
                }
            } else if (button_mode_[i] == "hold") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.position[index] = position_on_[i];
                } else {
                    command_joint_state_msg_.position[index] = position_off_[i];
                }
            }
        } else if (button_function_[i] == "joint_speed") {
            command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
            size_t index                          = 0;
            for (size_t j = 0; j < i; j++) {
                if (command_joint_state_msg_.name[j] == joint_name_[i]) {
                    index = j;
                    break;
                }
            }
            if (button_mode_[i] == "toggle_on") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.velocity[index] = speed_on_[i];
                }
            } else if (button_mode_[i] == "toggle_off") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.velocity[index] = speed_off_[i];
                }
            } else if (button_mode_[i] == "hold") {
                if (msg->buttons[i] == 1) {
                    command_joint_state_msg_.velocity[index] = speed_on_[i];
                } else {
                    command_joint_state_msg_.velocity[index] = speed_off_[i];
                }
            }
        } else if (button_function_[i] == "get_origin") {
            if (button_mode_[i] == "click") {
                if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                    std_msgs::msg::String origin_get_msg;
                    origin_get_msg.data = joint_name_[i];
                    origin_get_publisher_->publish (origin_get_msg);
                }
            }
        }
        last_button_state_[i] = msg->buttons[i];
    }
    if (zl_function_ != "none" && zl_mode_ != "none") {
        if (zl_function_ == "joint_position") {
            command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
            size_t index                          = 0;
            for (size_t j = 0; j < command_joint_state_msg_.name.size (); j++) {
                if (command_joint_state_msg_.name[j] == zl_joint_name_) {
                    index = j;
                    break;
                }
            }
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
            size_t index                          = 0;
            for (size_t j = 0; j < command_joint_state_msg_.name.size (); j++) {
                if (command_joint_state_msg_.name[j] == zl_joint_name_) {
                    index = j;
                    break;
                }
            }
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
    if (zr_function_ != "none" && zr_mode_ != "none") {
        if (zr_function_ == "joint_position") {
            command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
            size_t index                          = 0;
            for (size_t j = 0; j < command_joint_state_msg_.name.size (); j++) {
                if (command_joint_state_msg_.name[j] == zr_joint_name_) {
                    index = j;
                    break;
                }
            }
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
            size_t index                          = 0;
            for (size_t j = 0; j < command_joint_state_msg_.name.size (); j++) {
                if (command_joint_state_msg_.name[j] == zr_joint_name_) {
                    index = j;
                    break;
                }
            }
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