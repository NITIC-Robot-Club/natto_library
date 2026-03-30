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

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

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
    button_joint_state_index_.resize (num_button_);
    button_joint_enabled_.resize (num_button_);

    std::vector<int>  joint_priority_by_index;
    std::vector<int>  joint_owner_by_index;
    std::vector<bool> joint_publish_always_by_index;

    const auto get_string_at = [] (const std::vector<std::string> &values, size_t index, const std::string &fallback) {
        if (values.empty ()) {
            return fallback;
        }
        if (index < values.size ()) {
            return values[index];
        }
        if (values.size () == 1) {
            return values[0];
        }
        return fallback;
    };

    const auto get_double_at = [] (const std::vector<double> &values, size_t index, double fallback) {
        if (values.empty ()) {
            return fallback;
        }
        if (index < values.size ()) {
            return values[index];
        }
        if (values.size () == 1) {
            return values[0];
        }
        return fallback;
    };

    const auto get_bool_at = [] (const std::vector<bool> &values, size_t index, bool fallback) {
        if (values.empty ()) {
            return fallback;
        }
        if (index < values.size ()) {
            return values[index];
        }
        if (values.size () == 1) {
            return values[0];
        }
        return fallback;
    };

    rcl_interfaces::msg::ParameterDescriptor dynamic_param_desc;
    dynamic_param_desc.dynamic_typing = true;

    const auto as_double_vector = [this] (const rclcpp::ParameterValue &param, const std::string &name, double fallback) {
        if (param.get_type () == rclcpp::PARAMETER_DOUBLE_ARRAY) {
            return param.get<std::vector<double>> ();
        }
        if (param.get_type () == rclcpp::PARAMETER_INTEGER_ARRAY) {
            std::vector<double> result;
            for (const auto value : param.get<std::vector<int64_t>> ()) {
                result.push_back (static_cast<double> (value));
            }
            return result;
        }
        if (param.get_type () == rclcpp::PARAMETER_DOUBLE) {
            return std::vector<double> {param.get<double> ()};
        }
        if (param.get_type () == rclcpp::PARAMETER_INTEGER) {
            return std::vector<double> {static_cast<double> (param.get<int64_t> ())};
        }
        RCLCPP_WARN (this->get_logger (), "'%s' has invalid type. fallback to default.", name.c_str ());
        return std::vector<double> {fallback};
    };

    const auto as_bool_vector = [this] (const rclcpp::ParameterValue &param, const std::string &name, bool fallback) {
        if (param.get_type () == rclcpp::PARAMETER_BOOL_ARRAY) {
            return param.get<std::vector<bool>> ();
        }
        if (param.get_type () == rclcpp::PARAMETER_BOOL) {
            return std::vector<bool> {param.get<bool> ()};
        }
        RCLCPP_WARN (this->get_logger (), "'%s' has invalid type. fallback to default.", name.c_str ());
        return std::vector<bool> {fallback};
    };

    for (size_t i = 0; i < num_button_; i++) {
        button_mode_.push_back (this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".mode", "none"));
        std::string button_function_legacy = this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".function", "none");
        auto        button_functions       = this->declare_parameter<std::vector<std::string>> ("button_" + std::to_string (i) + ".functions", std::vector<std::string> {});
        RCLCPP_INFO (this->get_logger (), "button_%zu.mode: %s", i, button_mode_[i].c_str ());

        std::string              joint_name_legacy    = this->declare_parameter<std::string> ("button_" + std::to_string (i) + ".joint_name", "");
        std::vector<std::string> button_joint_names   = this->declare_parameter<std::vector<std::string>> ("button_" + std::to_string (i) + ".joint_names", std::vector<std::string> {});
        auto                     position_on_param    = this->declare_parameter ("button_" + std::to_string (i) + ".position_on", rclcpp::ParameterValue (1.0), dynamic_param_desc);
        auto                     position_off_param   = this->declare_parameter ("button_" + std::to_string (i) + ".position_off", rclcpp::ParameterValue (0.0), dynamic_param_desc);
        auto                     speed_on_param       = this->declare_parameter ("button_" + std::to_string (i) + ".speed_on", rclcpp::ParameterValue (1.0), dynamic_param_desc);
        auto                     speed_off_param      = this->declare_parameter ("button_" + std::to_string (i) + ".speed_off", rclcpp::ParameterValue (0.0), dynamic_param_desc);
        auto                     publish_always_param = this->declare_parameter ("button_" + std::to_string (i) + ".publish_always", rclcpp::ParameterValue (false), dynamic_param_desc);

        std::vector<double> position_on_values     = as_double_vector (position_on_param, "button_" + std::to_string (i) + ".position_on", 1.0);
        std::vector<double> position_off_values    = as_double_vector (position_off_param, "button_" + std::to_string (i) + ".position_off", 0.0);
        std::vector<double> speed_on_values        = as_double_vector (speed_on_param, "button_" + std::to_string (i) + ".speed_on", 1.0);
        std::vector<double> speed_off_values       = as_double_vector (speed_off_param, "button_" + std::to_string (i) + ".speed_off", 0.0);
        std::vector<bool>   publish_always_values  = as_bool_vector (publish_always_param, "button_" + std::to_string (i) + ".publish_always", false);

        priority_.push_back (static_cast<int> (this->declare_parameter<int> ("button_" + std::to_string (i) + ".priority", 0)));

        if (button_mode_[i] != "toggle" && button_mode_[i] != "toggle_on" && button_mode_[i] != "toggle_off" && button_mode_[i] != "hold" && button_mode_[i] != "none" && button_mode_[i] != "positive_edge") {
            RCLCPP_WARN (this->get_logger (), "button_%zu.mode: '%s' is invalid. selected 'none'.", i, button_mode_[i].c_str ());
            RCLCPP_INFO (this->get_logger (), "Please set 'toggle' or 'toggle_on' or 'toggle_off' or 'hold' or 'none' or 'positive_edge'.");
            button_mode_[i] = "none";
        }

        if (button_functions.empty ()) {
            if (button_function_legacy == "get_origin" && !button_joint_names.empty ()) {
                for (const auto &jn : button_joint_names) {
                    if (!jn.empty ()) {
                        button_functions.push_back ("get_origin");
                    }
                }
            } else {
                button_functions.push_back (button_function_legacy);
            }
        }

        button_function_.push_back ({});
        joint_name_.push_back ({});
        position_on_.push_back ({});
        position_off_.push_back ({});
        speed_on_.push_back ({});
        speed_off_.push_back ({});
        publish_always_.push_back ({});
        button_joint_state_index_[i].clear ();
        button_joint_enabled_[i].clear ();
        last_toggle_state_.push_back ({});

        for (size_t entry = 0; entry < button_functions.size (); entry++) {
            auto function = button_functions[entry];
            auto jn       = get_string_at (button_joint_names, entry, joint_name_legacy);
            auto pos_on   = get_double_at (position_on_values, entry, 1.0);
            auto pos_off  = get_double_at (position_off_values, entry, 0.0);
            auto spd_on   = get_double_at (speed_on_values, entry, 1.0);
            auto spd_off  = get_double_at (speed_off_values, entry, 0.0);
            auto pub_alw  = get_bool_at (publish_always_values, entry, false);

            if (function != "none" && function != "power" && function != "allow_auto_drive" && function != "joint_position" && function != "joint_speed" && function != "get_origin") {
                RCLCPP_WARN (this->get_logger (), "button_%zu.functions[%zu]: '%s' is invalid. selected 'none'.", i, entry, function.c_str ());
                function = "none";
            }

            button_function_[i].push_back (function);
            joint_name_[i].push_back (jn);
            position_on_[i].push_back (pos_on);
            position_off_[i].push_back (pos_off);
            speed_on_[i].push_back (spd_on);
            speed_off_[i].push_back (spd_off);
            publish_always_[i].push_back (pub_alw);
            button_joint_state_index_[i].push_back (-1);
            button_joint_enabled_[i].push_back (true);
            last_toggle_state_[i].push_back (false);

            if (function == "joint_position" || function == "joint_speed") {
                if (jn.empty ()) {
                    RCLCPP_WARN (this->get_logger (), "button_%zu.joint_names[%zu] is empty. please set joint_names.", i, entry);
                    throw std::runtime_error ("invalid parameter");
                }
                RCLCPP_INFO (this->get_logger (), "button_%zu.functions[%zu]: %s", i, entry, function.c_str ());
                RCLCPP_INFO (this->get_logger (), "button_%zu.joint_names[%zu]: %s", i, entry, jn.c_str ());
                RCLCPP_INFO (this->get_logger (), "button_%zu.position_on[%zu]: %f", i, entry, pos_on);
                RCLCPP_INFO (this->get_logger (), "button_%zu.position_off[%zu]: %f", i, entry, pos_off);
                RCLCPP_INFO (this->get_logger (), "button_%zu.speed_on[%zu]: %f", i, entry, spd_on);
                RCLCPP_INFO (this->get_logger (), "button_%zu.speed_off[%zu]: %f", i, entry, spd_off);

                auto   it_existing  = std::find (command_joint_state_msg_.name.begin (), command_joint_state_msg_.name.end (), jn);
                bool   has_existing = (it_existing != command_joint_state_msg_.name.end ());
                size_t joint_index  = 0;

                if (!has_existing) {
                    joint_index = command_joint_state_msg_.name.size ();
                    command_joint_state_msg_.name.push_back (jn);
                    command_joint_state_msg_.position.push_back (pos_off);
                    command_joint_state_msg_.velocity.push_back (spd_off);
                    joint_priority_by_index.push_back (priority_[i]);
                    joint_owner_by_index.push_back (static_cast<int> (i));
                    joint_publish_always_by_index.push_back (pub_alw);
                    button_joint_state_index_[i][entry] = static_cast<int> (joint_index);
                    button_joint_enabled_[i][entry]     = true;
                } else {
                    joint_index                                = static_cast<size_t> (std::distance (command_joint_state_msg_.name.begin (), it_existing));
                    joint_priority_by_index[joint_index]       = std::max (joint_priority_by_index[joint_index], priority_[i]);
                    joint_publish_always_by_index[joint_index] = joint_publish_always_by_index[joint_index] || pub_alw;
                    button_joint_state_index_[i][entry]        = static_cast<int> (joint_index);
                    button_joint_enabled_[i][entry]            = true;

                    RCLCPP_INFO (this->get_logger (), "joint_name '%s' is duplicated. button_%zu entry_%zu shares same command slot and runtime priority arbitration is applied.", jn.c_str (), i, entry);
                }
            } else if (function == "get_origin") {
                if (jn.empty ()) {
                    RCLCPP_WARN (this->get_logger (), "button_%zu.joint_names[%zu] is empty. please set joint_names.", i, entry);
                    throw std::runtime_error ("invalid parameter");
                }
                RCLCPP_INFO (this->get_logger (), "button_%zu.get_origin target[%zu]: %s", i, entry, jn.c_str ());
            }
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
                joint_publish_always_by_index[static_cast<size_t> (zr_joint_state_index_)] = true;
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
                joint_publish_always_by_index[static_cast<size_t> (zl_joint_state_index_)] = true;
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
    const auto has_higher_priority_pressed = [this, &msg] (size_t button_index, size_t entry_index) {
        const auto &target_function = button_function_[button_index][entry_index];
        const auto &target_joint    = joint_name_[button_index][entry_index];
        for (size_t k = 0; k < num_button_; k++) {
            if (button_mode_[k] != "hold") {
                continue;
            }
            if (priority_[k] <= priority_[button_index]) {
                continue;
            }
            if (k < msg->buttons.size () && msg->buttons[k] == 1) {
                for (size_t e = 0; e < button_function_[k].size (); e++) {
                    if (k == button_index && e == entry_index) {
                        continue;
                    }
                    if (button_function_[k][e] != target_function) {
                        continue;
                    }
                    if (joint_name_[k][e] != target_joint) {
                        continue;
                    }
                    return true;
                }
            }
        }
        return false;
    };

    const auto has_any_other_pressed_same_joint = [this, &msg] (size_t button_index, size_t entry_index) {
        const auto &target_function = button_function_[button_index][entry_index];
        const auto &target_joint    = joint_name_[button_index][entry_index];
        for (size_t k = 0; k < num_button_; k++) {
            if (button_mode_[k] != "hold") {
                continue;
            }
            if (k < msg->buttons.size () && msg->buttons[k] == 1) {
                for (size_t e = 0; e < button_function_[k].size (); e++) {
                    if (k == button_index && e == entry_index) {
                        continue;
                    }
                    if (button_function_[k][e] != target_function) {
                        continue;
                    }
                    if (joint_name_[k][e] != target_joint) {
                        continue;
                    }
                    return true;
                }
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
        if (button_function_[i].empty ()) {
            continue;
        }

        for (size_t entry = 0; entry < button_function_[i].size (); entry++) {
            if (button_function_[i][entry] == "none") {
                continue;
            }

            if (button_function_[i][entry] == "power") {
                if (button_mode_[i] == "toggle") {
                    if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                        last_toggle_state_[i][entry] = !last_toggle_state_[i][entry];
                    }
                    power_msg_.data = last_toggle_state_[i][entry];
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
            } else if (button_function_[i][entry] == "allow_auto_drive") {
                if (button_mode_[i] == "toggle") {
                    if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                        last_toggle_state_[i][entry] = !last_toggle_state_[i][entry];
                    }
                    allow_auto_drive_msg_.data = last_toggle_state_[i][entry];
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
            } else if (button_function_[i][entry] == "joint_position") {
                if (!button_joint_enabled_[i][entry] || button_joint_state_index_[i][entry] < 0) {
                    continue;
                }
                command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
                size_t index                          = static_cast<size_t> (button_joint_state_index_[i][entry]);
                if (button_mode_[i] == "toggle") {
                    if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                        last_toggle_state_[i][entry] = !last_toggle_state_[i][entry];
                    }
                    if (last_toggle_state_[i][entry]) {
                        command_joint_state_msg_.position[index] = position_on_[i][entry];
                    } else {
                        command_joint_state_msg_.position[index] = position_off_[i][entry];
                    }
                } else if (button_mode_[i] == "toggle_on") {
                    if (msg->buttons[i] == 1) {
                        command_joint_state_msg_.position[index] = position_on_[i][entry];
                    }
                } else if (button_mode_[i] == "toggle_off") {
                    if (msg->buttons[i] == 1) {
                        command_joint_state_msg_.position[index] = position_off_[i][entry];
                    }
                } else if (button_mode_[i] == "hold") {
                    if (msg->buttons[i] == 1) {
                        if (has_higher_priority_pressed (i, entry)) {
                            continue;
                        }
                        command_joint_state_msg_.position[index] = position_on_[i][entry];
                    } else {
                        if (has_any_other_pressed_same_joint (i, entry)) {
                            continue;
                        }
                        command_joint_state_msg_.position[index] = position_off_[i][entry];
                    }
                }
            } else if (button_function_[i][entry] == "joint_speed") {
                if (!button_joint_enabled_[i][entry] || button_joint_state_index_[i][entry] < 0) {
                    continue;
                }
                command_joint_state_msg_.header.stamp = this->get_clock ()->now ();
                size_t index                          = static_cast<size_t> (button_joint_state_index_[i][entry]);
                if (button_mode_[i] == "toggle") {
                    if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                        last_toggle_state_[i][entry] = !last_toggle_state_[i][entry];
                    }
                    if (last_toggle_state_[i][entry]) {
                        command_joint_state_msg_.velocity[index] = speed_on_[i][entry];
                    } else {
                        command_joint_state_msg_.velocity[index] = speed_off_[i][entry];
                    }
                } else if (button_mode_[i] == "toggle_on") {
                    if (msg->buttons[i] == 1) {
                        command_joint_state_msg_.velocity[index] = speed_on_[i][entry];
                    }
                } else if (button_mode_[i] == "toggle_off") {
                    if (msg->buttons[i] == 1) {
                        command_joint_state_msg_.velocity[index] = speed_off_[i][entry];
                    }
                } else if (button_mode_[i] == "hold") {
                    if (msg->buttons[i] == 1) {
                        if (has_higher_priority_pressed (i, entry)) {
                            continue;
                        }
                        command_joint_state_msg_.velocity[index] = speed_on_[i][entry];
                    } else {
                        if (has_any_other_pressed_same_joint (i, entry)) {
                            continue;
                        }
                        command_joint_state_msg_.velocity[index] = speed_off_[i][entry];
                    }
                }
            } else if (button_function_[i][entry] == "get_origin") {
                if (button_mode_[i] == "positive_edge") {
                    if (msg->buttons[i] == 1 && last_button_state_[i] == 0) {
                        std_msgs::msg::String origin_get_msg;
                        origin_get_msg.data = joint_name_[i][entry];
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