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

#ifndef __SPEED_PATH_LOADER_HPP__
#define __SPEED_PATH_LOADER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/speed_path.hpp"
#include "natto_msgs/msg/state_action.hpp"
#include "natto_msgs/msg/state_result.hpp"
#include "std_msgs/msg/bool.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace speed_path_loader {
class speed_path_loader : public rclcpp::Node {
   public:
    speed_path_loader (const rclcpp::NodeOptions &node_options);

   private:
    std::string                file_directory_;
    natto_msgs::msg::SpeedPath speed_path_;

    uint64_t set_speed_path_state_id_;
    bool     goal_reached_;

    void state_action_callback (const natto_msgs::msg::StateAction::SharedPtr msg);
    void goal_reached_callback (const std_msgs::msg::Bool::SharedPtr msg);
    void load_speed_path (std::string file_path);
    void timer_callback ();

    rclcpp::Publisher<natto_msgs::msg::SpeedPath>::SharedPtr      speed_path_publisher_;
    rclcpp::Publisher<natto_msgs::msg::StateResult>::SharedPtr    state_result_publisher_;
    rclcpp::Subscription<natto_msgs::msg::StateAction>::SharedPtr state_action_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          goal_reached_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
};
}  // namespace speed_path_loader

#endif  // __SPEED_PATH_LOADER_HPP__