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

#ifndef __CANABLE_HPP__
#define __CANABLE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "natto_msgs/msg/can.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>

namespace canable {
class canable : public rclcpp::Node {
   public:
    canable (const rclcpp::NodeOptions &node_options);
    ~canable ();

   private:
    int  init_can_socket ();                                    // Initialize CAN socket
    void read_can_socket ();                                    // Read messages from CAN socket
    void write_can_socket (const natto_msgs::msg::Can &frame);  // Write messages to CAN socket
    bool retry_open_can        = true;
    bool retry_write_can       = true;
    int  retry_write_count     = 0;
    int  max_retry_write_count = 5;
    int  can_socket_;

    std::string         can_interface;
    struct sockaddr_can addr_;
    struct ifreq        ifr_;

    rclcpp::Publisher<natto_msgs::msg::Can>::SharedPtr    canable_pub_;
    rclcpp::Subscription<natto_msgs::msg::Can>::SharedPtr canable_sub_;
};
}  // namespace canable

#endif  // __CANABLE_HPP__