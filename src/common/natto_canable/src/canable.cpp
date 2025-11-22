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

#include "natto_canable/canable.hpp"

namespace canable {

canable::canable (const rclcpp::NodeOptions &node_options) : Node ("canable", node_options) {
    can_interface         = this->declare_parameter<std::string> ("can_interface", "can0");
    retry_open_can        = this->declare_parameter ("retry_open_can", true);
    retry_write_can       = this->declare_parameter ("retry_write_can", true);
    max_retry_write_count = this->declare_parameter ("max_retry_write_count", 5);
    use_fd_               = this->declare_parameter ("use_fd", false);

    if (init_can_socket () != 0) {
        RCLCPP_FATAL (get_logger (), "Failed to initialize CAN socket");
        throw std::runtime_error ("CAN socket init failed");
    }

    canable_pub_ = this->create_publisher<natto_msgs::msg::Can> ("receive", 10);
    canable_sub_ = this->create_subscription<natto_msgs::msg::Can> ("transmit", 10, [this] (const natto_msgs::msg::Can::SharedPtr msg) { write_can_socket (*msg); });

    std::thread (&canable::read_can_socket, this).detach ();
}

canable::~canable () {
    if (can_socket_ >= 0) {
        close (can_socket_);
    }
}

int canable::init_can_socket () {
    while (rclcpp::ok ()) {
        can_socket_ = socket (PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR (get_logger (), "Failed to create CAN socket");
            if (!retry_open_can) return -1;
            std::this_thread::sleep_for (std::chrono::milliseconds (500));
            continue;
        }

        if (use_fd_) {
            int enable_canfd = 1;
            if (setsockopt (can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof (enable_canfd)) < 0) {
                RCLCPP_ERROR (get_logger (), "Failed to enable CAN FD mode");
                close (can_socket_);
                if (!retry_open_can) return -1;
                std::this_thread::sleep_for (std::chrono::milliseconds (500));
                continue;
            }
            RCLCPP_INFO (get_logger (), "FDCAN mode enabled");
        }

        std::strncpy (ifr_.ifr_name, can_interface.c_str (), IFNAMSIZ - 1);
        ifr_.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl (can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
            RCLCPP_ERROR (get_logger (), "Failed to get interface index for %s", can_interface.c_str ());
            close (can_socket_);
            if (!retry_open_can) return -1;
            std::this_thread::sleep_for (std::chrono::milliseconds (500));
            continue;
        }

        addr_.can_family  = AF_CAN;
        addr_.can_ifindex = ifr_.ifr_ifindex;

        if (bind (can_socket_, reinterpret_cast<struct sockaddr *> (&addr_), sizeof (addr_)) < 0) {
            RCLCPP_ERROR (get_logger (), "Failed to bind CAN socket to %s", can_interface.c_str ());
            close (can_socket_);
            if (!retry_open_can) return -1;
            std::this_thread::sleep_for (std::chrono::milliseconds (500));
            continue;
        }

        RCLCPP_INFO (get_logger (), "CAN socket initialized and bound to %s", can_interface.c_str ());
        return 0;
    }
    return -1;
}

void canable::read_can_socket () {
    while (rclcpp::ok ()) {
        if (use_fd_) {
            struct canfd_frame fd_frame {};
            int                nbytes = read (can_socket_, &fd_frame, sizeof (fd_frame));
            if (nbytes < 0) continue;

            natto_msgs::msg::Can msg;
            msg.id  = fd_frame.can_id;
            msg.dlc = fd_frame.len;
            std::copy (fd_frame.data, fd_frame.data + fd_frame.len, msg.data.begin ());
            msg.header.stamp = this->now ();
            canable_pub_->publish (msg);

        } else {
            struct can_frame frame {};
            int              nbytes = read (can_socket_, &frame, sizeof (frame));
            if (nbytes < 0) continue;

            natto_msgs::msg::Can msg;
            msg.id  = frame.can_id;
            msg.dlc = frame.can_dlc;
            std::copy (frame.data, frame.data + frame.can_dlc, msg.data.begin ());
            msg.header.stamp = this->now ();
            canable_pub_->publish (msg);
        }
    }
}

void canable::write_can_socket (const natto_msgs::msg::Can &msg) {
    int attempt = 0;

    if (use_fd_) {
        struct canfd_frame frame {};
        frame.can_id = msg.id;
        frame.len    = msg.dlc;
        std::copy (msg.data.begin (), msg.data.begin () + msg.dlc, frame.data);

        while (write (can_socket_, &frame, sizeof (frame)) < 0) {
            if (!retry_write_can || attempt >= max_retry_write_count) {
                RCLCPP_ERROR (get_logger (), "CAN write failed after %d retries on %s", attempt, can_interface.c_str ());
                init_can_socket ();
                return;
            }
            attempt++;
            RCLCPP_WARN (get_logger (), "CAN write failed, retry %d/%d on %s", attempt, max_retry_write_count, can_interface.c_str ());
            std::this_thread::sleep_for (std::chrono::milliseconds (100));
        }
    } else {
        struct can_frame frame {};
        frame.can_id  = msg.id;
        frame.can_dlc = msg.dlc;
        std::copy (msg.data.begin (), msg.data.begin () + msg.dlc, frame.data);

        while (write (can_socket_, &frame, sizeof (frame)) < 0) {
            if (!retry_write_can || attempt >= max_retry_write_count) {
                RCLCPP_ERROR (get_logger (), "CAN write failed after %d retries on %s", attempt, can_interface.c_str ());
                init_can_socket ();
                return;
            }
            attempt++;
            RCLCPP_WARN (get_logger (), "CAN write failed, retry %d/%d on %s", attempt, max_retry_write_count, can_interface.c_str ());
            std::this_thread::sleep_for (std::chrono::milliseconds (100));
        }
    }
}

}  // namespace canable

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (canable::canable)
