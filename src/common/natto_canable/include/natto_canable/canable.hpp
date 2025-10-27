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
    int                                                   init_can_socket ();                                    // Initialize CAN socket
    void                                                  read_can_socket ();                                    // Read messages from CAN socket
    void                                                  write_can_socket (const natto_msgs::msg::Can &frame);  // Write messages to CAN socket
    bool                                                  retry_open_can        = true;
    bool                                                  retry_write_can       = true;
    int                                                   retry_write_count     = 0;
    int                                                   max_retry_write_count = 5;
    std::string                                           can_interface;
    int                                                   can_socket_;
    struct sockaddr_can                                   addr_;
    struct ifreq                                          ifr_;
    rclcpp::Publisher<natto_msgs::msg::Can>::SharedPtr    canable_pub_;
    rclcpp::Subscription<natto_msgs::msg::Can>::SharedPtr canable_sub_;
};
}  // namespace canable

#endif  // __CANABLE_HPP__