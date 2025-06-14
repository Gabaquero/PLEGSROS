#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#include <cerrno>
#include <sched.h>
#include <pthread.h>

class CommandSenderNode : public rclcpp::Node {
public:
  CommandSenderNode() : Node("command_sender_node") {
    this->declare_parameter<std::string>("joint_name", "undefined_joint");
    this->declare_parameter<int>("can_id", 0x000);
    this->declare_parameter<std::vector<double>>("velocity_limits", {-3800.0, 3800.0});
    this->declare_parameter<std::string>("can_interface", "can0");

    joint_name_ = this->get_parameter("joint_name").as_string();
    can_id_ = this->get_parameter("can_id").as_int();
    velocity_limits_ = this->get_parameter("velocity_limits").as_double_array();
    can_interface_ = this->get_parameter("can_interface").as_string();

    if (!open_can_socket(can_interface_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface: %s", can_interface_.c_str());
      can_enabled_ = false;
    } else {
      can_enabled_ = true;
      RCLCPP_INFO(this->get_logger(), "[%s] CAN interface opened successfully", joint_name_.c_str());
    }

    sub_velocity_ = this->create_subscription<std_msgs::msg::Float64>(
      "/velocity_command/" + joint_name_, 10,
      std::bind(&CommandSenderNode::velocity_callback, this, std::placeholders::_1));
  }

  ~CommandSenderNode() override {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
    }
  }

private:
  bool open_can_socket(const std::string &iface) {
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
      return false;
    }
    
    // Optimize socket for RPi4 low-latency transmission
    int tx_buffer = 32768;  // 32KB transmit buffer
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &tx_buffer, sizeof(tx_buffer));
    
    // Enable TCP_NODELAY equivalent for CAN (reduce buffering)
    int nodelay = 1;
    setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &nodelay, sizeof(nodelay));
    
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';  // Ensure null termination
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    // Set CPU affinity for command sender to core 3 (dedicated I/O)
    if (getuid() == 0) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(3, &cpuset);
      pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    }
    
    return true;
  }

  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    double vel = std::clamp(msg->data, velocity_limits_[0], velocity_limits_[1]);
    uint16_t raw = static_cast<uint16_t>((vel - velocity_limits_[0]) /
      (velocity_limits_[1] - velocity_limits_[0]) * 65535.0);

    struct can_frame frame {};
    frame.can_id = can_id_;
    frame.can_dlc = 2;
    frame.data[0] = static_cast<uint8_t>(raw >> 8);
    frame.data[1] = static_cast<uint8_t>(raw & 0xFF);

    if (!can_enabled_) {
      return;  // CAN not available, skip sending
    }
    
    if (socket_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "[%s] CAN socket not available", joint_name_.c_str());
      return;
    }
    
    ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
      failed_write_count_++;
      if (failed_write_count_ % 100 == 1) {  // Log every 100th failure
        RCLCPP_WARN(this->get_logger(), 
          "[%s] Failed to send CAN frame (errno: %d, count: %d)", 
          joint_name_.c_str(), errno, failed_write_count_);
      }
      
      // If too many failures, try to reconnect
      if (failed_write_count_ > 1000) {
        RCLCPP_WARN(this->get_logger(), 
          "[%s] Too many CAN write failures, attempting reconnect", joint_name_.c_str());
        ::close(socket_fd_);
        socket_fd_ = -1;
        can_enabled_ = open_can_socket(can_interface_);
        failed_write_count_ = 0;
      }
    } else {
      failed_write_count_ = 0;  // Reset on successful write
    }
  }

  std::string joint_name_;
  int can_id_{};
  std::vector<double> velocity_limits_;
  std::string can_interface_;

  int socket_fd_{-1};
  bool can_enabled_{false};
  int failed_write_count_{0};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_velocity_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandSenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
