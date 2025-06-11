#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/float64_stamped.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>  // Added for fcntl, F_GETFL, F_SETFL, O_NONBLOCK
#include <cstring>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <cmath>  // Added for M_PI
#include <stdexcept>
#include <unistd.h>
#include <cerrno>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>

class ReceptionNode : public rclcpp::Node
{
public:
  ReceptionNode() : Node("reception_node"), running_(true)
  {
    // Parameters
    this->declare_parameter<std::string>("joint_name", "undefined_joint");
    this->declare_parameter<int>("can_id", 0x000);
    this->declare_parameter<std::vector<double>>("position_limits", {-180.0, 180.0});
    this->declare_parameter<std::vector<double>>("current_limits", {-7.0, 7.0});
    this->declare_parameter<std::vector<double>>("velocity_limits", {-3800.0, 3800.0});
    this->declare_parameter<std::vector<double>>("temperature_limits", {-40.0, 120.0});
    this->declare_parameter<bool>("use_simulated_data", true);
    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<double>("timeout_seconds", 0.1);
    this->declare_parameter<bool>("invert_position", false);

    joint_name_ = this->get_parameter("joint_name").as_string();
    can_id_ = this->get_parameter("can_id").as_int();
    position_limits_ = this->get_parameter("position_limits").as_double_array();
    current_limits_ = this->get_parameter("current_limits").as_double_array();
    velocity_limits_ = this->get_parameter("velocity_limits").as_double_array();
    temperature_limits_ = this->get_parameter("temperature_limits").as_double_array();
    use_simulated_data_ = this->get_parameter("use_simulated_data").as_bool();
    can_interface_ = this->get_parameter("can_interface").as_string();
    timeout_seconds_ = this->get_parameter("timeout_seconds").as_double();
    
    // Validate parameters
    if (joint_name_ == "undefined_joint") {
      RCLCPP_ERROR(this->get_logger(), "joint_name parameter must be set");
      throw std::runtime_error("Invalid joint_name parameter");
    }
    if (can_id_ <= 0 || can_id_ > 0x7FF) {
      RCLCPP_ERROR(this->get_logger(), "can_id must be between 0x001 and 0x7FF, got: 0x%03X", can_id_);
      throw std::runtime_error("Invalid can_id parameter");
    }
    if (position_limits_.size() != 2 || position_limits_[0] >= position_limits_[1]) {
      RCLCPP_ERROR(this->get_logger(), "position_limits must have 2 elements with min < max");
      throw std::runtime_error("Invalid position_limits parameter");
    }
    if (current_limits_.size() != 2 || current_limits_[0] >= current_limits_[1]) {
      RCLCPP_ERROR(this->get_logger(), "current_limits must have 2 elements with min < max");
      throw std::runtime_error("Invalid current_limits parameter");
    }
    if (velocity_limits_.size() != 2 || velocity_limits_[0] >= velocity_limits_[1]) {
      RCLCPP_ERROR(this->get_logger(), "velocity_limits must have 2 elements with min < max");
      throw std::runtime_error("Invalid velocity_limits parameter");
    }
    if (temperature_limits_.size() != 2 || temperature_limits_[0] >= temperature_limits_[1]) {
      RCLCPP_ERROR(this->get_logger(), "temperature_limits must have 2 elements with min < max");
      throw std::runtime_error("Invalid temperature_limits parameter");
    }
    if (timeout_seconds_ <= 0.0 || timeout_seconds_ > 10.0) {
      RCLCPP_ERROR(this->get_logger(), "timeout_seconds must be between 0.0 and 10.0, got: %.3f", timeout_seconds_);
      throw std::runtime_error("Invalid timeout_seconds parameter");
    }
    if (can_interface_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "can_interface parameter cannot be empty");
      throw std::runtime_error("Invalid can_interface parameter");
    }
    
    // Get position inversion from parameter
    invert_position_ = this->get_parameter("invert_position").as_bool();

    // Publishers
    pub_position_ = this->create_publisher<custom_msgs::msg::Float64Stamped>(
      "/joint_position/" + joint_name_, 10);
    pub_current_ = this->create_publisher<custom_msgs::msg::Float64Stamped>(
      "/joint_current/" + joint_name_, 10);
    pub_velocity_ = this->create_publisher<custom_msgs::msg::Float64Stamped>(
      "/joint_velocity/" + joint_name_, 10);
    pub_temperature_ = this->create_publisher<custom_msgs::msg::Float64Stamped>(
      "/joint_temperature/" + joint_name_, 10);

    // Initialize CAN or simulation
    if (!use_simulated_data_) {
      if (!initializeCANSocket()) {
        RCLCPP_ERROR(this->get_logger(), 
          "[%s] Failed to initialize CAN interface, falling back to simulation", 
          joint_name_.c_str());
        use_simulated_data_ = true;
      } else {
        // Start CAN receive thread
        can_thread_ = std::thread(&ReceptionNode::canReceiveLoop, this);
      }
    }

    // High-performance timer optimized for RPi4
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(2000),  // 500Hz with microsecond precision
      std::bind(&ReceptionNode::publishData, this)
    );

    RCLCPP_INFO(this->get_logger(), 
      "[%s] Node initialized (CAN ID: 0x%03X, Mode: %s, Inverted: %s)", 
      joint_name_.c_str(), can_id_, 
      use_simulated_data_ ? "SIMULATED" : "REAL CAN",
      invert_position_ ? "YES" : "NO");
  }

  ~ReceptionNode()
  {
    running_ = false;
    cv_.notify_all();
    
    if (can_thread_.joinable()) {
      can_thread_.join();
    }
    
    if (can_socket_ >= 0) {
      close(can_socket_);
    }
  }

private:
  // Data structure for CAN data - moved before its usage
  // ARM-optimized data structure with proper alignment
  struct alignas(64) CANData {  // 64-byte alignment for cache line optimization
    rclcpp::Time timestamp;
    uint16_t raw_position;
    uint16_t raw_current;
    uint16_t raw_velocity;
    uint16_t raw_temperature;
    uint32_t padding[12];  // Pad to cache line boundary for ARM
  };

  bool initializeCANSocket()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket: %s", 
        strerror(errno));
      return false;
    }

    // Get interface index
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));  // Zero the structure
    strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';  // Ensure null termination
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get index for %s: %s", 
        can_interface_.c_str(), strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    // Bind socket
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket: %s", 
        strerror(errno));
      close(can_socket_);
      can_socket_ = -1;
      return false;
    }

    // Set up filter to only receive messages for this joint
    struct can_filter filter;
    filter.can_id = can_id_;
    filter.can_mask = CAN_SFF_MASK;  // Standard frame format mask
    
    if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, 
                   &filter, sizeof(filter)) < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to set CAN filter: %s", 
        strerror(errno));
    }

    // Optimize socket for RPi4 performance
    int flags = fcntl(can_socket_, F_GETFL, 0);
    fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    
    // Set socket buffer sizes for burst performance
    int rx_buffer = 65536;  // 64KB receive buffer
    int tx_buffer = 32768;  // 32KB transmit buffer
    setsockopt(can_socket_, SOL_SOCKET, SO_RCVBUF, &rx_buffer, sizeof(rx_buffer));
    setsockopt(can_socket_, SOL_SOCKET, SO_SNDBUF, &tx_buffer, sizeof(tx_buffer));
    
    // Enable socket timestamping for precise timing
    int timestamp = 1;
    setsockopt(can_socket_, SOL_SOCKET, SO_TIMESTAMP, &timestamp, sizeof(timestamp));

    return true;
  }

  void canReceiveLoop()
  {
    struct can_frame frame;
    fd_set rfds;
    struct timeval tv;

    while (running_) {
      FD_ZERO(&rfds);
      FD_SET(can_socket_, &rfds);
      
      // Set timeout
      tv.tv_sec = 0;
      tv.tv_usec = 10000;  // 10ms timeout

      int retval = select(can_socket_ + 1, &rfds, NULL, NULL, &tv);
      
      if (retval > 0 && FD_ISSET(can_socket_, &rfds)) {
        ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
        
        if (nbytes == sizeof(frame)) {
          if (frame.can_id == (unsigned)can_id_ && frame.can_dlc >= 8) {
            // Process the CAN frame
            processCANFrame(frame);
          }
        }
      } else if (retval < 0 && errno != EINTR) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[%s] CAN select error: %s", joint_name_.c_str(), strerror(errno));
      }
    }
  }

  void processCANFrame(const struct can_frame& frame)
  {
    CANData data;
    data.timestamp = this->now();
    
    // Extract data from CAN frame
    data.raw_position = (frame.data[0] << 8) | frame.data[1];
    data.raw_current = (frame.data[2] << 8) | frame.data[3];
    data.raw_velocity = (frame.data[4] << 8) | frame.data[5];
    data.raw_temperature = (frame.data[6] << 8) | frame.data[7];
    
    // Store in thread-safe queue
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      can_data_queue_.push(data);
      
      // Keep queue size reasonable
      while (can_data_queue_.size() > 10) {
        can_data_queue_.pop();
      }
    }
    cv_.notify_one();
  }

  void publishData()
  {
    CANData data;
    bool has_data = false;

    rclcpp::Time now = this->now();
    
    if (!use_simulated_data_) {
      // Get latest real CAN data
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!can_data_queue_.empty()) {
          data = can_data_queue_.front();
          can_data_queue_.pop();
          has_data = true;
        }
      }
      
      // Update last data time with state lock
      if (has_data) {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        last_data_time_ = now;
      }
    } else {
      // Generate simulated data
      data = generateSimulatedData();
      has_data = true;
    }

    // Check for timeout with state lock
    if (!use_simulated_data_) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if ((now - last_data_time_).seconds() > timeout_seconds_) {
        if (!timeout_warned_) {
          RCLCPP_WARN(this->get_logger(), 
            "[%s] No CAN data received for %.1f seconds", 
            joint_name_.c_str(), timeout_seconds_);
          timeout_warned_ = true;
        }
        // Use last known values or safe defaults
        if (!has_data) {
          data = last_valid_data_;
          data.timestamp = now;
        }
      } else {
        timeout_warned_ = false;
      }
    }

    if (has_data) {
      // Convert raw values to physical units
      double position = map_value(data.raw_position, position_limits_[0], position_limits_[1]);
      double current = map_value(data.raw_current, current_limits_[0], current_limits_[1]);
      double velocity = map_value(data.raw_velocity, velocity_limits_[0], velocity_limits_[1]);
      double temperature = map_value(data.raw_temperature, temperature_limits_[0], temperature_limits_[1]);

      // Apply position inversion if needed
      if (invert_position_) {
        position = -position;
      }

      // Publish all data
      publishValue(pub_position_, position, data.timestamp);
      publishValue(pub_current_, current, data.timestamp);
      publishValue(pub_velocity_, velocity, data.timestamp);
      publishValue(pub_temperature_, temperature, data.timestamp);

      // Store as last valid data
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        last_valid_data_ = data;
      }

      // Periodic logging - reduced frequency for performance
      if (++log_counter_ % 2500 == 0) {  // Every 5 seconds at 500Hz
        RCLCPP_INFO(this->get_logger(), 
          "[%s] Pos: %.1f° | Curr: %.2fA | Vel: %.1f | Temp: %.1f°C",
          joint_name_.c_str(), position, current, velocity, temperature);
      }
    }
  }

  CANData generateSimulatedData()
  {
    CANData data;
    data.timestamp = this->now();
    
    // Generate more realistic simulated data
    double t = data.timestamp.seconds();
    double freq = 0.5;  // Hz
    
    // Simulate joint moving in sine wave pattern
    double position_norm = 0.5 + 0.3 * sin(2.0 * M_PI * freq * t);
    double velocity_norm = 0.5 + 0.3 * cos(2.0 * M_PI * freq * t);
    
    data.raw_position = static_cast<uint16_t>(position_norm * 65535);
    data.raw_current = 500 + rand() % 100;  // Small current variations
    data.raw_velocity = static_cast<uint16_t>(velocity_norm * 65535);
    data.raw_temperature = 7700 + rand() % 200;  // ~20-25°C
    
    return data;
  }

  double map_value(uint16_t raw, double min_val, double max_val)
  {
    return min_val + (raw / 65535.0) * (max_val - min_val);
  }

  void publishValue(rclcpp::Publisher<custom_msgs::msg::Float64Stamped>::SharedPtr pub,
                    double value, const rclcpp::Time& timestamp)
  {
    custom_msgs::msg::Float64Stamped msg;
    msg.header.stamp = timestamp;
    msg.data = value;
    pub->publish(msg);
  }

  // Parameters
  std::string joint_name_;
  int can_id_;
  std::vector<double> position_limits_;
  std::vector<double> current_limits_;
  std::vector<double> velocity_limits_;
  std::vector<double> temperature_limits_;
  bool use_simulated_data_;
  std::string can_interface_;
  double timeout_seconds_;
  bool invert_position_;

  // CAN communication
  int can_socket_ = -1;
  std::thread can_thread_;
  std::atomic<bool> running_;
  
  // Thread-safe data queue
  std::queue<CANData> can_data_queue_;
  std::mutex data_mutex_;
  std::condition_variable cv_;
  
  // State tracking (protected by state_mutex_)
  mutable std::mutex state_mutex_;
  rclcpp::Time last_data_time_;
  CANData last_valid_data_;
  bool timeout_warned_ = false;
  int log_counter_ = 0;

  // Publishers
  rclcpp::Publisher<custom_msgs::msg::Float64Stamped>::SharedPtr pub_position_;
  rclcpp::Publisher<custom_msgs::msg::Float64Stamped>::SharedPtr pub_current_;
  rclcpp::Publisher<custom_msgs::msg::Float64Stamped>::SharedPtr pub_velocity_;
  rclcpp::Publisher<custom_msgs::msg::Float64Stamped>::SharedPtr pub_temperature_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Optimize for Raspberry Pi 4 I/O performance
  if (getuid() == 0) {
    // Lock memory for RT performance
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
      RCLCPP_WARN(rclcpp::get_logger("reception_node"), 
        "Failed to lock memory (errno: %d)", errno);
    }
    
    // Set CPU affinity to core 1 (I/O operations)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);  // Use core 1 for reception nodes
    
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("reception_node"), "Failed to set CPU affinity");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("reception_node"), "CPU affinity set to core 1");
    }
    
    // Set real-time priority (lower than PD controller)
    struct sched_param param;
    param.sched_priority = 40;  // Lower for RPi4 stability
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
      RCLCPP_WARN(rclcpp::get_logger("reception_node"), 
        "Failed to set real-time priority (errno: %d)", errno);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("reception_node"), 
        "Real-time priority set successfully (priority: %d)", param.sched_priority);
    }
  }
  
  auto node = std::make_shared<ReceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}