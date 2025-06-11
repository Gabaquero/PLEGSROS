// safety_monitor_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/float64_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include <unordered_map>
#include <vector>
#include <cmath>

class SafetyMonitor : public rclcpp::Node
{
public:
    SafetyMonitor() : Node("safety_monitor")
    {
        // Parameters
        this->declare_parameter<double>("max_position_error", 10.0);  // degrees
        this->declare_parameter<double>("max_current", 6.5);  // amps
        this->declare_parameter<double>("max_temperature", 80.0);  // celsius
        this->declare_parameter<double>("min_temperature", -20.0);  // celsius
        this->declare_parameter<double>("max_velocity", 4000.0);  // units
        this->declare_parameter<double>("watchdog_timeout", 0.5);  // seconds
        this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
        this->declare_parameter<bool>("auto_reset", false);
        this->declare_parameter<double>("reset_delay", 2.0);  // seconds

        max_position_error_ = this->get_parameter("max_position_error").as_double();
        max_current_ = this->get_parameter("max_current").as_double();
        max_temperature_ = this->get_parameter("max_temperature").as_double();
        min_temperature_ = this->get_parameter("min_temperature").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();
        auto_reset_ = this->get_parameter("auto_reset").as_bool();
        reset_delay_ = this->get_parameter("reset_delay").as_double();
        
        joint_names_ = this->get_parameter("joints").as_string_array();
        
        if (joint_names_.empty()) {
            // Default joint configuration
            joint_names_ = {"right_hip", "right_knee", "right_ankle",
                           "left_hip", "left_knee", "left_ankle"};
        }

        // Initialize joint status
        for (const auto& joint : joint_names_) {
            joint_status_[joint] = JointStatus();
        }

        // Create subscribers for each joint
        for (const auto& joint : joint_names_) {
            // Position error subscriber
            auto error_sub = this->create_subscription<std_msgs::msg::Float64>(
                "/pd_error/" + joint, 10,
                [this, joint](const std_msgs::msg::Float64::SharedPtr msg) {
                    this->errorCallback(joint, msg);
                });
            error_subs_.push_back(error_sub);

            // Current subscriber
            auto current_sub = this->create_subscription<custom_msgs::msg::Float64Stamped>(
                "/joint_current/" + joint, 10,
                [this, joint](const custom_msgs::msg::Float64Stamped::SharedPtr msg) {
                    this->currentCallback(joint, msg);
                });
            current_subs_.push_back(current_sub);

            // Temperature subscriber
            auto temp_sub = this->create_subscription<custom_msgs::msg::Float64Stamped>(
                "/joint_temperature/" + joint, 10,
                [this, joint](const custom_msgs::msg::Float64Stamped::SharedPtr msg) {
                    this->temperatureCallback(joint, msg);
                });
            temp_subs_.push_back(temp_sub);

            // Velocity subscriber
            auto vel_sub = this->create_subscription<custom_msgs::msg::Float64Stamped>(
                "/joint_velocity/" + joint, 10,
                [this, joint](const custom_msgs::msg::Float64Stamped::SharedPtr msg) {
                    this->velocityCallback(joint, msg);
                });
            vel_subs_.push_back(vel_sub);
        }

        // Publishers
        emergency_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/emergency_stop", 10);
        safety_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/safety_status", 10);

        // Safety check timer
        safety_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz safety checks
            std::bind(&SafetyMonitor::safetyCheck, this));

        // Publish initial safe status
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        safety_status_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), 
            "Safety Monitor initialized for %zu joints", joint_names_.size());
    }

private:
    struct JointStatus {
        double position_error = 0.0;
        double current = 0.0;
        double temperature = 20.0;
        double velocity = 0.0;
        rclcpp::Time last_update = rclcpp::Time(0);
        bool has_error = false;
        std::string error_reason;
    };

    void errorCallback(const std::string& joint, const std_msgs::msg::Float64::SharedPtr msg)
    {
        joint_status_[joint].position_error = std::abs(msg->data);
        joint_status_[joint].last_update = this->now();
    }

    void currentCallback(const std::string& joint, 
                        const custom_msgs::msg::Float64Stamped::SharedPtr msg)
    {
        joint_status_[joint].current = std::abs(msg->data);
    }

    void temperatureCallback(const std::string& joint, 
                           const custom_msgs::msg::Float64Stamped::SharedPtr msg)
    {
        joint_status_[joint].temperature = msg->data;
    }

    void velocityCallback(const std::string& joint, 
                         const custom_msgs::msg::Float64Stamped::SharedPtr msg)
    {
        joint_status_[joint].velocity = std::abs(msg->data);
    }

    void safetyCheck()
    {
        bool system_safe = true;
        std::vector<std::string> error_messages;
        auto current_time = this->now();

        for (const auto& joint : joint_names_) {
            auto& status = joint_status_[joint];
            status.has_error = false;
            status.error_reason.clear();

            // Check watchdog timeout (like checking if sensors are still alive)
            double time_since_update = (current_time - status.last_update).seconds();
            if (time_since_update > watchdog_timeout_) {
                status.has_error = true;
                status.error_reason = "Timeout - no data for " + 
                                     std::to_string(time_since_update) + "s";
                system_safe = false;
                error_messages.push_back("[" + joint + "] " + status.error_reason);
                continue;
            }

            // Check position error
            if (status.position_error > max_position_error_) {
                status.has_error = true;
                status.error_reason = "Position error too large: " + 
                                     std::to_string(status.position_error) + "°";
                system_safe = false;
                error_messages.push_back("[" + joint + "] " + status.error_reason);
            }

            // Check current limits
            if (status.current > max_current_) {
                status.has_error = true;
                status.error_reason = "Current too high: " + 
                                     std::to_string(status.current) + "A";
                system_safe = false;
                error_messages.push_back("[" + joint + "] " + status.error_reason);
            }

            // Check temperature limits
            if (status.temperature > max_temperature_) {
                status.has_error = true;
                status.error_reason = "Temperature too high: " + 
                                     std::to_string(status.temperature) + "°C";
                system_safe = false;
                error_messages.push_back("[" + joint + "] " + status.error_reason);
            } else if (status.temperature < min_temperature_) {
                status.has_error = true;
                status.error_reason = "Temperature too low: " + 
                                     std::to_string(status.temperature) + "°C";
                system_safe = false;
                error_messages.push_back("[" + joint + "] " + status.error_reason);
            }

            // Check velocity limits
            if (status.velocity > max_velocity_) {
                status.has_error = true;
                status.error_reason = "Velocity too high: " + 
                                     std::to_string(status.velocity);
                system_safe = false;
                error_messages.push_back("[" + joint + "] " + status.error_reason);
            }
        }

        // Handle safety status changes
        if (!system_safe && system_safe_) {
            // System just became unsafe
            RCLCPP_ERROR(this->get_logger(), "SAFETY VIOLATION DETECTED!");
            for (const auto& msg : error_messages) {
                RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
            }
            
            // Trigger emergency stop
            auto estop_msg = std_msgs::msg::Bool();
            estop_msg.data = true;
            emergency_stop_pub_->publish(estop_msg);
            
            fault_time_ = current_time;
        } else if (system_safe && !system_safe_) {
            // System just became safe again
            RCLCPP_INFO(this->get_logger(), "System returned to safe state");
            
            if (auto_reset_) {
                // Wait before clearing emergency stop
                if ((current_time - fault_time_).seconds() > reset_delay_) {
                    auto estop_msg = std_msgs::msg::Bool();
                    estop_msg.data = false;
                    emergency_stop_pub_->publish(estop_msg);
                    RCLCPP_INFO(this->get_logger(), "Emergency stop cleared");
                }
            }
        }

        // Update and publish system status
        system_safe_ = system_safe;
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = system_safe;
        safety_status_pub_->publish(status_msg);

        // Periodic status report
        if (++report_counter_ % 50 == 0) {  // Every 5 seconds at 10Hz
            if (system_safe) {
                RCLCPP_INFO(this->get_logger(), "System operating safely");
            } else {
                RCLCPP_WARN(this->get_logger(), 
                    "System in fault state - %zu errors", error_messages.size());
            }
        }
    }

    // Parameters
    double max_position_error_;
    double max_current_;
    double max_temperature_;
    double min_temperature_;
    double max_velocity_;
    double watchdog_timeout_;
    bool auto_reset_;
    double reset_delay_;
    
    // Joint configuration
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, JointStatus> joint_status_;
    
    // State
    bool system_safe_ = true;
    rclcpp::Time fault_time_;
    int report_counter_ = 0;
    
    // ROS interfaces
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> error_subs_;
    std::vector<rclcpp::Subscription<custom_msgs::msg::Float64Stamped>::SharedPtr> current_subs_;
    std::vector<rclcpp::Subscription<custom_msgs::msg::Float64Stamped>::SharedPtr> temp_subs_;
    std::vector<rclcpp::Subscription<custom_msgs::msg::Float64Stamped>::SharedPtr> vel_subs_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_status_pub_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // High priority for safety
    struct sched_param param;
    param.sched_priority = 85;
    sched_setscheduler(0, SCHED_FIFO, &param);
    
    auto node = std::make_shared<SafetyMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}