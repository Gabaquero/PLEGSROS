#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/float64_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <message_filters/subscriber.hpp>  // Changed from .h to .hpp
#include <message_filters/sync_policies/approximate_time.hpp>  // Changed from .h to .hpp
#include <message_filters/synchronizer.hpp>  // Changed from .h to .hpp
#include <algorithm>
#include <deque>
#include <numeric>
#include <array>
#include <unistd.h>
#include <cerrno>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>

using std::placeholders::_1;
using std::placeholders::_2;

class PDControllerNode : public rclcpp::Node {
public:
    PDControllerNode() : Node("pd_controller_node") {
        // Parameters - keeping your values
        this->declare_parameter<std::string>("joint_name", "default_joint");
        this->declare_parameter<double>("kp", 250.0);
        this->declare_parameter<double>("kd", 1.5);
        this->declare_parameter<double>("max_velocity", 3800.0);
        this->declare_parameter<double>("velocity_filter_alpha", 0.8);
        this->declare_parameter<int>("derivative_window_size", 5);
        this->declare_parameter<double>("deadband", 0.1);  // Position error deadband
        this->declare_parameter<bool>("use_feedforward", false);
        this->declare_parameter<double>("feedforward_gain", 0.0);

        this->get_parameter("joint_name", joint_name_);
        this->get_parameter("kp", kp_);
        this->get_parameter("kd", kd_);
        this->get_parameter("max_velocity", max_velocity_);
        this->get_parameter("velocity_filter_alpha", filter_alpha_);
        // derivative_window_size_ is now a compile-time constant for ARM optimization
        // this->get_parameter("derivative_window_size", derivative_window_size_);
        this->get_parameter("deadband", deadband_);
        this->get_parameter("use_feedforward", use_feedforward_);
        this->get_parameter("feedforward_gain", feedforward_gain_);

        std::string desired_topic = "/trajectory/" + joint_name_;
        std::string current_topic = "/joint_position/" + joint_name_;
        std::string output_topic = "/velocity_command/" + joint_name_;

        // Message filter subscribers
        auto qos = rclcpp::QoS(10);
        desired_sub_.subscribe(this, desired_topic, qos);
        current_sub_.subscribe(this, current_topic, qos);

        // Synchronizer with adaptive queue size based on control frequency
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(20), desired_sub_, current_sub_);
        sync_->registerCallback(std::bind(&PDControllerNode::pd_callback, this, _1, _2));

        // Publishers
        command_pub_ = this->create_publisher<std_msgs::msg::Float64>(output_topic, 10);
        
        // Debug publishers
        error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pd_error/" + joint_name_, 10);
        derivative_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/pd_derivative/" + joint_name_, 10);

        // Initialize ARM-optimized derivative filter arrays
        position_history_.fill(0.0);
        time_history_.fill(0.0);

        RCLCPP_INFO(this->get_logger(), 
            "[%s] PD Controller initialized - Kp: %.1f, Kd: %.3f, Max Vel: %.1f", 
            joint_name_.c_str(), kp_, kd_, max_velocity_);
    }

private:
    void pd_callback(
        const custom_msgs::msg::Float64Stamped::ConstSharedPtr desired_msg,
        const custom_msgs::msg::Float64Stamped::ConstSharedPtr current_msg) 
    {
        // Get timestamps for proper derivative calculation
        double desired_time = desired_msg->header.stamp.sec + 
                             desired_msg->header.stamp.nanosec * 1e-9;
        double current_time = current_msg->header.stamp.sec + 
                             current_msg->header.stamp.nanosec * 1e-9;
        
        // Check for stale data (like checking if your sensors are still alive)
        double time_diff = std::abs(desired_time - current_time);
        if (time_diff > 0.1) {  // 100ms threshold
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[%s] Time sync issue: %.3f sec difference", 
                joint_name_.c_str(), time_diff);
        }

        double desired_pos = desired_msg->data;
        double current_pos = current_msg->data;
        double error = desired_pos - current_pos;

        // Apply deadband to reduce jitter when close to target
        if (std::abs(error) < deadband_) {
            error = 0.0;
        }

        // Proportional term
        double p_term = kp_ * error;

        // Calculate derivative using sliding window for noise reduction
        double d_term = 0.0;
        if (initialized_) {
            // Add to history
            position_history_[history_index_] = current_pos;
            time_history_[history_index_] = current_time;
            history_index_ = (history_index_ + 1) % derivative_window_size_;

            // Calculate derivative using least squares fit
            // (like finding the best straight line through noisy data points)
            d_term = calculateFilteredDerivative() * kd_;
        } else {
            // First iteration - initialize history
            std::fill(position_history_.begin(), position_history_.end(), current_pos);
            std::fill(time_history_.begin(), time_history_.end(), current_time);
            initialized_ = true;
        }

        // Feedforward term (optional - for trajectory tracking)
        double ff_term = 0.0;
        if (use_feedforward_ && last_desired_valid_) {
            double desired_velocity = (desired_pos - last_desired_pos_) / 
                                    std::max(0.001, current_time - last_time_);
            ff_term = feedforward_gain_ * desired_velocity;
        }

        // Calculate total velocity command
        double velocity = p_term - d_term + ff_term;

        // Apply velocity limits with rate limiting for smooth motion
        velocity = applyRateLimiting(velocity);
        velocity = std::clamp(velocity, -max_velocity_, max_velocity_);

        // Low-pass filter on output for smoothness
        filtered_velocity_ = filter_alpha_ * velocity + 
                           (1.0 - filter_alpha_) * filtered_velocity_;

        // Publish command
        auto msg = std_msgs::msg::Float64();
        msg.data = filtered_velocity_;
        command_pub_->publish(msg);

        // Publish debug info
        auto error_msg = std_msgs::msg::Float64();
        error_msg.data = error;
        error_pub_->publish(error_msg);

        auto deriv_msg = std_msgs::msg::Float64();
        deriv_msg.data = d_term / kd_;  // Actual derivative value
        derivative_pub_->publish(deriv_msg);

        // Update history
        last_desired_pos_ = desired_pos;
        last_time_ = current_time;
        last_desired_valid_ = true;
        last_velocity_command_ = filtered_velocity_;

        // Log every 5 seconds for debugging (reduce logging overhead)
        if (++log_counter_ % 2500 == 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                "[%s] Pos: %.2f->%.2f, Err: %.3f, Vel: %.1f (P:%.1f D:%.1f FF:%.1f)",
                joint_name_.c_str(), current_pos, desired_pos, error, 
                filtered_velocity_, p_term, d_term, ff_term);
        }
    }

    double calculateFilteredDerivative() {
        // Use simple finite difference with low-pass filtering
        // Much more efficient than least squares for real-time control
        
        if (history_index_ < 2) {
            return 0.0;  // Need at least 2 points
        }
        
        // Get the two most recent points
        int current_idx = (history_index_ - 1) % derivative_window_size_;
        int prev_idx = (history_index_ - 2) % derivative_window_size_;
        
        double dt = time_history_[current_idx] - time_history_[prev_idx];
        if (dt < 1e-6) {
            return filtered_derivative_;  // Return last value if dt too small
        }
        
        double raw_derivative = (position_history_[current_idx] - position_history_[prev_idx]) / dt;
        
        // Apply low-pass filter to reduce noise
        filtered_derivative_ = filter_alpha_ * filtered_derivative_ + (1.0 - filter_alpha_) * raw_derivative;
        
        return filtered_derivative_;
    }

    double applyRateLimiting(double velocity) {
        // Limit acceleration to prevent jerky motion
        const double max_accel = max_velocity_ * 2.0;  // Can accelerate to max in 0.5s
        const double dt = 0.002;  // Assuming 500Hz control rate
        
        double velocity_change = velocity - last_velocity_command_;
        double max_change = max_accel * dt;
        
        if (std::abs(velocity_change) > max_change) {
            velocity = last_velocity_command_ + 
                      std::copysign(max_change, velocity_change);
        }
        
        return velocity;
    }

    // Member variables
    std::string joint_name_;
    double kp_ = 250.0;
    double kd_ = 1.5;
    double max_velocity_ = 3800.0;
    double filter_alpha_ = 0.8;
    double deadband_ = 0.1;
    bool use_feedforward_ = false;
    double feedforward_gain_ = 0.0;
    
    // ARM-optimized derivative calculation with cache-friendly access
    static const int derivative_window_size_ = 8;  // Power of 2 for ARM efficiency
    alignas(64) std::array<double, derivative_window_size_> position_history_{};  // Cache-aligned
    alignas(64) std::array<double, derivative_window_size_> time_history_{};     // Cache-aligned
    int history_index_ = 0;
    bool initialized_ = false;
    
    // State tracking
    double last_desired_pos_ = 0.0;
    double last_time_ = 0.0;
    bool last_desired_valid_ = false;
    double last_velocity_command_ = 0.0;
    double filtered_velocity_ = 0.0;
    double filtered_derivative_ = 0.0;
    int log_counter_ = 0;

    // ROS interfaces
    message_filters::Subscriber<custom_msgs::msg::Float64Stamped> desired_sub_;
    message_filters::Subscriber<custom_msgs::msg::Float64Stamped> current_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        custom_msgs::msg::Float64Stamped,
        custom_msgs::msg::Float64Stamped>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr derivative_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Optimize for Raspberry Pi 4 performance
    if (getuid() == 0) {
        // Lock memory to prevent swapping (critical for RT performance)
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            RCLCPP_WARN(rclcpp::get_logger("pd_controller"), 
                "Failed to lock memory (errno: %d). Performance may be affected.", errno);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("pd_controller"), "Memory locked for real-time operation");
        }
        
        // Set CPU affinity to core 2 (less system interrupts than core 0/1)
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(2, &cpuset);  // Use core 2 for PD controllers
        
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            RCLCPP_WARN(rclcpp::get_logger("pd_controller"), "Failed to set CPU affinity");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("pd_controller"), "CPU affinity set to core 2");
        }
        
        // Set real-time priority (lower for RPi4 to prevent system lockup)
        struct sched_param param;
        param.sched_priority = 50;  // Reduced for RPi4 stability
        if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            RCLCPP_WARN(rclcpp::get_logger("pd_controller"), 
                "Failed to set real-time priority (errno: %d). Performance may be affected.", errno);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("pd_controller"), 
                "Real-time priority set successfully (priority: %d)", param.sched_priority);
        }
    } else {
        RCLCPP_DEBUG(rclcpp::get_logger("pd_controller"), 
            "Not running as root, skipping real-time optimizations");
    }
    
    rclcpp::spin(std::make_shared<PDControllerNode>());
    rclcpp::shutdown();
    return 0;
}