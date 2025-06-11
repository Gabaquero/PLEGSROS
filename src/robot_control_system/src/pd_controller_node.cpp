#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/float64_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <message_filters/subscriber.hpp>  // Changed from .h to .hpp
#include <message_filters/sync_policies/approximate_time.hpp>  // Changed from .h to .hpp
#include <message_filters/synchronizer.hpp>  // Changed from .h to .hpp
#include <algorithm>
#include <deque>
#include <numeric>

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
        this->get_parameter("derivative_window_size", derivative_window_size_);
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

        // Initialize derivative filter window
        position_history_.resize(derivative_window_size_);
        time_history_.resize(derivative_window_size_);

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

        // Log every N iterations for debugging
        if (++log_counter_ % 500 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "[%s] Pos: %.2f->%.2f, Err: %.3f, Vel: %.1f (P:%.1f D:%.1f FF:%.1f)",
                joint_name_.c_str(), current_pos, desired_pos, error, 
                filtered_velocity_, p_term, d_term, ff_term);
        }
    }

    double calculateFilteredDerivative() {
        // Use least squares to fit a line through position history
        // This is more robust to noise than simple finite differences
        
        double sum_t = 0.0, sum_p = 0.0, sum_tt = 0.0, sum_tp = 0.0;
        int n = derivative_window_size_;
        
        for (int i = 0; i < n; ++i) {
            sum_t += time_history_[i];
            sum_p += position_history_[i];
            sum_tt += time_history_[i] * time_history_[i];
            sum_tp += time_history_[i] * position_history_[i];
        }
        
        double denominator = n * sum_tt - sum_t * sum_t;
        if (std::abs(denominator) < 1e-10) {
            return 0.0;  // Avoid division by zero
        }
        
        // Slope of the fitted line is the derivative
        return (n * sum_tp - sum_t * sum_p) / denominator;
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
    
    // Derivative calculation
    int derivative_window_size_ = 5;
    std::vector<double> position_history_;
    std::vector<double> time_history_;
    int history_index_ = 0;
    bool initialized_ = false;
    
    // State tracking
    double last_desired_pos_ = 0.0;
    double last_time_ = 0.0;
    bool last_desired_valid_ = false;
    double last_velocity_command_ = 0.0;
    double filtered_velocity_ = 0.0;
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
    
    // Set up real-time scheduling
    struct sched_param param;
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        RCLCPP_WARN(rclcpp::get_logger("pd_controller"), 
            "Failed to set real-time priority. Performance may be affected.");
    }
    
    rclcpp::spin(std::make_shared<PDControllerNode>());
    rclcpp::shutdown();
    return 0;
}