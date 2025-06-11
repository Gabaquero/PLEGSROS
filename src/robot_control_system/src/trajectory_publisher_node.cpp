#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/float64_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <algorithm>

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher() : Node("trajectory_publisher")
  {
    // Parameters
    this->declare_parameter<std::string>("trajectory_file", "walk_cycle.csv");
    this->declare_parameter<double>("playback_speed", 1.0);
    this->declare_parameter<double>("publish_rate", 500.0);  // Hz
    this->declare_parameter<bool>("loop_trajectory", true);
    this->declare_parameter<bool>("interpolate", true);
    this->declare_parameter<double>("time_column_scale", 0.001);  // Convert ms to seconds
    
    std::string file_name;
    this->get_parameter("trajectory_file", file_name);
    this->get_parameter("playback_speed", playback_speed_);
    this->get_parameter("publish_rate", publish_rate_);
    this->get_parameter("loop_trajectory", loop_trajectory_);
    this->get_parameter("interpolate", interpolate_);
    this->get_parameter("time_column_scale", time_scale_);

    // Load trajectory file
    std::string package_share = ament_index_cpp::get_package_share_directory("robot_control_system");
    std::string file_path = package_share + "/trajectory/" + file_name;
    
    if (!loadCSV(file_path)) {
      // Try alternative path
      file_path = file_name;  // Try as absolute path
      if (!loadCSV(file_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file: %s", file_name.c_str());
        rclcpp::shutdown();
        return;
      }
    }

    // Joint configuration - matching your system
    joint_names_ = {"right_hip", "right_knee", "right_ankle",
                    "left_hip", "left_knee", "left_ankle"};
    
    // Assuming columns: time, right_hip, right_knee, right_ankle, left_hip, left_knee, left_ankle
    joint_indices_ = {
      {"right_hip", 1},
      {"right_knee", 2},
      {"right_ankle", 3},
      {"left_hip", 4},
      {"left_knee", 5},
      {"left_ankle", 6}
    };

    // Create publishers
    for (const auto &joint : joint_names_) {
      publishers_[joint] = this->create_publisher<custom_msgs::msg::Float64Stamped>(
        "/trajectory/" + joint, 10);
    }

    // Validate data
    if (!validateTrajectoryData()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid trajectory data format");
      rclcpp::shutdown();
      return;
    }

    // Start time
    start_time_ = this->now();
    
    // Publishing timer
    double period_ms = 1000.0 / publish_rate_;
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int>(period_ms * 1000)),
      std::bind(&TrajectoryPublisher::publishTrajectory, this));

    RCLCPP_INFO(this->get_logger(), 
      "Trajectory publisher started: %zu points, %.1f Hz, speed: %.1fx",
      trajectory_data_.size(), publish_rate_, playback_speed_);
  }

private:
  bool loadCSV(const std::string &path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      return false;
    }

    std::string line;
    bool first_line = true;
    
    while (std::getline(file, line)) {
      // Skip empty lines
      if (line.empty()) continue;
      
      // Skip header if present
      if (first_line) {
        first_line = false;
        // Check if it's a header (contains non-numeric characters)
        if (line.find_first_not_of("0123456789,.-+ \t") != std::string::npos) {
          RCLCPP_INFO(this->get_logger(), "Skipping header line");
          continue;
        }
      }

      std::vector<double> row;
      std::stringstream ss(line);
      std::string cell;
      
      while (std::getline(ss, cell, ',')) {
        try {
          // Trim whitespace
          cell.erase(0, cell.find_first_not_of(" \t"));
          cell.erase(cell.find_last_not_of(" \t") + 1);
          
          row.push_back(std::stod(cell));
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "Failed to parse value: %s", cell.c_str());
          row.push_back(0.0);
        }
      }
      
      if (!row.empty()) {
        trajectory_data_.push_back(row);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points from %s", 
      trajectory_data_.size(), path.c_str());
    return !trajectory_data_.empty();
  }

  bool validateTrajectoryData()
  {
    if (trajectory_data_.empty()) {
      return false;
    }

    // Check that all rows have enough columns
    size_t required_cols = 1 + joint_names_.size();  // time + joints
    for (size_t i = 0; i < trajectory_data_.size(); ++i) {
      if (trajectory_data_[i].size() < required_cols) {
        RCLCPP_ERROR(this->get_logger(), 
          "Row %zu has %zu columns, expected at least %zu",
          i, trajectory_data_[i].size(), required_cols);
        return false;
      }
    }

    // Extract time column and check monotonicity
    trajectory_times_.clear();
    for (const auto& row : trajectory_data_) {
      trajectory_times_.push_back(row[0] * time_scale_);
    }

    // Ensure times are monotonically increasing
    for (size_t i = 1; i < trajectory_times_.size(); ++i) {
      if (trajectory_times_[i] <= trajectory_times_[i-1]) {
        RCLCPP_WARN(this->get_logger(), 
          "Non-monotonic time at row %zu: %.3f <= %.3f",
          i, trajectory_times_[i], trajectory_times_[i-1]);
      }
    }

    trajectory_duration_ = trajectory_times_.back() - trajectory_times_.front();
    RCLCPP_INFO(this->get_logger(), "Trajectory duration: %.2f seconds", trajectory_duration_);

    return true;
  }

  void publishTrajectory()
  {
    // Calculate elapsed time with playback speed
    double elapsed = (this->now() - start_time_).seconds() * playback_speed_;
    
    // Handle looping
    if (loop_trajectory_ && trajectory_duration_ > 0) {
      elapsed = std::fmod(elapsed, trajectory_duration_);
    } else if (elapsed > trajectory_duration_) {
      // Stop at end if not looping
      if (!trajectory_ended_) {
        RCLCPP_INFO(this->get_logger(), "Trajectory playback completed");
        trajectory_ended_ = true;
      }
      elapsed = trajectory_duration_;
    }

    // Add offset to match trajectory time
    double current_traj_time = trajectory_times_.front() + elapsed;

    // Get interpolated values for each joint
    auto stamp = this->now();
    
    for (const auto &joint : joint_names_) {
      double value;
      
      if (interpolate_) {
        value = interpolateTrajectory(current_traj_time, joint_indices_[joint]);
      } else {
        // Find nearest point
        size_t idx = findNearestIndex(current_traj_time);
        value = trajectory_data_[idx][joint_indices_[joint]];
      }

      // Publish
      custom_msgs::msg::Float64Stamped msg;
      msg.header.stamp = stamp;
      msg.data = value;
      publishers_[joint]->publish(msg);
    }

    // Log progress periodically
    if (++log_counter_ % static_cast<int>(publish_rate_) == 0) {  // Every second
      RCLCPP_INFO(this->get_logger(), 
        "Trajectory progress: %.1f / %.1f seconds (%.1f%%)",
        elapsed, trajectory_duration_, 
        (elapsed / trajectory_duration_) * 100.0);
    }
  }

  double interpolateTrajectory(double time, size_t col_index)
  {
    // Handle edge cases
    if (time <= trajectory_times_.front()) {
      return trajectory_data_.front()[col_index];
    }
    if (time >= trajectory_times_.back()) {
      return trajectory_data_.back()[col_index];
    }

    // Binary search for the right interval
    auto it = std::lower_bound(trajectory_times_.begin(), trajectory_times_.end(), time);
    size_t idx2 = std::distance(trajectory_times_.begin(), it);
    size_t idx1 = idx2 - 1;

    // Linear interpolation (like drawing a straight line between two points)
    double t1 = trajectory_times_[idx1];
    double t2 = trajectory_times_[idx2];
    double v1 = trajectory_data_[idx1][col_index];
    double v2 = trajectory_data_[idx2][col_index];

    double alpha = (time - t1) / (t2 - t1);
    return v1 + alpha * (v2 - v1);

    // Note: Could implement cubic spline interpolation for smoother trajectories
    // but linear is often sufficient for high-rate trajectories
  }

  size_t findNearestIndex(double time)
  {
    auto it = std::lower_bound(trajectory_times_.begin(), trajectory_times_.end(), time);
    if (it == trajectory_times_.end()) {
      return trajectory_times_.size() - 1;
    }
    
    size_t idx = std::distance(trajectory_times_.begin(), it);
    if (idx > 0) {
      // Check if previous point is closer
      double diff_curr = std::abs(trajectory_times_[idx] - time);
      double diff_prev = std::abs(trajectory_times_[idx-1] - time);
      if (diff_prev < diff_curr) {
        return idx - 1;
      }
    }
    return idx;
  }

  // Member variables
  std::vector<std::vector<double>> trajectory_data_;
  std::vector<double> trajectory_times_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_indices_;
  std::unordered_map<std::string, rclcpp::Publisher<custom_msgs::msg::Float64Stamped>::SharedPtr> publishers_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  
  // Parameters
  double playback_speed_ = 1.0;
  double publish_rate_ = 500.0;
  bool loop_trajectory_ = true;
  bool interpolate_ = true;
  double time_scale_ = 0.001;
  
  // State
  double trajectory_duration_ = 0.0;
  bool trajectory_ended_ = false;
  int log_counter_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Set real-time priority
  struct sched_param param;
  param.sched_priority = 60;  // Lower priority than control nodes
  sched_setscheduler(0, SCHED_FIFO, &param);
  
  auto node = std::make_shared<TrajectoryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}