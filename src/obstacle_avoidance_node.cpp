#include <algorithm>
#include <cmath>
#include <string>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
  ObstacleAvoidanceNode()
  : rclcpp::Node("obstacle_avoidance_node"), node_start_time_(this->now()) {
    // Parameters
    linear_speed_ = declare_parameter<double>("linear_speed", 0.15);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.6);
    obstacle_distance_ = declare_parameter<double>("obstacle_distance", 0.6);
    startup_delay_sec_ = declare_parameter<double>("startup_delay_sec", 1.0);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleAvoidanceNode::onScan, this, std::placeholders::_1));

    is_turning_ = false;
    turn_direction_ = 1;
    scan_received_ = false;
    
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Obstacle Avoidance Node Started");
    RCLCPP_INFO(get_logger(), "Waiting for laser scan data...");
    RCLCPP_INFO(get_logger(), "========================================");
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!scan_received_) {
      scan_received_ = true;
      RCLCPP_INFO(get_logger(), "Laser scan received! Robot will start moving.");
    }

    const auto & ranges = msg->ranges;
    if (ranges.empty()) {
      RCLCPP_WARN(get_logger(), "Empty scan!");
      publishStop();
      return;
    }

    // Check startup delay
    const rclcpp::Time now_time = now();
    const double since_start = (now_time - node_start_time_).seconds();
    if (since_start < startup_delay_sec_) {
      publishStop();
      return;
    }

    const size_t n = ranges.size();
    const float angle_min = msg->angle_min;
    const float angle_inc = msg->angle_increment;

    // Get minimum distance in front sector
    auto getMinInSector = [&](float start_deg, float end_deg) -> float {
      const float start_rad = start_deg * M_PI / 180.0f;
      const float end_rad = end_deg * M_PI / 180.0f;
      int start_idx = static_cast<int>((start_rad - angle_min) / angle_inc);
      int end_idx = static_cast<int>((end_rad - angle_min) / angle_inc);
      
      start_idx = std::max(0, std::min(static_cast<int>(n) - 1, start_idx));
      end_idx = std::max(0, std::min(static_cast<int>(n) - 1, end_idx));
      
      if (start_idx > end_idx) std::swap(start_idx, end_idx);
      
      float min_dist = 10.0f;
      for (int i = start_idx; i <= end_idx; ++i) {
        float r = ranges[i];
        if (std::isfinite(r) && r > 0.05f && r < 10.0f && r < min_dist) {
          min_dist = r;
        }
      }
      return min_dist;
    };

    // Check three sectors
    const float front_center = getMinInSector(-20.0f, 20.0f);
    const float front_left = getMinInSector(20.0f, 80.0f);
    const float front_right = getMinInSector(-80.0f, -20.0f);

    geometry_msgs::msg::Twist cmd;
    
    // Simple logic: if obstacle in front, turn. Otherwise, go forward.
    if (front_center < static_cast<float>(obstacle_distance_)) {
      // OBSTACLE DETECTED - TURN
      if (!is_turning_) {
        // Just started detecting obstacle
        is_turning_ = true;
        turn_start_time_ = now_time;
        
        // Decide direction
        if (front_left > front_right) {
          turn_direction_ = 1;  // Turn left
          RCLCPP_INFO(get_logger(), "OBSTACLE at %.2fm! Turning LEFT (L:%.2f R:%.2f)", 
                     front_center, front_left, front_right);
        } else {
          turn_direction_ = -1;  // Turn right
          RCLCPP_INFO(get_logger(), "OBSTACLE at %.2fm! Turning RIGHT (L:%.2f R:%.2f)", 
                     front_center, front_left, front_right);
        }
      }
      
      // Execute turn
      cmd.linear.x = 0.0;
      cmd.angular.z = angular_speed_ * turn_direction_;
      
      double turn_time = (now_time - turn_start_time_).seconds();
      
      // If been turning for more than 3 seconds, try opposite direction
      if (turn_time > 3.0) {
        turn_direction_ = -turn_direction_;
        turn_start_time_ = now_time;
        RCLCPP_WARN(get_logger(), "Switching turn direction!");
      }
      
    } else {
      // NO OBSTACLE - MOVE FORWARD
      if (is_turning_) {
        is_turning_ = false;
        RCLCPP_INFO(get_logger(), "Path clear! Moving forward.");
      }
      
      cmd.linear.x = linear_speed_;
      cmd.angular.z = 0.0;
      
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, 
                          "Moving forward (front: %.2fm)", front_center);
    }

    // Publish
    cmd_pub_->publish(cmd);
  }

  void publishStop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  double linear_speed_;
  double angular_speed_;
  double obstacle_distance_;
  double startup_delay_sec_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  
  rclcpp::Time node_start_time_;
  rclcpp::Time turn_start_time_;
  
  bool is_turning_;
  int turn_direction_;
  bool scan_received_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleAvoidanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
