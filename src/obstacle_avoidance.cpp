#include <memory>
#include <vector>
#include <string>
#include <cmath> // 用於 isinf

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class ObstacleAvoidance : public rclcpp::Node
{
public:
  ObstacleAvoidance()
  : Node("obstacle_avoidance")
  {
    // 使用 BestEffort QoS
    auto sensor_qos = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", sensor_qos, std::bind(&ObstacleAvoidance::scan_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "自動避障節點修正版已啟動！");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty()) return;
    
    int size = msg->ranges.size();
    
    // 定義區域索引
    int right_idx_start = size * 0.125;  // 縮小範圍，專注於右前方
    int right_idx_end = size * 0.375;
    
    int front_idx_start = size * 0.375;  // 正前方
    int front_idx_end = size * 0.625;
    
    int left_idx_start = size * 0.625;   // 左前方
    int left_idx_end = size * 0.875;

    float min_right = 100.0;
    float min_front = 100.0;
    float min_left = 100.0;

    // Helper lambda: 檢查數值是否有效
    auto is_valid_range = [&](float range) {
        // 忽略 inf, nan, 以及錯誤的 0.0 (Gazebo 有時會有雜訊)
        if (std::isinf(range) || std::isnan(range) || range == 0.0) return false;
        return true;
    };

    // 掃描各區最小距離
    for (int i = right_idx_start; i < right_idx_end; i++) {
        if (is_valid_range(msg->ranges[i]) && msg->ranges[i] < min_right) 
            min_right = msg->ranges[i];
    }
    for (int i = front_idx_start; i < front_idx_end; i++) {
        if (is_valid_range(msg->ranges[i]) && msg->ranges[i] < min_front) 
            min_front = msg->ranges[i];
    }
    for (int i = left_idx_start; i < left_idx_end; i++) {
        if (is_valid_range(msg->ranges[i]) && msg->ranges[i] < min_left) 
            min_left = msg->ranges[i];
    }

    // 將未偵測到的區域設為安全值
    if (min_right == 100.0) min_right = 5.0;
    if (min_front == 100.0) min_front = 5.0;
    if (min_left == 100.0) min_left = 5.0;

    // 避障邏輯
    auto twist = geometry_msgs::msg::Twist();
    float stop_distance = 0.5; // 開始減速/停下的距離
    float backup_distance = 0.25; // 必須倒車的距離

    // 簡單的狀態機
    if (min_front < stop_distance) {
      RCLCPP_WARN(this->get_logger(), "前方障礙: %.2f (左:%.2f, 右:%.2f)", min_front, min_left, min_right);
      
      if (min_front < backup_distance) {
          // 太近了，倒車
          twist.linear.x = -0.15;
          twist.angular.z = 0.0; 
      } else {
          // 還有空間，原地旋轉避開
          twist.linear.x = 0.0;
          // 判斷往哪轉
          if (min_left > min_right) {
              twist.angular.z = 0.5; // 左邊空，左轉
          } else {
              twist.angular.z = -0.5; // 右邊空，右轉
          }
      }
    } else {
      // 檢查側面是否太近
      if (min_right < 0.3) {
          twist.linear.x = 0.1;
          twist.angular.z = 0.3; // 左轉遠離
      } else if (min_left < 0.3) {
          twist.linear.x = 0.1;
          twist.angular.z = -0.3; // 右轉遠離
      } else {
          // 前方與側面都安全
          twist.linear.x = 0.3;
          twist.angular.z = 0.0;
      }
    }

    publisher_->publish(twist);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}