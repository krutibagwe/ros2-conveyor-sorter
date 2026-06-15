#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "conveyor_sort/msg/object_position.hpp"
#include "conveyor_sort/msg/jam_alert.hpp"

class JamMonitorNode : public rclcpp::Node {
public:
  JamMonitorNode() : Node("jam_monitor_node") {

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/conveyor_speed", 10,
      std::bind(&JamMonitorNode::speed_callback, this, std::placeholders::_1));
    belt_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/belt_status", 10,
      std::bind(&JamMonitorNode::belt_status_callback, this, std::placeholders::_1));

    jam_pub_ = this->create_publisher<conveyor_sort::msg::JamAlert>(
      "/jam_alert", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&JamMonitorNode::check_jam, this));

    startup_time_ = this->now();
    last_time_    = this->now();

    RCLCPP_INFO(this->get_logger(), "Jam monitor node started.");
  }

private:
  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    measured_speed_ = msg->data;
    speed_received_ = true;
  }

  void belt_status_callback(const std_msgs::msg::String::SharedPtr msg) {
    belt_state_ = msg->data;
    // Reset jam timer whenever belt is not actively running
    if (belt_state_ != "RUNNING") {
      time_speed_low_ = 0.0;
      jam_duration_   = 0.0;
    }
  }

  void check_jam() {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0) return;

    double uptime = (now - startup_time_).seconds();
    bool jam = false;

    // Only check jam when belt is actively RUNNING
    if (uptime > 35.0 && belt_state_ == "RUNNING" && speed_received_) {
      if (measured_speed_ < speed_threshold_) {
        time_speed_low_ += dt;
      } else {
        time_speed_low_ = 0.0;
      }

      if (time_speed_low_ > 5.0) {
        jam = true;
        jam_duration_ += dt;
      } else {
        jam_duration_ = 0.0;
      }
    } else {
      time_speed_low_ = 0.0;
      jam_duration_   = 0.0;
    }

    conveyor_sort::msg::JamAlert alert;
    alert.header.stamp     = this->now();
    alert.jam_detected     = jam;
    alert.reason           = jam ? "speed_too_low" : "none";
    alert.duration_seconds = static_cast<float>(jam_duration_);
    jam_pub_->publish(alert);

    if (jam) {
      RCLCPP_ERROR(this->get_logger(),
        "[JAM MONITOR] Speed too low for %.1fs (speed=%.3f)",
        jam_duration_, measured_speed_);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr    speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     belt_status_sub_;
  rclcpp::Publisher<conveyor_sort::msg::JamAlert>::SharedPtr jam_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double measured_speed_  = 0.0;
  double time_speed_low_  = 0.0;
  double jam_duration_    = 0.0;
  double speed_threshold_ = 0.15;
  bool   speed_received_  = false;
  std::string belt_state_ = "STOPPED";
  rclcpp::Time last_time_;
  rclcpp::Time startup_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JamMonitorNode>());
  rclcpp::shutdown();
  return 0;
}