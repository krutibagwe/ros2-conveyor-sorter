#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "conveyor_sort/msg/object_position.hpp"
#include "conveyor_sort/msg/jam_alert.hpp"
#include "conveyor_sort/pid_controller.hpp"

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode()
    : Node("controller_node"),
      pid_(1.2, 0.1, 0.05),
      target_speed_(1.0) {

    pos_sub_ = this->create_subscription<conveyor_sort::msg::ObjectPosition>(
      "/object_position", 10,
      std::bind(&ControllerNode::pos_callback, this, std::placeholders::_1));
    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/conveyor_speed", 10,
      std::bind(&ControllerNode::speed_callback, this, std::placeholders::_1));

    conveyor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/conveyor_cmd", 10);
    sorter_cmd_pub_   = this->create_publisher<std_msgs::msg::String>(
      "/sorter_cmd", 10);
    jam_pub_          = this->create_publisher<conveyor_sort::msg::JamAlert>(
      "/jam_alert", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControllerNode::control_loop, this));

    startup_time_   = this->now();
    last_time_      = this->now();
    last_sort_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Controller node started.");
  }

private:
  void pos_callback(const conveyor_sort::msg::ObjectPosition::SharedPtr msg) {
    object_pos_       = msg->position_on_conveyor;
    current_color_    = msg->color;
    approaching_gate_ = msg->approaching_gate;
  }

  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    measured_speed_ = msg->data;
  }

  void control_loop() {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0) return;

    // PID speed command
    double cmd = pid_.compute(target_speed_, measured_speed_, dt);
    std_msgs::msg::Float32 cmd_msg;
    cmd_msg.data = static_cast<float>(cmd);
    conveyor_cmd_pub_->publish(cmd_msg);

    // Jam detection — speed only, sustained 5 seconds
    double uptime = (now - startup_time_).seconds();
    bool jam = false;
    if (uptime > 35.0) {
      if (measured_speed_ < 0.15) {
        time_speed_low_ += dt;
      } else {
        time_speed_low_ = 0.0;
      }
      jam = (time_speed_low_ > 5.0);
    } else {
      time_speed_low_ = 0.0;
    }

    if (jam) {
      conveyor_sort::msg::JamAlert alert;
      alert.header.stamp     = this->now();
      alert.jam_detected     = true;
      alert.reason           = "speed_too_low";
      alert.duration_seconds = static_cast<float>(time_speed_low_);
      jam_pub_->publish(alert);
      RCLCPP_WARN(this->get_logger(),
        "JAM DETECTED: speed=%.3f for %.1fs",
        measured_speed_, time_speed_low_);
    }

    // Sorting trigger
    double since_last_sort = (now - last_sort_time_).seconds();
    bool cooldown_ready    = (since_last_sort > 2.0);
    bool new_color         = (current_color_ != last_sorted_color_);

    if (approaching_gate_ && !current_color_.empty()
        && (cooldown_ready || new_color)) {
      std_msgs::msg::String sort_msg;
      sort_msg.data = current_color_;
      sorter_cmd_pub_->publish(sort_msg);
      RCLCPP_INFO(this->get_logger(), "Sorting: %s", current_color_.c_str());
      last_sort_time_    = now;
      last_sorted_color_ = current_color_;
    }
  }

  rclcpp::Subscription<conveyor_sort::msg::ObjectPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr             speed_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                conveyor_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 sorter_cmd_pub_;
  rclcpp::Publisher<conveyor_sort::msg::JamAlert>::SharedPtr          jam_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  PIDController pid_;
  double target_speed_      = 1.0;
  double measured_speed_    = 0.0;
  double object_pos_        = 0.0;
  double time_speed_low_    = 0.0;
  std::string current_color_;
  std::string last_sorted_color_;
  bool approaching_gate_    = false;
  rclcpp::Time last_time_;
  rclcpp::Time startup_time_;
  rclcpp::Time last_sort_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}