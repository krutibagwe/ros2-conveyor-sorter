#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "conveyor_sort/pid_controller.hpp"

class ConveyorNode : public rclcpp::Node {
public:
  ConveyorNode()
    : Node("conveyor_node"),
      pid_(1.0, 0.05, 0.02),
      simulated_speed_(0.0),
      last_time_(this->now()) {
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/conveyor_cmd", 10,
      std::bind(&ConveyorNode::cmd_callback, this, std::placeholders::_1));
    speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/conveyor_speed", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ConveyorNode::simulate, this));
    RCLCPP_INFO(this->get_logger(), "Conveyor node started.");
  }

private:
  void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    commanded_speed_ = msg->data;
  }

  void simulate() {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0) return;

    // Simulate motor inertia: speed ramps toward command
    double error = commanded_speed_ - simulated_speed_;
    simulated_speed_ += error * 0.3 * dt * 10.0;

    // Add small noise
    simulated_speed_ += ((rand() % 100) - 50) * 0.0002;

    std_msgs::msg::Float32 spd;
    spd.data = static_cast<float>(simulated_speed_);
    speed_pub_->publish(spd);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  PIDController pid_;
  double simulated_speed_ = 0.0;
  double commanded_speed_ = 0.0;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConveyorNode>());
  rclcpp::shutdown();
  return 0;
}