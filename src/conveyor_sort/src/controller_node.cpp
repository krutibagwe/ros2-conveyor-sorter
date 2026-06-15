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
      target_speed_(0.0),
      belt_state_("STOPPED") {

    pos_sub_ = this->create_subscription<conveyor_sort::msg::ObjectPosition>(
      "/object_position", 10,
      std::bind(&ControllerNode::pos_callback, this, std::placeholders::_1));
    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/conveyor_speed", 10,
      std::bind(&ControllerNode::speed_callback, this, std::placeholders::_1));
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/belt_command", 10,
      std::bind(&ControllerNode::cmd_callback, this, std::placeholders::_1));
    reset_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/belt_reset", 10,
      std::bind(&ControllerNode::reset_callback, this, std::placeholders::_1));

    conveyor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/conveyor_cmd", 10);
    sorter_cmd_pub_   = this->create_publisher<std_msgs::msg::String>(
      "/sorter_cmd", 10);
    jam_pub_          = this->create_publisher<conveyor_sort::msg::JamAlert>(
      "/jam_alert", 10);
    status_pub_       = this->create_publisher<std_msgs::msg::String>(
      "/belt_status", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControllerNode::control_loop, this));
    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ControllerNode::publish_status, this));

    startup_time_   = this->now();
    last_time_      = this->now();
    last_sort_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "Controller node started. State: STOPPED. Send START to begin.");
  }

private:
  void cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string cmd = msg->data;

    if (cmd == "START") {
      if (belt_state_ == "STOPPED") {
        belt_state_     = "RUNNING";
        target_speed_   = 1.0;
        time_speed_low_ = 0.0;
        jam_active_     = false;
        pid_.reset();
        RCLCPP_INFO(this->get_logger(), "Belt STARTED.");
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Cannot START from state %s. "
          "Use RESUME if paused/jammed, or RESET first.",
          belt_state_.c_str());
      }

    } else if (cmd == "STOP") {
      if (belt_state_ == "RUNNING" || belt_state_ == "PAUSED") {
        belt_state_     = "STOPPED";
        target_speed_   = 0.0;
        time_speed_low_ = 0.0;
        pid_.reset();
        RCLCPP_INFO(this->get_logger(),
          "Belt STOPPED. Box frozen on belt. "
          "Send RESET then START to begin again.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Belt already stopped.");
      }

    } else if (cmd == "PAUSE") {
      if (belt_state_ == "RUNNING") {
        belt_state_     = "PAUSED";
        target_speed_   = 0.0;
        time_speed_low_ = 0.0;  // prevent false jam after pause
        pid_.reset();
        RCLCPP_INFO(this->get_logger(), "Belt PAUSED — frozen in place.");
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Belt not running — cannot pause.");
      }

    } else if (cmd == "RESUME") {
      if (belt_state_ == "PAUSED") {
        belt_state_     = "RUNNING";
        target_speed_   = 1.0;
        time_speed_low_ = 0.0;
        pid_.reset();
        RCLCPP_INFO(this->get_logger(), "Belt RESUMED from pause.");
      } else if (jam_active_) {
        jam_active_     = false;
        belt_state_     = "RUNNING";
        target_speed_   = 1.0;
        time_speed_low_ = 0.0;
        pid_.reset();
        RCLCPP_INFO(this->get_logger(),
          "Jam cleared by RESUME — belt running.");
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Belt not paused/jammed — RESUME has no effect.");
      }
    }
  }

  void reset_callback(const std_msgs::msg::String::SharedPtr /*msg*/) {
    belt_state_       = "STOPPED";
    target_speed_     = 0.0;
    measured_speed_   = 0.0;
    object_pos_       = 0.0;
    time_speed_low_   = 0.0;
    jam_active_       = false;
    approaching_gate_ = false;
    current_color_.clear();
    last_sorted_color_.clear();
    pid_.reset();
    RCLCPP_INFO(this->get_logger(),
      "Controller RESET. Belt STOPPED. Send START to begin again.");
  }

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

    // PID only when RUNNING and not jammed
    if (belt_state_ == "RUNNING" && !jam_active_) {
      double cmd = pid_.compute(target_speed_, measured_speed_, dt);
      std_msgs::msg::Float32 cmd_msg;
      cmd_msg.data = static_cast<float>(cmd);
      conveyor_cmd_pub_->publish(cmd_msg);
    } else {
      std_msgs::msg::Float32 cmd_msg;
      cmd_msg.data = 0.0f;
      conveyor_cmd_pub_->publish(cmd_msg);
    }

    // Jam detection — only when RUNNING, not already jammed
    double uptime = (now - startup_time_).seconds();
    if (belt_state_ == "RUNNING" && !jam_active_ && uptime > 35.0) {
      if (measured_speed_ < 0.15) {
        time_speed_low_ += dt;
      } else {
        time_speed_low_ = 0.0;
      }
      if (time_speed_low_ > 5.0) {
        jam_active_ = true;
        conveyor_sort::msg::JamAlert alert;
        alert.header.stamp     = this->now();
        alert.jam_detected     = true;
        alert.reason           = "speed_too_low";
        alert.duration_seconds = static_cast<float>(time_speed_low_);
        jam_pub_->publish(alert);
        RCLCPP_WARN(this->get_logger(),
          "JAM DETECTED: speed=%.3f. "
          "Belt frozen until RESUME or RESET.",
          measured_speed_);
      }
    } else if (belt_state_ != "RUNNING") {
      time_speed_low_ = 0.0;
    }

    // Sorting — only when RUNNING and not jammed
    if (belt_state_ == "RUNNING" && !jam_active_) {
      double since_last = (now - last_sort_time_).seconds();
      bool cooldown     = (since_last > 2.0);
      bool new_color    = (current_color_ != last_sorted_color_);

      if (approaching_gate_ && !current_color_.empty()
          && (cooldown || new_color)) {
        std_msgs::msg::String sort_msg;
        sort_msg.data = current_color_;
        sorter_cmd_pub_->publish(sort_msg);
        RCLCPP_INFO(this->get_logger(),
          "Sorting: %s", current_color_.c_str());
        last_sort_time_    = now;
        last_sorted_color_ = current_color_;
      }
    }
  }

  void publish_status() {
    std_msgs::msg::String msg;
    msg.data = jam_active_ ? "JAMMED" : belt_state_;
    status_pub_->publish(msg);
  }

  rclcpp::Subscription<conveyor_sort::msg::ObjectPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr             speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              reset_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                conveyor_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 sorter_cmd_pub_;
  rclcpp::Publisher<conveyor_sort::msg::JamAlert>::SharedPtr          jam_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  PIDController pid_;
  double target_speed_      = 0.0;
  double measured_speed_    = 0.0;
  double object_pos_        = 0.0;
  double time_speed_low_    = 0.0;
  bool   jam_active_        = false;
  std::string belt_state_;
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