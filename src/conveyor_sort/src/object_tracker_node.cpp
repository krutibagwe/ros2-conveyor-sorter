#include "rclcpp/rclcpp.hpp"
#include "conveyor_sort/msg/detected_object.hpp"
#include "conveyor_sort/msg/object_position.hpp"

class ObjectTrackerNode : public rclcpp::Node {
public:
  ObjectTrackerNode() : Node("object_tracker_node"),
    gate_x_(550.0f), conveyor_speed_px_per_sec_(50.0f) {
    sub_ = this->create_subscription<conveyor_sort::msg::DetectedObject>(
      "/detected_color", 10,
      std::bind(&ObjectTrackerNode::detect_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<conveyor_sort::msg::ObjectPosition>(
      "/object_position", 10);
    RCLCPP_INFO(this->get_logger(), "Object tracker node started.");
  }

private:
  void detect_callback(const conveyor_sort::msg::DetectedObject::SharedPtr msg) {
    conveyor_sort::msg::ObjectPosition pos_msg;
    pos_msg.header.stamp = this->now();
    pos_msg.color = msg->color;
    pos_msg.position_on_conveyor = msg->centroid_x;

    float dist_to_gate = gate_x_ - msg->centroid_x;
    pos_msg.estimated_time_to_gate =
      (conveyor_speed_px_per_sec_ > 0.0f)
        ? dist_to_gate / conveyor_speed_px_per_sec_
        : 999.0f;
    pos_msg.approaching_gate = (dist_to_gate > 0.0f && dist_to_gate < 150.0f);

    pub_->publish(pos_msg);
  }

  rclcpp::Subscription<conveyor_sort::msg::DetectedObject>::SharedPtr sub_;
  rclcpp::Publisher<conveyor_sort::msg::ObjectPosition>::SharedPtr pub_;
  float gate_x_;
  float conveyor_speed_px_per_sec_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTrackerNode>());
  rclcpp::shutdown();
  return 0;
}