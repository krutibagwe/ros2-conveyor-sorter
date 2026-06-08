#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SortingNode : public rclcpp::Node {
public:
  SortingNode() : Node("sorting_node") {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/sorter_cmd", 10,
      std::bind(&SortingNode::sort_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Sorting node started.");
  }

private:
  void sort_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string color = msg->data;
    if (color == "red") {
      RCLCPP_INFO(this->get_logger(), "ACTUATOR -> Bin 1 (Red)");
    } else if (color == "blue") {
      RCLCPP_INFO(this->get_logger(), "ACTUATOR -> Bin 2 (Blue)");
    } else if (color == "green") {
      RCLCPP_INFO(this->get_logger(), "ACTUATOR -> Bin 3 (Green)");
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown color: %s", color.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SortingNode>());
  rclcpp::shutdown();
  return 0;
}