#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node"), use_gazebo_(false) {
    this->declare_parameter<bool>("use_gazebo_camera", false);
    use_gazebo_ = this->get_parameter("use_gazebo_camera").as_bool();

    if (use_gazebo_) {
      gazebo_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/conveyor_camera/image_raw", 10,
        std::bind(&CameraNode::gazebo_image_callback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Camera node: Gazebo camera mode.");
    } else {
      // Subscribe to active_box to sync color with spawner
      active_box_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/active_box", 10,
        std::bind(&CameraNode::active_box_callback, this, std::placeholders::_1));

      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CameraNode::publish_frame, this));
      RCLCPP_INFO(this->get_logger(), "Camera node: Simulated camera mode.");
    }
  }

private:
  void gazebo_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!publisher_) {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10);
    }
    publisher_->publish(*msg);
  }

  void active_box_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Parse "color,x,y,state"
    std::string data = msg->data;
    std::vector<std::string> parts;
    std::stringstream ss(data);
    std::string token;
    while (std::getline(ss, token, ',')) parts.push_back(token);
    if (parts.size() < 4) return;

    active_color_ = parts[0];
    try {
      active_x_    = std::stod(parts[1]);
      active_y_    = std::stod(parts[2]);
    } catch (...) {}
    active_state_ = parts[3];
    has_active_box_ = true;
  }

  void publish_frame() {
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(80, 80, 80));

    // Belt outline
    cv::rectangle(frame,
      cv::Point(0, 190), cv::Point(640, 290),
      cv::Scalar(60, 60, 60), cv::FILLED);
    cv::rectangle(frame,
      cv::Point(0, 190), cv::Point(640, 290),
      cv::Scalar(40, 40, 40), 2);

    // Bin markers on frame bottom
    // Red bin at ~X=150, Blue at ~X=370, Green at ~X=560
    cv::rectangle(frame, cv::Point(100, 340), cv::Point(200, 420),
      cv::Scalar(40, 40, 180), cv::FILLED);   // Red bin (BGR)
    cv::putText(frame, "RED", cv::Point(120, 390),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);

    cv::rectangle(frame, cv::Point(280, 340), cv::Point(380, 420),
      cv::Scalar(180, 40, 40), cv::FILLED);   // Blue bin
    cv::putText(frame, "BLUE", cv::Point(295, 390),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);

    cv::rectangle(frame, cv::Point(460, 340), cv::Point(560, 420),
      cv::Scalar(40, 180, 40), cv::FILLED);   // Green bin
    cv::putText(frame, "GREEN", cv::Point(470, 390),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
      

    // Gate line
    cv::line(frame, cv::Point(530, 180), cv::Point(530, 300),
      cv::Scalar(0, 255, 255), 2);
    cv::putText(frame, "GATE", cv::Point(535, 200),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,255,255), 1);

    if (has_active_box_ && !active_color_.empty()) {
      // Map belt X (-1.35 to 1.6) → pixel X (30 to 610)
      int obj_x = static_cast<int>(
        (active_x_ - (-1.35)) / (1.6 - (-1.35)) * (610 - 30) + 30);
      obj_x = std::max(30, std::min(610, obj_x));

      // Map belt Y (0.05 to -0.65) → pixel Y (240 to 340)
      int obj_y = static_cast<int>(
        (active_y_ - 0.05) / (-0.65 - 0.05) * (340 - 240) + 240);
      obj_y = std::max(200, std::min(350, obj_y));

      // Color in BGR
      cv::Scalar box_color;
      if      (active_color_ == "red")   box_color = cv::Scalar(0,   0,   220);
      else if (active_color_ == "blue")  box_color = cv::Scalar(220, 0,   0);
      else if (active_color_ == "green") box_color = cv::Scalar(0,   220, 0);
      else                               box_color = cv::Scalar(200, 200, 200);

      // Draw box
      cv::rectangle(frame,
        cv::Point(obj_x - 25, obj_y - 25),
        cv::Point(obj_x + 25, obj_y + 25),
        box_color, cv::FILLED);

      // Label
      cv::putText(frame, active_color_,
        cv::Point(obj_x - 20, obj_y - 30),
        cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar(255, 255, 255), 1);

      // State label
      cv::putText(frame, "[" + active_state_ + "]",
        cv::Point(obj_x - 30, obj_y + 45),
        cv::FONT_HERSHEY_SIMPLEX, 0.4,
        cv::Scalar(200, 200, 0), 1);

      // Pusher arrow when pushing
      if (active_state_ == "pushing") {
        cv::arrowedLine(frame,
          cv::Point(obj_x, obj_y),
          cv::Point(obj_x, obj_y + 60),
          cv::Scalar(0, 255, 255), 3, 8, 0, 0.3);
      }
    } else {
      // No box — show waiting message
      cv::putText(frame, "Waiting for box...",
        cv::Point(220, 240),
        cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(180, 180, 180), 1);
    }

    // Header text
    cv::putText(frame, "Conveyor Camera Feed",
      cv::Point(10, 25),
      cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(200, 200, 200), 1);

    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr   gazebo_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     active_box_sub_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  bool        use_gazebo_     = false;
  std::string active_color_;
  double      active_x_       = -1.35;
  double      active_y_       =  0.05;
  std::string active_state_   = "moving";
  bool        has_active_box_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}