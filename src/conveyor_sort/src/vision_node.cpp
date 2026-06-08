#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "conveyor_sort/msg/detected_object.hpp"
#include <opencv2/opencv.hpp>

class VisionNode : public rclcpp::Node {
public:
  VisionNode() : Node("vision_node") {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&VisionNode::image_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<conveyor_sort::msg::DetectedObject>(
      "/detected_color", 10);
    RCLCPP_INFO(this->get_logger(), "Vision node started.");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // HSV ranges
    struct ColorDef {
      std::string name;
      cv::Scalar lower, upper;
    };

    std::vector<ColorDef> colors = {
      {"red",   cv::Scalar(0, 100, 100),   cv::Scalar(10, 255, 255)},
      {"blue",  cv::Scalar(110, 100, 100), cv::Scalar(130, 255, 255)},
      {"green", cv::Scalar(50, 100, 100),  cv::Scalar(70, 255, 255)},
    };

    for (auto &cd : colors) {
      cv::Mat mask;
      cv::inRange(hsv, cd.lower, cd.upper, mask);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      if (contours.empty()) continue;

      // Largest contour
      auto &c = *std::max_element(contours.begin(), contours.end(),
        [](const auto &a, const auto &b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });

      if (cv::contourArea(c) < 500) continue;

      cv::Rect bbox = cv::boundingRect(c);
      cv::Moments M = cv::moments(c);

      conveyor_sort::msg::DetectedObject det_msg;
      det_msg.header.stamp = this->now();
      det_msg.color = cd.name;
      det_msg.centroid_x = (M.m10 / M.m00);
      det_msg.centroid_y = (M.m01 / M.m00);
      det_msg.bbox_x = bbox.x;
      det_msg.bbox_y = bbox.y;
      det_msg.bbox_width = bbox.width;
      det_msg.bbox_height = bbox.height;

      pub_->publish(det_msg);
      return; // One object at a time for now
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<conveyor_sort::msg::DetectedObject>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}