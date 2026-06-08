#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "conveyor_sort/msg/object_position.hpp"
#include "conveyor_sort/msg/jam_alert.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <vector>

class RvizDisplayNode : public rclcpp::Node {
public:
  RvizDisplayNode() : Node("rviz_display_node") {

    obj_sub_ = this->create_subscription<conveyor_sort::msg::ObjectPosition>(
      "/object_position", 10,
      std::bind(&RvizDisplayNode::obj_callback, this, std::placeholders::_1));
    jam_sub_ = this->create_subscription<conveyor_sort::msg::JamAlert>(
      "/jam_alert", 10,
      std::bind(&RvizDisplayNode::jam_callback, this, std::placeholders::_1));
    sort_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/sorter_cmd", 10,
      std::bind(&RvizDisplayNode::sort_callback, this, std::placeholders::_1));
    count_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/bin_counts", 10,
      std::bind(&RvizDisplayNode::count_callback, this, std::placeholders::_1));
    active_box_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/active_box", 10,
      std::bind(&RvizDisplayNode::active_box_callback, this, std::placeholders::_1));
    jam_sim_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/jam_sim_status", 10,
      std::bind(&RvizDisplayNode::jam_sim_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/conveyor_markers", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RvizDisplayNode::publish_markers, this));

    RCLCPP_INFO(this->get_logger(), "RViz display node started.");
  }

private:
  // ── Callbacks ──────────────────────────────────────────────────────────────

  void obj_callback(const conveyor_sort::msg::ObjectPosition::SharedPtr msg) {
    object_pos_       = msg->position_on_conveyor;
    current_color_    = msg->color;
    approaching_gate_ = msg->approaching_gate;
    object_visible_   = true;
  }

  void jam_callback(const conveyor_sort::msg::JamAlert::SharedPtr msg) {
    jam_detected_ = msg->jam_detected;
    jam_reason_   = msg->reason;
    jam_duration_ = msg->duration_seconds;
  }

  void sort_callback(const std_msgs::msg::String::SharedPtr msg) {
    last_sorted_ = msg->data;
  }

  void count_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string data = msg->data;
    auto parse = [&](const std::string & key) -> int {
      auto pos = data.find(key + ":");
      if (pos == std::string::npos) return 0;
      pos += key.size() + 1;
      auto end = data.find(',', pos);
      try {
        return std::stoi(data.substr(
          pos, end == std::string::npos ? end : end - pos));
      } catch (...) { return 0; }
    };
    bin_red_   = parse("RED");
    bin_blue_  = parse("BLUE");
    bin_green_ = parse("GREEN");
    bin_misc_  = parse("MISC");
  }

  void active_box_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string data = msg->data;
    std::vector<std::string> parts;
    std::stringstream ss(data);
    std::string token;
    while (std::getline(ss, token, ',')) parts.push_back(token);
    if (parts.size() < 4) return;
    active_color_ = parts[0];
    try {
      active_x_ = std::stod(parts[1]);
      active_y_ = std::stod(parts[2]);
    } catch (...) {}
    active_state_       = parts[3];
    active_box_visible_ = true;
  }

  void jam_sim_callback(const std_msgs::msg::String::SharedPtr msg) {
    jam_simulated_ = (msg->data == "JAM_ACTIVE");
  }

  // ── Marker helpers ─────────────────────────────────────────────────────────

  visualization_msgs::msg::Marker make_marker(
    int id, int type,
    double px, double py, double pz,
    double sx, double sy, double sz,
    float r, float g, float b, float a,
    const std::string & frame = "world")
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id    = frame;
    m.header.stamp       = this->now();
    m.ns                 = "conveyor";
    m.id                 = id;
    m.type               = type;
    m.action             = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x    = px;
    m.pose.position.y    = py;
    m.pose.position.z    = pz;
    m.pose.orientation.w = 1.0;
    m.scale.x = sx; m.scale.y = sy; m.scale.z = sz;
    m.color.r = r;  m.color.g = g;  m.color.b = b; m.color.a = a;
    return m;
  }

  visualization_msgs::msg::Marker make_text(
    int id,
    double px, double py, double pz,
    const std::string & text,
    float r, float g, float b,
    float size = 0.12f)
  {
    auto m = make_marker(id,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      px, py, pz, 0.0, 0.0, size,
      r, g, b, 1.0f);
    m.text = text;
    return m;
  }

  void color_from_name(const std::string & name,
    float & r, float & g, float & b)
  {
    if      (name == "red")    { r=0.9f; g=0.1f; b=0.1f; }
    else if (name == "blue")   { r=0.1f; g=0.3f; b=0.9f; }
    else if (name == "green")  { r=0.1f; g=0.8f; b=0.1f; }
    else if (name == "purple") { r=0.6f; g=0.1f; b=0.8f; }
    else if (name == "yellow") { r=0.9f; g=0.9f; b=0.1f; }
    else if (name == "orange") { r=0.9f; g=0.5f; b=0.1f; }
    else if (name == "white")  { r=0.9f; g=0.9f; b=0.9f; }
    else                       { r=0.7f; g=0.7f; b=0.7f; }
  }

  void push_arrow(visualization_msgs::msg::MarkerArray & arr,
    int id, double x, bool active)
  {
    float r, g, b, a;
    double sx;
    if (active) {
      r=1.0f; g=1.0f; b=0.0f; a=1.0f; sx=0.32;
    } else {
      r=0.5f; g=0.5f; b=0.5f; a=0.25f; sx=0.16;
    }
    arr.markers.push_back(make_marker(
      id, visualization_msgs::msg::Marker::ARROW,
      x, 0.18, 0.58,
      sx, 0.05, 0.05,
      r, g, b, a));
  }

  // ── Main publish loop ──────────────────────────────────────────────────────

  void publish_markers() {
    visualization_msgs::msg::MarkerArray arr;

    // 1. Conveyor belt
    arr.markers.push_back(make_marker(
      0, visualization_msgs::msg::Marker::CUBE,
      0.175, 0.05, 0.5,
      3.55, 0.4, 0.05,
      0.25f, 0.25f, 0.25f, 1.0f));

    // 2. Back rail
    arr.markers.push_back(make_marker(
      1, visualization_msgs::msg::Marker::CUBE,
      0.175, 0.25, 0.6,
      3.55, 0.03, 0.12,
      0.5f, 0.5f, 0.5f, 1.0f));

    // 3. Pusher arrows — red, blue, green, misc (no pusher for misc)
    bool pushing       = (active_state_ == "pushing");
    bool is_misc_push  = pushing && (active_color_ != "red" &&
                                     active_color_ != "blue" &&
                                     active_color_ != "green");
    push_arrow(arr, 3, -0.8, pushing && active_color_ == "red");
    push_arrow(arr, 4,  0.2, pushing && active_color_ == "blue");
    push_arrow(arr, 5,  1.2, pushing && active_color_ == "green");

    // Misc "drop" indicator at end of belt
    arr.markers.push_back(make_marker(
      25, visualization_msgs::msg::Marker::ARROW,
      1.7, 0.18, 0.58,
      is_misc_push ? 0.32 : 0.16,
      0.05, 0.05,
      is_misc_push ? 0.8f : 0.5f,
      is_misc_push ? 0.8f : 0.5f,
      is_misc_push ? 0.1f : 0.5f,
      is_misc_push ? 1.0f : 0.25f));

    // 4. Bins on the side
    // Red bin
    arr.markers.push_back(make_marker(
      6, visualization_msgs::msg::Marker::CUBE,
      -0.8, -0.65, 0.12,
      0.35, 0.3, 0.25,
      0.7f, 0.1f, 0.1f, 0.6f));
    // Blue bin
    arr.markers.push_back(make_marker(
      7, visualization_msgs::msg::Marker::CUBE,
      0.2, -0.65, 0.12,
      0.35, 0.3, 0.25,
      0.1f, 0.2f, 0.8f, 0.6f));
    // Green bin
    arr.markers.push_back(make_marker(
      8, visualization_msgs::msg::Marker::CUBE,
      1.2, -0.65, 0.12,
      0.35, 0.3, 0.25,
      0.1f, 0.7f, 0.1f, 0.6f));
    // Misc bin — grey, at end of belt
    arr.markers.push_back(make_marker(
      9, visualization_msgs::msg::Marker::CUBE,
      1.7, -0.65, 0.12,
      0.35, 0.3, 0.25,
      0.6f, 0.6f, 0.6f, 0.6f));

    // 5. Bin labels with live counts
    arr.markers.push_back(make_text(
      10, -0.8, -0.95, 0.52,
      "RED\n" + std::to_string(bin_red_),
      0.9f, 0.3f, 0.3f, 0.16f));
    arr.markers.push_back(make_text(
      11, 0.2, -0.95, 0.52,
      "BLUE\n" + std::to_string(bin_blue_),
      0.3f, 0.4f, 0.9f, 0.16f));
    arr.markers.push_back(make_text(
      12, 1.2, -0.95, 0.52,
      "GREEN\n" + std::to_string(bin_green_),
      0.2f, 0.8f, 0.2f, 0.16f));
    arr.markers.push_back(make_text(
      14, 1.7, -0.95, 0.52,
      "MISC\n" + std::to_string(bin_misc_),
      0.7f, 0.7f, 0.7f, 0.16f));

    // 6. Total count
    int total = bin_red_ + bin_blue_ + bin_green_ + bin_misc_;
    arr.markers.push_back(make_text(
      13, 0.2, 0.05, 2.5,
      "Total sorted: " + std::to_string(total),
      0.9f, 0.9f, 0.9f, 0.13f));

    // 7. Active box
    if (active_box_visible_ && !active_color_.empty()) {
      float r, g, b;
      color_from_name(active_color_, r, g, b);

      arr.markers.push_back(make_marker(
        20, visualization_msgs::msg::Marker::CUBE,
        active_x_, active_y_, 0.6,
        0.1, 0.1, 0.1,
        r, g, b, 1.0f));

      arr.markers.push_back(make_text(
        21, active_x_, active_y_, 0.82,
        active_color_ + "\n[" + active_state_ + "]",
        r, g, b, 0.12f));
    }

    // 8. Jam status — simulated jam takes priority over sensor jam
    bool show_jam    = jam_detected_ || jam_simulated_;
    std::string jam_text = jam_simulated_
      ? "SIMULATED JAM!\nBelt Frozen\n(auto-clears in 8s)"
      : ("JAM DETECTED\n" + jam_reason_
          + "\nDuration: "
          + std::to_string(static_cast<int>(jam_duration_)) + "s");

    if (show_jam) {
      arr.markers.push_back(make_marker(
        30, visualization_msgs::msg::Marker::CUBE,
        0.175, 0.05, 1.2,
        3.55, 0.4, 0.06,
        1.0f, 0.0f, 0.0f, 0.7f));
      arr.markers.push_back(make_text(
        31, 0.175, 0.05, 1.65,
        jam_text,
        1.0f, 0.2f, 0.2f, 0.18f));
    } else {
      arr.markers.push_back(make_marker(
        30, visualization_msgs::msg::Marker::CUBE,
        0.175, 0.05, 1.1,
        3.55, 0.4, 0.03,
        0.0f, 1.0f, 0.0f, 0.3f));
      arr.markers.push_back(make_text(
        31, 0.175, 0.05, 2.0,
        "System OK",
        0.2f, 1.0f, 0.2f, 0.14f));
    }

    marker_pub_->publish(arr);
  }

  // ── Members ────────────────────────────────────────────────────────────────

  rclcpp::Subscription<conveyor_sort::msg::ObjectPosition>::SharedPtr obj_sub_;
  rclcpp::Subscription<conveyor_sort::msg::JamAlert>::SharedPtr       jam_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              sort_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              count_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              active_box_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              jam_sim_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double object_pos_          = 320.0;
  std::string current_color_;
  bool object_visible_        = false;
  bool approaching_gate_      = false;

  double active_x_            = -1.35;
  double active_y_            =  0.05;
  std::string active_color_;
  std::string active_state_   = "moving";
  bool active_box_visible_    = false;

  bool jam_detected_          = false;
  bool jam_simulated_         = false;
  std::string jam_reason_;
  float jam_duration_         = 0.0f;

  std::string last_sorted_;

  int bin_red_   = 0;
  int bin_blue_  = 0;
  int bin_green_ = 0;
  int bin_misc_  = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizDisplayNode>());
  rclcpp::shutdown();
  return 0;
}