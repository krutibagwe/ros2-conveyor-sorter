#include "conveyor_sort/jam_detector.hpp"

JamDetector::JamDetector(double speed_threshold, double position_timeout_sec)
  : speed_threshold_(speed_threshold),
    position_timeout_sec_(position_timeout_sec),
    last_object_position_(0.0),
    time_position_unchanged_(0.0),
    jam_duration_(0.0),
    jam_active_(false),
    reason_("none") {}

bool JamDetector::update(double commanded_speed, double measured_speed,
                          double object_position, double dt) {
  // Condition 1: conveyor commanded to move but speed too low
  bool speed_jam = (commanded_speed > 0.1) && (measured_speed < speed_threshold_);

  // Condition 2: object not moving
  bool position_unchanged = (std::abs(object_position - last_object_position_) < 0.001);
  if (position_unchanged && commanded_speed > 0.1) {
    time_position_unchanged_ += dt;
  } else {
    time_position_unchanged_ = 0.0;
  }
  last_object_position_ = object_position;

  bool position_jam = (time_position_unchanged_ >= position_timeout_sec_);

  if (speed_jam || position_jam) {
    jam_active_ = true;
    jam_duration_ += dt;
    reason_ = speed_jam ? "speed_too_low" : "object_not_moving";
  } else {
    jam_active_ = false;
    jam_duration_ = 0.0;
    reason_ = "none";
  }

  return jam_active_;
}

std::string JamDetector::getReason() const { return reason_; }
double JamDetector::getJamDuration() const { return jam_duration_; }
void JamDetector::reset() {
  last_object_position_ = 0.0;
  time_position_unchanged_ = 0.0;
  jam_duration_ = 0.0;
  jam_active_ = false;
  reason_ = "none";
}