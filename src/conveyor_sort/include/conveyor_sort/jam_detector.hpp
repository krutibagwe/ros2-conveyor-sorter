#pragma once
#include <string>
#include <chrono>

class JamDetector {
public:
  JamDetector(double speed_threshold, double position_timeout_sec);

  // Returns true if jam is detected
  bool update(double commanded_speed, double measured_speed,
              double object_position, double dt);

  std::string getReason() const;
  double getJamDuration() const;
  void reset();

private:
  double speed_threshold_;
  double position_timeout_sec_;
  double last_object_position_;
  double time_position_unchanged_;
  double jam_duration_;
  bool jam_active_;
  std::string reason_;
};