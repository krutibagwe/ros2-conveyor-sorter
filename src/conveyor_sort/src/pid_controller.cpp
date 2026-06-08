#include "conveyor_sort/pid_controller.hpp"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd)
  : kp_(kp), ki_(ki), kd_(kd),
    prev_error_(0.0), integral_(0.0),
    output_min_(-100.0), output_max_(100.0) {}

double PIDController::compute(double setpoint, double measured, double dt) {
  if (dt <= 0.0) return 0.0;

  double error = setpoint - measured;
  integral_ += error * dt;
  double derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  return std::clamp(output, output_min_, output_max_);
}

void PIDController::reset() {
  prev_error_ = 0.0;
  integral_ = 0.0;
}

void PIDController::setGains(double kp, double ki, double kd) {
  kp_ = kp; ki_ = ki; kd_ = kd;
}

void PIDController::setOutputLimits(double min, double max) {
  output_min_ = min;
  output_max_ = max;
}