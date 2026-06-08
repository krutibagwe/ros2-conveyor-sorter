#pragma once

class PIDController {
public:
  PIDController(double kp, double ki, double kd);

  double compute(double setpoint, double measured, double dt);
  void reset();
  void setGains(double kp, double ki, double kd);
  void setOutputLimits(double min, double max);

private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
  double output_min_;
  double output_max_;
};