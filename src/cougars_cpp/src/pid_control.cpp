#include "pid_control.h"

PID_Control::PID_Control() { }

void PID_Control::calibrate(float p, float i, float d, int min, int max,
                         float timer_interval, int adjust) {

  kp = p;
  ki = i;
  kd = d;
  min_output = min;
  max_output = max;
  interval = timer_interval;
  bias = adjust;

  error = 0;
  error_prior = 0;
  integral = 0;
  integral_prior = 0;
  derivative = 0;

  integral_index = 0;
  // initialize the integral array with zeros
  for (int i = 0; i < INTEGRAL_ARRAY_SIZE; i++) {
    integralArray[i] = 0;
  }
}

int PID_Control::compute(float desired, float actual) {

  // PROPORTIONAL CALCULATIONS
  error = desired - actual;

  // INTEGRAL CALCULATIONS
  // add the new error to the integral sum and subtract the oldest
  integral =
      integral_prior + (error * interval) - integralArray[integral_index];
  integralArray[integral_index] = error;
  integral_index = (integral_index + 1) % INTEGRAL_ARRAY_SIZE;
  integral_prior = integral;

  // DERIVATIVE CALCULATIONS
  derivative = (error - error_prior) / interval;
  error_prior = error;

  // SUM IT ALL TOGETHER
  float output = error * kp + integral * ki + derivative * kd + bias;

  // clamp the output so we don't exceed the limit
  if (output > max_output) {
    output = max_output;
  } else if (output < min_output) {
    output = min_output;
  }
  return (int)output;
}