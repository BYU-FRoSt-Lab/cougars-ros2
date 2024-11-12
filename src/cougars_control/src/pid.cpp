/**
 * @brief A simple PID controller class.
 * @author Nelson Durrant, Braden Meyers
 * @date September 2024
 *
 * This class is a simple implementation of a PID controller. It is designed to
 * be used in a feedback loop to control a system. The PID controller is
 * initialized with the PID constants, the output limits, the interval at which
 * the PID controller is called, and the bias value. The PID controller can then
 * be used to compute the output value based on the desired and actual values.
 */
#define INTEGRAL_ARRAY_SIZE 20 // memory size of integral term

class PID {

public:
  PID() {};

  void calibrate(float p, float i, float d, int min, int max,
                 float timer_interval, int adjust, float integral_threshold = 5.0, float integral_time = 3.0) {
    kp = p;
    ki = i;
    kd = d;
    min_output = min;
    max_output = max;
    interval = timer_interval * 0.001;
    bias = adjust;

    // Reset error and integral terms
    reset();

    // Integral threshold parameters
    integral_threshold_ = integral_threshold; //Error threshold value where integrator is added
    integral_time_ = integral_time;  //Time in seconds that error is below threshold to start adding
    integral_time_count_ = 0;
  }

  // Main compute method
  float compute(float desired, float actual) {
    float error = desired - actual;
    updateIntegral(error);
    float derivative = computeDerivative(error);

    // Compute output and apply saturation
    float output = saturate(error * kp + integral * ki - derivative * kd + bias);
    error_prior = error;
    return output;
  }

  // Overloaded compute method using externally provided derivative
  float compute(float desired, float actual, float measured_derivative) {
    float error = desired - actual;
    updateIntegral(error);

    // Compute output and apply saturation
    float output = saturate(error * kp + integral * ki - measured_derivative * kd + bias);
    error_prior = error;
    return output;
  }

  // Resets the controller state
  void reset() {
    error_prior = 0;
    integral = 0;
    integral_prior = 0;
    integral_index = 0;
    integral_time_count_ = 0;

    for (int i = 0; i < INTEGRAL_ARRAY_SIZE; i++) {
      integralArray[i] = 0;
    }
  }

private:
  float kp;
  float ki;
  float kd;
  int min_output;
  int max_output;
  float interval;
  int bias;
  float integral;
  float integral_prior;
  float error_prior;

  int integral_index;
  float integralArray[INTEGRAL_ARRAY_SIZE];

  // Integral threshold parameters
  float integral_threshold_;
  float integral_time_;
  float integral_time_count_;

  // Limits the output within the specified min and max
  float saturate(float value) const {
    if (value > max_output) return max_output;
    if (value < min_output) return min_output;
    return value;
  }

  // Updates the integral only if the error is within the threshold for the specified time
  void updateIntegral(float error) {
    if (std::abs(error) < integral_threshold_) {
      integral_time_count_ += interval;
      if (integral_time_count_ >= integral_time_) {
        // Update integral using a moving window of errors
        integral = integral_prior + (error * interval) - integralArray[integral_index];
        integralArray[integral_index] = error;
        integral_index = (integral_index + 1) % INTEGRAL_ARRAY_SIZE;
        integral_prior = integral;
      }
    } else {
      integral_time_count_ = 0;  // Reset count if error goes out of threshold
    }
  }

  // Computes the derivative term
  float computeDerivative(float error) const {
    return (error - error_prior) / interval;
  }
};
