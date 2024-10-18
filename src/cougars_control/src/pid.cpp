#define INTEGRAL_ARRAY_SIZE 20 // memory size of integral term

/**
 * @brief A simple PID controller class.
 * @author Nelson Durrant
 * @date September 2024
 *
 * This class is a simple implementation of a PID controller. It is designed to
 * be used in a feedback loop to control a system. The PID controller is
 * initialized with the PID constants, the output limits, the interval at which
 * the PID controller is called, and the bias value. The PID controller can then
 * be used to compute the output value based on the desired and actual values.
 */
class PID {

public:
  /**
   * Creates a new PID controller.
   */
  PID() {};

  /**
   * This method calibrates the PID controller with the specified PID constants,
   * output limits, interval, and bias value.
   *
   * @param p The proportional constant.
   * @param i The integral constant.
   * @param d The derivative constant.
   * @param min The minimum output value.
   * @param max The maximum output value.
   */
  void calibrate(float p, float i, float d, int min, int max, float interval) {
    kp = p;
    ki = i;
    kd = d;
    min_output = min;
    max_output = max;
    interval = interval;

    x_dot = 0.0;  // estimated derivative of x
    x_d1 = 0.0;  // x delayed by one sample
    error_d1 = 0.0;  // error delayed by one sample
    integrator = 0.0; // integrator
  }

  /**
   * This method computes the output value based on the desired and actual
   * values. The output value is computed using the PID constants, output
   * limits, interval, and bias value that were previously set.
   *
   * @param desired The desired value.
   * @param actual The actual value.
   * @return The output value.
   */
  float compute(float desired, float actual) {

    error = desired - actual;

    // integrate error in x
    integrator = integrator + (interval / 2) * (error + error_d1);

    // differentiate x
    x_dot = beta * x_dot + (1 - beta) * ((x - x_d1) / interval);

    // calculate the force
    force_unsat = kp * error + ki * integrator - kd * x_dot;

    // saturate the force

    // integrator anti-windup
    if (ki != 0.0) {
      integrator = integrator + interval / ki * (force_sat - force_unsat);
    }

    // update delayed variables
    error_d1 = error
    x_d1 = actual

    return force_sat
  }

private:
  
  float kp;
  float ki;
  float kd;
  float min_output;
  float max_output;

  float x_dot = 0.0
  float x_d1 = 0.0
  float error_d1 = 0.0
  float integrator = 0.0
};