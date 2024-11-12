/**
 * @author Nelson Durrant
 * @date September 2024
 *
 * This class is a simple implementation of a PID controller based on the BYU ECEn 483 approach.
 */
class PID {

public:
  /**
   * Creates a new PID controller.
   */
  PID() {};

  /**
   * This method initializes the PID controller with the given constants.
   *
   * @param p The proportional constant.
   * @param i The integral constant.
   * @param d The derivative constant.
   * @param min The minimum output value.
   * @param max The maximum output value.
   * @param interval The interval at which the PID controller is called.
   */
  void initialize(float p, float i, float d, float min, float max, float interval) {
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

    float sigma = 0.05; // cutoff freq for dirty derivative
    beta = (2 * sigma - interval) / (2 * sigma + interval);  
  }

  /**
   * Simple PID control, based on the approach in BYU ECEn 483.
   *
   * @param x_r The desired value.
   * @param x The actual value.
   * @return The output value.
   */
  float compute(float x_r, float x) {

    float error = x_r - x;

    // integrate error in x
    integrator = integrator + (interval / 2) * (error + error_d1);

    // differentiate x
    x_dot = beta * x_dot + (1 - beta) * ((x - x_d1) / interval);

    // calculate the force
    float force_unsat = kp * error + ki * integrator - kd * x_dot;

    // saturate the force
    float force_sat;
    if (force_unsat > max_output) {
      force_sat = max_output;
    } else if (force_unsat < min_output) {
      force_sat = min_output;
    } else {
      force_sat = force_unsat;
    }

    // integrator anti-windup
    if (ki != 0.0) {
      integrator = integrator + interval / ki * (force_sat - force_unsat);
    }

    // update delayed variables
    error_d1 = error;
    x_d1 = x;

    return force_sat;
  }

private:
  
  float kp;
  float ki;
  float kd;
  float min_output;
  float max_output;
  float interval;

  float beta = 0.0;
  float x_dot = 0.0;
  float x_d1 = 0.0;
  float error_d1 = 0.0;
  float integrator = 0.0;
};