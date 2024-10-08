#ifndef PID_CLASS
#define PID_CLASS

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
   * @brief Creates a new PID controller.
   *
   * This constructor creates a new PID controller with default values.
   */
  PID();

  /**
   * @brief Calibrates the PID controller.
   *
   * This method calibrates the PID controller with the specified PID constants,
   * output limits, interval, and bias value.
   *
   * @param p The proportional constant.
   * @param i The integral constant.
   * @param d The derivative constant.
   * @param min The minimum output value.
   * @param max The maximum output value.
   * @param timer_interval The interval at which the PID controller is called.
   * @param adjust The bias value.
   */
  void calibrate(float p, float i, float d, int min, int max,
                 float timer_interval, int adjust);

  /**
   * @brief Computes the output value.
   *
   * This method computes the output value based on the desired and actual
   * values. The output value is computed using the PID constants, output
   * limits, interval, and bias value that were previously set.
   *
   * @param desired The desired value.
   * @param actual The actual value.
   * @return The output value.
   */
  int compute(float desired, float actual);

private:
  /**
   * @brief The proportional constant.
   *
   * This constant is used to compute the proportional term of the PID
   * controller.
   */
  float kp;

  /**
   * @brief The integral constant.
   *
   * This constant is used to compute the integral term of the PID controller.
   */
  float ki;

  /**
   * @brief The derivative constant.
   *
   * This constant is used to compute the derivative term of the PID controller.
   */
  float kd;

  /**
   * @brief The minimum output value.
   *
   * This value is the minimum output value that the PID controller can output.
   */
  int min_output;

  /**
   * @brief The maximum output value.
   *
   * This value is the maximum output value that the PID controller can output.
   */
  int max_output;

  float interval;
  int bias;
  float integral;
  float integral_prior;
  float error;
  float error_prior;
  float derivative;

  int integral_index;
  float integralArray[INTEGRAL_ARRAY_SIZE];
};

#endif // PID_CLASS