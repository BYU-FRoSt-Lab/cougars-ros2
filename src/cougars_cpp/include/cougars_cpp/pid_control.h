#ifndef PID_CONTROL
#define PID_CONTROL

#define INTEGRAL_ARRAY_SIZE 20 // memory size of integral term

class PID_Control {

public:
  PID_Control();
  void calibrate(float p, float i, float d, int min, int max, float timer_interval,
              int adjust);
  int compute(float desired, float actual);

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
  float error;
  float error_prior;
  float derivative;

  int integral_index;
  float integralArray[INTEGRAL_ARRAY_SIZE];
};

#endif // PID_CONTROL