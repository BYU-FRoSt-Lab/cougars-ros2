/**
 * @author Nelson Durrant
 * @date September 2024
 *
 * This class is a simple implementation of a PID controller based on the BYU
 * ECEn 483 approach.
 * 
 * It uses the integrator on/off anti-windup strategy.
 */

#include <iostream>

class PID {

public:
  /**
   * Creates a new PID controller.
   */
  PID(){};

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
  void initialize(float p, float i, float d, float min, float max,
                  float interval, float scalar=1.0) {
    this->kp = p;
    this->ki = i;
    this->kd = d;
    this->min_output = min;
    this->max_output = max;
    this->pid_interval = interval;

    this->x_dot = 0.0;      // estimated derivative of x
    this->x_d1 = 0.0;       // x delayed by one sample
    this->error_d1 = 0.0;   // error delayed by one sample
    this->integrator = 0.0; // integrator

    float sigma = 0.05; // cutoff freq for dirty derivative
    this->beta = (2.0 * sigma - interval) / (2.0 * sigma + interval);
    multiplier = scalar;
  }

  float getKp() const { return kp; }
  void reset_int(){
    this->integrator = 0.0;
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

    // integrate error in x with anti-windup
    if (std::abs(this->x_dot) < 0.08) {
      this->integrator = this->integrator + (this->pid_interval / 2.0) * (error + this->error_d1);
    }

    // std::cout << "integrator " << this->integrator << std::endl;

    // differentiate x
    this->x_dot = this->beta * this->x_dot + (1.0 - this->beta) * ((x - this->x_d1) / this->pid_interval);

    // std::cout << "x_dot " << this->x_dot << std::endl;

    // calculate the force
    float force_unsat = this->kp * error + this->ki * this->integrator - this->kd * this->x_dot;

    // saturate the force
    float force_sat;
    if (force_unsat > this->max_output) {
      force_sat = this->max_output;
    } else if (force_unsat < this->min_output) {
      force_sat = this->min_output;
    } else {
      force_sat = force_unsat;
    }

    // update delayed variables
    this->error_d1 = error;
    this->x_d1 = x;

    return force_sat;
  }

  float compute(float x_r, float x, float x_dot) {
    
    float error = x_r - x;

    this->integrator = this->integrator + (this->pid_interval / 2.0) * (error + this->error_d1);

    // std::cout << "integrator " << this->integrator << std::endl;

    // std::cout << "x_dot " << this->x_dot << std::endl;

    // calculate the force
    float force_unsat = this->kp * error + this->ki * this->integrator - this->kd * x_dot;

    // saturate the force
    float force_sat;
    if (force_unsat > this->max_output) {
      force_sat = this->max_output;
    } else if (force_unsat < this->min_output) {
      force_sat = this->min_output;
    } else {
      force_sat = force_unsat;
    }

    // update delayed variables
    this->error_d1 = error;

    return force_sat;
  }

    float compute(float x_r, float x, float x_dot, float velocity) {
    
    float error = x_r - x;

    this->integrator = this->integrator + (this->pid_interval / 2.0) * (error + this->error_d1);

    // std::cout << "integrator " << this->integrator << std::endl;

    // calculate the force
    float scalar = (velocity * velocity * multiplier);

    float pd = (this->kp * error - this->kd * x_dot) / scalar;
    float force_unsat =  pd + ((this->ki * this->integrator) / scalar);

    // saturate the force and integrator error in x
    float force_sat;

    if (force_unsat > this->max_output) {
      force_sat = this->max_output;
      if (pd > this->max_output){
        this->integrator = 0.0;
      } else {
        this->integrator = ((this->max_output - pd) * scalar)/this->ki;
      }
    } else if (force_unsat < this->min_output) {
      force_sat = this->min_output;
      if (pd < this->min_output){
        this->integrator = 0.0;
      } else {
        this->integrator = ((this->min_output - pd) * scalar) / this->ki;
      }
    } else {
      force_sat = force_unsat;
    }

    float pid = pd + ((this->ki * this->integrator) / scalar);
    // calculate the force
    std::cout << "PD: " << pd << " PID: " << pid << std::endl;

    // update delayed variables
    this->error_d1 = error;

    return force_sat;
  }

private:
  float kp;
  float ki;
  float kd;
  float min_output;
  float max_output;
  float pid_interval;

  float beta;
  float x_dot;
  float x_d1;
  float error_d1;
  float integrator;
  float multiplier;
};