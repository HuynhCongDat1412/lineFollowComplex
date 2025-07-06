#include "pid.h"

void PID_init(PIDController* pid, float kp, float ki, float kd, float out_min, float out_max) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0;
  pid->previous_error = 0;
  pid->output_min = out_min;
  pid->output_max = out_max;
}

void PID_reset(PIDController* pid) {
  pid->integral = 0;
  pid->previous_error = 0;
}

float PID_compute(PIDController* pid, float setpoint, float measured) {
  float error = setpoint - measured;
  pid->integral += error;
  float derivative = error - pid->previous_error;
  pid->previous_error = error;

  float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  if (output > pid->output_max) output = pid->output_max;
  else if (output < pid->output_min) output = pid->output_min;
  return output;
}
