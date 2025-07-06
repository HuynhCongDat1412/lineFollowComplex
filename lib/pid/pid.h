// pid_controller.h
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float output_min;
  float output_max;
} PIDController;

void PID_init(PIDController* pid, float kp, float ki, float kd, float out_min, float out_max);
void PID_reset(PIDController* pid);
float PID_compute(PIDController* pid, float setpoint, float measured);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H