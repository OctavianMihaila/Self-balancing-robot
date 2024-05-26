#include "movement_controller.h"

float calculate_pid_result(float gyro_angle, float prev_pid_error) {
  float curr_pid_error = gyro_angle;
  float pid_result = PID_KP * curr_pid_error + PID_KI * (curr_pid_error + prev_pid_error) + PID_KD * (curr_pid_error - prev_pid_error);

  // Limit the PID result so that it doesn't become unmanageable.
  if (pid_result > MAX_PID_RESULT) {
    pid_result = MAX_PID_RESULT;
  } else if (pid_result < -MAX_PID_RESULT) {
    pid_result = -MAX_PID_RESULT;
  }

  prev_pid_error = curr_pid_error;

  return pid_result;
}