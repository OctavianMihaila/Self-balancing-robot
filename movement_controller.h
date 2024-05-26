#include <avr/io.h>
#include <util/delay.h>

// Gain values for the PID controller.
#define PID_KP 15
#define PID_KI 3.7
#define PID_KD 23
#define MAX_PID_RESULT 500
float calculate_pid_result(float gyro_angle, float prev_pid_error);