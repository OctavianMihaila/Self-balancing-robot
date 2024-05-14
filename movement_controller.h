#include <avr/io.h>
#include <util/delay.h>

// TODO: Adjust as needed.
// Gain values for the PID controller.
#define PID_KP 15
#define PID_KI 1.5
#define PID_KD 30
#define MAX_YAW 400 // TODO: FOR NOW UNUSED, might be needed.
#define MAX_PID_RESULT 400 // TODO: ADJUST

float calculate_pid_result(float gyro_angle);