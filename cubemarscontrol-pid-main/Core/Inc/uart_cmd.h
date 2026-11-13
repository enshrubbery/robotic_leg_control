#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>

#define NUM_MOTORS 6

typedef enum {
    MOTOR_TYPE_AK40 = 40,
    MOTOR_TYPE_AK60 = 60,
    MOTOR_TYPE_AK70 = 70,
} MotorType;

typedef struct {
    float pos;
    float vel;
    float kp;           // inner MIT spring gain (set via K command)
    float kd;           // inner MIT damper gain (set via D command)
    float tff;          // torque feedforward (computed by outer PID; also settable via T command)
    uint8_t can_id;
    MotorType type;
    uint8_t enable_pending;
    uint8_t disable_pending;
    uint8_t enabled;
    uint8_t set_origin_pending;
    volatile float fb_pos;
    volatile uint8_t fb_received;
    volatile uint32_t fb_tick;
    float pos_offset;
    // Outer STM32 PID (set via G/H/J commands)
    float pid_kp;
    float pid_kd;
    float pid_ki;
    float pid_integral;
    float pid_prev_error;
    uint8_t pid_active;
    uint8_t fb_only;     // 1 = poll state via 0xFC, no torque (set via R command)
} MotorState;

void         uart_cmd_init(void);
void         uart_cmd_feed(uint8_t byte);
MotorState  *uart_cmd_motor(uint8_t idx);
uint8_t      uart_cmd_e_stop_requested(void);
void         uart_cmd_clear_e_stop(void);
uint32_t     uart_cmd_last_activity(void);
float        uart_cmd_pos_rate_limit(void);

#endif
