#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>

#define NUM_MOTORS 6
#define NUM_SERVOS 3

typedef enum {
    MOTOR_TYPE_AK40 = 40,
    MOTOR_TYPE_AK60 = 60,
    MOTOR_TYPE_AK70 = 70,
    MOTOR_TYPE_AK80_64 = 80,
} MotorType;

typedef enum {
    MOTOR_BUS_CAN = 0,
    MOTOR_BUS_RS485 = 1,
} MotorBus;

typedef struct {
    float pos;
    float vel;
    float kp;           // inner MIT spring gain (set via K command)
    float kd;           // inner MIT damper gain (set via D command)
    float tff;          // torque feedforward (computed by outer PID; also settable via T command)
    uint8_t can_id;       // Fixed joint node ID; not programmed into the motor
    uint8_t id_configured;
    MotorBus bus;
    MotorType type;
    uint8_t enable_pending;
    uint8_t enabled;
    uint8_t set_origin_pending;
    volatile float fb_pos;
    volatile uint8_t fb_received;
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

uint8_t      uart_cmd_motor_can_ready(const MotorState *m);
void         uart_cmd_init(void);
void         uart_cmd_feed(uint8_t byte);
MotorState  *uart_cmd_motor(uint8_t idx);
float       *uart_cmd_servo_pos(void);
uint8_t      uart_cmd_e_stop_requested(void);
uint8_t      uart_cmd_service_reset(void);
void         uart_cmd_latch_e_stop(void);
void         uart_cmd_estop_sample(uint8_t circuit_closed);

#endif
