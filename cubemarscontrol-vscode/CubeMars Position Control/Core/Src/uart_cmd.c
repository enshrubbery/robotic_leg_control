#include "uart_cmd.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CMD_BUF_SIZE 32

static MotorState motors[NUM_MOTORS];
static float servo_pos[NUM_SERVOS] = {0.0f, 0.0f, 0.0f};
static char buf[CMD_BUF_SIZE];
static uint8_t idx = 0;
// Boot inhibited; releasing the button never rearms automatically.
static volatile uint8_t e_stop_pending = 1;
static volatile uint8_t reset_pending = 0;

void uart_cmd_latch_e_stop(void)
{
    e_stop_pending = 1;
    reset_pending = 0;
}

// Compatibility hook: external contact monitoring is disabled in this build.
void uart_cmd_estop_sample(uint8_t circuit_closed)
{
    (void)circuit_closed;
}

uint8_t uart_cmd_e_stop_requested(void) { return e_stop_pending; }
static void clear_motor_commands(void)
{
    for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        motors[i].enabled = 0;
        motors[i].enable_pending = 0;
        motors[i].set_origin_pending = 0;
        motors[i].fb_only = 0;
        motors[i].pid_active = 0;
        motors[i].pid_integral = 0.0f;
        motors[i].pid_prev_error = 0.0f;
        motors[i].vel = 0.0f;
        motors[i].tff = 0.0f;
        motors[i].pos = motors[i].fb_pos;
    }
}

// Main-loop only, with interrupts masked.
// Deferring reset prevents an interrupted soft-start from resuming after reset.
uint8_t uart_cmd_service_reset(void)
{
    if (!reset_pending) return 0;
    reset_pending = 0;
    if (!e_stop_pending) return 0;
    clear_motor_commands();
    e_stop_pending = 0;
    return 1;
}

void uart_cmd_init(void)
{
    // Fixed joint routing: M2 stays reserved and cannot transmit.
    static const uint8_t ids[NUM_MOTORS] = {10, 99, 12, 106, 107, 108};
    static const MotorType types[NUM_MOTORS] = {
        MOTOR_TYPE_AK70, MOTOR_TYPE_AK80_64, MOTOR_TYPE_AK70,
        MOTOR_TYPE_AK60, MOTOR_TYPE_AK60, MOTOR_TYPE_AK60
    };
    memset(motors, 0, sizeof(motors));
    memset(servo_pos, 0, sizeof(servo_pos));
    for (unsigned i = 0; i < NUM_MOTORS; ++i) {
        MotorState *m = &motors[i];
        m->can_id = ids[i];
        m->id_configured = 1;
        m->type = types[i];
        m->bus = MOTOR_BUS_CAN;
        m->kp = i >= 3 ? 3.0f : 5.0f;
        m->kd = i >= 3 ? 0.2f : 0.3f;
        m->pid_kp = i >= 3 ? 25.0f : 5.0f;
        m->pid_ki = i >= 3 ? 0.3f : 0.0f;
    }

    e_stop_pending = 1;
    reset_pending = 0;
    idx = 0;
}

uint8_t uart_cmd_motor_can_ready(const MotorState *m)
{
    return m && m != &motors[1] && m->bus == MOTOR_BUS_CAN && m->id_configured;
}

MotorState *uart_cmd_motor(uint8_t motor_idx)
{
    if (motor_idx >= NUM_MOTORS) return &motors[0];
    return &motors[motor_idx];
}

float *uart_cmd_servo_pos(void)
{
    return servo_pos;
}

static void parse_cmd(void)
{
    if (idx < 2) return;
    buf[idx] = '\0';

    // Global emergency stop: "0X"
    if (buf[0] == '0' && (buf[1] == 'X' || buf[1] == 'x')) {
        uart_cmd_latch_e_stop();
        return;
    }

    // Explicit reset does not require an external contact.
    // It clears pending motion but does not enable any motor.
    if (idx == 2 && buf[0] == '0' && (buf[1] == 'Z' || buf[1] == 'z')) {
        if (e_stop_pending) {
            reset_pending = 1;
        }
        return;
    }
    if (e_stop_pending) return;

    uint8_t motor_num = (uint8_t)(buf[0] - '1');
    char cmd = buf[1];
    float val = (idx > 2) ? (float)atof(&buf[2]) : 0.0f;

    if (!isfinite(val)) return;

    // Servo indices: '7','8','9'; slot 8 has no output (A1 E-stop).
    if (motor_num >= NUM_MOTORS && motor_num < NUM_MOTORS + NUM_SERVOS) {
        if (motor_num != NUM_MOTORS + 1 && (cmd == 'P' || cmd == 'p')) {
            if (val < -120.0f) val = -120.0f;
            if (val > 120.0f) val = 120.0f;
            servo_pos[motor_num - NUM_MOTORS] = val;
        }
        return;
    }

    if (motor_num >= NUM_MOTORS) return;

    if (motor_num == 1) return; // M2 Hip Roll is permanently disabled.
    MotorState *m = &motors[motor_num];

    switch (cmd) {
        case 'P': case 'p':
            m->pos = val;
            m->pid_integral = 0.0f;
            m->pid_prev_error = m->fb_pos;
            m->pid_active = 1;
            break;
        case 'V': case 'v': m->vel = val; break;
        case 'K': case 'k': m->kp  = val; break;  // inner MIT spring gain
        case 'D': case 'd': m->kd  = val; break;  // inner MIT damper gain
        case 'T': case 't': m->tff = val; break;
        case 'G': case 'g': m->pid_kp = val; break;              // outer PID P gain
        case 'H': case 'h': m->pid_kd = val; break;              // outer PID D gain
        case 'J': case 'j':
            m->pid_ki = val;
            m->pid_integral = 0.0f;
            break;
        case 'E': case 'e':
            if (!uart_cmd_motor_can_ready(m)) break;
            m->enable_pending = 1;
            m->fb_only = 0;
            break;
        case 'R': case 'r':
            if (!uart_cmd_motor_can_ready(m)) break;
            m->fb_only = 1;
            m->enabled = 0;
            m->enable_pending = 0;
            break;
        case 'O': case 'o':
            if (uart_cmd_motor_can_ready(m)) m->set_origin_pending = 1;
            break;
        case 'I': case 'i':
        case 'A': case 'a':
            // IDs and models are fixed for this leg; GUI cannot reroute them.
            break;
        default: break;
    }
}

void uart_cmd_feed(uint8_t byte)
{
    if (byte == '\n' || byte == '\r') {
        parse_cmd();
        idx = 0;
    } else {
        if (idx < CMD_BUF_SIZE - 1)
            buf[idx++] = (char)byte;
    }
}
