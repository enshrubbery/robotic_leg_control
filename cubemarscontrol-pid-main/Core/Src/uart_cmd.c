#include "uart_cmd.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>

#define CMD_BUF_SIZE 32

static MotorState motors[NUM_MOTORS];
// USART2 fills complete lines. Only the control loop parses numbers and
// changes setpoints, so a CAN send cannot overwrite a newly arrived target.
#define QUEUE_SIZE 32U
static char lines[QUEUE_SIZE][CMD_BUF_SIZE];
static volatile unsigned head, tail;
volatile uint32_t uart_rx_faults, uart_queue_faults;
uint32_t uart_lines_processed;
static uint8_t idx = 0;
static uint8_t discard_line;
static volatile uint8_t e_stop_pending = 0;
static volatile uint32_t last_activity_tick = 0;
static float pos_rate_limit = 0.3f;   // rad/s â€” set via GUI with "0L<value>"

uint8_t uart_cmd_take_stop(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint8_t reason = e_stop_pending;
    e_stop_pending = 0;
    __set_PRIMASK(mask);
    return reason;
}
uint32_t uart_cmd_last_activity(void)    { return last_activity_tick; }
float    uart_cmd_pos_rate_limit(void)   { return pos_rate_limit; }

void uart_cmd_init(void)
{
    memset(motors, 0, sizeof(motors));
    last_activity_tick = HAL_GetTick();

    // M1: Hip Pitch, AK70-10, CAN ID 1 (needs enable)
    motors[0].pos = 0.0f;
    motors[0].vel = 0.0f;
    motors[0].kp  = 5.0f;    // inner MIT spring
    motors[0].kd  = 0.3f;    // inner MIT damper
    motors[0].tff = 0.0f;
    motors[0].can_id = 1;
    motors[0].type = MOTOR_TYPE_AK70;
    motors[0].enable_pending = 0;
    motors[0].enabled = 0;
    motors[0].set_origin_pending = 0;
    motors[0].pos_offset = 0.0f;
    motors[0].pid_kp = 5.0f;
    motors[0].pid_kd = 0.0f;
    motors[0].pid_ki = 0.0f;
    motors[0].pid_integral = 0.0f;
    motors[0].pid_prev_error = 0.0f;
    motors[0].pid_active = 0;
    motors[0].fb_only = 0;

    // M2: Hip Roll, AK80-64, CAN ID 2 (AK80 shares AK70 limits)
    motors[1].pos = 0.0f;
    motors[1].vel = 0.0f;
    motors[1].kp  = 5.0f;    // inner MIT spring
    motors[1].kd  = 0.3f;    // inner MIT damper
    motors[1].tff = 0.0f;
    motors[1].can_id = 2;
    motors[1].type = MOTOR_TYPE_AK70;
    motors[1].enable_pending = 0;
    motors[1].enabled = 0;
    motors[1].set_origin_pending = 0;
    motors[1].pos_offset = 0.0f;
    motors[1].pid_kp = 5.0f;
    motors[1].pid_kd = 0.0f;
    motors[1].pid_ki = 0.0f;
    motors[1].pid_integral = 0.0f;
    motors[1].pid_prev_error = 0.0f;
    motors[1].pid_active = 0;
    motors[1].fb_only = 0;

    // M3: Hip Yaw, AK70-10, CAN ID 3 (needs enable)
    motors[2].pos = 0.0f;
    motors[2].vel = 0.0f;
    motors[2].kp  = 5.0f;    // inner MIT spring
    motors[2].kd  = 0.3f;    // inner MIT damper
    motors[2].tff = 0.0f;
    motors[2].can_id = 3;
    motors[2].type = MOTOR_TYPE_AK70;
    motors[2].enable_pending = 0;
    motors[2].enabled = 0;
    motors[2].set_origin_pending = 0;
    motors[2].pos_offset = 0.0f;
    motors[2].pid_kp = 5.0f;
    motors[2].pid_kd = 0.0f;
    motors[2].pid_ki = 0.0f;
    motors[2].pid_integral = 0.0f;
    motors[2].pid_prev_error = 0.0f;
    motors[2].pid_active = 0;
    motors[2].fb_only = 0;

    // M4: Knee, AK60, CAN ID 4 (needs enable)
    motors[3].pos = 0.0f;
    motors[3].vel = 0.0f;
    motors[3].kp  = 3.0f;    // inner MIT spring
    motors[3].kd  = 0.2f;    // inner MIT damper
    motors[3].tff = 0.0f;
    motors[3].can_id = 4;
    motors[3].type = MOTOR_TYPE_AK60;
    motors[3].enable_pending = 0;
    motors[3].enabled = 0;
    motors[3].set_origin_pending = 0;
    motors[3].pos_offset = 0.0f;
    motors[3].pid_kp = 25.0f;
    motors[3].pid_kd = 0.0f;
    motors[3].pid_ki = 0.3f;
    motors[3].pid_integral = 0.0f;
    motors[3].pid_prev_error = 0.0f;
    motors[3].pid_active = 0;
    motors[3].fb_only = 0;

    // M5: Foot Diff 1, AK60, CAN ID 5 (needs enable)
    motors[4].pos = 0.0f;
    motors[4].vel = 0.0f;
    motors[4].kp  = 3.0f;    // inner MIT spring
    motors[4].kd  = 0.2f;    // inner MIT damper
    motors[4].tff = 0.0f;
    motors[4].can_id = 5;
    motors[4].type = MOTOR_TYPE_AK60;
    motors[4].enable_pending = 0;
    motors[4].enabled = 0;
    motors[4].set_origin_pending = 0;
    motors[4].pos_offset = 0.0f;
    motors[4].pid_kp = 25.0f;
    motors[4].pid_kd = 0.0f;
    motors[4].pid_ki = 0.3f;
    motors[4].pid_integral = 0.0f;
    motors[4].pid_prev_error = 0.0f;
    motors[4].pid_active = 0;
    motors[4].fb_only = 0;

    // M6: Foot Diff 2, AK60, CAN ID 6 (needs enable)
    motors[5].pos = 0.0f;
    motors[5].vel = 0.0f;
    motors[5].kp  = 3.0f;    // inner MIT spring
    motors[5].kd  = 0.2f;    // inner MIT damper
    motors[5].tff = 0.0f;
    motors[5].can_id = 6;
    motors[5].type = MOTOR_TYPE_AK60;
    motors[5].enable_pending = 0;
    motors[5].enabled = 0;
    motors[5].set_origin_pending = 0;
    motors[5].pos_offset = 0.0f;
    motors[5].pid_kp = 25.0f;
    motors[5].pid_kd = 0.0f;
    motors[5].pid_ki = 0.3f;
    motors[5].pid_integral = 0.0f;
    motors[5].pid_prev_error = 0.0f;
    motors[5].pid_active = 0;
    motors[5].fb_only = 0;


    idx = 0;
}

MotorState *uart_cmd_motor(uint8_t motor_idx)
{
    if (motor_idx >= NUM_MOTORS) return &motors[0];
    return &motors[motor_idx];
}

volatile uint8_t uart_cmd_diag_requested;

static void parse_cmd(const char *buf)
{

    // Global commands: "0X" emergency stop, "0H" host heartbeat.
    if (buf[0] == '0') {
        if (buf[1] == 'Q') uart_cmd_diag_requested = 1;
        if (buf[1] == 'L' || buf[1] == 'l') {
            float v = (buf[2] != 0) ? (float)atof(&buf[2]) : 0.0f;
            if (v > 0.0f && v <= 20.0f) pos_rate_limit = v;
        }
        if (buf[1] == 'X' || buf[1] == 'x' || buf[1] == 'H' || buf[1] == 'h')
            last_activity_tick = HAL_GetTick();
        return;
    }

    uint8_t motor_num = (uint8_t)(buf[0] - '1');
    char cmd = buf[1];
    float val = (buf[2] != 0) ? (float)atof(&buf[2]) : 0.0f;

    if (motor_num >= NUM_MOTORS) return;

    MotorState *m = &motors[motor_num];
    last_activity_tick = HAL_GetTick();

    switch (cmd) {
        case 'P': case 'p':
            m->pos = val;
            m->pid_integral = 0.0f;
            m->pid_prev_error = m->fb_pos;
            m->pid_active = m->fb_received
                         && ((uint32_t)(HAL_GetTick() - m->fb_tick) <= 100U);
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
            m->enable_pending = 1;
            m->disable_pending = 0;
            m->fb_only = 0;
            break;
        case 'R': case 'r':
            m->fb_only = 1;
            m->disable_pending = 1;
            m->enable_pending = 0;
            break;
        case 'O': case 'o': m->set_origin_pending = 1; break;
        case 'I': case 'i': m->can_id = (uint8_t)val; break;
        case 'A': case 'a':
            if ((int)val == 70)
                m->type = MOTOR_TYPE_AK70;
            else if ((int)val == 40)
                m->type = MOTOR_TYPE_AK40;
            else
                m->type = MOTOR_TYPE_AK60;
            m->disable_pending = 1;
            m->enable_pending = 0;
            break;
        default: break;
    }
}

static void discard_commands(void)
{
    idx = 0;
    tail = head;
    e_stop_pending = 2;
}

void uart_cmd_rx_error(void)
{
    ++uart_rx_faults;
    discard_commands();
    discard_line = 1;
}

void uart_cmd_feed(uint8_t byte)
{
    if (discard_line) {
        if (byte == '\n' || byte == '\r') discard_line = 0;
        return;
    }
    char *line = lines[head];
    if (byte != '\n' && byte != '\r') {
        if (idx < CMD_BUF_SIZE - 1) {
            line[idx++] = (char)byte;
        } else {
            ++uart_queue_faults;
            discard_commands();
            discard_line = 1;
        }
        return;
    }
    if (idx >= 2) {
        line[idx] = 0;
        if (line[0] == '0' && (line[1] == 'X' || line[1] == 'x')) {
            tail = head;
            if (!e_stop_pending) e_stop_pending = 1;
        } else if (!e_stop_pending) {
            unsigned next = (head + 1U) % QUEUE_SIZE;
            if (next == tail) {
                ++uart_queue_faults;
                discard_commands();
            } else {
                head = next;
            }
        }
    }
    idx = 0;
}

void uart_cmd_process(void)
{
    char line[CMD_BUF_SIZE];
    for (unsigned count = 0; count < QUEUE_SIZE; ++count) {
        // An RX fault or emergency stop can invalidate the queue at any byte.
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (tail == head || e_stop_pending) {
            __set_PRIMASK(mask);
            return;
        }
        memcpy(line, lines[tail], sizeof(line));
        tail = (tail + 1U) % QUEUE_SIZE;
        __set_PRIMASK(mask);
        parse_cmd(line);
        ++uart_lines_processed;
    }
}
