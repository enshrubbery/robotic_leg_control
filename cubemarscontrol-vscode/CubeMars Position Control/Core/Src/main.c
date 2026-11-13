/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Robot arm UART remote control (6 motor slots + 3 servo slots) — PID version
  *
  * Controls AK70-10 Hip Pitch/Yaw and three AK60-6 joints over CAN.
  * M2 Hip Roll (AK80-64, CAN 99) is reserved and cannot transmit.
  * and 3 MG995 servos via TIM2 PWM (end effector).
  *
  * STM32-side outer PID loop feeds torque (tff) into the MIT CAN frame.
  * Inner MIT kp/kd are small fixed values for high-frequency stability only.
  *
  * Protocol: <motor_num><cmd><value>\n
  *   motor_num = 1 Hip Pitch, 2 disabled Hip Roll, 3 Hip Yaw, 4 Knee, 5-6 Foot Diff
  *   motor_num = 7-9 (servos; 8 disabled because A1 is E-stop)
  *   P=position (rad for motors, degrees for servos)
  *   V=velocity feedforward, K=inner MIT kp, D=inner MIT kd, T=torque offset
  *   G=outer PID Kp, H=outer PID Kd, J=outer PID Ki
  *   E=enable, R=passive feedback (0xFC poll, no torque), I=CAN_ID, A=type
  *   0X = latched zero-gain/zero-feedforward stop; 0Z = reset after release
  *
  * AK60 uses extended CAN ID:  ExtId = CAN_ID | (8 << 8)
  * AK70/80 uses standard CAN ID:  StdId = CAN_ID
  * AK40 uses standard CAN ID (same frame as AK70, different limits)
  *
  * Servo PWM: TIM2 @ 50Hz (prescaler=84-1, period=20000-1)
  *   CH1 (PA0) = Wrist 1, CH2 disabled: PA1/A1 is the NC E-stop input, CH3 (PB10) = Gripper
  *   Pulse 500 = 0 deg, Pulse 2500 = 180 deg (MG995 TowerPro)
  *
  * Hardware: STM32F446RE Nucleo + Waveshare CAN shield
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "uart_cmd.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// AK60 parameter limits
#define AK60_P_MIN    -12.56f
#define AK60_P_MAX     12.56f
#define AK60_V_MIN    -60.0f
#define AK60_V_MAX     60.0f
#define AK60_T_MIN    -12.0f
#define AK60_T_MAX     12.0f

// AK70/80 parameter limits
#define AK70_P_MIN    -12.5f
#define AK70_P_MAX     12.5f
#define AK70_V_MIN    -30.0f
#define AK70_V_MAX     30.0f
#define AK70_T_MIN    -18.0f
#define AK70_T_MAX     18.0f

// CubeMars AK80-64 MIT protocol encoding ranges (not bench torque limits).
#define AK80_64_V_MAX 8.0f
#define AK80_64_T_MAX 144.0f

// AK40 parameter limits
#define AK40_P_MIN    -12.5f
#define AK40_P_MAX     12.5f
#define AK40_V_MIN    -45.5f
#define AK40_V_MAX     45.5f
#define AK40_T_MIN    -5.0f
#define AK40_T_MAX     5.0f

// Shared limits
#define KP_MIN    0.0f
#define KP_MAX    500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f

#define CAN_PACKET_MIT 8

// Position rate limit: max rad/s the commanded position can change
// At 2 rad/s with ~10ms loop, max step per iteration = 0.02 rad
#define POS_RATE_LIMIT  2.0f
#define LOOP_DT         0.012f

// Servo PWM: 50Hz, 1us resolution (prescaler=84-1, period=20000-1)
// MG995: center at 1500us, ~8.33us per degree
// Pulse clamped to 500-2500us for safety
#define SERVO_PULSE_CENTER 1500
#define SERVO_US_PER_DEG   8.333f
#define SERVO_PULSE_CLAMP_MIN  500
#define SERVO_PULSE_CLAMP_MAX  2500

// CAN handles
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
uint8_t  txData[8];
uint8_t  rxData[8];
uint32_t txMailbox;

uint8_t uart_rx_byte;

// Rate-limited commanded position for each motor (ramps toward m->pos)
static float cmd_pos[NUM_MOTORS] = {0};
static volatile uint8_t estop_gpio_ready = 0;
static volatile uint8_t estop_can_ready = 0;

// Runs during blocking soft-start delays as well as the normal control loop.
void app_safety_tick(void)
{
    static uint8_t was_stopped = 0;
    if (!estop_gpio_ready) return;
    if (uart_cmd_e_stop_requested()) {
        TIM2->CCER = 0;
        if (estop_can_ready && !was_stopped)
            CAN1->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
    }
    was_stopped = uart_cmd_e_stop_requested();
}

// Feedback ring buffer — ISR writes, main loop transmits
#define FB_BUF_SIZE 512
static char fb_buf[FB_BUF_SIZE];
static volatile uint16_t fb_head = 0;
static volatile uint16_t fb_tail = 0;

static void fb_write(const char *str, int len)
{
    for (int i = 0; i < len; i++) {
        uint16_t next = (fb_head + 1) % FB_BUF_SIZE;
        if (next == fb_tail) break;  // full, drop
        fb_buf[fb_head] = str[i];
        fb_head = next;
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
static float        uint_to_float(int x_int, float x_min, float x_max, int bits);
static void         send_can_frame(void);
static void         send_ak60_cmd(MotorState *m);
static void         send_ak70_cmd(MotorState *m);
static void         send_ak40_cmd(MotorState *m);
static void         send_ak70_enable(MotorState *m);
static void         send_ak60_set_origin(MotorState *m);
static void         send_ak70_set_origin(MotorState *m);
static void         unpack_ak60_reply(uint8_t motor_idx);
static void         unpack_ak70_reply(uint8_t motor_idx);
static void         unpack_ak40_reply(uint8_t motor_idx);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static uint32_t servo_angle_to_pulse(float angle_deg)
{
    if (angle_deg < -120.0f) angle_deg = -120.0f;
    if (angle_deg > 120.0f) angle_deg = 120.0f;
    float pulse = (float)SERVO_PULSE_CENTER + angle_deg * SERVO_US_PER_DEG;
    return (uint32_t)pulse;
}

static void update_servos(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (uart_cmd_e_stop_requested()) {
        TIM2->CCER = 0;
    } else {
        float *spos = uart_cmd_servo_pos();
        TIM2->CCR1 = servo_angle_to_pulse(spos[0]);
        // PA1 is reserved for the E-stop; servo command 8 has no output.
        TIM2->CCR3 = servo_angle_to_pulse(spos[2]);
        TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC3E;
    }
    __set_PRIMASK(primask);
}

static unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (unsigned int)((x - x_min) / span * (float)((1u << bits) - 1));
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

static void send_can_frame(void)
{
    uint32_t timeout = HAL_GetTick() + 5;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if (HAL_GetTick() > timeout) return;
    }
    // Final gate covers every command path, including soft-start and origin.
    // Poll the pin here too so an opening cannot wait for the next tick.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (uart_cmd_e_stop_requested()) {
        if (txHeader.IDE == CAN_ID_EXT) {
            if ((txHeader.ExtId >> 8) != CAN_PACKET_MIT) {
                __set_PRIMASK(primask);
                return;
            }
            // AK60: kp=kd=0, position midpoint, velocity/torque midpoint.
            const uint8_t zero[8] = {0, 0, 0, 0x7f, 0xff, 0x7f, 0xf7, 0xff};
            memcpy(txData, zero, sizeof(zero));
        } else {
            // MIT: position/velocity midpoint, kp=kd=0, torque midpoint.
            const uint8_t zero[8] = {0x7f, 0xff, 0x7f, 0xf0, 0, 0, 0x07, 0xff};
            memcpy(txData, zero, sizeof(zero));
        }
        txHeader.DLC = 8;
    }
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
    __set_PRIMASK(primask);
}

// AK60: extended CAN ID, KP/KD/Pos/Vel/Torque byte order
static void send_ak60_cmd(MotorState *m)
{
    if (!uart_cmd_motor_can_ready(m)) return;
    uint16_t kp_int = float_to_uint(m->kp,  KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(m->kd,  KD_MIN, KD_MAX, 12);
    uint16_t p_int  = float_to_uint(m->pos + m->pos_offset, AK60_P_MIN, AK60_P_MAX, 16);
    uint16_t v_int  = float_to_uint(m->vel, AK60_V_MIN, AK60_V_MAX, 12);
    uint16_t t_int  = float_to_uint(m->tff, AK60_T_MIN, AK60_T_MAX, 12);

    txData[0] = (kp_int >> 4) & 0xFF;
    txData[1] = ((kp_int & 0xF) << 4) | ((kd_int >> 8) & 0xF);
    txData[2] = kd_int & 0xFF;
    txData[3] = (p_int >> 8) & 0xFF;
    txData[4] = p_int & 0xFF;
    txData[5] = (v_int >> 4) & 0xFF;
    txData[6] = ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    txData[7] = t_int & 0xFF;

    txHeader.StdId              = 0;
    txHeader.ExtId              = (uint32_t)m->can_id | ((uint32_t)CAN_PACKET_MIT << 8);
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK70/80: standard CAN ID, Pos/Vel/KP/KD/Torque byte order
static void send_ak70_cmd(MotorState *m)
{
    if (!uart_cmd_motor_can_ready(m)) return;
    float v_max = m->type == MOTOR_TYPE_AK80_64 ? AK80_64_V_MAX : AK70_V_MAX;
    float t_max = m->type == MOTOR_TYPE_AK80_64 ? AK80_64_T_MAX : AK70_T_MAX;
    unsigned int p_int  = float_to_uint(m->pos, AK70_P_MIN, AK70_P_MAX, 16);
    unsigned int v_int  = float_to_uint(m->vel, -v_max, v_max, 12);
    unsigned int kp_int = float_to_uint(m->kp,  KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(m->kd,  KD_MIN, KD_MAX, 12);
    unsigned int t_int  = float_to_uint(m->tff, -t_max, t_max, 12);

    txData[0] = p_int >> 8;
    txData[1] = p_int & 0xFF;
    txData[2] = v_int >> 4;
    txData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    txData[4] = kp_int & 0xFF;
    txData[5] = kd_int >> 4;
    txData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    txData[7] = t_int & 0xFF;

    txHeader.StdId              = m->can_id;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK40: standard CAN ID, same byte order as AK70 but different limits
static void send_ak40_cmd(MotorState *m)
{
    if (!uart_cmd_motor_can_ready(m)) return;
    unsigned int p_int  = float_to_uint(m->pos, AK40_P_MIN, AK40_P_MAX, 16);
    unsigned int v_int  = float_to_uint(m->vel, AK40_V_MIN, AK40_V_MAX, 12);
    unsigned int kp_int = float_to_uint(m->kp,  KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(m->kd,  KD_MIN, KD_MAX, 12);
    unsigned int t_int  = float_to_uint(m->tff, AK40_T_MIN, AK40_T_MAX, 12);

    txData[0] = p_int >> 8;
    txData[1] = p_int & 0xFF;
    txData[2] = v_int >> 4;
    txData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    txData[4] = kp_int & 0xFF;
    txData[5] = kd_int >> 4;
    txData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    txData[7] = t_int & 0xFF;

    txHeader.StdId              = m->can_id;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK70/80 enable: 0xFF x7 + 0xFC
static void send_ak70_enable(MotorState *m)
{
    if (!uart_cmd_motor_can_ready(m)) return;
    txData[0] = 0xFF;
    txData[1] = 0xFF;
    txData[2] = 0xFF;
    txData[3] = 0xFF;
    txData[4] = 0xFF;
    txData[5] = 0xFF;
    txData[6] = 0xFF;
    txData[7] = 0xFC;

    txHeader.StdId              = m->can_id;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK60 set origin: servo mode, extended frame, CAN_PACKET_SET_ORIGIN_HERE (id=5)
// Data[0]=0 → temporary (cleared on power loss); Data[0]=1 → permanent
static void send_ak60_set_origin(MotorState *m)
{
    if (!uart_cmd_motor_can_ready(m)) return;
    txData[0] = 0x02;

    txHeader.StdId              = 0;
    txHeader.ExtId              = (uint32_t)m->can_id | ((uint32_t)5 << 8);
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 1;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK70/80/AK40 set origin: 0xFF x7 + 0xFE
static void send_ak70_set_origin(MotorState *m)
{
    if (!uart_cmd_motor_can_ready(m)) return;
    txData[0] = 0xFF;
    txData[1] = 0xFF;
    txData[2] = 0xFF;
    txData[3] = 0xFF;
    txData[4] = 0xFF;
    txData[5] = 0xFF;
    txData[6] = 0xFF;
    txData[7] = 0xFE;

    txHeader.StdId              = m->can_id;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK60 feedback: servo-mode format (writes to ring buffer, not UART directly)
static void unpack_ak60_reply(uint8_t motor_idx)
{
    int16_t pos_int = (int16_t)((rxData[0] << 8) | rxData[1]);
    int16_t spd_int = (int16_t)((rxData[2] << 8) | rxData[3]);
    int16_t cur_int = (int16_t)((rxData[4] << 8) | rxData[5]);

    float pos = (float)pos_int * 0.1f;
    float spd = (float)spd_int * 10.0f;
    float cur = (float)cur_int * 0.01f;
    int8_t temp = (int8_t)rxData[6];
    int8_t err  = (int8_t)rxData[7];

    MotorState *m = uart_cmd_motor(motor_idx);
    m->fb_pos = pos * 0.01745329f - m->pos_offset;
    m->fb_received = 1;

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "[M%d] pos=%.1f deg  spd=%.0f eRPM  I=%.2f A  T=%d  err=%d\r\n",
        motor_idx + 1, (double)(m->fb_pos / 0.01745329f), (double)spd, (double)cur, temp, err);
    fb_write(buf, len);
}

// AK70/80 feedback: MIT mode format (writes to ring buffer)
static void unpack_ak70_reply(uint8_t motor_idx)
{
    MotorState *m = uart_cmd_motor(motor_idx);
    float v_max = m->type == MOTOR_TYPE_AK80_64 ? AK80_64_V_MAX : AK70_V_MAX;
    float t_max = m->type == MOTOR_TYPE_AK80_64 ? AK80_64_T_MAX : AK70_T_MAX;
    int p_int = (rxData[1] << 8) | rxData[2];
    int v_int = (rxData[3] << 4) | (rxData[4] >> 4);
    int i_int = ((rxData[4] & 0xF) << 8) | rxData[5];

    float pos = uint_to_float(p_int, AK70_P_MIN, AK70_P_MAX, 16);
    float vel = uint_to_float(v_int, -v_max, v_max, 12);
    float tau = uint_to_float(i_int, -t_max, t_max, 12);
    int8_t temp = (int8_t)(rxData[6] - 40);
    int8_t err  = (int8_t)rxData[7];

    m->fb_pos = pos;
    m->fb_received = 1;

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "[M%d] pos=%.3f rad  vel=%.2f  tau=%.2f  T=%d  err=%d\r\n",
        motor_idx + 1, (double)pos, (double)vel, (double)tau, temp, err);
    fb_write(buf, len);
}

// AK40 feedback: same format as AK70 but different limits
static void unpack_ak40_reply(uint8_t motor_idx)
{
    int p_int = (rxData[1] << 8) | rxData[2];
    int v_int = (rxData[3] << 4) | (rxData[4] >> 4);
    int i_int = ((rxData[4] & 0xF) << 8) | rxData[5];

    float pos = uint_to_float(p_int, AK40_P_MIN, AK40_P_MAX, 16);
    float vel = uint_to_float(v_int, AK40_V_MIN, AK40_V_MAX, 12);
    float tau = uint_to_float(i_int, -AK40_T_MAX, AK40_T_MAX, 12);
    int8_t temp = (int8_t)(rxData[6] - 40);
    int8_t err  = (int8_t)rxData[7];

    MotorState *m = uart_cmd_motor(motor_idx);
    m->fb_pos = pos;
    m->fb_received = 1;

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "[M%d] pos=%.3f rad  vel=%.2f  tau=%.2f  T=%d  err=%d\r\n",
        motor_idx + 1, (double)pos, (double)vel, (double)tau, temp, err);
    fb_write(buf, len);
}

// Identify which motor sent feedback based on CAN ID
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;

    // Try multiple ways to identify source motor:
    //  - Extended frame (AK60): motor ID in lower byte of ExtId
    //  - Standard frame (AK70): motor ID in rxData[0]
    uint8_t rx_id;
    if (rxHeader.IDE == CAN_ID_EXT)
        rx_id = (uint8_t)(rxHeader.ExtId & 0xFF);
    else
        rx_id = rxData[0];

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        MotorState *m = uart_cmd_motor(i);
        if (uart_cmd_motor_can_ready(m) && m->can_id == rx_id &&
            ((m->type == MOTOR_TYPE_AK60) == (rxHeader.IDE == CAN_ID_EXT))) {
            if (m->type == MOTOR_TYPE_AK60)
                unpack_ak60_reply(i);
            else if (m->type == MOTOR_TYPE_AK40)
                unpack_ak40_reply(i);
            else
                unpack_ak70_reply(i);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            return;
        }
    }

    // No match — buffer debug line
    char dbg[64];
    int len = snprintf(dbg, sizeof(dbg), "[M0] unknown CAN id=%d ide=%lu\r\n",
        (int)rx_id, (unsigned long)rxHeader.IDE);
    fb_write(dbg, len);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uart_cmd_feed(uart_rx_byte);
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

// Interrupts only latch the stop; CAN transmission stays in main context.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) {
        uart_cmd_latch_e_stop();
        TIM2->CCER = 0;
        if (estop_can_ready)
            CAN1->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_CAN1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    HAL_CAN_Start(&hcan1);
    estop_can_ready = 1;
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    // Servos already started by MX_TIM2_Init (direct register config)
    update_servos();

    uart_cmd_init();
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    // Enable B1 button (PC13) as emergency stop
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_Delay(100);

    // Send zero-torque to all motors on startup to suppress power-on snap.
    // Motors that retained MIT mode from a previous session will receive kp=0,
    // kd=0, tff=0 and stay limp instead of snapping to their stale setpoint.
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        MotorState *m = uart_cmd_motor(i);
        float saved_kp = m->kp;
        float saved_kd = m->kd;
        m->kp  = 0.0f;
        m->kd  = 0.0f;
        m->tff = 0.0f;
        for (int j = 0; j < 3; j++) {
            if (m->type == MOTOR_TYPE_AK60)
                send_ak60_cmd(m);
            else if (m->type == MOTOR_TYPE_AK40)
                send_ak40_cmd(m);
            else
                send_ak70_cmd(m);
            HAL_Delay(10);
        }
        m->kp = saved_kp;
        m->kd = saved_kd;
    }

    /* USER CODE END 2 */

    const char startup[] = "\r\nSTOP LATCHED: send 0Z to reset. Motors remain disabled.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)startup, sizeof(startup) - 1, 100);
    uint8_t stop_reported = 1;

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (uart_cmd_e_stop_requested()) {
            for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
                MotorState *m = uart_cmd_motor(i);
                m->enabled = m->enable_pending = m->set_origin_pending = 0;
                m->fb_only = m->pid_active = 0;
                m->vel = m->tff = m->pid_integral = 0.0f;
                if (m->type == MOTOR_TYPE_AK60) send_ak60_cmd(m);
                else if (m->type == MOTOR_TYPE_AK40) send_ak40_cmd(m);
                else send_ak70_cmd(m);
            }
            update_servos();
            if (!stop_reported) {
                const char msg[] = "STOP LATCHED: send 0Z to reset.\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, 100);
                stop_reported = 1;
            }
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            uint8_t reset = uart_cmd_service_reset();
            __set_PRIMASK(primask);
            if (reset) {
                const char msg[] = "STOP RESET: all motors disabled; fresh E commands required.\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, 100);
                stop_reported = 0;
            }
            HAL_Delay(2);
            continue;
        }

        for (uint8_t i = 0; i < NUM_MOTORS; i++)
        {
            MotorState *m = uart_cmd_motor(i);

            if (uart_cmd_e_stop_requested()) break;

            if (!uart_cmd_motor_can_ready(m)) continue;

            // Handle set origin
            if (m->set_origin_pending) {
                if (m->type == MOTOR_TYPE_AK60) {
                    // Software-only origin: offset translates user coords to MIT coords
                    m->pos_offset += m->fb_pos;
                } else {
                    send_ak70_set_origin(m);
                }
                HAL_Delay(10);
                m->pos = 0.0f;
                cmd_pos[i] = 0.0f;
                m->fb_pos = 0.0f;
                m->pid_integral   = 0.0f;
                m->pid_prev_error = 0.0f;
                m->set_origin_pending = 0;
            }

            // Handle pending enable with soft-start
            if (m->enable_pending) {
                // AK70/AK40 need MIT mode entry frame
                if (m->type != MOTOR_TYPE_AK60) {
                    for (int j = 0; j < 10 && !uart_cmd_e_stop_requested(); j++) {
                        send_ak70_enable(m);
                        HAL_Delay(10);
                    }
                    HAL_Delay(100);
                }

                m->fb_received = 0;
                float kp_target = m->kp;
                float kd_target = m->kd;

                // Probe with zero gain for all motor types so the ramp
                // starts at the actual position instead of 0.
                m->kp  = 0.0f;
                m->kd  = 0.0f;
                m->tff = 0.0f;
                uint32_t probe_end = HAL_GetTick() + 300;
                while (!uart_cmd_e_stop_requested() && !m->fb_received && HAL_GetTick() < probe_end) {
                    if (m->type == MOTOR_TYPE_AK60)
                        send_ak60_cmd(m);
                    else if (m->type == MOTOR_TYPE_AK40)
                        send_ak40_cmd(m);
                    else
                        send_ak70_cmd(m);
                    HAL_Delay(10);
                }
                if (m->fb_received) {
                    m->pos     = m->fb_pos;
                    cmd_pos[i] = m->fb_pos;
                }
                m->kp = kp_target;
                m->kd = kd_target;
                if (!m->fb_received) {
                    m->enable_pending = 0;
                    m->enabled = 0;
                    continue;
                }

                for (int j = 1; j <= 50 && !uart_cmd_e_stop_requested(); j++) {
                    if (m->fb_received && j <= 5) {
                        m->pos     = m->fb_pos;
                        cmd_pos[i] = m->fb_pos;
                    }

                    m->kp = kp_target * (float)j / 50.0f;
                    m->kd = kd_target * (float)j / 50.0f;

                    if (m->type == MOTOR_TYPE_AK60)
                        send_ak60_cmd(m);
                    else if (m->type == MOTOR_TYPE_AK40)
                        send_ak40_cmd(m);
                    else
                        send_ak70_cmd(m);

                    HAL_Delay(20);
                }
                m->kp = kp_target;
                m->kd = kd_target;

                m->pid_integral   = 0.0f;
                m->pid_prev_error = m->fb_pos;
                m->pid_active     = 0;
                m->enable_pending = 0;
                m->enabled = !uart_cmd_e_stop_requested();
            }

            // Passive feedback mode: poll state via 0xFC, no torque applied
            if (m->fb_only) {
                if (m->type != MOTOR_TYPE_AK60)
                    send_ak70_enable(m);  // 0xFC — triggers state response, no control
                continue;
            }

            // Non-enabled motors get a continuous zero-torque packet so that
            // if they power on after the STM32 they still snap to nothing.
            if (!m->enabled) {
                float saved_kp = m->kp, saved_kd = m->kd, saved_tff = m->tff;
                m->kp = 0.0f; m->kd = 0.0f; m->tff = 0.0f;
                if (m->type == MOTOR_TYPE_AK60)
                    send_ak60_cmd(m);
                else if (m->type == MOTOR_TYPE_AK40)
                    send_ak40_cmd(m);
                else
                    send_ak70_cmd(m);
                m->kp = saved_kp; m->kd = saved_kd; m->tff = saved_tff;
                continue;
            }

            // Rate-limit position: ramp cmd_pos toward m->pos
            float target = m->pos;
            float pos_err = target - cmd_pos[i];
            float max_step = POS_RATE_LIMIT * LOOP_DT;
            if (pos_err > max_step)
                cmd_pos[i] += max_step;
            else if (pos_err < -max_step)
                cmd_pos[i] -= max_step;
            else
                cmd_pos[i] = target;

            // Clamp torque to motor limits
            float t_max;
            if (m->type == MOTOR_TYPE_AK60)
                t_max = AK60_T_MAX;
            else if (m->type == MOTOR_TYPE_AK40)
                t_max = AK40_T_MAX;
            else if (m->type == MOTOR_TYPE_AK80_64)
                t_max = AK80_64_T_MAX;
            else
                t_max = AK70_T_MAX;

            float tff_pid = 0.0f;
            if (m->pid_active) {
                float pid_error = cmd_pos[i] - m->fb_pos;

                // Safety clamp: if error exceeds 1 rad (~57 deg), kill outer PID
                // and let the inner MIT spring handle recovery alone
                if (pid_error > 1.0f || pid_error < -1.0f) {
                    m->pid_active = 0;
                    m->pid_integral = 0.0f;
                    m->tff = 0.0f;
                } else {
                    float dfb = (m->fb_pos - m->pid_prev_error) / LOOP_DT;
                    m->pid_prev_error = m->fb_pos;
                    m->pid_integral += pid_error * LOOP_DT;
                    float i_max = (m->pid_ki > 0.001f) ? (t_max / m->pid_ki) : 0.0f;
                    if (m->pid_integral > i_max) m->pid_integral = i_max;
                    if (m->pid_integral < -i_max) m->pid_integral = -i_max;
                    tff_pid = m->pid_kp * pid_error
                            - m->pid_kd * dfb
                            + m->pid_ki * m->pid_integral;
                    if (tff_pid > t_max) tff_pid = t_max;
                    if (tff_pid < -t_max) tff_pid = -t_max;
                }
            }

            // Send CAN frame with rate-limited position and PID torque
            float saved_pos = m->pos;
            float saved_tff = m->tff;
            m->pos = cmd_pos[i];
            m->tff = tff_pid + saved_tff;
            if (m->tff > t_max) m->tff = t_max;
            if (m->tff < -t_max) m->tff = -t_max;

            if (m->type == MOTOR_TYPE_AK60)
                send_ak60_cmd(m);
            else if (m->type == MOTOR_TYPE_AK40)
                send_ak40_cmd(m);
            else
                send_ak70_cmd(m);

            m->pos = saved_pos;
            m->tff = saved_tff;
            HAL_Delay(2);
        }

        // Update servo PWM outputs
        update_servos();

        // Drain feedback buffer over UART (non-ISR context)
        unsigned drain_budget = 64;
        while (fb_tail != fb_head && drain_budget-- && !uart_cmd_e_stop_requested()) {
            uint8_t c = (uint8_t)fb_buf[fb_tail];
            HAL_UART_Transmit(&huart2, &c, 1, 2);
            fb_tail = (fb_tail + 1) % FB_BUF_SIZE;
        }

        HAL_Delay(2);
    }
    /* USER CODE END WHILE */
}

/* -------------------------------------------------------------------------- */
/*  Peripheral init                                                           */
/* -------------------------------------------------------------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_CAN1_Init(void)
{
    hcan1.Instance              = CAN1;
    hcan1.Init.Prescaler        = 3;
    hcan1.Init.Mode             = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth    = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1         = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2         = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = DISABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();

    CAN_FilterTypeDef filter;
    filter.FilterActivation     = ENABLE;
    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}

static void MX_TIM2_Init(void)
{
    // Direct register setup — bypasses HAL state machine issues

    // 1. Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    __DSB();  // ensure clock is active before accessing registers

    // PA0 remains TIM2_CH1. PA1/A1 must remain an input for the NC E-stop.
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER0) | (2U << 0);
    GPIOA->OSPEEDR |= (1U << 0);
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~0xFU) | 1U;

    // 3. Configure PB10 (TIM2_CH3) as AF1
    GPIOB->MODER   &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER   |=  (2U << (10*2));  // AF mode
    GPIOB->OSPEEDR |=  (1U << (10*2));  // Medium speed
    GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT10);  // Push-pull
    GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD10);  // No pull
    GPIOB->AFR[1]  &= ~(0xFU << ((10-8)*4));
    GPIOB->AFR[1]  |=  (1U << ((10-8)*4));  // AF1 for PB10

    // 4. Configure TIM2: 50Hz PWM, 1us resolution
    TIM2->PSC  = 84 - 1;     // 84MHz / 84 = 1MHz
    TIM2->ARR  = 20000 - 1;  // 1MHz / 20000 = 50Hz
    TIM2->CCR1 = 1500;       // 1.5ms = 90 deg
    TIM2->CCR2 = 1500;
    TIM2->CCR3 = 1500;

    // PWM mode 1 on CH1, CH2 (CCMR1), CH3 (CCMR2)
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE
                 | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

    // Enable CH1, CH2, CH3 outputs
    TIM2->CCER = 0; // Outputs stay inhibited until explicit reset.

    // Generate update event to load prescaler/ARR, then start counter
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    // PA1 is unused; no external E-stop contact is required.
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    estop_gpio_ready = 1;

    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
