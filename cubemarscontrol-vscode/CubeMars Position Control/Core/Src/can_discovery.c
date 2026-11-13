/* Passive CAN diagnostic: silent mode, no CAN TX, PWM, or motor parser calls. */
#include "main.h"
#include "can_discovery.h"
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static uint32_t bitrate = 1000000;
static uint32_t frames, last_report;
static uint32_t seen_id[32];
static uint8_t seen_kind[32];
static unsigned seen_count;

static void print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void start_can(uint32_t rate)
{
    if (hcan1.Instance != NULL) {
        if (HAL_CAN_Stop(&hcan1) != HAL_OK) Error_Handler();
        if (HAL_CAN_DeInit(&hcan1) != HAL_OK) Error_Handler();
    }
    hcan1.Instance = CAN1;
    /* APB1 = 42 MHz; 14 time quanta per bit. */
    hcan1.Init.Prescaler = 3000000U / rate;
    hcan1.Init.Mode = CAN_MODE_SILENT;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();
    CAN_FilterTypeDef filter = {0};
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
    bitrate = rate;
    frames = 0;
    seen_count = 0;
    last_report = HAL_GetTick();
    char line[96];
    snprintf(line, sizeof(line), "Listening at %lu bit/s; silent mode (no ACK or commands).\r\n",
             (unsigned long)bitrate);
    print(line);
}

static void receive_frame(void)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8] = {0};
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, data) != HAL_OK) return;
    ++frames;
    uint8_t ext = rx.IDE == CAN_ID_EXT;
    uint8_t remote = rx.RTR == CAN_RTR_REMOTE;
    uint32_t id = ext ? rx.ExtId : rx.StdId;
    uint8_t kind = (uint8_t)(ext | (remote << 1));
    for (unsigned i = 0; i < seen_count; ++i)
        if (seen_id[i] == id && seen_kind[i] == kind) return;
    /* Print each identifier once per bitrate selection; bound UART traffic. */
    if (seen_count == 32) return;
    seen_id[seen_count] = id;
    seen_kind[seen_count++] = kind;
    char line[160];
    int len = snprintf(line, sizeof(line), "CAN %s %s id=0x%08lX dlc=%lu data=",
                       ext ? "EXT" : "STD", remote ? "RTR" : "DATA",
                       (unsigned long)id, (unsigned long)rx.DLC);
    if (!remote) {
        for (unsigned i = 0; i < rx.DLC && i < 8; ++i)
            len += snprintf(line + len, sizeof(line) - (unsigned)len, "%02X ", data[i]);
    }
    snprintf(line + len, sizeof(line) - (unsigned)len, "\r\n");
    print(line);
    uint8_t motor_id;
    if (can_discovery_candidate(ext, remote, id, (uint8_t)rx.DLC, data, &motor_id)) {
        snprintf(line, sizeof(line), "Candidate motor ID: %u (servo %s; verify with one motor connected).\r\n",
                 motor_id, (id >> 8) == 0x29 ? "status" : "start");
        print(line);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio); // A1 stays an input; no timer/PWM initialization.
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    print("\r\nCAN DISCOVERY ONLY - no motor control or power cutoff.\r\n"
          "USB keys: 1=1M, 5=500k, 2=250k, 8=125k, ?=help.\r\n"
          "A1 E-stop is reported only; this build cannot stop a powered motor.\r\n");
    start_can(1000000);
    while (1) {
        uint8_t key;
        if (HAL_UART_Receive(&huart2, &key, 1, 1) == HAL_OK) {
            switch (key) {
            case '1': start_can(1000000); break;
            case '5': start_can(500000); break;
            case '2': start_can(250000); break;
            case '8': start_can(125000); break;
            case '?': print("1=1M, 5=500k, 2=250k, 8=125k. No motor commands accepted.\r\n"); break;
            default: break;
            }
        }
        for (unsigned i = 0; i < 3 && HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0); ++i)
            receive_frame();
        if ((uint32_t)(HAL_GetTick() - last_report) >= 5000) {
            char line[192];
            snprintf(line, sizeof(line),
                     "%lu bit/s: frames=%lu, identifiers=%u/32, E-stop=%s, LEC=%lu, fifo_overrun=%s%s\r\n",
                     (unsigned long)bitrate, (unsigned long)frames, seen_count,
                     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET ? "closed" : "OPEN",
                     (unsigned long)((CAN1->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos),
                     (CAN1->RF0R & CAN_RF0R_FOVR0) ? "yes" : "no",
                     frames ? "" : "; no feedback detected");
            print(line);
            last_report = HAL_GetTick();
        }
    }
}

void Error_Handler(void)
{
    /* Stop on init failure; never fall back to normal CAN mode. */
    __disable_irq();
    while (1) {}
}
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

