/* Diagnostic USB (USART2) / motor UART (USART1) bridge, 115200 8N1.
 * PA9 TX -> motor RX; PA10 RX <- motor TX; common signal ground.
 * No CAN or PWM initialization. No generated motor commands.
 * Electrical compatibility must be checked before connecting hardware.
 */
#include "main.h"
CAN_HandleTypeDef hcan1; /* Required by shared interrupt source; never started. */
UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart1;
static uint8_t rx1, rx2;
typedef struct {
    uint8_t data[1024];
    volatile unsigned head, tail, dropped;
} Queue;
static Queue to_usb, to_motor;
static void push(Queue *q, uint8_t b) {
    unsigned next = (q->head + 1) % sizeof(q->data);
    if (next == q->tail) { q->dropped++; return; }
    q->data[q->head] = b;
    q->head = next;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
    if (h->Instance == USART1) {
        push(&to_usb, rx1);
        HAL_UART_Receive_IT(h, &rx1, 1);
    } else if (h->Instance == USART2) {
        push(&to_motor, rx2);
        HAL_UART_Receive_IT(h, &rx2, 1);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
    __HAL_UART_CLEAR_OREFLAG(h);
    HAL_UART_Receive_IT(h, h->Instance == USART1 ? &rx1 : &rx2, 1);
}
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
static void drain(Queue *q, UART_HandleTypeDef *h) {
    if (q->tail != q->head) {
        uint8_t byte = q->data[q->tail];
        if (HAL_UART_Transmit(h, &byte, 1, 10) == HAL_OK)
            q->tail = (q->tail + 1) % sizeof(q->data);
    }
}
static void init_uart(UART_HandleTypeDef *h, USART_TypeDef *instance) {
    h->Instance = instance;
    h->Init.BaudRate = 115200;
    h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits = UART_STOPBITS_1;
    h->Init.Parity = UART_PARITY_NONE;
    h->Init.Mode = UART_MODE_TX_RX;
    h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(h) != HAL_OK) Error_Handler();
}
int main(void) {
    HAL_Init(); /* Reset-default HSI clock, 16 MHz. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);
    init_uart(&huart1, USART1);
    init_uart(&huart2, USART2);
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart1, &rx1, 1);
    HAL_UART_Receive_IT(&huart2, &rx2, 1);
    while (1) {
        drain(&to_usb, &huart2);
        drain(&to_motor, &huart1);
    }
}
void Error_Handler(void) { __disable_irq(); while (1) {} }
