/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 4-Unit Planter System (F411 Compatible)
  * @details        :
  * [System Overview]
  * - 4 Independent Planter Units (파종기 4개 독립 제어)
  * - Adaptive Optical Window based on Gear Rotation Speed (속도 반응형 윈도우)
  * - Jitter Normalization for 5/6 Pulse Pattern (5/6 펄스 패턴 정규화)
  *
  * [Pin Mapping]
  * - Unit 1: Optical(PA6), Gear(PB14)
  * - Unit 2: Optical(PA7), Gear(PB15)
  * - Unit 3: Optical(PB1), Gear(PA8)
  * - Unit 4: Optical(PB2), Gear(PA9)
  *
  * [Adaptive Logic]
  * - Ratio: 40% (Optimized for Acceleration)
  * - Min Window: 20ms (Optimized for High Speed)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 파종기 1개 유닛의 상태를 관리하는 구조체
typedef struct {
    // --- Gear Sensor State (기어 회전 감지) ---
    volatile uint32_t gear_cnt;          // 총 회전 수 (HMI 표시: n_gearX)
    volatile uint32_t gear_edge_cnt;     // 현재 섹션 내 펄스 카운트
    volatile uint32_t gear_target;       // 현재 섹션 목표 펄스 (5 또는 6)
    volatile bool gear_is_target_6;      // 5/6 패턴 토글 플래그
    volatile bool gear_state;            // 동작 상태 (true:주행중, false:멈춤)
    volatile uint32_t last_edge_time;    // 디바운싱용 마지막 펄스 시간
    GPIO_PinState last_stable_pin;       // 노이즈 필터링된 이전 핀 상태

    // ★ [Adaptive Window Variables] (가변 윈도우 핵심 변수)
    volatile uint32_t last_gear_count_time; // 이전 섹션 완료 시간 (속도 계산용)
    volatile uint32_t current_window_ms;    // 현재 계산된 차단 윈도우 시간

    // --- Optical Sensor State (씨앗 낙하 감지) ---
    volatile uint32_t opt_cnt;           // 씨앗 카운트 (HMI 표시: n_countX)
    volatile uint32_t last_opt_time;     // 윈도우 적용을 위한 마지막 감지 시간
} PlanterState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- [Sensor Parameters] ---
#define GEAR_DEBOUNCE_MS    5        // 기어 센서 기계적 진동 무시 시간
#define GEAR_TIMEOUT_MS     2000     // 2초 이상 신호 없으면 '멈춤'으로 간주

// ★ [Adaptive Window Tuning] (가변 윈도우 튜닝)
#define OPTICAL_WINDOW_DEFAULT 88    // 기본 윈도우 값 (초기/정지 시)
#define WINDOW_MIN_MS       20       // 최소 윈도우 (고속 주행 시 씨앗 놓침 방지)
#define WINDOW_MAX_MS       300      // 최대 윈도우 (저속 주행 시 중복 카운트 방지)
#define WINDOW_RATIO        40       // 섹션 시간 대비 윈도우 비율 (40%가 급가속 대응에 최적)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1; // HMI (Nextion)
UART_HandleTypeDef huart2; // Debug (PC)

/* USER CODE BEGIN PV */
PlanterState planters[4]; // 4개 유닛 객체 배열
char uart_buf[200];
uint8_t hmi_rx_byte;
const uint8_t nextion_end[3] = { 0xFF, 0xFF, 0xFF }; // Nextion 종료 커맨드
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Process_Gear_Signal(int unit_idx, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Process_Optical_Signal(int unit_idx, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 모든 파종기 유닛 초기화
void Init_Planters(void) {
    uint32_t now = HAL_GetTick();
    for(int i=0; i<4; i++) {
        planters[i].gear_cnt = 0;
        planters[i].gear_edge_cnt = 0;
        planters[i].gear_target = 5;      // 첫 패턴은 5펄스부터 시작
        planters[i].gear_is_target_6 = true;
        planters[i].gear_state = false;
        planters[i].last_edge_time = now;
        planters[i].last_stable_pin = GPIO_PIN_SET;

        // 가변 윈도우 변수 초기화
        planters[i].last_gear_count_time = now;
        planters[i].current_window_ms = OPTICAL_WINDOW_DEFAULT;

        planters[i].opt_cnt = 0;
        planters[i].last_opt_time = now;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  Init_Planters(); // 변수 초기화 호출

  sprintf(uart_buf, "\r\n=== 4-Unit Planter System (Adaptive Window) ===\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);

  // HMI 수신 인터럽트 활성화
  HAL_UART_Receive_IT(&huart1, &hmi_rx_byte, 1);

  uint32_t time_display = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    // ============================================================
    // [TASK 1] 기어 센서 타임아웃 체크 (멈춤 감지)
    // ============================================================
    for(int i=0; i<4; i++) {
        if (planters[i].gear_state) {
            // 마지막 신호로부터 2초 지났으면 멈춤으로 판단
            if (now - planters[i].last_edge_time > GEAR_TIMEOUT_MS) {
                planters[i].gear_state = false;
                // 멈췄을 때는 윈도우를 안전하게 기본값(88ms)으로 복구
                planters[i].current_window_ms = OPTICAL_WINDOW_DEFAULT;
            }
        }
    }

    // ============================================================
    // [TASK 2] 데이터 HMI 및 PC 전송 (250ms 주기)
    // ============================================================
    if (now - time_display >= 250)
    {
        time_display = now;

        // 인터럽트 충돌 방지를 위해 데이터 스냅샷 생성
        PlanterState snap[4];
        __disable_irq();
        for(int i=0; i<4; i++) snap[i] = planters[i];
        __enable_irq();

        // 1. PC Debug Output - [W]값(현재 윈도우) 모니터링
        sprintf(uart_buf, "U1[W:%lu,G:%lu,O:%lu] U2[W:%lu] U3[W:%lu] U4[W:%lu]\r\n",
                snap[0].current_window_ms, snap[0].gear_cnt, snap[0].opt_cnt,
                snap[1].current_window_ms, snap[2].current_window_ms, snap[3].current_window_ms);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);

        // 2. HMI Output (Nextion Display Update)
        for(int i=0; i<4; i++) {
            int idx = i + 1;
            // 회전수
            sprintf(uart_buf, "n_gear%d.val=%lu", idx, snap[i].gear_cnt);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
            HAL_UART_Transmit(&huart1, nextion_end, 3, 100);

            // 파종수
            sprintf(uart_buf, "n_count%d.val=%lu", idx, snap[i].opt_cnt);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
            HAL_UART_Transmit(&huart1, nextion_end, 3, 100);

            // 상태 (RUN/STOP)
            sprintf(uart_buf, "t_status%d.txt=\"%s\"", idx, snap[i].gear_state ? "RUN" : "STOP");
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
            HAL_UART_Transmit(&huart1, nextion_end, 3, 100);
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (F411 100MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16; // HSI=16MHz, /16 = 1MHz Input for PLL
  RCC_OscInitStruct.PLL.PLLN = 200; // 1MHz * 200 = 200MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 200MHz / 2 = 100MHz (SYSCLK)
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // PCLK1 max 50MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // PCLK2 max 100MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function (F4 Compatible)
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function (F4 Compatible)
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA6 PA7 (Optical Unit 1, 2) */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 (Optical Unit 3, 4) */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 (Gear Unit 3, 4) */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 (Gear Unit 1, 2) */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI Interrupt Init for F411 */
  // PB1 -> EXTI1
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  // PB2 -> EXTI2
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  // PA6, PA7, PA8, PA9 -> EXTI9_5 (통합 벡터)
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  // PB14, PB15 -> EXTI15_10 (통합 벡터)
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
// [Interrupt] 통합 콜백 (핀별로 함수 분배)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Unit 1
    if (GPIO_Pin == GPIO_PIN_6)       Process_Optical_Signal(0, GPIOA, GPIO_PIN_6);
    else if (GPIO_Pin == GPIO_PIN_14) Process_Gear_Signal(0, GPIOB, GPIO_PIN_14);
    // Unit 2
    else if (GPIO_Pin == GPIO_PIN_7)  Process_Optical_Signal(1, GPIOA, GPIO_PIN_7);
    else if (GPIO_Pin == GPIO_PIN_15) Process_Gear_Signal(1, GPIOB, GPIO_PIN_15);
    // Unit 3
    else if (GPIO_Pin == GPIO_PIN_1)  Process_Optical_Signal(2, GPIOB, GPIO_PIN_1);
    else if (GPIO_Pin == GPIO_PIN_8)  Process_Gear_Signal(2, GPIOA, GPIO_PIN_8);
    // Unit 4
    else if (GPIO_Pin == GPIO_PIN_2)  Process_Optical_Signal(3, GPIOB, GPIO_PIN_2);
    else if (GPIO_Pin == GPIO_PIN_9)  Process_Gear_Signal(3, GPIOA, GPIO_PIN_9);
}

// ------------------------------------------------------------
// [Logic] 기어 센서 처리 (5/6 패턴 인식 및 가변 윈도우 계산)
// ------------------------------------------------------------
void Process_Gear_Signal(int unit_idx, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    PlanterState *p = &planters[unit_idx];
    GPIO_PinState target_state = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

    // [1] 소프트웨어 글리치 필터 (매우 짧은 노이즈 무시)
    bool is_valid = true;
    for (int k = 0; k < 3; k++) {
        for(volatile int i = 0; i < 1500; i++) { __NOP(); }
        if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != target_state) {
            is_valid = false;
            break;
        }
    }

    if (is_valid)
    {
        // [2] 상태 변화 감지 및 디바운싱
        if (target_state == p->last_stable_pin) return;
        p->last_stable_pin = target_state;

        unsigned long now = HAL_GetTick();
        if (now - p->last_edge_time < GEAR_DEBOUNCE_MS) return;

        p->last_edge_time = now;
        p->gear_state = true;
        p->gear_edge_cnt++;

        // [3] 패턴 완료 체크 (5펄스 or 6펄스)
        if (p->gear_edge_cnt >= p->gear_target)
        {
            p->gear_cnt++;        // 전체 회전수 증가
            p->gear_edge_cnt = 0; // 펄스 카운트 초기화

            // 다음 타겟 토글 (5 <-> 6)
            if (p->gear_target == 6) p->gear_target = 5;
            else p->gear_target = 6;
            p->gear_is_target_6 = !p->gear_is_target_6;

            // ★★★ [가변 윈도우 계산 핵심 로직] ★★★
            uint32_t duration = now - p->last_gear_count_time;
            p->last_gear_count_time = now;

            // 유효한 주행 속도 범위 내일 때만 계산 (50ms ~ 3000ms)
            if (duration > 50 && duration < 3000)
            {
                // 1. 방금 지나온 섹션의 펄스 수 (5 or 6)
                uint32_t past_pulses = (p->gear_target == 6) ? 5 : 6;

                // 2. 펄스 1개당 시간 평균 계산 (정규화)
                uint32_t time_per_pulse = duration / past_pulses;

                // 3. 표준 5.5펄스 기준으로 시간 환산
                uint32_t normalized_duration = (time_per_pulse * 55) / 10;

                // 4. 비율(40%) 적용하여 윈도우 값 산출
                uint32_t new_win = (normalized_duration * WINDOW_RATIO) / 100;

                // 5. 안전 범위 클램핑 (최소 20ms, 최대 300ms)
                if (new_win < WINDOW_MIN_MS) new_win = WINDOW_MIN_MS;
                if (new_win > WINDOW_MAX_MS) new_win = WINDOW_MAX_MS;

                p->current_window_ms = new_win;
            }
        }
    }
}

// ------------------------------------------------------------
// [Logic] 광학 센서 처리 (가변 윈도우 적용)
// ------------------------------------------------------------
void Process_Optical_Signal(int unit_idx, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    PlanterState *p = &planters[unit_idx];
    unsigned long now = HAL_GetTick();

    // ★ 현재 계산된 가변 윈도우(current_window_ms)보다 시간이 더 흘렀을 때만 카운트
    if (now - p->last_opt_time > p->current_window_ms)
    {
        // 간단한 노이즈 필터 (3회 연속 High 체크)
        int pass_count = 0;
        for (int k = 0; k < 3; k++) {
            for(volatile int i = 0; i < 1500; i++) { __NOP(); }
            if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) pass_count++;
            else break;
        }

        if (pass_count == 3) {
            p->opt_cnt++;
            p->last_opt_time = now;
        }
    }
}

// ------------------------------------------------------------
// [Interrupt] HMI 리셋 명령 처리
// ------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 0x01: 전체 리셋 (기어, 윈도우 포함)
        if (hmi_rx_byte == 0x01) {
            __disable_irq();
            uint32_t now = HAL_GetTick();
            for(int i=0; i<4; i++) {
                planters[i].gear_cnt = 0;
                planters[i].gear_edge_cnt = 0;
                planters[i].gear_target = 5;
                planters[i].gear_is_target_6 = true;
                planters[i].gear_state = false;
                planters[i].current_window_ms = OPTICAL_WINDOW_DEFAULT;
                planters[i].last_gear_count_time = now;
            }
            __enable_irq();
        }
        // 0x02: 카운트만 리셋 (파종 수량 초기화)
        else if (hmi_rx_byte == 0x02) {
            __disable_irq();
            for(int i=0; i<4; i++) {
                planters[i].opt_cnt = 0;
                planters[i].last_opt_time = 0;
            }
            __enable_irq();
        }
        HAL_UART_Receive_IT(&huart1, &hmi_rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
