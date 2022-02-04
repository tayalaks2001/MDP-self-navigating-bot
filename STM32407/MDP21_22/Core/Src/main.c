/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ShowTask */
osThreadId_t ShowTaskHandle;
const osThreadAttr_t ShowTask_attributes = {
  .name = "ShowTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Move_LR */
osThreadId_t Move_LRHandle;
const osThreadAttr_t Move_LR_attributes = {
  .name = "Move_LR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UltraSonic */
osThreadId_t UltraSonicHandle;
const osThreadAttr_t UltraSonic_attributes = {
  .name = "UltraSonic",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART3 */
osThreadId_t UART3Handle;
const osThreadAttr_t UART3_attributes = {
  .name = "UART3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void encoder_task(void *argument);
void move_left_right(void *argument);
void ultra_sonic_task(void *argument);
void uart3(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*UART Variables*/
uint8_t aRxBuffer[20] = {0};
uint8_t uart_ready = 1;

/* Motor Variables */
int global_pwmL = 0;
int global_pwmR = 0;
int motor_case = 0;

/*Encoder Variables*/
int Rcnt1, Rcnt2, Rdiff;
int Lcnt1, Lcnt2, Ldiff;

/* Ultrasonic Variables ------------------------------------------------------*/
uint32_t F_IC_Val1 = 0;
uint32_t F_IC_Val2 = 0;
uint32_t F_Difference = 0;
uint8_t F_Is_First_Captured = 0;  // is the first value captured ?
uint8_t F_Distance  = 0;
uint8_t Front_US_Ready = 1;

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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_UART_Receive_DMA (&huart3, aRxBuffer, 20);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ShowTask */
  ShowTaskHandle = osThreadNew(show, NULL, &ShowTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder_task, NULL, &EncoderTask_attributes);

  /* creation of Move_LR */
  Move_LRHandle = osThreadNew(move_left_right, NULL, &Move_LR_attributes);

  /* creation of UltraSonic */
  UltraSonicHandle = osThreadNew(ultra_sonic_task, NULL, &UltraSonic_attributes);

  /* creation of UART3 */
  UART3Handle = osThreadNew(uart3, NULL, &UART3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




  while (1){
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  HAL_Delay(2000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  //USART3->CR1 |= (USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE);
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CIN1_Pin */
  GPIO_InitStruct.Pin = CIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CIN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : User_btn_Pin */
  GPIO_InitStruct.Pin = User_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_PIN_Pin */
  GPIO_InitStruct.Pin = TRIG_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_PIN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void motor_readjust()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 130; // left
	osDelay(100);
	htim1.Instance->CCR4 = 170; //Right
	osDelay(100);
	htim1.Instance->CCR4 = 148;//Center;
	osDelay(1000);
}


void motor_forward(int pwmL , int pwmR)
{
	//LEFT WHEELS
	 HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

	//RIGHT WHEELS
	 HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
}

void motor_backward(int pwmL, int pwmR)
{

	//LEFT WHEELS
	 HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
     HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

	//RIGHT WHEELS
	 HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
}

void motor_90degreesleft(int pwmL, int pwmR)
{
	htim1.Instance->CCR4 = 90;
	osDelay(10);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

	//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	osDelay(1800);

	htim1.Instance->CCR4 = 150;
	osDelay(100);
	motor_forward(pwmL+2000, pwmR);
	osDelay(2000);

}

void motor_90degreesright(int pwmL, int pwmR)
{
	htim1.Instance->CCR4 = 230;
	osDelay(10);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

	//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	osDelay(1800);

	htim1.Instance->CCR4 = 150;
	osDelay(100);
	motor_forward(pwmL, pwmR+2000);
	osDelay(2000);
}

void motor_180degrees_right(int pwmL, int pwmR)
{
	htim1.Instance->CCR4 = 230;
	osDelay(10);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

	//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	osDelay(3800);

	htim1.Instance->CCR4 = 150;
	osDelay(100);
	motor_forward(pwmL, pwmR+2000);
	osDelay(2000);
}

void motor_180_degrees_left(int pwmL, int pwmR)
{
	htim1.Instance->CCR4 = 90;
		osDelay(10);
		//LEFT WHEELS
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

		//RIGHT WHEELS
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

		osDelay(3800);

		htim1.Instance->CCR4 = 150;
		osDelay(100);
		motor_forward(pwmL, pwmR+2000);
		osDelay(2000);
}

void motor_stop()
{
	//LEFT WHEELS
	 HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
     HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

	//RIGHT WHEELS
	 HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

}


/*void HAL_UART_RxCpltCallBack(UART_HandleTypeDef *huart)
{
	Prevent unused argument(s) compilation warning
	UNUSED(huart);
		HAL_UART_Transmit(&huart,"OK\n",20,0xFFFF);
		  HAL_UART_Receive_DMA (&huart3, aRxBuffer, 20);

}*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	OLED_ShowString(10, 50, (uint8_t*)"CALLBACK");

    HAL_UART_Transmit(&huart3, aRxBuffer, 20, 100);
    HAL_UART_Receive_DMA(&huart3, aRxBuffer, 20);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == htim4.Instance && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is tim4chl1
	{
		//OLED_ShowString(0, 40, testMsg);
		if (F_Is_First_Captured==0) // if the first value is not captured
		{
			F_IC_Val1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1); // read the first value
			F_Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (F_Is_First_Captured==1)   // if the first is already captured
		{
			F_IC_Val2 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(&htim4, 0);  // reset the counter

			if (F_IC_Val2 > F_IC_Val1)
			{
				F_Difference = F_IC_Val2-F_IC_Val1;
			}

			else if (F_IC_Val1 > F_IC_Val2)
			{
				F_Difference = (0xffff - F_IC_Val1) + F_IC_Val2;
			}

			F_Distance = F_Difference * .034/2;
			F_Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			//__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
			Front_US_Ready = 1;
		}

	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t ch = 'A';
  /* Infinite loop */
  OLED_ShowString(10, 50, (uint8_t*)"DEFAULT TASK");
  for(;;)
  {
	  //HAL_UART_Transmit(&huart3, (uint8_t *)&ch,1,0xFFFF);
	  //HAL_UART_Receive_IT(&huart3, (uint8_t*)aRxBuffer, 20);
	  osDelay(1000);
	  if(HAL_GPIO_ReadPin(User_btn_GPIO_Port, User_btn_Pin)==0){
	  	osDelay(300);
	  	motor_case++;
	  	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  	//osDelay(200);
	  }
   }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the ShowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
	uint8_t hello[20] = "Hello World!\0";
	uint8_t ultra[20];
  /* Infinite loop */
  for(;;)
  {
    sprintf(ultra,"Distance :%5d\0",F_Distance);
    OLED_ShowString(10,10,ultra);
    OLED_Refresh_Gram();
     osDelay(100);  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_encoder_task */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_task */
void encoder_task(void *argument)
{
  /* USER CODE BEGIN encoder_task */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


	uint8_t Ltemp[20];
	uint16_t Ldir;
	Lcnt1 = __HAL_TIM_GET_COUNTER(&htim2);



	uint32_t tick;
	uint8_t Rtemp[20];
	uint16_t Rdir;
	Rcnt1 = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {
	 if(HAL_GetTick()-tick > 1000)
	 {
		 Lcnt2 = __HAL_TIM_GET_COUNTER(&htim2);
		 Rcnt2 = __HAL_TIM_GET_COUNTER(&htim3);

		 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
		 {
			 if(Lcnt2 < Lcnt1){
				 Ldiff = Lcnt1- Lcnt2;
			 }
			 else{
				 Ldiff = (65535 - Lcnt2) + Lcnt1;
				 if(Ldiff==65535)
				 {
					 Ldiff = 0;
				 }
			 }
		 }

		 else
		 {
			 if(Lcnt2 > Lcnt1){
				 Ldiff = Lcnt2 - Lcnt1;
			 }

			 else{
				 Ldiff = (65535-Lcnt1)+Lcnt2;
				 if(Ldiff == 65535){
					 Ldiff = 0;
				 }
			 }
		 }

		 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
			 if(Rcnt2<Rcnt1)
			 {
				 Rdiff = Rcnt1 -Rcnt2;
			 }
			 else
			 {
				 Rdiff = (65535-Rcnt2)+Rcnt1;
				 if(Rdiff == 65535)
				 {
					 Rdiff = 0;
				 }
			 }
		 }

		 else
		 {
			 if(Rcnt2 > Rcnt1){
				 Rdiff = Rcnt2 - Rcnt1;
			 }
			 else{
				 Rdiff = (65535-Rcnt1)+Rcnt2;
				 if(Rdiff == 65535)
				 {
					 Rdiff = 0;
				 }
			 }
		 }
	  	  sprintf(Ltemp, "LSpeed: %5d\0",Ldiff);
		  OLED_ShowString(10,20,Ltemp);
		  Ldir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
//		  sprintf(Ltemp, "LDir: %5d \0", Ldir);
//		  OLED_ShowString(10,30, Ltemp);

		  sprintf(Rtemp, "RSpeed: %5d\0",Rdiff);
		  OLED_ShowString(10,30,Rtemp);
		  Rdir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
//		  sprintf(Rtemp, "RDir: %5d \0", Rdir);
//		  OLED_ShowString(10,50, Rtemp);



		 Lcnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		 Rcnt1 = __HAL_TIM_GET_COUNTER(&htim3);
		 tick = HAL_GetTick();



		  osDelay(1);
	  }
  }
  osDelay(1000);
  /* USER CODE END encoder_task */
}

/* USER CODE BEGIN Header_move_left_right */
/**
* @brief Function implementing the Move_LR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_move_left_right */
void move_left_right(void *argument)
{
  /* USER CODE BEGIN move_left_right */
  /* Infinite loop */

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	motor_readjust();

	uint8_t desired_enc = 100;
	float kp = 0.8;
	uint8_t error =0;
	uint8_t a[20];

	//Test 90/180 degree turn
	osDelay(1000);
	motor_case = 9;
	for(;;){
//		error = desired_enc - Ldiff;
//		global_pwmL += kp * error;
//		sprintf(a,"LError: %d\0",error);
//		OLED_ShowString(10,40,a);
//
//		error = desired_enc - Rdiff;
//		global_pwmR += kp * error;
//		sprintf(a,"RError: %d\0",error);
//		OLED_ShowString(10,50,a);
//
//		motor_forward(global_pwmL, global_pwmR);
		switch(motor_case){

			case 0://Motor_stop
				motor_stop();
				break;

			case 1://MOVE FORWARD
				//motor_readjust();
				motor_forward(global_pwmL, global_pwmR);
				//osDelay(10);
				break;
			case 2://MOVE BACKWARD
				//motor_readjust();
				global_pwmR = 1000;
				global_pwmL = 1000;
				motor_backward(global_pwmL, global_pwmR);
				//osDelay(2000);
				break;

			case 3://MOVE FORWARD-LEFT
				//motor_readjust();
				global_pwmR = 1000;
				global_pwmL = 1000;
				htim1.Instance->CCR4 = 130;
				osDelay(100);
				motor_forward(global_pwmL, global_pwmR);
				//osDelay(2000);
				break;

			case 4: //MOVE FORWARD-RIGHT
				//motor_readjust();
				global_pwmR = 1000;
				global_pwmL = 1000;
				htim1.Instance->CCR4 = 180;
				osDelay(100);
				motor_forward(global_pwmL, global_pwmR);
				//osDelay(2000);
				break;

			case 5: //MOVE BACKWARDS LEFT
				//motor_readjust();
				global_pwmR = 1000;
				global_pwmL = 1000;
				htim1.Instance->CCR4 = 130;
				osDelay(100);
				motor_backward(global_pwmL, global_pwmR);
				//osDelay(2000);
				break;

			case 6: //MOVE BACKWARDS RIGHT
				//motor_readjust();
				global_pwmR = 1000;
				global_pwmL = 1000;
				htim1.Instance->CCR4 = 180;
				osDelay(100);
				motor_backward(global_pwmL, global_pwmR);
				//osDelay(2000);
				break;

			case 7: //90 DEG MOVE LEFT
				//motor_readjust();
				global_pwmR = 2000;
				global_pwmL = 0;
				motor_90degreesleft(global_pwmL, global_pwmR);
			case 8: // 90 DEG MOVE RIGHT
				global_pwmR = 0;
				global_pwmL = 2000;
				motor_90degreesright(global_pwmL, global_pwmR);
			case 9:
				global_pwmR = 0;
				global_pwmL = 2000;
				motor_180degrees_right(global_pwmL, global_pwmR);
			default:
				htim1.Instance->CCR4 = 150;
				osDelay(100);
				motor_stop();
				motor_case = 0;
				break;


		}

		osDelay(10);


	}

  /* USER CODE END move_left_right */
}

/* USER CODE BEGIN Header_ultra_sonic_task */
/**
* @brief Function implementing the UltraSonic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultra_sonic_task */
void ultra_sonic_task(void *argument)
{
  /* USER CODE BEGIN ultra_sonic_task */
  /* Infinite loop */
	int triggerCount = 0;
	int detectionCount = 0;
	int reqDetectionCount = 1;
  while(Front_US_Ready == 0){
	  osDelay(200);
  }
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
	for(;;)
  {
		Front_US_Ready = 0;
		HAL_GPIO_WritePin(GPIOD, TRIG_PIN_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		osDelay(10);  // wait for 10 us
		HAL_GPIO_WritePin(GPIOD, TRIG_PIN_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low


		osDelay(100);

  }
  /* USER CODE END ultra_sonic_task */
}

/* USER CODE BEGIN Header_uart3 */
/**
* @brief Function implementing the UART3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart3 */
void uart3(void *argument)
{
  /* USER CODE BEGIN uart3 */
  /* Infinite loop */

  for(;;)
  {
	if(uart_ready!=1 ){
		//HAL_UART_Transmit_IT(&huart3, (uint8_t*)''
	}

    osDelay(1);
  }
  /* USER CODE END uart3 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

