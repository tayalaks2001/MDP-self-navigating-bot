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
/* define ICM-20948 Device I2C address*/
#define I2C_ADD_ICM20948            0xD0
#define I2C_ADD_ICM20948_AK09916    0x0C
#define I2C_ADD_ICM20948_AK09916_READ  0x80
#define I2C_ADD_ICM20948_AK09916_WRITE 0x00
/* define ICM-20948 Register */
/* user bank 0 register */
#define REG_ADD_WIA             0x00
#define REG_VAL_WIA             0xEA
#define REG_ADD_USER_CTRL       0x03
#define REG_VAL_BIT_DMP_EN          0x80
#define REG_VAL_BIT_FIFO_EN         0x40
#define REG_VAL_BIT_I2C_MST_EN      0x20
#define REG_VAL_BIT_I2C_IF_DIS      0x10
#define REG_VAL_BIT_DMP_RST         0x08
#define REG_VAL_BIT_DIAMOND_DMP_RST 0x04
#define REG_ADD_PWR_MIGMT_1     0x06
#define REG_VAL_ALL_RGE_RESET   0x01
#define REG_VAL_RUN_MODE        0x01    //Non low-power mode
#define REG_ADD_LP_CONFIG       0x05
#define REG_ADD_PWR_MGMT_1      0x06
#define REG_ADD_PWR_MGMT_2      0x07
#define REG_ADD_ACCEL_XOUT_H    0x2D
#define REG_ADD_ACCEL_XOUT_L    0x2E
#define REG_ADD_ACCEL_YOUT_H    0x2F
#define REG_ADD_ACCEL_YOUT_L    0x30
#define REG_ADD_ACCEL_ZOUT_H    0x31
#define REG_ADD_ACCEL_ZOUT_L    0x32
#define REG_ADD_GYRO_XOUT_H     0x33
#define REG_ADD_GYRO_XOUT_L     0x34
#define REG_ADD_GYRO_YOUT_H     0x35
#define REG_ADD_GYRO_YOUT_L     0x36
#define REG_ADD_GYRO_ZOUT_H     0x37
#define REG_ADD_GYRO_ZOUT_L     0x38
#define REG_ADD_EXT_SENS_DATA_00 0x3B
#define REG_ADD_REG_BANK_SEL    0x7F
#define REG_VAL_REG_BANK_0  0x00
#define REG_VAL_REG_BANK_1  0x10
#define REG_VAL_REG_BANK_2  0x20
#define REG_VAL_REG_BANK_3  0x30

/* user bank 1 register */
/* user bank 2 register */
#define REG_ADD_GYRO_SMPLRT_DIV 0x00
#define REG_ADD_GYRO_CONFIG_1   0x01
#define REG_VAL_BIT_GYRO_DLPCFG_2   0x10 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_4   0x20 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_6   0x30 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_FS_250DPS  0x00 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_500DPS  0x02 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_DLPF       0x01 /* bit[0]   */
#define REG_ADD_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ADD_ACCEL_CONFIG        0x14
#define REG_VAL_BIT_ACCEL_DLPCFG_2  0x10 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_4  0x20 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_6  0x30 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_FS_2g     0x00 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_4g     0x02 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_8g     0x04 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_16g    0x06 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_DLPF      0x01 /* bit[0]   */

/* user bank 3 register */
#define REG_ADD_I2C_SLV0_ADDR   0x03
#define REG_ADD_I2C_SLV0_REG    0x04
#define REG_ADD_I2C_SLV0_CTRL   0x05
#define REG_VAL_BIT_SLV0_EN     0x80
#define REG_VAL_BIT_MASK_LEN    0x07
#define REG_ADD_I2C_SLV0_DO     0x06
#define REG_ADD_I2C_SLV1_ADDR   0x07
#define REG_ADD_I2C_SLV1_REG    0x08
#define REG_ADD_I2C_SLV1_CTRL   0x09
#define REG_ADD_I2C_SLV1_DO     0x0A

/* define ICM-20948 Register  end */

/* define ICM-20948 MAG Register  */
#define REG_ADD_MAG_WIA1    0x00
#define REG_VAL_MAG_WIA1    0x48
#define REG_ADD_MAG_WIA2    0x01
#define REG_VAL_MAG_WIA2    0x09
#define REG_ADD_MAG_ST2     0x10
#define REG_ADD_MAG_DATA    0x11
#define REG_ADD_MAG_CNTL2   0x31
#define REG_VAL_MAG_MODE_PD     0x00
#define REG_VAL_MAG_MODE_SM     0x01
#define REG_VAL_MAG_MODE_10HZ   0x02
#define REG_VAL_MAG_MODE_20HZ   0x04
#define REG_VAL_MAG_MODE_50HZ   0x05
#define REG_VAL_MAG_MODE_100HZ  0x08
#define REG_VAL_MAG_MODE_ST     0x10
/* define ICM-20948 MAG Register  end */

typedef struct icm20948_st_sensor_data_tag
{
	uint16_t s16X;
	uint16_t s16Y;
	uint16_t s16Z;
}ICM20948_ST_SENSOR_DATA;

typedef struct icm20948_st_avg_data_tag
{
	uint8_t u8Index;
	uint16_t s16AvgBuffer[8];
}ICM20948_ST_AVG_DATA;

typedef struct imu_st_sensor_data_tag
{
	uint16_t s16X;
	uint16_t s16Y;
	uint16_t s16Z;
}IMU_ST_SENSOR_DATA;
#define MAG_DATA_LEN    6
#define CONTROL_DELAY		1000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IMU_I2C_ADDRESS (0x68 << 1)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

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
/* Definitions for UltraSonic */
osThreadId_t UltraSonicHandle;
const osThreadAttr_t UltraSonic_attributes = {
  .name = "UltraSonic",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motortask */
osThreadId_t motortaskHandle;
const osThreadAttr_t motortask_attributes = {
  .name = "motortask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ICM_Task */
osThreadId_t ICM_TaskHandle;
const osThreadAttr_t ICM_Task_attributes = {
  .name = "ICM_Task",
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
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void encoder_task(void *argument);
void ultra_sonic_task(void *argument);
void Motor_Task(void *argument);
void ICM_20948_Task(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ICM20948 Variables --------------------------------------------------------*/
uint8_t icmData = 0x1;
uint8_t icmTempMsg[16] = {};
ICM20948_ST_SENSOR_DATA gstGyroOffset ={0,0,0};
IMU_ST_SENSOR_DATA gstMagOffset = {0,0,0};
int16_t TestmagnBuff[9]={0};
int16_t magn[3];
int16_t Deviation_gyro[3],Original_gyro[3];
int16_t s16Gyro[3], s16Accel[3], s16Magn[3];
int16_t gyro[3], accel[3],magnet[3];
uint16_t Deviation_Count = 0;
uint16_t expected_gyro =0;
int gyrosum = 0;


/*UART Variables*/
 uint8_t buffer_byte;
 uint8_t aRxBuffer[5] = {0};
 uint8_t aTxBuffer[1]= {0};
 uint8_t buffer_indx = 0;
uint8_t uart_ready = 1;
uint8_t flag = '0';

/* Motor Variables */
int motor_case = 0;
uint16_t pwmL = 0;
uint16_t pwmR = 0;
uint16_t time_taken = 0;
uint16_t distance = 0;

uint8_t direction = 1;


const float hallPerMM = 1.62;
int lCounter = 0;
int lCounterPrev=0;
int lCounterCum = 0;
uint32_t lLastDiff = 0;
uint32_t rLastDiff = 0;

/*Encoder Variables*/
const float circum = 2*3.142*3.4;
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
  //ICMInit();
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
// MX_USART3_UART_Init();
  MX_TIM3_Init();
 // MX_DMA_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MX_DMA_Init();
  MX_USART3_UART_Init();
  OLED_Init();
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  //HAL_UART_Receive_IT (&huart3, aRxBuffer, 12);

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

  /* creation of UltraSonic */
  UltraSonicHandle = osThreadNew(ultra_sonic_task, NULL, &UltraSonic_attributes);

  /* creation of motortask */
  motortaskHandle = osThreadNew(Motor_Task, NULL, &motortask_attributes);

  /* creation of ICM_Task */
  ICM_TaskHandle = osThreadNew(ICM_20948_Task, NULL, &ICM_Task_attributes);

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


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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

  /*Configure GPIO pins : CIN1_Pin Enable_Switch_Pin */
  GPIO_InitStruct.Pin = CIN1_Pin|Enable_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
void ICMWriteOneByte(uint8_t RegAddr, uint8_t Data)
{
	HAL_I2C_Mem_Write(&hi2c1, I2C_ADD_ICM20948, RegAddr, I2C_MEMADD_SIZE_8BIT, &Data, 1, 0xffff);
}

uint8_t ICMReadOneByte(uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	HAL_I2C_Mem_Read(&hi2c1, I2C_ADD_ICM20948, RegAddr, I2C_MEMADD_SIZE_8BIT, &TempVal, 1, 0xffff);
	return TempVal;
}

void ICMReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
    uint8_t i;
    uint8_t u8Temp;

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    ICMWriteOneByte(REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV0_REG,  u8RegAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = ICMReadOneByte(REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);
    osDelay(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);

    for(i=0; i<u8Len; i++)
    {
        *(pu8data+i) = ICMReadOneByte(REG_ADD_EXT_SENS_DATA_00+i);

    }
    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = ICMReadOneByte(REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL,  u8Temp);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

}

void ICMWriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
    uint8_t u8Temp;
    ICMWriteOneByte(REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    ICMWriteOneByte(REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_REG,  u8RegAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_DO,   u8data);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = ICMReadOneByte(REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);
    osDelay(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = ICMReadOneByte(REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL,  u8Temp);

   ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    return;
}

void ICMWhoIAm()
{
	uint8_t ICM_OK_Msg[8] = "ICM OK";
	if (REG_VAL_WIA == ICMReadOneByte(REG_ADD_WIA))
	{
		OLED_ShowString(0,20,ICM_OK_Msg);
	}
}

void ICMCalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
	uint8_t i;

	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex &= 0x07;

  	*pOutVal = 0;
	for(i = 0; i < 8; i ++)
  	{
    	*pOutVal += *(pAvgBuffer + i);
  	}
  	*pOutVal >>= 3;
}

void ICMGyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    static int16_t ss16c = 0;
    ss16c++;

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_XOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_XOUT_H);
    s16Buf[0] =	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_YOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_YOUT_H);
    s16Buf[1] =	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_ZOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_ZOUT_H);
    s16Buf[2] =	(u8Buf[1]<<8)|u8Buf[0];

#if 1
    for(i = 0; i < 3; i ++)
    {
        ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
    *ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
    *ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;
#else
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif
    return;
}

void ICMAccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
   uint8_t u8Buf[2];
   int16_t s16Buf[3] = {0};
   uint8_t i;
   int32_t s32OutBuf[3] = {0};
   static ICM20948_ST_AVG_DATA sstAvgBuf[3];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_XOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_XOUT_H);
   s16Buf[0] = (u8Buf[1]<<8)|u8Buf[0];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_YOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_YOUT_H);
   s16Buf[1] = (u8Buf[1]<<8)|u8Buf[0];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_ZOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_ZOUT_H);
   s16Buf[2] = (u8Buf[1]<<8)|u8Buf[0];

#if 1
   for(i = 0; i < 3; i ++)
   {
       ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
   }
   *ps16X = s32OutBuf[0];
   *ps16Y = s32OutBuf[1];
   *ps16Z = s32OutBuf[2];

#else
   *ps16X = s16Buf[0];
   *ps16Y = s16Buf[1];
   *ps16Z = s16Buf[2];
#endif
   return;

}

void ICMMagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
   uint8_t counter = 20;
   uint8_t u8Data[MAG_DATA_LEN];
   int16_t s16Buf[3] = {0};
   uint8_t i;
   int32_t s32OutBuf[3] = {0};
   static ICM20948_ST_AVG_DATA sstAvgBuf[3];
   while( counter>0 )
   {
       osDelay(10);
       ICMReadSecondary(I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_ST2, 1, u8Data);

       if ((u8Data[0] & 0x01) != 0)
           break;

       counter--;
   }

   if(counter != 0)
   {
       ICMReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_DATA,
                                   MAG_DATA_LEN,
                                   u8Data);
       s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
       s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
       s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];
   }
   else
   {
       printf("\r\n Mag is busy \r\n");
   }
#if 1
   for(i = 0; i < 3; i ++)
   {
       ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
   }

   *ps16X =  s32OutBuf[0];
   *ps16Y = -s32OutBuf[1];
   *ps16Z = -s32OutBuf[2];
#else
   *ps16X = s16Buf[0];
   *ps16Y = -s16Buf[1];
   *ps16Z = -s16Buf[2];
#endif
   return;
}

void ICMGyroOffset()
{
	uint8_t i;
    int16_t	s16Gx = 0, s16Gy = 0, s16Gz = 0;
	int32_t	s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
    for(i = 0; i < 32; i ++)
 	{
        ICMGyroRead(&s16Gx, &s16Gy, &s16Gz);
        s32TempGx += s16Gx;
		s32TempGy += s16Gy;
		s32TempGz += s16Gz;
        osDelay(10);
    }
    gstGyroOffset.s16X = s32TempGx >> 5;
	gstGyroOffset.s16Y = s32TempGy >> 5;
	gstGyroOffset.s16Z = s32TempGz >> 5;
    return;
}

void MPU_Get_Gyroscope(void)
{
	ICMGyroRead(&gyro[0], &gyro[1], &gyro[2]);
//	if(Deviation_Count==CONTROL_DELAY)
//	{
		//Save the raw data to update zero by clicking the user button
		Original_gyro[0] = gyro[0];
		Original_gyro[1] = gyro[1];
		Original_gyro[2] = gyro[2];

		//Removes zero drift data
		gyro[0] = Original_gyro[0]-Deviation_gyro[0];
		gyro[1] = Original_gyro[1]-Deviation_gyro[1];
		gyro[2] = Original_gyro[2]-Deviation_gyro[2];
//	}
	return;
}

void ICMInit()
{
	/* user bank 0 register */
	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
	ICMWriteOneByte(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
	osDelay(10);
	ICMWriteOneByte(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);

	/* user bank 2 register */

	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);

	ICMWriteOneByte(REG_ADD_GYRO_SMPLRT_DIV, 0x07);
	ICMWriteOneByte(REG_ADD_GYRO_CONFIG_1,REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_500DPS | REG_VAL_BIT_GYRO_DLPF);

	ICMWriteOneByte(REG_ADD_ACCEL_SMPLRT_DIV_2,  0x07);
	ICMWriteOneByte(REG_ADD_ACCEL_CONFIG,REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);

	/* user bank 0 register */
	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

	osDelay(100);
	/* offset */
	ICMGyroOffset();

	ICMWriteSecondary(I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);
}



int e1_error = 0;
int e2_error = 0;
uint16_t target = 3000;
uint16_t m1_speed = 0;
uint16_t m2_speed = 0;

float KP =0.0008;

void wheels_straight(){
	htim1.Instance->CCR4 = 147;
	osDelay(10);
}
void wheels_right(int degree){
	htim1.Instance->CCR4 = 147 + degree;
	osDelay(10);
}
void wheels_left(int degree){
	htim1.Instance->CCR4 = 147 - degree;
	osDelay(10);
}

void motor_readjust()
{
	//return;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	wheels_left(10);
	osDelay(500);
	wheels_right(10);
	osDelay(500);
	wheels_straight();
	osDelay(500);
}

/*---------------------------------------Week 8 Task(Indoor)of pwm 2000------------------------------------------------------*/

void move_forward_indoor_dist(int distance_mm)
{
	pwmL = 1.03*1500;
	pwmR = 1500;

	wheels_straight();
	osDelay(1);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

	//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
	osDelay(1);

	/* First 10 cm is  385 ms */
	/* Distance of 20 cm is 815 ms
	 * Distance of 30 cm is 1210 ms
	 * Distance of 50 cm is 1910 ms
	 * Distance of 80 cm is 3200 ms
	 * Distance of 100 cm is 4000ms
	 * Equation for PWM 1500
	 */

	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	//ticks = 7.4436x + 0.0087
	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

	uint8_t en_msg[10];

//	int time_needed = 4*distance_mm + 2.2293;
//    uint32_t t_start = HAL_GetTick();
    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		sprintf(en_msg,"diff: %d",encoderDiff);
		OLED_ShowString(10,20, en_msg);
		outdoor_fakePID();
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
		osDelay(10);
		wheels_straight();
		osDelay(10);
	}
	motor_stop();
	osDelay(1);
}

void move_forward_outdoor_dist(int distance_mm)
{
	pwmL = 1500;
	pwmR = 1500;

	wheels_straight();
	osDelay(1);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

	//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
	osDelay(1);

	/* First 10 cm is  390 ms */
	/* Distance of 20 cm is 830 ms
	 * Distance of 30 cm is 1210 ms
	 * Distance of 50 cm is 1910 ms
	 * Distance of 80 cm is 3200 ms
	 * Distance of 100 cm is 3800ms
	 * Equation for PWM 1500
	 */


	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	//ticks = 7.4436x + 0.0087
	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

	uint8_t en_msg[10];

//	int time_needed = 4*distance_mm + 2.2293;
//    uint32_t t_start = HAL_GetTick();
    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		sprintf(en_msg,"diff: %d",encoderDiff);
		OLED_ShowString(10,20, en_msg);
		outdoor_fakePID();
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
		osDelay(10);
		wheels_straight();
		osDelay(10);
	}
	motor_stop();
	osDelay(1);
}

void move_backward_indoor_dist(int distance_mm)
{
	pwmL = 1500;
	pwmR = 1500;

	wheels_straight();
	osDelay(1);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

	//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
	osDelay(1);

	/* First 10 cm is  390 ms */
	/* Distance of 20 cm is 830 ms
	 * Distance of 30 cm is 1210 ms
	 * Distance of 50 cm is 1910 ms
	 * Distance of 80 cm is 3200 ms
	 * Distance of 100 cm is 3800ms
	 * Equation for PWM 1500
	 */


	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	//ticks = 7.4436x + 0.0087
	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

	uint8_t en_msg[10];

//	int time_needed = 4*distance_mm + 2.2293;
//    uint32_t t_start = HAL_GetTick();
    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		sprintf(en_msg,"diff: %d",encoderDiff);
		OLED_ShowString(10,20, en_msg);
		indoor_fakePID();
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
		osDelay(10);
		wheels_straight();
		osDelay(10);
	}
	motor_stop();
	osDelay(1);
}
/*---------------------------------------Week 9 Task(outdoor)of pwm 2000------------------------------------------------------*/

void move_backward_outdoor_dist(int distance_mm)
{
	pwmL = 1500;
	pwmR = 1500;

	wheels_straight();
	osDelay(1);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

	//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
	osDelay(1);

	/* First 10 cm is  390 ms */
	/* Distance of 20 cm is 830 ms
	 * Distance of 30 cm is 1210 ms
	 * Distance of 50 cm is 1910 ms
	 * Distance of 80 cm is 3200 ms
	 * Distance of 100 cm is 3800ms
	 * Equation for PWM 1500
	 */


	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	//ticks = 7.4436x + 0.0087
	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

	uint8_t en_msg[10];

//	int time_needed = 4*distance_mm + 2.2293;
//    uint32_t t_start = HAL_GetTick();
    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		sprintf(en_msg,"diff: %d",encoderDiff);
		OLED_ShowString(10,20, en_msg);
		outdoor_fakePID();
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);
		osDelay(10);
		wheels_straight();
		osDelay(10);
	}
	motor_stop();
	osDelay(1);
}

/*-----------------------------------Assessment Requirement----------------------------------------*/

/*-----------------------------------Code for 3 point Turn ----------------------------------------*/
void indoor_three_point_turnR()
{
	move_indoor_forward_right(80,150);
	osDelay(500);
	move_indoor_backward_left(40, 100);
	osDelay(200);
	move_indoor_forward_right(55,180);
	osDelay(200);
	move_indoor_backward_left(45, 140);
	osDelay(200);
}

void indoor_three_point_turnL()
{
	move_indoor_forward_left(45, 140);
	osDelay(200);
	move_indoor_backward_right(55, 180);
	osDelay(200);
	move_indoor_forward_left(40, 100);
	osDelay(200);
	move_indoor_backward_right(80,160);
	osDelay(200);
}

void outdoor_three_point_turnR()
{
	move_outdoor_forward_right(80,150);
	osDelay(500);
	move_outdoor_backward_left(40, 100);
	osDelay(200);
	move_outdoor_forward_right(55,180);
	osDelay(200);
	move_outdoor_backward_left(45, 140);
	osDelay(200);
}

void outdoor_three_point_turnL()
{
	move_outdoor_forward_left(45, 140);
	osDelay(200);
	move_outdoor_backward_right(55, 180);
	osDelay(200);
	move_outdoor_forward_left(40, 100);
	osDelay(200);
	move_outdoor_backward_right(80,160);
	osDelay(200);
}

void random_path()
{
	move_forward_outdoor_dist(1000);
	osDelay(200);
	outdoor_three_point_turnR();
	osDelay(200);
	outdoor_three_point_turnL();
	osDelay(200);
	move_forward_outdoor_dist(1000);
	osDelay(200);
	outdoor_three_point_turnL();
	osDelay(200);
	move_forward_outdoor_dist(1000);
}
void move_indoor_forward_right(uint8_t angle, int distance_mm)
{
	pwmL = 2000;
	pwmR = 1500;
	direction = 1;
	wheels_straight();
	osDelay(200);
	wheels_right(angle);
	osDelay(500);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

			//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

	uint8_t en_msg[10];

    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		osDelay(10);
	}
	motor_stop();
	osDelay(1);


}

void move_indoor_backward_left(uint8_t angle, int distance_mm)
{
	pwmL = 1500;
	pwmR = 2000;
	direction = 0;
	wheels_straight();
	osDelay(200);
	wheels_left(angle);
	osDelay(500);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

			//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);



	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}


    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		osDelay(10);
	}
	motor_stop();
	osDelay(1);
}


void move_indoor_forward_left(uint8_t angle, int distance_mm)
{
	pwmL = 1500;
	pwmR = 2000;
	direction = 1;
	wheels_straight();
	osDelay(200);
	wheels_left(angle);
	osDelay(500);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

			//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		osDelay(10);
	}
	motor_stop();
	osDelay(1);
}

void move_indoor_backward_right(uint8_t angle, int distance_mm)
{
	pwmL = 2000;
	pwmR = 1500;
	direction = 0;
	wheels_straight();
	osDelay(200);
	wheels_right(angle);
	osDelay(500);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

			//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

	uint8_t en_msg[10];

    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		osDelay(10);
	}
	motor_stop();
	osDelay(1);

}

/***************************************************************************************OUTDOOR-----------------------*/
void move_outdoor_forward_left(uint8_t angle, int distance_mm)
{
	pwmL = 1500;
	pwmR = 2000;
	direction = 1;
	wheels_straight();
	osDelay(200);
	wheels_left(angle);
	osDelay(500);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

			//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		osDelay(10);
	}
	motor_stop();
	osDelay(1);
}

void move_outdoor_backward_right(uint8_t angle, int distance_mm)
{
	pwmL = 2000;
	pwmR = 1500;
	direction = 0;
	wheels_straight();
	osDelay(200);
	wheels_right(angle);
	osDelay(500);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

			//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

	uint8_t en_msg[10];

    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		osDelay(10);
	}
	motor_stop();
	osDelay(1);

}
void move_outdoor_forward_right(uint8_t angle, int distance_mm)
{
	pwmL = 2000;
	pwmR = 1500;
	direction = 1;
	wheels_straight();
	osDelay(200);
	wheels_right(angle);
	osDelay(500);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

			//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}

	uint8_t en_msg[10];

    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		osDelay(10);
	}
	motor_stop();
	osDelay(1);


}

void move_outdoor_backward_left(uint8_t angle, int distance_mm)
{
	pwmL = 1500;
	pwmR = 2000;
	direction = 0;
	wheels_straight();
	osDelay(200);
	wheels_left(angle);
	osDelay(500);
	//LEFT WHEELS
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);

			//RIGHT WHEELS
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);



	int targetdiff =  7.4436*distance_mm + 0.0087;
	int encoderDiff = 0;

	int L2 = 0;

	int L1 = __HAL_TIM_GET_COUNTER(&htim2);
	if(L1 == 0 && direction == 1)
	{
		L1 = 65535;
	}


    while(encoderDiff < (targetdiff -300))
	{
    	L2 = __HAL_TIM_GET_COUNTER(&htim2);
		if(direction == 1)
		{
			if(L2 == 0)
			{
				L2 = 65535;
			}

			if(L1 >= L2)
			{
				encoderDiff = L1-L2;
			}
			else
			{
				encoderDiff = (65535 -L2) + L1;
			}
		}

		else
		{
			if(L2 == 65535)
			{
				L2 =0;
			}
			if(L2 >= L1)
			{
				encoderDiff = L2 - L1;
			}
			else
			{
				encoderDiff = (65535 - L1 ) + L2;
			}

		}
		osDelay(10);
	}
	motor_stop();
	osDelay(1);
}
/*-------------------------------------------------------------------------------------------------*/

void move_90turnR()
{
	pwmL = 5000;
	pwmR = 500;

	time_taken = 560;

	//LEFT WHEELS
	 HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

	//RIGHT WHEELS
	 HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	wheels_right(120);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	uint32_t t_start = HAL_GetTick();

	while(HAL_GetTick()-t_start < time_taken )
	{
		osDelay(1);
	}


	wheels_straight();
	osDelay(100);
	motor_stop();
}

void move_90turnL()
{

	pwmL = 800;
	pwmR = 4000;
	int time_taken = 925;
	//LEFT WHEELS
	 HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
     HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);

	//RIGHT WHEELS
	 HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);

	wheels_left(35);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);

	uint32_t t_start = HAL_GetTick();

	while(HAL_GetTick()-t_start < time_taken)
	{
		osDelay(1);
	}
	wheels_straight();
	osDelay(100);
	motor_stop();
}

void indoor_fakePID()
{

	int expected;
	expected = 1800;

	if(direction == 1)
	{
		if(Ldiff > expected)
		{
			wheels_left(4);
			pwmL = pwmL - 5;
		}
		else if(Ldiff < expected)
		{
			wheels_right(5);
			pwmL = pwmL + 5;

		}

		if(Rdiff > expected)
		{
			wheels_right(4);
			pwmR = pwmR - 5;
		}
		else if(Rdiff < expected)
		{
			wheels_left(4);
			pwmR = pwmR + 5;

		}
	}
	else
	{
		if(Ldiff > expected)
		{
			wheels_right(4);
			pwmL = pwmL - 5;
		}
		else if(Ldiff < expected)
		{
			wheels_left(5);
			pwmL = pwmL + 5;

		}

		if(Rdiff > expected)
		{
			wheels_left(4);
			pwmR = pwmR - 5;
		}
		else if(Rdiff < expected)
		{
			wheels_right(4);
			pwmR = pwmR + 5;

		}
	}
//	int16_t actual_gyro = gyro[2];
//	if(actual_gyro < expected_gyro)
//	{
//		wheels_left(10);
//		wheels_straight();
//	}
//	else if (actual_gyro > expected_gyro)
//	{
//		wheels_right(20);
//		wheels_straight();
//	}



}

void outdoor_fakePID()
{
//	if(gyro[2] > 50)
//	{
//		wheels_right(8);
//		osDelay(80);
//		wheels_straight();
//	}
//	else if (gyro[2] < -50)
//	{
//		wheels_left(5);
//		osDelay(80);
//		wheels_straight();
//	}
	int expected;
	expected = 1800;

	if(direction == 1)
	{
		if(Ldiff > expected)
		{
			wheels_left(4);
			pwmL = pwmL - 5;
		}
		else if(Ldiff < expected)
		{
			wheels_right(10);
			pwmL = pwmL + 5;

		}

		if(Rdiff > expected)
		{
			wheels_right(10);
			pwmR = pwmR - 5;
		}
		else if(Rdiff < expected)
		{
			wheels_left(4);
			pwmR = pwmR + 5;

		}
	}
	else
	{
		if(Ldiff > expected)
		{
			wheels_right(10);
			pwmL = pwmL - 5;
		}
		else if(Ldiff < expected)
		{
			wheels_left(5);
			pwmL = pwmL + 5;

		}

		if(Rdiff > expected)
		{
			wheels_left(4);
			pwmR = pwmR - 5;
		}
		else if(Rdiff < expected)
		{
			wheels_right(10);
			pwmR = pwmR + 5;

		}
	}

}

/*-----------------------------------OUTDOOR FASTEST CAR CODE--------------------------------------------------*/

void outdoor_fastestcar_150cm()
{
	motor_readjust();
	//move_90turnL(1000,4550,2500,5);
	//move_90turnR(4500,1000,750,80);
	/*First 500ms is 34.5cm  for 4000 pwm
	 * Subsequent 100ms is 42.1 cm so 7.5cm per 100ms
	 */

}


/*-----------------------------------Assessment Requirement----------------------------------------*/

void motor_stop()
{
	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	 wheels_straight();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	//OLED_ShowString(20, 50, (uint8_t*)"CALLBACK");
	osDelay(1000);
   if(huart->Instance == USART3)
   {
	   HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	   osDelay(100);
   }
   motor_case = aRxBuffer[0];
   distance = ((uint16_t)(aRxBuffer[1]-48))*1000 + (uint16_t)(aRxBuffer[2]-48)*100 + ((uint16_t)aRxBuffer[3]-48)*10 + ((uint16_t)aRxBuffer[4]-48);
   HAL_UART_Receive_DMA(&huart3, aRxBuffer,5);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	aTxBuffer[0] = flag;
	HAL_UART_Transmit_DMA(&huart3, aTxBuffer, 1);
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
  /* Infinite loop */

   HAL_UART_Receive_DMA(&huart3, aRxBuffer, 5);
   osDelay(100);
   HAL_UART_Transmit_DMA(&huart3, aTxBuffer, 1);
  for(;;)
  {
	  if(HAL_GPIO_ReadPin(User_btn_GPIO_Port, User_btn_Pin)==0)
	  {
		  osDelay(200);
		  motor_case++;
		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		  osDelay(100);
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
	uint8_t ultra[20];
	uint8_t pwm[10];
	uint8_t switchcase[10];
	uint8_t Distance_travelled[10];
	uint8_t Mode[20];
	char mode_ind[10] = "Indoor";
	char mode_out[10] = "Outdoor";
  /* Infinite loop */
  for(;;)
  {
    sprintf(switchcase,"MotorCase: %d\0",(char)motor_case);
    OLED_ShowString(10, 10, switchcase);
    OLED_Refresh_Gram();
    osDelay(10);

//    sprintf(Distance_travelled,"Dist_Tr: %d\0",distance);
//    OLED_ShowString(10, 20, Distance_travelled);
//    OLED_Refresh_Gram();
//    osDelay(10);

	OLED_ShowString(10,30,aRxBuffer);
	OLED_Refresh_Gram();
	osDelay(10);

	if(HAL_GPIO_ReadPin(Enable_Switch_GPIO_Port, Enable_Switch_Pin)==0)
	{
		OLED_Refresh_Gram();
		osDelay(10);
		sprintf(Mode, "Mode : %s\0", mode_ind);
		OLED_ShowString(10, 40, Mode);
		osDelay(100);
	}
	else if (HAL_GPIO_ReadPin(Enable_Switch_GPIO_Port, Enable_Switch_Pin)==1)
	{
		OLED_Refresh_Gram();
		osDelay(10);
		sprintf(Mode, "Mode : %s\0", mode_out);
		OLED_ShowString(10, 40, Mode);
		osDelay(100);
	}
	sprintf(ultra,"UDistance:%5d\0",F_Distance);
	OLED_ShowString(10,50,ultra);
    OLED_Refresh_Gram();
    osDelay(10);
  }
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

	uint8_t tick_count[10];
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
			 else
			 {
				 Ldiff = (65535 - Lcnt2) + Lcnt1;
				 if(Ldiff == 65535)
				 {
					 Ldiff = 0;
				 }
			 }
		 }
		 else
		 {
			 if(Lcnt2 > Lcnt1)
			 {
				 Ldiff = Lcnt2 - Lcnt1;
				 if(Ldiff == 65535)
				 {
					 Ldiff = 0;
				 }
			 }

			 else
			 {
				 Ldiff = (65535-Lcnt1)+Lcnt2;
				 if(Ldiff == 65535)
				 {
					 Ldiff = 0;
				 }
			 }
		 }

		 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
		 {
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
			 if(Rcnt2 > Rcnt1)
			 {
				 Rdiff = Rcnt2 - Rcnt1;
			 }
			 else
			 {
				 Rdiff = (65535-Rcnt1)+Rcnt2;
				 if(Rdiff == 65535)
				 {
					 Rdiff = 0;
				 }

			 }
		 }

	  	  sprintf(Ltemp, "LSpeed: %5d\0",Ldiff);
		  //OLED_ShowString(10,20,Ltemp);
//		  Ldir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
//		  sprintf(Ltemp, "LDir: %5d \0", Ldir);
//		  OLED_ShowString(10,30, Ltemp);
//
		  sprintf(Rtemp, "RSpeed: %5d\0",Rdiff);
		  //OLED_ShowString(10,30,Rtemp);
		  Rdir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);

//		  sprintf(Rtemp, "RDir: %5d \0", Rdir);
//		  OLED_ShowString(10,50, Rtemp);
		 Lcnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		 Rcnt1 = __HAL_TIM_GET_COUNTER(&htim3);
		 tick = HAL_GetTick();
		 osDelay(1);
	  }
	  lCounterCum=__HAL_TIM_GET_COUNTER(&htim3);
	  char output[12];

	  sprintf(output, "%8d\0",lCounterCum);
	  //OLED_ShowString(10, 40,output);
	  //osDelay(1);

  }
  /* USER CODE END encoder_task */
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

/* USER CODE BEGIN Header_Motor_Task */
/**
* @brief Function implementing the motortask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task */
void Motor_Task(void *argument)
{
  /* USER CODE BEGIN Motor_Task */
  /* Infinite loop */
/*---------------------------MOTOR MOVEMENT-------------------------------------------------*/
  for(;;)
  {
	  /*************************************************INDOOR*************************************************************************/
	  if(HAL_GPIO_ReadPin(Enable_Switch_GPIO_Port, Enable_Switch_Pin)==0)//INDOOR
	  {
		  switch(motor_case){
			/*---------------------------MOTOR STOP-------------------------------------------------*/
				case 'H'://Motor_stop;
					if(motor_case != 'H'){break;}
					motor_stop();
					wheels_straight();
					break;
			/*---------------------------MOTOR FORWARD-------------------------------------------------*/
				case 'F'://MOVE FORWARD
					if(motor_case != 'F' && flag != '1'){break;}
					flag = '0';
					direction = 1;
					move_forward_indoor_dist(distance);
					motor_case =0;
					break;
			/*---------------------------MOTOR BACKWARD-------------------------------------------------*/
				case 'B'://MOVE BACKWARD
					if(motor_case != 'B'&& flag!='1'){break;}
					flag = '0';
					direction = 0;
					move_backward_indoor_dist(distance);
					motor_case =0;
					break;
			/*---------------------------MOTOR 90 LEFT-------------------------------------------------*/
				case 'L':
					if(motor_case != 'L' && flag != '1'){break;}
					flag = '0';
					indoor_three_point_turnL();
					motor_case =0;
					break;
				/*---------------------------MOTOR 90 RIGHT-------------------------------------------------*/
				case 'R':
					if(motor_case != 'R' && flag !='1'){break;}
					flag = '0';
					indoor_three_point_turnR();
					motor_case =0;
					break;
				/*-------------------------CALCULATE TURNING RADIUS(TURN RIGHT/LEFT) -------------------------------------------------------------------------*/

				case 2:
					osDelay(2000);
					if(motor_case != 2){break;}
					indoor_three_point_turnR();
					motor_case =0;
					break;
				case 3:
					osDelay(2000);
					if(motor_case != 3){break;}
					indoor_three_point_turnL();
					motor_case =0;
					break;
			/*------------------------Calculate Distance-------------------------------------------------------------------------------------------*/
				case 4: //INDOOR FORWARD
					osDelay(2000);
					if(motor_case != 4){break;}
					distance = 100;
					direction = 1;
					expected_gyro = gyro[2];
					move_forward_indoor_dist(distance);
					motor_case =0;
					break;

				case 5://INDOOR BACKWARDS
					osDelay(2000);
					if(motor_case != 5){break;}
					distance = 1000;
					direction = 0;
					move_backward_indoor_dist(distance);
					motor_case =0;
					break;

			/*------------------------ASSESSMENT FOR TASK_2 150 CM (OUTDOOR)---------------------------------------------------*/
				case 1:
					osDelay(10);
					break;

				default:
					wheels_straight();
					motor_stop();
					osDelay(10);
					break;
		  	  }

	  }
	  /**********************************************************OUTDOOR******************************************************************************/
	  else if(HAL_GPIO_ReadPin(Enable_Switch_GPIO_Port, Enable_Switch_Pin)==1) //OUTDOOR
	  {
		  switch(motor_case){
			/*---------------------------MOTOR STOP-------------------------------------------------*/
				case 'H'://Motor_stop;
					if(motor_case != 'H'){break;}
					motor_stop();
					wheels_straight();
					break;
			/*---------------------------MOTOR FORWARD-------------------------------------------------*/
				case 'F'://MOVE FORWARD
					if(motor_case != 'F' && flag != '1'){break;}
					flag = '0';
					direction = 1;
					move_forward_outdoor_dist(distance);
					motor_case =0;
					break;
			/*---------------------------MOTOR BACKWARD-------------------------------------------------*/
				case 'B'://MOVE BACKWARD
					if(motor_case != 'B'&& flag!='1'){break;}
					flag = '0';
					direction = 0;
					move_backward_outdoor_dist(distance);
					motor_case =0;
					break;
			/*---------------------------MOTOR 90 LEFT-------------------------------------------------*/
				case 'L':
					if(motor_case != 'L' && flag != '1'){break;}
					flag = '0';
					outdoor_three_point_turnL();
					motor_case =0;
					break;
				/*---------------------------MOTOR 90 RIGHT-------------------------------------------------*/
				case 'R':
					if(motor_case != 'R' && flag !='1'){break;}
					flag = '0';
					outdoor_three_point_turnR();
					motor_case =0;
					break;
				/*-------------------------CALCULATE TURNING RADIUS(TURN RIGHT/LEFT) -------------------------------------------------------------------------*/
				case 2:
					osDelay(2000);
					if(motor_case != 2){break;}
					outdoor_three_point_turnR();
					motor_case =0;
					break;
				case 3:
					osDelay(2000);
					if(motor_case != 3){break;}
					outdoor_three_point_turnL();
					motor_case =0;
					break;
			/*----------------------	CalculateDistance------------------------------------------------------------------------------------------*/
				case 4:
					osDelay(2000);
					if(motor_case != 4){break;}
					distance = 1000;
					osDelay(20);
					direction = 1;
					move_forward_outdoor_dist(distance);
					motor_case =0;
					break;

				case 5:
					osDelay(2000);
					if(motor_case != 5){break;}
					distance = 1000;
					direction = 0;
					move_backward_outdoor_dist(distance);
					motor_case =0;
					break;

			/*------------------------ASSESSMENT FOR TASK_2 150 CM (OUTDOOR)---------------------------------------------------*/
				case 1:
					osDelay(2000);
					if(motor_case !=1){break;}
					direction = 1;
					osDelay(10);
					motor_case = 0;
					break;


				default:
					wheels_straight();
					motor_stop();
					osDelay(10);
					break;
		  }
	  }
	osDelay(10);
	flag = '1';

  }
  /* USER CODE END Motor_Task */
}

/* USER CODE BEGIN Header_ICM_20948_Task */
/**
* @brief Function implementing the ICM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ICM_20948_Task */
void ICM_20948_Task(void *argument)
{
  /* USER CODE BEGIN ICM_20948_Task */
  /* Infinite loop */
	ICMInit();
	osDelay(100);

	uint8_t whoami;
	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
	whoami = ICMReadOneByte(0u);
	while(whoami != 0xea)
	{
		sprintf(icmTempMsg,"whoami: %x",whoami);
		//OLED_ShowString(10, 20, icmTempMsg);
	}

  for(;;)
  {
	  //vTaskDelayUntil(&lastWakeTime, 0.01);
  	if(Deviation_Count<CONTROL_DELAY)
	{
		Deviation_Count++;
		memcpy(Deviation_gyro,gyro,sizeof(gyro));
	}
	  //Get acceleration sensor data
//	  ICMAccelRead(&accel[0], &accel[1], &accel[2]);
	  //Get gyroscope data
  	//ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
	MPU_Get_Gyroscope();


	sprintf(icmTempMsg,"%+05d",gyro[2]);
	OLED_ShowString(10, 30, icmTempMsg);


	  //Get magnetometer data
//	  ICMMagRead(&magnet[0], &magnet[1], &magnet[2]);
//	  sprintf(icmTempMsg,"%d %d %d      ",magnet[0],magnet[1],magnet[2]);
	osDelay(10);
  }
  /* USER CODE END ICM_20948_Task */
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

