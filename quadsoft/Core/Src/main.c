/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

// Bibliotheken
// - HAL
// - MotionFX
// - LL

// Taktgeber
// - PLL

// Timer
// - TIM1
// - TIM3

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_mems.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "com.h"
#include "demo_serial.h"
#include "bsp_ip_conf.h"
#include "fw_version.h"
#include "motion_fx_manager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define X 0
#define Y 1
#define Z 2
#define QW 3
#define QX 0
#define QY 1
#define QZ 2
#define NORDEN 0
#define OSTEN 1
#define UNTEN 2
#define SENSOR_NORDEN 1
#define SENSOR_WESTEN 0
#define SENSOR_OBEN 2
#define M1 0
#define M2 1
#define M3 2
#define M4 3
#define ROLL 0
#define PITCH 1
#define YAW 2
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MG_TO_MPSS  0.001f / 9.81f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define FROM_MGAUSS_TO_UT50  (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS  500.0f
#define USEPWM 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
typedef struct {
	double w;
	double x;
	double y;
	double z;
} Quaternion;

typedef struct {
    double rollen, nicken, gieren;
} Eulerwinkel;

typedef struct {
    double norden, osten, unten;
} Geschwindigkeit;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static void DWT_Start(void);
static uint32_t DWT_Stop(void);
void LED_Init(void);
void LED_Off(void);
void LED_On(void);
void LED_Blinking(uint32_t);
static void MX_USART2_UART_Init(void);
void USART_TransmitString(USART_TypeDef *USARTx, const char* str);
Eulerwinkel Quaternion2Eulerwinkel(Quaternion q);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void READ_ACCELEROMETER(void);
static void READ_GYRO(void);
static void READ_MAG(void);
void regelschritt(void);

// Sensorfusion
static MOTION_SENSOR_Axes_t AccValue;
static MOTION_SENSOR_Axes_t GyrValue;
static MOTION_SENSOR_Axes_t MagValue;
uint8_t joystick_schub = 0;
int joystick_nicken = 0;
int joystick_rollen = 0;
uint8_t rx_buffer[32];
uint8_t rx_index = 0;
float pwmunteregrenze = 800.0f;
float pwmoberegrenze = 880.0f;
uint8_t regleran = 0;
float pegel = 0.3;

MFX_input_t data_in;
MFX_input_t *fusionin = &data_in;
MFX_output_t data_out;
MFX_output_t *fusionout = &data_out;

// Volatile Parameter
double beschleunigung[3] = {0};
double geschwindigkeit[3] = {0};
Quaternion drehlage;

// Umweltparameter
double gravitation_constant = 9.81;

// Quadcopterparameter
double masse = 1.0;
double body_x[3];
double body_y[3];
double body_z[3];
double y_C[3];
double quad_dcm[3][3] = {0};
double thr_int[3] = {0.0, 0.0, 0.0};
double mixerFM[4][4] = {
	{  23000,  64000,  64000, -1530000},
	{  23000, -64000,  64000,  1530000},
	{  23000, -64000, -64000, -1530000},
	{  23000,  64000, -64000,  1530000}
};

// PID-Parameter
double vel_p_gain[3] = {5.0, 5.0, 4.0};
double vel_d_gain[3] = {0.5, 0.5, 0.5};
double rate_p_gain[3] = {1.5, 1.5, 1.0};
double rate_d_gain[3] = {0.04, 0.04, 0.1};
double attitute_p_gain[3] = {8.0, 8.0, 1.5};
// double vel_i_gain[3] = {5.0, 5.0, 5.0};

// Limits
//	double uMax = 5.0;
//	double vMax = 5.0;
//	double wMax = 5.0;
//	double pMax = 3.5;
//	double qMax = 3.5;
//	double rMax = 2.62;
//	double vMaxAll = 5.0;
//	double velMax[3] = {5.0, 5.0, 5.0};
//	double rateMax[3] = {3.5, 3.5, 2.62};
double minWMotor = 0.0;
double maxWMotor = 1000.0;
// double minThr = 300.0;
double maxThr = 500.0;
double tiltMax = 0.8;
Geschwindigkeit geschwindigkeitsfehlerlimit;
Geschwindigkeit sollgeschwindigkeit;
float schub = 0.0f;

// Sollpunkte
double sollschub[3] = {0.0, 0.0, 0.0};
Eulerwinkel drehratensollwert;
double yaw_setpoint = 0.0;
double acc_setpoint[3] = {0.0, 0.0, 0.0};

double delta_t_s;
Eulerwinkel drehlage_tminus1;
Eulerwinkel drehrate;
Eulerwinkel drehgeschwindigkeit_tminus1;
Eulerwinkel omega_dot;

float skalar;
int schubnur = 1;

double e_z[3];
double e_z_d[3];
Quaternion quaternionfehler_reduziert;
double w_cmd[4];
double w_cmd_tminus1[4];
double w_cmd_tminus2[4];
double geschwindigkeitsfehler[3] = {0.0, 0.0, 0.0};
double yaw_w = 0.0;
Eulerwinkel drehratenfehler;
Eulerwinkel drehratenstellwert;

// Normen
double e_z_norm;
double qe_norm;
double sollschub_norm;

// UART
char buffer[50];


double normQuaternion(Quaternion q) {
	return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quaternion inverseQuaternion(Quaternion q) {
	double norm = normQuaternion(q);
	q.w =  q.w / norm;
	q.x = -q.x / norm;
	q.y = -q.y / norm;
	q.z = -q.z / norm;
	return q;
}

Quaternion scaleQuaternion(Quaternion q, double scalar) {
	q.w = scalar * q.w;
	q.x = scalar * q.x;
	q.y = scalar * q.y;
	q.z = scalar * q.z;
	return q;
}

Quaternion kreuzproduktQuaternion(Quaternion q1, Quaternion q2) {
	Quaternion returnQuaternion;
	returnQuaternion.w = (
		q1.w * q2.w -
		q1.x * q2.x -
		q1.y * q2.y -
		q1.z * q2.z
	);
	returnQuaternion.x = (
		q1.x * q2.w +
		q1.w * q2.x -
		q1.z * q2.y +
		q1.y * q2.z
	);
	returnQuaternion.y = (
		q1.y * q2.w +
		q1.z * q2.x +
		q1.w * q2.y -
		q1.x * q2.z
	);
	returnQuaternion.z = (
		q1.z * q2.w -
		q1.y * q2.x +
		q1.x * q2.y +
		q1.w * q2.z
	);
	return returnQuaternion;
}

Quaternion kardanwinkel2quaternion(double gierwinkel, double nickwinkel, double rollwinkel) {
	double cr1 = cos(0.5 * gierwinkel);
	double cr2 = cos(0.5 * nickwinkel);
	double cr3 = cos(0.5 * rollwinkel);
	double sr1 = sin(0.5 * gierwinkel);
	double sr2 = sin(0.5 * nickwinkel);
	double sr3 = sin(0.5 * rollwinkel);

	Quaternion q;
	q.w = cr1 * cr2 * cr3 + sr1 * sr2 * sr3;
	q.x = cr1 * cr2 * sr3 - sr1 * sr2 * cr3;
	q.y = cr1 * sr2 * cr3 + sr1 * cr2 * sr3;
	q.z = sr1 * cr2 * cr3 - cr1 * sr2 * sr3;

	double qn = normQuaternion(q);
	q.w /= qn;
	q.x /= qn;
	q.y /= qn;
	q.z /= qn;
	return q;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* USER CODE BEGIN 1 */
	uint16_t timer_val;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	// MotionFX_manager_init();

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CRC_Init();
	MX_TIM3_Init();
	MX_RTC_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_MEMS_Init();
	/* USER CODE BEGIN 2 */
	MX_USART2_UART_Init();

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	pegel = 0.5;

	//	while (pegel < (1.0f)) {
	//		if (regleran) {
	//			pegel += 0.001;
	//		}
	//		HAL_Delay(100);
	//
	//		TIM1->CCR1 = pegel * 30259;
	//		TIM1->CCR2 = pegel * 30259;
	//		TIM1->CCR3 = pegel * 30259;
	//		TIM1->CCR4 = pegel * 30259;
	//	}

	TIM1->CCR1 = 0.0;
	TIM1->CCR2 = 0.0;
	TIM1->CCR3 = 0.0;
	TIM1->CCR4 = 0.0;
	DWT_Start();

	// HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim2);

	timer_val = __HAL_TIM_GET_COUNTER(&htim2);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (__HAL_TIM_GET_COUNTER(&htim2) - timer_val >= 20)
		{
			timer_val = __HAL_TIM_GET_COUNTER(&htim2);
			regelschritt();
			LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
		}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	*/
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	*/
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 8;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 30259;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 8000-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65536-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// UART

void USART_TransmitString(USART_TypeDef *USARTx, const char* str) {
    while (*str != '\0') {
        LL_USART_TransmitData8(USARTx, *str);
        while (!LL_USART_IsActiveFlag_TXE(USARTx));
        str++;
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */
	/* (1) Enable GPIO clock and configures the USART pins *********************/

	/* Enable the peripheral clock of GPIO Port */
	USARTx_GPIO_CLK_ENABLE();

	/* Configure Tx Pin as : Alternate function, High Speed, Push pull, Pull up */
	LL_GPIO_SetPinMode(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_MODE_ALTERNATE);
	USARTx_SET_TX_GPIO_AF();
	LL_GPIO_SetPinSpeed(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_PULL_UP);

	/* Configure Rx Pin as : Alternate function, High Speed, Push pull, Pull up */
	LL_GPIO_SetPinMode(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_MODE_ALTERNATE);
	USARTx_SET_RX_GPIO_AF();
	LL_GPIO_SetPinSpeed(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_PULL_UP);

	/* (2) NVIC Configuration for USART interrupts */
	/*  - Set priority for USARTx_IRQn */
	/*  - Enable USARTx_IRQn */
	NVIC_SetPriority(USARTx_IRQn, 0);
	NVIC_EnableIRQ(USARTx_IRQn);

	/* (3) Enable USART peripheral clock and clock source ***********************/
	USARTx_CLK_ENABLE();

	/* (4) Configure USART functional parameters ********************************/

	/* Disable USART prior modifying configuration registers */
	/* Note: Commented as corresponding to Reset value */
	// LL_USART_Disable(USARTx_INSTANCE);

	/* TX/RX direction */
	LL_USART_SetTransferDirection(USARTx_INSTANCE, LL_USART_DIRECTION_TX_RX);

	/* 8 data bit, 1 start bit, 1 stop bit, no parity */
	LL_USART_ConfigCharacter(USARTx_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

	/* No Hardware Flow control */
	/* Reset value is LL_USART_HWCONTROL_NONE */
	// LL_USART_SetHWFlowCtrl(USARTx_INSTANCE, LL_USART_HWCONTROL_NONE);

	/* Oversampling by 16 */
	/* Reset value is LL_USART_OVERSAMPLING_16 */
	//LL_USART_SetOverSampling(USARTx_INSTANCE, LL_USART_OVERSAMPLING_16);

	/* Set Baudrate to 115200 using APB frequency set to 100000000/APB_Div Hz */
	/* Frequency available for USART peripheral can also be calculated through LL RCC macro */
	/* Ex :
	  Periphclk = LL_RCC_GetUSARTClockFreq(Instance); or LL_RCC_GetUARTClockFreq(Instance); depending on USART/UART instance

	  In this example, Peripheral Clock is expected to be equal to 100000000/APB_Div Hz => equal to SystemCoreClock/APB_Div
	*/
	LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock/APB_Div, LL_USART_OVERSAMPLING_16, 115200);

	/* (5) Enable USART *********************************************************/
	LL_USART_Enable(USARTx_INSTANCE);

	/* Enable RXNE and Error interrupts */
	LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
	LL_USART_EnableIT_ERROR(USARTx_INSTANCE);

	/* USER CODE END USART2_Init 1 */
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
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}


void USART_CharReception_Callback(void)
{
__IO uint32_t received_char;

  /* Auslesen des Zeichens. RXNE flag wird gecleared durch das lesen des DR Registers */
  received_char = LL_USART_ReceiveData8(USARTx_INSTANCE);

  if (received_char == '\r') {
	  rx_index = 0;

	  joystick_schub = (100 * ((rx_buffer[0] - '0') - 1) + 10 * (rx_buffer[1] - '0') + (rx_buffer[2] - '0'));
	  joystick_nicken = ((100 * (rx_buffer[3] - '0') + 10 * (rx_buffer[4] - '0') + (rx_buffer[5] - '0')) - 500);
	  joystick_rollen = ((100 * (rx_buffer[6] - '0') + 10 * (rx_buffer[7] - '0') + (rx_buffer[8] - '0')) - 500);
	  regleran = rx_buffer[9] - '0';
	  schubnur = rx_buffer[10] - '0';
  } else {
	  rx_buffer[rx_index] = received_char;

	  rx_index += 1;
  }

  LL_USART_TransmitData8(USARTx_INSTANCE, received_char);
}

// Regler
void regelschritt(void) {
	READ_ACCELEROMETER();
	READ_GYRO();
	READ_MAG();

	/* Convert angular velocity from [md/s] to [deg/s] */
	// Gieren (Uhrzeigersinn)
	data_in.gyro[0] = (float) GyrValue.x * FROM_MDPS_TO_DPS;
	// Nicken (Nase nach unten ist negatives nicken)
	data_in.gyro[1] = (float) GyrValue.y * FROM_MDPS_TO_DPS;
	// Rollen (Mit Blick von hinten im Uhrzeigersinn)
	data_in.gyro[2] = (float) GyrValue.z * FROM_MDPS_TO_DPS;

	// Norden in g
	data_in.acc[0] = (float) AccValue.x * FROM_MG_TO_G;
	// Osten in g
	data_in.acc[1] = (float) AccValue.y * FROM_MG_TO_G;
	// Oben in g
	data_in.acc[2] = (float) AccValue.z * FROM_MG_TO_G;

	data_in.mag[0] = (float) MagValue.x * FROM_MGAUSS_TO_UT50;
	data_in.mag[1] = (float) MagValue.y * FROM_MGAUSS_TO_UT50;
	data_in.mag[2] = (float) MagValue.z * FROM_MGAUSS_TO_UT50;

	delta_t_s = DWT_Stop() * 1e-6f;
	DWT_Start();

	MotionFX_manager_run(fusionin, fusionout, 0.01f);

	sollgeschwindigkeit.norden = joystick_nicken / 50.0f;
	sollgeschwindigkeit.osten = joystick_rollen / 50.0f;
	sollgeschwindigkeit.unten = -joystick_schub / 50.0f;

	// [m/s]
	geschwindigkeit[NORDEN] += delta_t_s * fusionout->linear_acceleration[SENSOR_NORDEN] / 9.81f;
	// [m/s]
	geschwindigkeit[OSTEN] += delta_t_s * fusionout->linear_acceleration[SENSOR_WESTEN] / 9.81f;
	// [m/s]
	geschwindigkeit[UNTEN] += delta_t_s * fusionout->linear_acceleration[SENSOR_OBEN] / 9.81f;

	drehlage.w = fusionout->quaternion[QW];
	drehlage.x = fusionout->quaternion[QX];
	drehlage.y = fusionout->quaternion[QY];
	drehlage.z = fusionout->quaternion[QZ];

//	drehlage = kardanwinkel2quaternion(
//			0.0f * fusionout->rotation[0] * 3.14f / 180.0f,
//			-fusionout->rotation[1] * 3.14f / 180.0f,
//			fusionout->rotation[2] * 3.14f / 180.0f
//	);

	if (drehlage.x != drehlage.x) {
		LED_Off();
	} else {
		LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
	}

	// Controller
	// Rollwinkel [deg/s]
	drehrate.rollen = (fusionout->rotation[2] - drehlage_tminus1.rollen) / delta_t_s;
	// Nickwinkel [deg/s]
	drehrate.nicken = (fusionout->rotation[1] - drehlage_tminus1.nicken) / delta_t_s;
	// Gierwinkel [deg/s]
	drehrate.gieren = 0.0f * data_in.gyro[0];

	drehlage_tminus1.rollen = fusionout->rotation[2];
	drehlage_tminus1.nicken = fusionout->rotation[1];

	omega_dot.rollen = 0.0f; // (drehrate.rollen - drehgeschwindigkeit_tminus1.rollen) / delta_t_s;
	omega_dot.nicken = 0.0f; // (drehrate.nicken - drehgeschwindigkeit_tminus1.nicken) / delta_t_s;
	omega_dot.gieren = 0.0f;

	drehgeschwindigkeit_tminus1.rollen = drehrate.rollen;
	drehgeschwindigkeit_tminus1.nicken = drehrate.nicken;


	// Geschwindigkeitsregler

	// m/s
	geschwindigkeitsfehler[UNTEN] = 0.0f; // sollgeschwindigkeit.unten - geschwindigkeit[UNTEN];

	sollschub[UNTEN] = vel_p_gain[UNTEN] * geschwindigkeitsfehler[UNTEN] - vel_d_gain[UNTEN] * beschleunigung[UNTEN] + masse * (acc_setpoint[UNTEN] - gravitation_constant) + thr_int[UNTEN];

	double uMax = -0.4;
	double uMin = -16 * gravitation_constant;

	// sollschub[2] = thrust_z;
	if (sollschub[UNTEN] < uMin) {
		sollschub[UNTEN] = uMin;
	}
	if (sollschub[UNTEN] > uMax) {
		sollschub[UNTEN] = uMax;
	}

	// XY Velocity Control (Thrust in NE-direction)
	geschwindigkeitsfehler[NORDEN] = sollgeschwindigkeit.norden - geschwindigkeit[NORDEN];
	geschwindigkeitsfehler[OSTEN] = sollgeschwindigkeit.osten - geschwindigkeit[OSTEN];
	sollschub[X] = (
		vel_p_gain[NORDEN] * geschwindigkeitsfehler[NORDEN] -
		vel_d_gain[NORDEN] *  beschleunigung[NORDEN] +
		masse * acc_setpoint[NORDEN] +
		thr_int[NORDEN]
	);
	sollschub[Y] = (
		vel_p_gain[OSTEN] * geschwindigkeitsfehler[OSTEN] -
		vel_d_gain[OSTEN] *  beschleunigung[OSTEN] +
		masse * acc_setpoint[OSTEN] +
		thr_int[OSTEN]
	);

	double thrust_max_xy_tilt = (
		abs(sollschub[UNTEN]) * tan(tiltMax)
	);
	double thrust_max_xy = sqrt(
		pow(maxThr, 2) -
		pow(sollschub[UNTEN], 2)
	);

	if (thrust_max_xy > thrust_max_xy_tilt) {
		thrust_max_xy = thrust_max_xy_tilt;
	}

	if (
		sollschub[NORDEN] * sollschub[NORDEN] + sollschub[OSTEN] * sollschub[OSTEN] >
		pow(thrust_max_xy, 2)) {

		double mag = sqrt(
			pow(sollschub[NORDEN], 2) +
			pow(sollschub[OSTEN], 2)
		);

		sollschub[NORDEN] = (
			sollschub[NORDEN] * thrust_max_xy / mag
		);
		sollschub[OSTEN] = (
			sollschub[OSTEN] * thrust_max_xy / mag
		);
	}

	// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990

	geschwindigkeitsfehlerlimit.norden = geschwindigkeitsfehler[NORDEN] - (
		sollschub[NORDEN] - sollschub[NORDEN]
		) * 2.0 / vel_p_gain[NORDEN];
	geschwindigkeitsfehlerlimit.osten = geschwindigkeitsfehler[OSTEN] - (
		sollschub[OSTEN] - sollschub[OSTEN]
		) * 2.0 / vel_p_gain[OSTEN];

	double normsollschub = sqrt(
		sollschub[NORDEN] * sollschub[NORDEN] +
		sollschub[OSTEN] * sollschub[OSTEN] +
		sollschub[UNTEN] * sollschub[UNTEN]
	);

	body_z[0] = -sollschub[0] / normsollschub;
	body_z[1] = -sollschub[1] / normsollschub;
	body_z[2] = -sollschub[2] / normsollschub;

	y_C[0] = -sin(yaw_setpoint);
	y_C[1] = cos(yaw_setpoint);
	y_C[2] = 0.0;

	// Kreuzprodukt
	body_x[0] = y_C[1] * body_z[2] - y_C[2] * body_z[1];
	body_x[1] = y_C[2] * body_z[0] - y_C[0] * body_z[2];
	body_x[2] = y_C[0] * body_z[1] - y_C[1] * body_z[0];

	double normBodyX = sqrt(
		body_x[0] * body_x[0] +
		body_x[1] * body_x[1] +
		body_x[2] * body_x[2]
	);

	body_x[0] = body_x[0] / normBodyX;
	body_x[1] = body_x[1] / normBodyX;
	body_x[2] = body_x[2] / normBodyX;

	body_y[0] = (
		body_z[1] * body_x[2] -
		body_z[2] * body_x[1]
	);
	body_y[1] = (
		body_z[2] * body_x[0] -
		body_z[0] * body_x[2]
	);
	body_y[2] = (
		body_z[0] * body_x[1] -
		body_z[1] * body_x[0]
	);

	double R_sp[3][3];

	// Desired rotation matrix
	R_sp[0][0] = body_x[0];
	R_sp[1][0] = body_x[1];
	R_sp[2][0] = body_x[2];

	R_sp[0][1] = body_y[0];
	R_sp[1][1] = body_y[1];
	R_sp[2][1] = body_y[2];

	R_sp[0][2] = body_z[0];
	R_sp[1][2] = body_z[1];
	R_sp[2][2] = body_z[2];

	// Full desired quaternion (
	//    full because it considers the desired Yaw angle
	// )
	// RotToQuat(R_sp, q_target);

	// double quaternion_ganz[4];

	Quaternion quaternion_ganz;
	double tr = R_sp[0][0] + R_sp[1][1] + R_sp[2][2];
	double r, e0, e1, e2, e3;

	if (tr > R_sp[0][0] && tr > R_sp[1][1] && tr > R_sp[2][2]) {
		e0 = 0.5 * sqrt(1 + tr);
		r = 0.25 / e0;
		e1 = (R_sp[2][1] - R_sp[1][2]) * r;
		e2 = (R_sp[0][2] - R_sp[2][0]) * r;
		e3 = (R_sp[1][0] - R_sp[0][1]) * r;
	} else if (R_sp[0][0] > R_sp[1][1] && R_sp[0][0] > R_sp[2][2]) {
		e1 = 0.5 * sqrt(1 - tr + 2 * R_sp[0][0]);
		r = 0.25 / e1;
		e0 = (R_sp[2][1] - R_sp[1][2]) * r;
		e2 = (R_sp[0][1] + R_sp[1][0]) * r;
		e3 = (R_sp[0][2] + R_sp[2][0]) * r;
	} else if (R_sp[1][1] > R_sp[2][2]) {
		e2 = 0.5 * sqrt(1 - tr + 2 * R_sp[1][1]);
		r = 0.25 / e2;
		e0 = (R_sp[0][2] - R_sp[2][0]) * r;
		e1 = (R_sp[0][1] + R_sp[1][0]) * r;
		e3 = (R_sp[1][2] + R_sp[2][1]) * r;
	} else {
		e3 = 0.5 * sqrt(1 - tr + 2 * R_sp[2][2]);
		r = 0.25 / e3;
		e0 = (R_sp[1][0] - R_sp[0][1]) * r;
		e1 = (R_sp[0][2] + R_sp[2][0]) * r;
		e2 = (R_sp[1][2] + R_sp[2][1]) * r;
	}

	quaternion_ganz.w = e0;
	quaternion_ganz.x = e1;
	quaternion_ganz.y = e2;
	quaternion_ganz.z = e3;

	if (e0 < 0) {
		quaternion_ganz.w = -quaternion_ganz.w;
		quaternion_ganz.x = -quaternion_ganz.x;
		quaternion_ganz.y = -quaternion_ganz.y;
		quaternion_ganz.z = -quaternion_ganz.z;
	}

	double norm = normQuaternion(quaternion_ganz);

	quaternion_ganz = scaleQuaternion(quaternion_ganz, 1 / norm);

	// Maybe not correct
	// quaternion_ganz[0] = q_target[0];
	// quaternion_ganz[1] = q_target[1];
	// quaternion_ganz[2] = q_target[2];
	// quaternion_ganz[3] = q_target[3];

	// Current thrust orientation e_z and desired thrust orientation e_z_d

	quad_dcm[0][0] = 1.0;
	quad_dcm[1][1] = 1.0;
	quad_dcm[2][2] = 1.0;

	e_z[0] = 0.0;
	e_z[1] = 0.0;
	e_z[2] = 1.0;

	double normSollschub = sqrt(
		sollschub[0] * sollschub[0] +
		sollschub[1] * sollschub[1] +
		sollschub[2] * sollschub[2]
	);

	e_z_d[0] = -sollschub[0] / normSollschub;
	e_z_d[1] = -sollschub[1] / normSollschub;
	e_z_d[2] = -sollschub[2] / normSollschub;

	double quaternionfehler_reduziert_dot_part = (
		e_z[0] * e_z_d[0] +
		e_z[1] * e_z_d[1] +
		e_z[2] * e_z_d[2]
	);

	e_z_norm = sqrt(
		e_z[0] * e_z[0] +
		e_z[1] * e_z[1] +
		e_z[2] * e_z[2]
	);

	e_z[0] = -e_z[0] / e_z_norm;
	e_z[1] = -e_z[1] / e_z_norm;
	e_z[2] = -e_z[2] / e_z_norm;

	double e_z_d_norm = sqrt(
		e_z_d[0] * e_z_d[0] +
		e_z_d[1] * e_z_d[1] +
		e_z_d[2] * e_z_d[2]
	);

	e_z_d[0] = -e_z_d[0] / e_z_d_norm;
	e_z_d[1] = -e_z_d[1] / e_z_d_norm;
	e_z_d[2] = -e_z_d[2] / e_z_d_norm;

	quaternionfehler_reduziert.w = quaternionfehler_reduziert_dot_part + sqrt(
		e_z[0] * e_z[0] +
		e_z[1] * e_z[1] +
		e_z[2] * e_z[2] +
		e_z_d[0] * e_z_d[0] +
		e_z_d[1] * e_z_d[1] +
		e_z_d[2] * e_z_d[2]
	);

	quaternionfehler_reduziert.x = (
		e_z[1] * e_z_d[2] -
		e_z[2] * e_z_d[1]
	);
	quaternionfehler_reduziert.y = (
		e_z[2] * e_z_d[0] -
		e_z[0] * e_z_d[2]
	);
	quaternionfehler_reduziert.z = (
		e_z[0] * e_z_d[1] -
		e_z[1] * e_z_d[0]
	);

	qe_norm = normQuaternion(quaternionfehler_reduziert);

	quaternionfehler_reduziert.w /= qe_norm;
	quaternionfehler_reduziert.x /= qe_norm;
	quaternionfehler_reduziert.y /= qe_norm;
	quaternionfehler_reduziert.z /= qe_norm;

	// Reduced desired quaternion (reduced because it doesn't consider the desired Yaw angle)
	// Mixed desired quaternion (between reduced and full) and resulting desired quaternion qd

	Quaternion qd_reduziert;
	qd_reduziert.w = (
		quaternionfehler_reduziert.w * drehlage.w -
		quaternionfehler_reduziert.x * drehlage.x -
		quaternionfehler_reduziert.y * drehlage.y -
		quaternionfehler_reduziert.z * drehlage.z
	);
	qd_reduziert.x = (
		quaternionfehler_reduziert.x * drehlage.w +
		quaternionfehler_reduziert.w * drehlage.x -
		quaternionfehler_reduziert.z * drehlage.y +
		quaternionfehler_reduziert.y * drehlage.z
	);
	qd_reduziert.y = (
		quaternionfehler_reduziert.y * drehlage.w +
		quaternionfehler_reduziert.z * drehlage.x +
		quaternionfehler_reduziert.w * drehlage.y -
		quaternionfehler_reduziert.x * drehlage.z
	);
	qd_reduziert.z = (
		quaternionfehler_reduziert.z * drehlage.w -
		quaternionfehler_reduziert.y * drehlage.x +
		quaternionfehler_reduziert.x * drehlage.y +
		quaternionfehler_reduziert.w * drehlage.z
	);

	Quaternion qd_reduziert_inverse = inverseQuaternion(qd_reduziert);

	Quaternion q_mix = kreuzproduktQuaternion(qd_reduziert_inverse, quaternion_ganz);


	if (q_mix.w < -1.0) {
		q_mix.w = -1.0;
	}
	if (q_mix.w > 1.0) {
		q_mix.w = 1.0;
	}
	if (q_mix.z < -1.0) {
		q_mix.z = -1.0;
	}
	if (q_mix.z > 1.0) {
		q_mix.z = 1.0;
	}

	Quaternion qd;
	qd.w = (
		qd_reduziert.w * cos(yaw_w * acos(q_mix.w)) -
		qd_reduziert.z * sin(yaw_w * asin(q_mix.z))
	);
	qd.x = (
		qd_reduziert.x * cos(yaw_w * acos(q_mix.w)) +
		qd_reduziert.y * sin(yaw_w * asin(q_mix.z))
	);
	qd.y = (
		qd_reduziert.y * cos(yaw_w * acos(q_mix.w)) -
		qd_reduziert.x * sin(yaw_w * asin(q_mix.z))
	);
	qd.z = (
		qd_reduziert.z * cos(yaw_w * acos(q_mix.w)) +
		qd_reduziert.w * sin(yaw_w * asin(q_mix.z))
	);

	Quaternion drehlage_inverse = inverseQuaternion(drehlage);

	Quaternion drehlagefehler;
	drehlagefehler = kreuzproduktQuaternion(drehlage_inverse, qd);

	if (drehlagefehler.w > 0) {
		drehratensollwert.rollen = (2.0 * drehlagefehler.x * attitute_p_gain[0]);
		drehratensollwert.nicken = (2.0 * drehlagefehler.y * attitute_p_gain[1]);
		drehratensollwert.gieren = (2.0 * drehlagefehler.z * attitute_p_gain[2]);
	} else {
		drehratensollwert.rollen = -(2.0 * drehlagefehler.x * attitute_p_gain[0]);
		drehratensollwert.nicken = -(2.0 * drehlagefehler.y * attitute_p_gain[1]);
		drehratensollwert.gieren = -(2.0 * drehlagefehler.z * attitute_p_gain[2]);
	}

	// Rate Control
	drehratenfehler.rollen = drehratensollwert.rollen - drehrate.rollen;
	drehratenstellwert.rollen = (
		rate_p_gain[0] * drehratenfehler.rollen -
		rate_d_gain[0] * omega_dot.rollen
	);
	drehratenfehler.nicken = drehratensollwert.nicken - drehrate.nicken;
	drehratenstellwert.nicken = (
		rate_p_gain[1] * drehratenfehler.nicken -
		rate_d_gain[1] * omega_dot.nicken
	);
	drehratenfehler.gieren = drehratensollwert.gieren - drehrate.gieren;
	drehratenstellwert.gieren = (
		rate_p_gain[2] * drehratenfehler.gieren -
		rate_d_gain[2] * omega_dot.gieren
	);

	sollschub_norm = sqrt(
		pow(sollschub[0], 2) +
		pow(sollschub[1], 2) +
		pow(sollschub[2], 2)
	);

	w_cmd[M1] = (
		mixerFM[0][0] * sollschub_norm +
		mixerFM[0][1] * drehratenstellwert.rollen +
		mixerFM[0][2] * drehratenstellwert.nicken +
		mixerFM[0][3] * drehratenstellwert.gieren
	);
	w_cmd[M2] = (
		mixerFM[1][0] * sollschub_norm +
		mixerFM[1][1] * drehratenstellwert.rollen +
		mixerFM[1][2] * drehratenstellwert.nicken +
		mixerFM[1][3] * drehratenstellwert.gieren
	);
	w_cmd[M3] = (
		mixerFM[2][0] * sollschub_norm +
		mixerFM[2][1] * drehratenstellwert.rollen +
		mixerFM[2][2] * drehratenstellwert.nicken +
		mixerFM[2][3] * drehratenstellwert.gieren
	);
	w_cmd[M4] = (
		mixerFM[3][0] * sollschub_norm +
		mixerFM[3][1] * drehratenstellwert.rollen +
		mixerFM[3][2] * drehratenstellwert.nicken +
		mixerFM[3][3] * drehratenstellwert.gieren
	);

	for (int i = 0; i < 4; i++) {
		if (w_cmd[i] < pow(minWMotor, 2)) {
			w_cmd[i] = pow(minWMotor, 2);
		};
		if (w_cmd[i] > pow(maxWMotor, 2)) {
			w_cmd[i] = pow(maxWMotor, 2);
		};

		w_cmd[i] = sqrt(w_cmd[i]);
	};

	// PWM
	schub = joystick_schub / 100.0f;
	if (regleran) {
		skalar = (pwmoberegrenze - pwmunteregrenze) / 1000.0f;
		// Pin PA8
		TIM1->CCR1 = ((skalar * schub * ((w_cmd[M1] + w_cmd_tminus1[M1] + w_cmd_tminus2[M1]) / 3) + pwmunteregrenze) / 1000.0f) * 30259;
		// Pin PA9
		TIM1->CCR2 = ((skalar * schub * ((w_cmd[M2] + w_cmd_tminus1[M2] + w_cmd_tminus2[M2]) / 3) + pwmunteregrenze) / 1000.0f) * 30259;
		// Pin PA10
		TIM1->CCR3 = ((skalar * schub * ((w_cmd[M3] + w_cmd_tminus1[M3] + w_cmd_tminus2[M3]) / 3) + pwmunteregrenze) / 1000.0f) * 30259;
		// Pin PA11
		TIM1->CCR4 = ((skalar * schub * ((w_cmd[M4] + w_cmd_tminus1[M4] + w_cmd_tminus2[M4]) / 3) + pwmunteregrenze) / 1000.0f) * 30259;
	} else if (schubnur) {
		TIM1->CCR1 = schub * 30259;
		TIM1->CCR2 = schub * 30259;
		TIM1->CCR3 = schub * 30259;
		TIM1->CCR4 = schub * 30259;
	} else {
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
		TIM1->CCR4 = 0;
	}

	w_cmd_tminus1[0] = w_cmd[0];
	w_cmd_tminus1[1] = w_cmd[1];
	w_cmd_tminus1[2] = w_cmd[2];
	w_cmd_tminus1[3] = w_cmd[3];

	w_cmd_tminus2[0] = w_cmd_tminus1[0];
	w_cmd_tminus2[1] = w_cmd_tminus1[1];
	w_cmd_tminus2[2] = w_cmd_tminus1[2];
	w_cmd_tminus2[3] = w_cmd_tminus1[3];
}

static void READ_ACCELEROMETER(void)
{
    if (UseOfflineData == 1)
    {
		AccValue.x = OfflineData[OfflineDataReadIndex].acceleration_x_mg;
		AccValue.y = OfflineData[OfflineDataReadIndex].acceleration_y_mg;
		AccValue.z = OfflineData[OfflineDataReadIndex].acceleration_z_mg;
    }
    else
    {
		BSP_SENSOR_ACC_GetAxes(&AccValue);
    }
}

static void READ_GYRO(void)
{
    if (UseOfflineData == 1)
    {
		GyrValue.x = OfflineData[OfflineDataReadIndex].angular_rate_x_mdps;QM
		GyrValue.y = OfflineData[OfflineDataReadIndex].angular_rate_y_mdps;
		GyrValue.z = OfflineData[OfflineDataReadIndex].angular_rate_z_mdps;
    }
    else
    {
    	BSP_SENSOR_GYR_GetAxes(&GyrValue);
    }
}

static void READ_MAG(void)
{

    if (UseOfflineData == 1)
    {
     MagValue.x = OfflineData[OfflineDataReadIndex].magnetic_field_x_mgauss;
     MagValue.y = OfflineData[OfflineDataReadIndex].magnetic_field_y_mgauss;
     MagValue.z = OfflineData[OfflineDataReadIndex].magnetic_field_z_mgauss;
    }
    else
    {
      BSP_SENSOR_MAG_GetAxes(&MagValue);
    }
}

// Quaternion

Eulerwinkel Quaternion2Eulerwinkel(Quaternion q) {
    Eulerwinkel winkel;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    winkel.rollen = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    winkel.nicken = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    winkel.gieren = atan2(siny_cosp, cosy_cosp);

    return winkel;
}

// LED

void LED_On(void)
{
	LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

void LED_Off(void)
{
	LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

void LED_Blinking(uint32_t Period)
{
	while (1)
	{
		LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
		// LL_mDelay(Period);
	}
}

static void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

/**
  * @brief  Start counting clock cycles
  * @param  None
  * @retval None
  */
static void DWT_Start(void)
{
	DWT->CYCCNT = 0; /* Clear count of clock cycles */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* Enable counter */
}

/**
  * @brief  Stop counting clock cycles and calculate elapsed time in [us]
  * @param  None
  * @retval Elapsed time in [us]
  */
static uint32_t DWT_Stop(void)
{
	volatile uint32_t cycles_count = 0U;
	uint32_t system_core_clock_mhz = 0U;

	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
	cycles_count = DWT->CYCCNT;

	/* Calculate elapsed time in [us] */
	system_core_clock_mhz = SystemCoreClock / 1000000U;
	return cycles_count / system_core_clock_mhz;
}

/* USER CODE END 4 */

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
