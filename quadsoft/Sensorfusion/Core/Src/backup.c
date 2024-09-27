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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_mems.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "com.h"
#include "demo_serial.h"
#include "bsp_ip_conf.h"
#include "fw_version.h"
#include "motion_fx_manager.h"

#include <stdio.h>

#define TIM_DUTY_CYCLES_NB 11

/* Private function prototypes -----------------------------------------------*/
__STATIC_INLINE void     SystemClock_Config(void);

/* USER CODE BEGIN PV */
float roll, pitch, yaw;  // Attitude (Euler angles)
float qw, qx, qy, qz;    // Quaternion
float accX, accY, accZ;  // Linear acceleration

#define DWT_LAR_KEY  0xC5ACCE55 /* DWT register unlock key */
#define ALGO_FREQ  100U /* Algorithm frequency 100Hz */
#define ACC_ODR  ((float)ALGO_FREQ)
#define ACC_FS  4 /* FS = <-4g, 4g> */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define FROM_MGAUSS_TO_UT50  (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS  500.0f

volatile uint8_t DataLoggerActive = 0;
volatile uint32_t SensorsEnabled = 0;
char LibVersion[35];
int LibVersionLen;
volatile uint8_t SensorReadRequest = 0;
uint8_t UseOfflineData = 0;
offline_data_t OfflineData[OFFLINE_DATA_SIZE];
int OfflineDataReadIndex = 0;
int OfflineDataWriteIndex = 0;
int OfflineDataCount = 0;
uint32_t AlgoFreq = ALGO_FREQ;
uint8_t Enabled6X = 0;
static int32_t PushButtonState = GPIO_PIN_RESET;

static MOTION_SENSOR_Axes_t AccValue;
static MOTION_SENSOR_Axes_t GyrValue;
static MOTION_SENSOR_Axes_t MagValue;
static float TempValue;
static float HumValue;
static volatile uint32_t TimeStamp = 0;
static volatile uint8_t MagCalRequest = 0;
static MOTION_SENSOR_Axes_t MagOffset;
static uint8_t MagCalStatus = 0;

static void Init_Sensors(void);
static void RTC_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);
static void Temperature_Sensor_Handler(TMsg *Msg);
static void Humidity_Sensor_Handler(TMsg *Msg);
static void TIM_Config(uint32_t Freq);
static void DWT_Init(void);
static void DWT_Start(void);
static uint32_t DWT_Stop(void);
void UART_SendString(const char* str);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

CRC_HandleTypeDef hcrc;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void setPwmDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float dutyCycle);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
void MX_TIM3_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
const char * tx_buffer = "Test2313\n\r";
uint8_t rx_index;
uint8_t rx_data[8];
uint8_t rx_buffer[128];
uint8_t transfer_done;

MFX_output_t data_out;
MFX_output_t *pdata_out = &data_out;

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PWM_CHANNEL 1
#define PWM_CTLR_NODE DT_NODELABEL(pwm2)

#define X 0
#define Y 1
#define Z 2
#define NORTH 0
#define EAST 1
#define DOWN 2
#define ROLL 0
#define PITCH 1
#define YAW 2
#define PI 3.14159265358979323846
#define RAD2DEG 180.0 / math.PI
#define DEG2RAD math.pi / 180.0
#define RAD_TO_DEG (180.0 / PI)
#define DEG_TO_RAD (PI / 180.0)
#define ACCELEROMETER_CUTOFF 100
#define COMPLEMENTARY_FILTER_ALPHA 0.98
// State vector: [position, velocity, orientation]
#define STATE_DIM 9
// Measurement vector: [accelerometer, gyroscope, magnetometer]
#define MEASURE_DIM 9

// Global variables
double acc_a_x = 0;
double acc_a_y = 0;
double acc_a_z = 0;
double acc_v_x = 0;
double acc_v_y = 0;
double acc_v_z = 0;
double acc_s_x = 0;
double acc_s_y = 0;
double acc_s_z = 0;

double gyro_a_x = 0;
double gyro_a_y = 0;
double gyro_a_z = 0;
double gyro_v_x = 0;
double gyro_v_y = 0;
double gyro_v_z = 0;
double gyro_s_x = 0;
double gyro_s_y = 0;
double gyro_s_z = 0;
double mag_a[3];
double delta_t = 0.001;
double t = 0;
double pos_p_gain[3] = {1.0, 1.0, 1.0};
double vel_p_gain[3] = {5.0, 5.0, 4.0};
double vel_d_gain[3] = {0.5, 0.5, 0.5};
double vel_i_gain[3] = {5.0, 5.0, 5.0};
double attitute_p_gain[3] = {8.0, 8.0, 1.5};
double Pp = 1.5;
double Dp = 0.4;
double Pq = 1.5;
double Dq = 0.4;
double Pr = 1.0;
double Dr = 0.1;
double rate_p_gain[3] = {1.5, 0.4, 1.5};
double rate_d_gain[3] = {0.4, 1.0, 0.1};
double uMax = 5.0;
double vMax = 5.0;
double wMax = 5.0;
double pMax = 3.5;
double qMax = 3.5;
double rMax = 2.62;
double velMax[3] = {5.0, 5.0, 5.0};
double vMaxAll = 5.0;
double rateMax[3] = {3.5, 3.5, 2.62};
double thr_int[3];
double tiltMax = 0.8;
int saturateVel_separetely = 0;
double gravitation_constant = 9.81;
double quadcopter_mass = 1.0;
double rate_control[3] = {0.0, 0.0, 0.0};
double minWMotor = 0.0;
double maxWMotor = 1000.0;
double minThr = 0.0;
double maxThr = 1000.0;
double q_target[4] = {1.0, 0.0, 0.0, 0.0};
double acc_setpoint[3];
double yawW = 0.0;
double rate_error[3];
double roll_pitch_gain = 0.0;
double pos_sp[3] = {0.0, 0.0, 0.0};
double vel_sp[3] = {1.0, 0.0, 0.0};
double acc_sp[3] = {0.0, 0.0, 0.0};
double thrust_sp[3] = {0.0, 0.0, 0.0};
double eul_sp[3] = {0.0, 0.0, 0.0};
double pqr_sp[3] = {0.0, 0.0, 0.0};
double yawFF[3] = {0.0, 0.0, 0.0};
double gain = 0;
double yaw_w;
double v_error[3];
double v_error_lim[3];
double body_x[3];
double body_y[3];
double body_z[3];
double rate_setpoint[4];
double yaw_setpoint;
double y_C[3];
double quad_dcm[3][3];
double quad_quat_state[4] = {1.0, 0.0, 0.0, 0.0};
double e_z[3];
double e_z_d[3];
double qe_red[4];
double qd_red[4];
double qd[4];
double result[4];
double qd_red_inverse[4];
double q_mix[4];
double quadternion_error[4];
double quad_quat_state_inverse[4];
double thrust_sp_norm;
double w_cmd_clipped[4];
double w_cmd[4];

// Normierte Werte
double qd_norm;
double e_z_norm;
double nor;
double norm;
double qe_norm;

// Attitude structure
typedef struct {
    double roll;
    double pitch;
    double yaw;
} Attitude;

typedef struct {
	double w;
	double x;
	double y;
	double z;
} Quaternion;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == BSP_IP_TIM_Handle.Instance)
  {
    SensorReadRequest = 1;
  }
}

static void MX_Sensorfusion_init(void)
{
	float ans_float;

	/* Initialize button */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	/* Check what is the Push Button State when the button is not pressed. It can change across families */
	PushButtonState = (BSP_PB_GetState(BUTTON_KEY)) ?  0 : 1;

	/* Initialize LED */
	BSP_LED_Init(LED2);

	/* Initialize Virtual COM Port */
	BSP_COM_Init(COM1);

	/* Initialize Timer */
	BSP_IP_TIM_Init();

	/* Configure Timer to run with desired algorithm frequency */
	TIM_Config(ALGO_FREQ);

	/* Initialize (disabled) sensors */
	Init_Sensors();

	/* Sensor Fusion API initialization function */
	MotionFX_manager_init();

	/* OPTIONAL */
	/* Get library version */
	MotionFX_manager_get_version(LibVersion, &LibVersionLen);

	/* Enable magnetometer calibration */
	MotionFX_manager_MagCal_start(ALGO_PERIOD);

	/* Test if calibration data are available */
	MFX_MagCal_output_t mag_cal_test;
	MotionFX_MagCal_getParams(&mag_cal_test);

	/* If calibration data are available load HI coefficients */
	if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
	{
		ans_float = (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
		MagOffset.x = (int32_t)ans_float;
		ans_float = (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
		MagOffset.y = (int32_t)ans_float;
		ans_float = (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);
		MagOffset.z = (int32_t)ans_float;

		MagCalStatus = 1;
	}

	DWT_Init();

	BSP_LED_On(LED2);
	HAL_Delay(500);
	BSP_LED_Off(LED2);

	/* Start receiving messages via DMA */
	UART_StartReceiveMsg();
}

static void Init_Sensors(void)
{
	BSP_SENSOR_ACC_Init();
	BSP_SENSOR_GYR_Init();
	BSP_SENSOR_MAG_Init();
	BSP_SENSOR_PRESS_Init();
	BSP_SENSOR_TEMP_Init();
	BSP_SENSOR_HUM_Init();

	BSP_SENSOR_ACC_SetOutputDataRate(ACC_ODR);
	BSP_SENSOR_ACC_SetFullScale(ACC_FS);
}

static void RTC_Handler(TMsg *Msg)
{
	uint8_t sub_sec = 0;
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructure;
	uint32_t ans_uint32;
	int32_t ans_int32;
	uint32_t RtcSynchPrediv = hrtc.Init.SynchPrediv;

	if (UseOfflineData == 1)
	{
		Msg->Data[3] = (uint8_t)OfflineData[OfflineDataReadIndex].hours;
		Msg->Data[4] = (uint8_t)OfflineData[OfflineDataReadIndex].minutes;
		Msg->Data[5] = (uint8_t)OfflineData[OfflineDataReadIndex].seconds;
		Msg->Data[6] = (uint8_t)OfflineData[OfflineDataReadIndex].subsec;
	}
	else
	{
		(void)HAL_RTC_GetTime(&hrtc, &stimestructure, FORMAT_BIN);
		(void)HAL_RTC_GetDate(&hrtc, &sdatestructureget, FORMAT_BIN);

		/* To be MISRA C-2012 compliant the original calculation:
		   sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
		   has been split to separate expressions */
		ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
		ans_int32 /= RtcSynchPrediv + 1;
		ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
		sub_sec = (uint8_t)ans_uint32;

		Msg->Data[3] = (uint8_t)stimestructure.Hours;
		Msg->Data[4] = (uint8_t)stimestructure.Minutes;
		Msg->Data[5] = (uint8_t)stimestructure.Seconds;
		Msg->Data[6] = sub_sec;
	}
}

void BSP_PB_Callback(Button_TypeDef Button)
{
	MagCalRequest = 1U;
}

static void Accelero_Sensor_Handler(TMsg *Msg)
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

    Serialize_s32(&Msg->Data[19], (int32_t)AccValue.x, 4);
    Serialize_s32(&Msg->Data[23], (int32_t)AccValue.y, 4);
    Serialize_s32(&Msg->Data[27], (int32_t)AccValue.z, 4);
}

static void Gyro_Sensor_Handler(TMsg *Msg)
{
    if (UseOfflineData == 1)
    {
		GyrValue.x = OfflineData[OfflineDataReadIndex].angular_rate_x_mdps;
		GyrValue.y = OfflineData[OfflineDataReadIndex].angular_rate_y_mdps;
		GyrValue.z = OfflineData[OfflineDataReadIndex].angular_rate_z_mdps;
    }
    else
    {
		BSP_SENSOR_GYR_GetAxes(&GyrValue);
    }

    Serialize_s32(&Msg->Data[31], GyrValue.x, 4);
    Serialize_s32(&Msg->Data[35], GyrValue.y, 4);
    Serialize_s32(&Msg->Data[39], GyrValue.z, 4);
}

static void Magneto_Sensor_Handler(TMsg *Msg)
{
	float ans_float;
	MFX_MagCal_input_t mag_data_in;
	MFX_MagCal_output_t mag_data_out;

	if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
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

			if (MagCalStatus == 0U)
			{
				mag_data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
				mag_data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
				mag_data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

				mag_data_in.time_stamp = (int)TimeStamp;
				TimeStamp += (uint32_t)ALGO_PERIOD;

				MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

				if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
				{
					MagCalStatus = 1;

					ans_float = (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
					MagOffset.x = (int32_t)ans_float;
					ans_float = (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
					MagOffset.y = (int32_t)ans_float;
					ans_float = (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);
					MagOffset.z = (int32_t)ans_float;

					/* Disable magnetometer calibration */
					MotionFX_manager_MagCal_stop(ALGO_PERIOD);
				}
			}

			MagValue.x = (int32_t)(MagValue.x - MagOffset.x);
			MagValue.y = (int32_t)(MagValue.y - MagOffset.y);
			MagValue.z = (int32_t)(MagValue.z - MagOffset.z);
		}

		Serialize_s32(&Msg->Data[43], MagValue.x, 4);
		Serialize_s32(&Msg->Data[47], MagValue.y, 4);
		Serialize_s32(&Msg->Data[51], MagValue.z, 4);
	}
}

/**
  * @brief  Handles the TEMP axes data getting/sending
  * @param  Msg the TEMP part of the stream
  * @retval None
  */
static void Temperature_Sensor_Handler(TMsg *Msg)
{
	if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
	{
		if (UseOfflineData == 1)
		{
			TempValue = OfflineData[OfflineDataReadIndex].temperature;
		}
		else
		{
			BSP_SENSOR_TEMP_GetValue(&TempValue);
		}

		(void)memcpy(&Msg->Data[11], (void *)&TempValue, sizeof(float));
	}
}

/**
  * @brief  Handles the HUM axes data getting/sending
  * @param  Msg the HUM part of the stream
  * @retval None
  */
static void Humidity_Sensor_Handler(TMsg *Msg)
{
	if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
	{
		if (UseOfflineData == 1)
		{
			HumValue = OfflineData[OfflineDataReadIndex].humidity;
		}
		else
		{
			BSP_SENSOR_HUM_GetValue(&HumValue);
		}

		(void)memcpy(&Msg->Data[15], (void *)&HumValue, sizeof(float));;
	}
}

/**
  * @brief  Timer configuration
  * @param  Freq the desired Timer frequency
  * @retval None
  */
static void TIM_Config(uint32_t Freq)
{
	const uint32_t tim_counter_clock = 2000; /* TIM counter clock 2 kHz */
	uint32_t prescaler_value = (uint32_t)((SystemCoreClock / tim_counter_clock) - 1);
	uint32_t period = (tim_counter_clock / Freq) - 1;

	BSP_IP_TIM_Handle.Init.Prescaler = prescaler_value;
	BSP_IP_TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	BSP_IP_TIM_Handle.Init.Period = period;
	BSP_IP_TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	BSP_IP_TIM_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&BSP_IP_TIM_Handle) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  Initialize DWT register for counting clock cycles purpose
  * @param  None
  * @retval None
  */
static void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */
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


void UART_SendChar(uint8_t c)
{
    HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
}

void UART_SendString(const char* str)
{
    while (*str)
    {
        UART_SendChar(*str++);
    }
}

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

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CRC_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_RTC_Init();
	MX_Sensorfusion_init();

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


	int cnt = 1;

	struct euler_angles {
		double roll;
		double pitch;
		double yaw;
	};

	// init_complementary_filter();

	roll_pitch_gain = (0.5 * (
        attitute_p_gain[X] +
        attitute_p_gain[Y]
    ));
    yaw_w = attitute_p_gain[Z] / roll_pitch_gain;

    if (yaw_w < 0.0) {
        yaw_w = 0.0;
    }
    if (yaw_w > 1.0) {
        yaw_w = 1.0;
    }

    thr_int[0] = 0.0;
    thr_int[1] = 0.0;
    thr_int[2] = 0.0;

    attitute_p_gain[2] = roll_pitch_gain;

    double v_target[3] = {0.0, 0.0, 0.0};

    double rate_setpoint[3];
    double quad_rate[3] = {0.0, 0.0, 0.0};
    double omega_dot[3] = {0.0, 0.0, 0.0};
    double rateCtrl[3];

    double mixerFM[4][4] = {
        {  23000,  64000,  64000, -1530000},
        {  23000, -64000,  64000,  1530000},
        {  23000, -64000, -64000, -1530000},
        {  23000,  64000, -64000,  1530000}
    };

	DWT_Start();

	while (1)
	{

		static TMsg msg_dat;
		static TMsg msg_cmd;
		static int discarded_count = 0;
		uint32_t dt_us = 0U;
		MFX_input_t data_in;
		MFX_input_t *pdata_in = &data_in;

		if (UART_ReceivedMSG((TMsg *)&msg_cmd) == 1)
		{
			if (msg_cmd.Data[0] == DEV_ADDR)
			{
				(void)HandleMSG((TMsg *)&msg_cmd);
			}
		}

		if (MagCalRequest == 1U)
		{
			/* Debouncing */
			HAL_Delay(50);

			/* Wait until the button is released */
			while ((BSP_PB_GetState( BUTTON_KEY ) == PushButtonState));

			/* Debouncing */
			HAL_Delay(50);

			MagCalRequest = 0;

			/* Reset magnetometer calibration value*/
			MagCalStatus = 0;
			MagOffset.x = 0;
			MagOffset.y = 0;
			MagOffset.z = 0;

			/* Enable magnetometer calibration */
			MotionFX_manager_MagCal_start(ALGO_PERIOD);
		}
		SensorReadRequest = 1U;
		if (SensorReadRequest == 1U)
		{
			SensorReadRequest = 0;

			/* Acquire data from enabled sensors and fill Msg stream */
			RTC_Handler(&msg_dat);
			Accelero_Sensor_Handler(&msg_dat);
			Gyro_Sensor_Handler(&msg_dat);
			Magneto_Sensor_Handler(&msg_dat);
			Humidity_Sensor_Handler(&msg_dat);
			Temperature_Sensor_Handler(&msg_dat);

			/* Convert angular velocity from [mdps] to [dps] */
			data_in.gyro[0] = (float)GyrValue.x * FROM_MDPS_TO_DPS;
			data_in.gyro[1] = (float)GyrValue.y * FROM_MDPS_TO_DPS;
			data_in.gyro[2] = (float)GyrValue.z * FROM_MDPS_TO_DPS;

			/* Convert acceleration from [mg] to [g] */
			data_in.acc[0] = (float)AccValue.x * FROM_MG_TO_G;
			data_in.acc[1] = (float)AccValue.y * FROM_MG_TO_G;
			data_in.acc[2] = (float)AccValue.z * FROM_MG_TO_G;

			/* Convert magnetic field intensity from [mGauss] to [uT / 50] */
			data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
			data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
			data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

			/* Run Sensor Fusion algorithm */
			// BSP_LED_On(LED2);
			MotionFX_manager_run(pdata_in, pdata_out, MOTION_FX_ENGINE_DELTATIME);
			// BSP_LED_Off(LED2);
//
//			(void) memcpy(&msg_dat->Data[55], (void *)pdata_out->quaternion, 4U * sizeof(float));
//			(void) memcpy(&msg_dat->Data[71], (void *)pdata_out->rotation, 3U * sizeof(float));
//			(void) memcpy(&msg_dat->Data[83], (void *)pdata_out->gravity, 3U * sizeof(float));
//			(void) memcpy(&msg_dat->Data[95], (void *)pdata_out->linear_acceleration, 3U * sizeof(float));
//
//			(void) memcpy(&msg_dat->Data[107], (void *) & (pdata_out->heading), sizeof(float));
//			(void) memcpy(&msg_dat->Data[111], (void *) & (pdata_out->headingErr), sizeof(float));
//
//			Serialize_s32(&msg_dat->Data[115], (int32_t)dt_us, 4);

			/* Send data stream */
			INIT_STREAMING_HEADER(&msg_dat);
			msg_dat.Len = STREAMING_MSG_LENGTH;

			if (UseOfflineData == 1U)
			{
				OfflineDataCount--;
				if (OfflineDataCount < 0)
				{
					OfflineDataCount = 0;
				}

				OfflineDataReadIndex++;
				if (OfflineDataReadIndex >= OFFLINE_DATA_SIZE)
				{
					OfflineDataReadIndex = 0;
				}

				if (OfflineDataCount > 0)
				{
					SensorReadRequest = 1;
				}
			}

			if (discarded_count >= SAMPLETODISCARD)
			{
				UART_SendMsg(&msg_dat);
			}
			else
			{
				discarded_count++;
			}
		}

		acc_a_x = pdata_out->linear_acceleration[0];
		acc_a_y = pdata_out->linear_acceleration[1];
		acc_a_z = pdata_out->linear_acceleration[2];


		dt_us = DWT_Stop();


		DWT_Start();

		acc_v_x = acc_v_x + dt_us * (
			acc_a_x
		);
		acc_v_y = acc_v_y + dt_us * (
			acc_a_y
		);
		acc_v_z = acc_v_z + dt_us * (
			acc_a_z
		);

		Quaternion quad_quat_staten;
		quad_quat_staten.w = pdata_out->quaternion[3];
		quad_quat_staten.x = pdata_out->quaternion[2];
		quad_quat_staten.y = pdata_out->quaternion[1];
		quad_quat_staten.z = pdata_out->quaternion[0];

//		acc_v_x = 1;
//		acc_v_y = 0;
//		acc_v_z = 0;

		// Controller
		quad_rate[0] = gyro_v_x;
		quad_rate[1] = gyro_v_y;
		quad_rate[2] = gyro_v_z;

		omega_dot[0] = gyro_a_x;
		omega_dot[1] = gyro_a_y;
		omega_dot[2] = gyro_a_z;

		// Controller v_z
		v_error[Z] = v_target[Z] - acc_v_z;
		// double thrust_z;

		thrust_sp[2] = vel_p_gain[2] * v_error[2] - vel_d_gain[2] * acc_a_z + quadcopter_mass * (acc_setpoint[2] - gravitation_constant) + thr_int[2];

		double uMax = -0.4;
		double uMin = -16 * 9.18;

		// thrust_sp[2] = thrust_z;
		if (thrust_sp[2] < uMin) {
			thrust_sp[2] = uMin;
		}
		if (thrust_sp[2] > uMax) {
			thrust_sp[2] = uMax;
		}

		// Controller v_x, v_y

		// XY Velocity Control (Thrust in NE-direction)
		v_error[X] = v_target[X] - acc_v_x;
		v_error[Y] = v_target[Y] - acc_v_y;
		thrust_sp[X] = (
			vel_p_gain[X] * v_error[X] -
			vel_d_gain[X] *  acc_a_x +
			quadcopter_mass * acc_setpoint[X] +
			thr_int[X]
		);
		thrust_sp[Y] = (
			vel_p_gain[Y] * v_error[Y] -
			vel_d_gain[Y] *  acc_a_y +
			quadcopter_mass * acc_setpoint[Y] +
			thr_int[Y]
		);

		double thrust_max_xy_tilt = (
			abs(thrust_sp[2]) * tan(tiltMax)
		);
		double thrust_max_xy = sqrt(
			pow(maxThr, 2) -
			pow(thrust_sp[Z], 2)
		);

		if (thrust_max_xy > thrust_max_xy_tilt) {
			thrust_max_xy = thrust_max_xy_tilt;
		}

		if (
			thrust_sp[X] * thrust_sp[X] + thrust_sp[Y] * thrust_sp[Y] >
			pow(thrust_max_xy, 2)) {

			double mag = sqrt(
				pow(thrust_sp[X], 2) +
				pow(thrust_sp[Y], 2)
			);

			thrust_sp[X] = (
				thrust_sp[X] * thrust_max_xy / mag
			);
			thrust_sp[Y] = (
				thrust_sp[Y] * thrust_max_xy / mag
			);
		}

		// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990

		v_error_lim[X] = v_error[X] - (
			thrust_sp[X] - thrust_sp[X]
			) * 2.0 / vel_p_gain[X];
		v_error_lim[Y] = v_error[X] - (
			thrust_sp[Y] - thrust_sp[Y]
			) * 2.0 / vel_p_gain[Y];

		double tmpnorm = sqrt(
			thrust_sp[0] * thrust_sp[0] +
			thrust_sp[1] * thrust_sp[1] +
			thrust_sp[2] * thrust_sp[2]
		);

		body_z[0] = -thrust_sp[0] / tmpnorm;
		body_z[1] = -thrust_sp[1] / tmpnorm;
		body_z[2] = -thrust_sp[2] / tmpnorm;

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

		e_z[0] = quad_dcm[0][2];
		e_z[1] = quad_dcm[1][2];
		e_z[2] = quad_dcm[2][2];

		double normThrustSp = sqrt(
			thrust_sp[0] * thrust_sp[0] +
			thrust_sp[1] * thrust_sp[1] +
			thrust_sp[2] * thrust_sp[2]
		);

		e_z_d[0] = -thrust_sp[0] / normThrustSp;
		e_z_d[1] = -thrust_sp[1] / normThrustSp;
		e_z_d[2] = -thrust_sp[2] / normThrustSp;

		double qe_red_dot_part = (
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

		qe_red[0] = qe_red_dot_part + sqrt(
			e_z[0] * e_z[0] +
			e_z[1] * e_z[1] +
			e_z[2] * e_z[2] +
			e_z_d[0] * e_z_d[0] +
			e_z_d[1] * e_z_d[1] +
			e_z_d[2] * e_z_d[2]
		);

		qe_red[1] = (
			e_z[1] * e_z_d[2] -
			e_z[2] * e_z_d[1]
		);
		qe_red[2] = (
			e_z[2] * e_z_d[0] -
			e_z[0] * e_z_d[2]
		);
		qe_red[3] = (
			e_z[0] * e_z_d[1] -
			e_z[1] * e_z_d[0]
		);

		qe_norm = sqrt(
			qe_red[0] * qe_red[0] +
			qe_red[1] * qe_red[1] +
			qe_red[2] * qe_red[2] +
			qe_red[3] * qe_red[3]
		);

		qe_red[0] = qe_red[0] / qe_norm;
		qe_red[1] = qe_red[1] / qe_norm;
		qe_red[2] = qe_red[2] / qe_norm;
		qe_red[3] = qe_red[3] / qe_norm;

		// Reduced desired quaternion (reduced because it doesn't consider the desired Yaw angle)

		qd_red[0] = (
			qe_red[0] * quad_quat_state[0] -
			qe_red[1] * quad_quat_state[1] -
			qe_red[2] * quad_quat_state[2] -
			qe_red[3] * quad_quat_state[3]
		);
		qd_red[1] = (
			qe_red[1] * quad_quat_state[0] +
			qe_red[0] * quad_quat_state[1] -
			qe_red[3] * quad_quat_state[2] +
			qe_red[2] * quad_quat_state[3]
		);
		qd_red[2] = (
			qe_red[2] * quad_quat_state[0] +
			qe_red[3] * quad_quat_state[1] +
			qe_red[0] * quad_quat_state[2] -
			qe_red[1] * quad_quat_state[3]
		);
		qd_red[3] = (
			qe_red[3] * quad_quat_state[0] -
			qe_red[2] * quad_quat_state[1] +
			qe_red[1] * quad_quat_state[2] +
			qe_red[0] * quad_quat_state[3]
		);

		// Mixed desired quaternion (between reduced and full) and resulting desired quaternion qd

		qd_norm = sqrt(
			qd_red[0] * qd_red[0] +
			qd_red[1] * qd_red[1] +
			qd_red[2] * qd_red[2] +
			qd_red[3] * qd_red[3]
		);



		Quaternion qd_reduziert;
		qd_reduziert.w = qd_red[0];
		qd_reduziert.x = qd_red[1];
		qd_reduziert.y = qd_red[2];
		qd_reduziert.z = qd_red[3];

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

		Quaternion quad_quat_state_inverse = inverseQuaternion(quad_quat_staten);

		Quaternion quaternion_error;
		quaternion_error = kreuzproduktQuaternion(quad_quat_state_inverse, qd);

		if (quaternion_error.w > 0) {
			rate_setpoint[0] = (2.0 * quaternion_error.x * attitute_p_gain[0]);
			rate_setpoint[1] = (2.0 * quaternion_error.y * attitute_p_gain[1]);
			rate_setpoint[2] = (2.0 * quaternion_error.z * attitute_p_gain[2]);
		} else {
			rate_setpoint[0] = -(2.0 * quaternion_error.x * attitute_p_gain[0]);
			rate_setpoint[1] = -(2.0 * quaternion_error.y * attitute_p_gain[1]);
			rate_setpoint[2] = -(2.0 * quaternion_error.z * attitute_p_gain[2]);
		}

		// Rate Control
		for (int i = 0; i < 3; i++) {
			rate_error[i] = rate_setpoint[i] - quad_rate[i];
			rateCtrl[i] = (
				rate_p_gain[i] * rate_error[i] -
				rate_d_gain[i] * omega_dot[i]
			);
		}

		thrust_sp_norm = sqrt(
			pow(thrust_sp[0], 2) +
			pow(thrust_sp[1], 2) +
			pow(thrust_sp[2], 2)
		);

		w_cmd[0] = (
			mixerFM[0][0] * thrust_sp_norm +
			mixerFM[0][1] * rateCtrl[X] +
			mixerFM[0][2] * rateCtrl[Y] +
			mixerFM[0][3] * rateCtrl[Z]
		);
		w_cmd[1] = (
			mixerFM[1][0] * thrust_sp_norm +
			mixerFM[1][1] * rateCtrl[X] +
			mixerFM[1][2] * rateCtrl[Y] +
			mixerFM[1][3] * rateCtrl[Z]
		);
		w_cmd[2] = (
			mixerFM[2][0] * thrust_sp_norm +
			mixerFM[2][1] * rateCtrl[X] +
			mixerFM[2][2] * rateCtrl[Y] +
			mixerFM[2][3] * rateCtrl[Z]
		);
		w_cmd[3] = (
			mixerFM[3][0] * thrust_sp_norm +
			mixerFM[3][1] * rateCtrl[X] +
			mixerFM[3][2] * rateCtrl[Y] +
			mixerFM[3][3] * rateCtrl[Z]
		);

		for (int i = 0; i < 4; i++) {
			w_cmd_clipped[i] = sqrt(w_cmd[i]);
			if (w_cmd_clipped[i] < minWMotor) {
				w_cmd_clipped[i] = minWMotor;
			};
			if (w_cmd_clipped[i] > maxWMotor) {
				w_cmd_clipped[i] = maxWMotor;
			};
		};

		// PWM Motor control

	    setPwmDutyCycle(&htim1, TIM_CHANNEL_1, 20.0f / 100.f * w_cmd_clipped[0] + 0.4f);
	    setPwmDutyCycle(&htim1, TIM_CHANNEL_2, 20.0f / 100.f * w_cmd_clipped[1] + 0.4f);
	    setPwmDutyCycle(&htim2, TIM_CHANNEL_1, 20.0f / 100.f * w_cmd_clipped[2] + 0.4f);
	    setPwmDutyCycle(&htim2, TIM_CHANNEL_2, 20.0f / 100.f * w_cmd_clipped[3] + 0.4f);

	     HAL_Delay(100);


		t += delta_t;

		cnt++;
	}
}

void setPwmDutyCycle(TIM_HandleTypeDef *htim, uint32_t channel, float dutyCycle)
{
	if (dutyCycle < 0.4f) dutyCycle = 0.4f;
	if (dutyCycle > 0.6f) dutyCycle = 0.6f;

	uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);
	uint32_t pulse = (uint32_t)(dutyCycle * period);

	__HAL_TIM_SET_COMPARE(htim, channel, pulse);
}


void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Configure GPIO pins : PA2 PA3 (TIM1 CH1 and CH2)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure GPIO pins : PB2 PB3 (TIM2 CH1 and CH2)
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(htim->Instance==TIM1)
	{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**TIM1 GPIO Configuration
	PA8     ------> TIM1_CH1
	PA9     ------> TIM1_CH2
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else if(htim->Instance==TIM2)
	{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**TIM2 GPIO Configuration
	PB8     ------> TIM2_CH1
	PB9     ------> TIM2_CH2
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}

static void MX_TIM1_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 83;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 83;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
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
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
	Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
	Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
	Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim2);
}

static void MX_CRC_Init(void)
{
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

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
}

void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure GPIO pins : PA8 PA9 (TIM1 CH1 and CH2)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure GPIO pins : PB8 PB9 (TIM2 CH1 and CH2)
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
//__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
//  /* NOTE: This function should not be modified, when the callback is needed,
//           the HAL_UART_RxCpltCallback could be implemented in the user file
//   */
//  HAL_UART_Transmit(&huart2, rx_buffer, sizeof(rx_buffer), 10);
//}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
/*
 * backup.c
 *
 *  Created on: Aug 20, 2024
 *      Author: tim
 */


