/**
  ******************************************************************************
  * @file    iks4a1_mems_control.c
  * @author  MEMS Software Solutions Team
  * @brief   This file contains the MEMS sensors interface for X-NUCLEO-IKS4A1
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

#include "iks4a1_mems_control.h"

/**
  * @brief  Initializes accelerometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_ACC_Init(void)
{
  (void)IKS4A1_MOTION_SENSOR_Init(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO);
}

/**
  * @brief  Initializes gyroscope
  * @param  None
  * @retval None
  */
void BSP_SENSOR_GYR_Init(void)
{
  (void)IKS4A1_MOTION_SENSOR_Init(IKS4A1_LSM6DSV16X_0, MOTION_GYRO);
}

/**
  * @brief  Initializes magnetometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_MAG_Init(void)
{
  (void)IKS4A1_MOTION_SENSOR_Init(IKS4A1_LIS2MDL_0, MOTION_MAGNETO);
}

/**
  * @brief  Initializes pressure sensor
  * @param  None
  * @retval None
  */
void BSP_SENSOR_PRESS_Init(void)
{
  (void)IKS4A1_ENV_SENSOR_Init(IKS4A1_LPS22DF_0, ENV_PRESSURE);
}

/**
  * @brief  Initializes temperature sensor
  * @param  None
  * @retval None
  */
void BSP_SENSOR_TEMP_Init(void)
{
  (void)IKS4A1_ENV_SENSOR_Init(IKS4A1_STTS22H_0, ENV_TEMPERATURE);
}

/**
  * @brief  Initializes humidity sensor
  * @param  None
  * @retval None
  */
void BSP_SENSOR_HUM_Init(void)
{
  (void)IKS4A1_ENV_SENSOR_Init(IKS4A1_SHT40AD1B_0, ENV_HUMIDITY);
}

/**
  * @brief  Enables accelerometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_ACC_Enable(void)
{
  (void)IKS4A1_MOTION_SENSOR_Enable(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO);
}

/**
  * @brief  Enables gyroscope
  * @param  None
  * @retval None
  */
void BSP_SENSOR_GYR_Enable(void)
{
  (void)IKS4A1_MOTION_SENSOR_Enable(IKS4A1_LSM6DSV16X_0, MOTION_GYRO);
}

/**
  * @brief  Enables magnetometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_MAG_Enable(void)
{
  (void)IKS4A1_MOTION_SENSOR_Enable(IKS4A1_LIS2MDL_0, MOTION_MAGNETO);
}

/**
  * @brief  Enables pressure sensor
  * @param  None
  * @retval None
  */
void BSP_SENSOR_PRESS_Enable(void)
{
  (void)IKS4A1_ENV_SENSOR_Enable(IKS4A1_LPS22DF_0, ENV_PRESSURE);
}

/**
  * @brief  Enables temperature sensor
  * @param  None
  * @retval None
  */
void BSP_SENSOR_TEMP_Enable(void)
{
  (void)IKS4A1_ENV_SENSOR_Enable(IKS4A1_STTS22H_0, ENV_TEMPERATURE);
}

/**
  * @brief  Enables humidity sensors
  * @param  None
  * @retval None
  */
void BSP_SENSOR_HUM_Enable(void)
{
  (void)IKS4A1_ENV_SENSOR_Enable(IKS4A1_SHT40AD1B_0, ENV_HUMIDITY);
}

/**
  * @brief  Disables accelerometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_ACC_Disable(void)
{
  (void)IKS4A1_MOTION_SENSOR_Disable(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO);
}

/**
  * @brief  Disables gyroscope
  * @param  None
  * @retval None
  */
void BSP_SENSOR_GYR_Disable(void)
{
  (void)IKS4A1_MOTION_SENSOR_Disable(IKS4A1_LSM6DSV16X_0, MOTION_GYRO);
}

/**
  * @brief  Disables magnetometer
  * @param  None
  * @retval None
  */
void BSP_SENSOR_MAG_Disable(void)
{
  (void)IKS4A1_MOTION_SENSOR_Disable(IKS4A1_LIS2MDL_0, MOTION_MAGNETO);
}

/**
  * @brief  Disables pressure sensor
  * @param  None
  * @retval None
  */
void BSP_SENSOR_PRESS_Disable(void)
{
  (void)IKS4A1_ENV_SENSOR_Disable(IKS4A1_LPS22DF_0, ENV_PRESSURE);
}

/**
  * @brief  Disables temperature sensor
  * @param  None
  * @retval None
  */
void BSP_SENSOR_TEMP_Disable(void)
{
  (void)IKS4A1_ENV_SENSOR_Disable(IKS4A1_STTS22H_0, ENV_TEMPERATURE);
}

/**
  * @brief  Disables humidity sensor
  * @param  None
  * @retval None
  */
void BSP_SENSOR_HUM_Disable(void)
{
  (void)IKS4A1_ENV_SENSOR_Disable(IKS4A1_SHT40AD1B_0, ENV_HUMIDITY);
}

/**
  * @brief  Get accelerometer data
  * @param  Axes pointer to axes data structure
  * @retval None
  */
void BSP_SENSOR_ACC_GetAxes(IKS4A1_MOTION_SENSOR_Axes_t *Axes)
{
  (void)IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, Axes);
}

/**
  * @brief  Get gyroscope data
  * @param  Axes pointer to axes data structure
  * @retval None
  */
void BSP_SENSOR_GYR_GetAxes(IKS4A1_MOTION_SENSOR_Axes_t *Axes)
{
  (void)IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, Axes);
}

/**
  * @brief  Get magnetometer data
  * @param  Axes pointer to axes data structure
  * @retval None
  */
void BSP_SENSOR_MAG_GetAxes(IKS4A1_MOTION_SENSOR_Axes_t *Axes)
{
  (void)IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, Axes);
}

/**
  * @brief  Get pressure sensor data
  * @param  Value pointer to pressure value
  * @retval None
  */
void BSP_SENSOR_PRESS_GetValue(float *Value)
{
  (void)IKS4A1_ENV_SENSOR_GetValue(IKS4A1_LPS22DF_0, ENV_PRESSURE, Value);
}

/**
  * @brief  Get temperature sensor data
  * @param  Value pointer to temperature value
  * @retval None
  */
void BSP_SENSOR_TEMP_GetValue(float *Value)
{
  (void)IKS4A1_ENV_SENSOR_GetValue(IKS4A1_STTS22H_0, ENV_TEMPERATURE, Value);
}

/**
  * @brief  Get humidity sensor data
  * @param  Value pointer to humidity value
  * @retval None
  */
void BSP_SENSOR_HUM_GetValue(float *Value)
{
  (void)IKS4A1_ENV_SENSOR_GetValue(IKS4A1_SHT40AD1B_0, ENV_HUMIDITY, Value);
}

/**
  * @brief  Set output data rate for accelerometer
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_ACC_SetOutputDataRate(float Odr)
{
  (void)IKS4A1_MOTION_SENSOR_SetOutputDataRate(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, Odr);
}

/**
  * @brief  Set output data rate for gyroscope
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_GYR_SetOutputDataRate(float Odr)
{
  (void)IKS4A1_MOTION_SENSOR_SetOutputDataRate(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, Odr);
}

/**
  * @brief  Set output data rate for magnetometer
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_MAG_SetOutputDataRate(float Odr)
{
  (void)IKS4A1_MOTION_SENSOR_SetOutputDataRate(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, Odr);
}

/**
  * @brief  Set output data rate for pressure sensor
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_PRESS_SetOutputDataRate(float Odr)
{
  (void)IKS4A1_ENV_SENSOR_SetOutputDataRate(IKS4A1_LPS22DF_0, ENV_PRESSURE, Odr);
}

/**
  * @brief  Set output data rate for temperature sensors
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_TEMP_SetOutputDataRate(float Odr)
{
  (void)IKS4A1_ENV_SENSOR_SetOutputDataRate(IKS4A1_STTS22H_0, ENV_TEMPERATURE, Odr);
}

/**
  * @brief  Set output data rate for humidity sensor
  * @param  Odr Output Data Rate value to be set
  * @retval None
  */
void BSP_SENSOR_HUM_SetOutputDataRate(float Odr)
{
  (void)IKS4A1_ENV_SENSOR_SetOutputDataRate(IKS4A1_SHT40AD1B_0, ENV_HUMIDITY, Odr);
}

/**
  * @brief  Get output data rate for accelerometer
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_ACC_GetOutputDataRate(float *Odr)
{
  (void)IKS4A1_MOTION_SENSOR_GetOutputDataRate(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, Odr);
}

/**
  * @brief  Get output data rate for gyroscope
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_GYR_GetOutputDataRate(float *Odr)
{
  (void)IKS4A1_MOTION_SENSOR_GetOutputDataRate(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, Odr);
}

/**
  * @brief  Get output data rate for magnetometer
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_MAG_GetOutputDataRate(float *Odr)
{
  (void)IKS4A1_MOTION_SENSOR_GetOutputDataRate(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, Odr);
}

/**
  * @brief  Get output data rate for pressure sensor
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_PRESS_GetOutputDataRate(float *Odr)
{
  (void)IKS4A1_ENV_SENSOR_GetOutputDataRate(IKS4A1_LPS22DF_0, ENV_PRESSURE, Odr);
}

/**
  * @brief  Get output data rate for temperature sensors
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_TEMP_GetOutputDataRate(float *Odr)
{
  (void)IKS4A1_ENV_SENSOR_GetOutputDataRate(IKS4A1_STTS22H_0, ENV_TEMPERATURE, Odr);
}

/**
  * @brief  Get output data rate for humidity sensor
  * @param  Odr Output Data Rate value
  * @retval None
  */
void BSP_SENSOR_HUM_GetOutputDataRate(float *Odr)
{
  (void)IKS4A1_ENV_SENSOR_GetOutputDataRate(IKS4A1_SHT40AD1B_0, ENV_HUMIDITY, Odr);
}

/**
  * @brief  Set full scale for accelerometer
  * @param  Fullscale Fullscale value to be set
  * @retval None
  */
void BSP_SENSOR_ACC_SetFullScale(int32_t Fullscale)
{
  (void)IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, Fullscale);
}

/**
  * @brief  Set full scale for gyroscope
  * @param  Fullscale Fullscale value to be set
  * @retval None
  */
void BSP_SENSOR_GYR_SetFullScale(int32_t Fullscale)
{
  (void)IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, Fullscale);
}

/**
  * @brief  Set full scale for magnetometer
  * @param  Fullscale Fullscale value to be set
  * @retval None
  */
void BSP_SENSOR_MAG_SetFullScale(int32_t Fullscale)
{
  (void)IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, Fullscale);
}

/**
  * @brief  Get full scale for accelerometer
  * @param  Fullscale Fullscale value
  * @retval None
  */
void BSP_SENSOR_ACC_GetFullScale(int32_t *Fullscale)
{
  (void)IKS4A1_MOTION_SENSOR_GetFullScale(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, Fullscale);
}

/**
  * @brief  Get full scale for gyroscope
  * @param  Fullscale Fullscale value
  * @retval None
  */
void BSP_SENSOR_GYR_GetFullScale(int32_t *Fullscale)
{
  (void)IKS4A1_MOTION_SENSOR_GetFullScale(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, Fullscale);
}

/**
  * @brief  Get full scale for magnetometer
  * @param  Fullscale Fullscale value
  * @retval None
  */
void BSP_SENSOR_MAG_GetFullScale(int32_t *Fullscale)
{
  (void)IKS4A1_MOTION_SENSOR_GetFullScale(IKS4A1_LIS2MDL_0, MOTION_MAGNETO, Fullscale);
}
