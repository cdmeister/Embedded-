/**
  ******************************************************************************
  * @file    pid.h
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    30-April-2018
  * @brief   PID algorithm
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
uint32_t lastTime;
uint32_t Setpoint;
float errSum, lastErr;
float kp, ki, kd;

uint16_t Compute(float Input);
void SetTunings(float Kp, float Ki, float Kd);
void SetSetpoint(uint32_t setpoint);

#endif /* __PID_H */
