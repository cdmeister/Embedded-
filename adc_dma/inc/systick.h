/**
  ******************************************************************************
  * @file    systick.h
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    30-April-2018
  * @brief   Systick ARM M4
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTICK_H
#define __SYSTICK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Delay(uint32_t nTime);
void SysTick_Handler(void);
void SysTick_Init(uint32_t ticks);
/* Private variables -------------------------------------------------------- */
volatile uint32_t TimeDelay;

#endif /* __SYSTICK_H */
