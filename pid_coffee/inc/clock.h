/**
  ******************************************************************************
  * @file    lcd.h
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    30-April-2018
  * @brief   Header for clocks.
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CLOCK_H
#define __CLOCK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "flash.h"
#include "systick.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern void SystemHSIenable(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler);
extern void SystemHSEenable(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler);

extern void SystemPLLClockEnable(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler,
                            uint8_t pll_m, uint8_t pll_n,
                            uint8_t pll_p, uint8_t pll_q);

extern void SystemPLLClockUpdate(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler,
                            uint8_t pll_m, uint8_t pll_n,
                            uint8_t pll_p, uint8_t pll_q);

extern void SystemClockPrescaler(uint16_t ahb_prescaler,
                            uint8_t  apb1_prescaler,
                            uint8_t apb2_prescaler);

extern void SystemPLLPrescaler(uint8_t pll_m, uint8_t pll_n,
                              uint8_t pll_p, uint8_t pll_q);

extern uint16_t GetAHBPrescaler(void);
extern uint8_t GetAPB1Prescaler(void);
extern uint8_t GetAPB2Prescaler(void);
extern uint32_t GetHCLK(void);
extern uint32_t GetPCLK1(void);
extern uint32_t GetPCLK2(void);

#endif /* __CLOCK_H */
