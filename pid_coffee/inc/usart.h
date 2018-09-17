/**
  ******************************************************************************
  * @file    lcd.h
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    30-April-2018
  * @brief   Header for lcd.c module, compatitable with HD44780
             tested on JHD 162A
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H
#define __USART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "systick.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define BufferSize 512
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void USARTx_Init(USART_TypeDef * USARTx);

void USART_print(USART_TypeDef * USARTx, const char * fmt, ...);

#endif /* __USART_H */
