/**
  ******************************************************************************
  * @file    main.c
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    17-September-2017
  * @brief   Main program body
  ******************************************************************************

*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */

#define PORTE_14 0x00004000

#define PORTA_0 0x00000000

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  /** In order to work with pins, the RCC(Reset and Clock Control) must be
    * enabled
    */

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; /* for external led */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* for external  button 1*/

  /* Setup the Pin for GPIOA - Push button */
  GPIOA->MODER &= ~0x3UL; /* Input Mode */
  GPIOA->OTYPER |= 0x00000; /*Configure as output as default push-pull*/
  GPIOA->OSPEEDR |=0x00000002; /*Configure as high speed*/
  GPIOA->PUPDR &= ~0x3UL; /*Configure as no pullup or no pulldown*/

  // Set mode of all pins as digital output
  // 00 = digital input       01 = digital output
  // 10 = alternate function  11 = analog (default)
  GPIOE->MODER &= ~(0x3<<28); /* Clear mode bits */
  GPIOE->MODER |= (1<<28);/* LED 5-8 are on GPIOD Pins 12-15 */

  // Set output type of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOE->OTYPER &= ~(0x4<<3); /*Configure as output open-drain */

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOE->OSPEEDR &=~(0x01<<3); /* Configure as medium speed */
  GPIOE->OSPEEDR |= (0x01<<3);

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOE->PUPDR &= ~(0x01<<3); /*no pul-up, no pull-down*/


  /* Setup the Pin for GPIOE - LED connected to PIN PE14 */

   /*   GPIOE->ODR |= PORTE_14;*/

  while(1) {
    if((GPIOA->IDR & 0x1) == 0x0){

      GPIOE->ODR ^= PORTE_14;
      while((GPIOA->IDR &0x1) != 0x1);
    }

  }

  return 0;
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
