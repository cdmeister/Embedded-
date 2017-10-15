/**
  ******************************************************************************
  * @file    main.c
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    9-September-2017
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

#define PORTD_15 0x00008000
#define PORTD_14 0x00004000
#define PORTD_13 0x00002000
#define PORTD_12 0x00001000
#define PORTD_ALL 0x0000F000

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  /* Enable Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;


  // Set mode of all pins as digital output
  // 00 = digital input       01 = digital output
  // 10 = alternate function  11 = analog (default)
  GPIOD->MODER &= ~(0xFF<<24); /* Clear mode bits */
  GPIOD->MODER |= 85UL<<24;/* LED 5-8 are on GPIOD Pins 12-15 */

  // Set output type of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(0xF<<3); /*Configure as output open-drain */

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(0xFF<<3); /* Configure as high speed */
  GPIOD->OSPEEDR |= (0xFF<<3);

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(0xFF<<3); /*no pul-up, no pull-down*/

  unsigned int delay =0;

  /* Infinite loop */
  while (1)
  {
    GPIOD->ODR |=PORTD_12;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_12;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR |=PORTD_13;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_13;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR |=PORTD_14;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_14;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR |=PORTD_15;
    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_15;

    for(delay= 0; delay < 1066667; delay++);

    GPIOD->ODR |=PORTD_ALL;

    for(delay= 0; delay < 1066667; delay++);
    GPIOD->ODR &=~PORTD_ALL;
    for(delay= 0; delay < 1066667; delay++);

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
