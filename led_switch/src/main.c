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

#define PORTD_15 0x00008000
#define PORTD_14 0x00004000
#define PORTD_13 0x00002000
#define PORTD_12 0x00001000
#define PORTD_ALL 0x0000F000

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

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; /* for Leds 5-6 */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* for user button 1*/

  /* Setup the Pin for GPIOA - Push button */
  GPIOA->MODER |= 0x00000000; /* Input Mode */
  GPIOA->OTYPER |= 0x00000000; /*Configure as output as default push-pull*/
  GPIOA->OSPEEDR |=0x00000002; /*Configure as high speed*/
  GPIOA->PUPDR |= 0x00000002; /*Configure as pull-down */

  /* Setup the Pin for GPIOD - LEDS on the board */
  GPIOD->MODER |= 0x55000000;/* LED 5-8 are on GPIOD Pins 12-15 */
  GPIOD->OTYPER |= 0x00000000; /*Configure as output push-pull */
  GPIOD->OSPEEDR |=0xAA000000; /* Configure as high speed */
  GPIOD->PUPDR |= 0x00000000; /*No pull-up or pull down*/



  unsigned int i = 0;
  unsigned int index= 0;
  while(1) {
    /** If you hold down button, it will cycle,rate if controled by
      * for loop at the end
      */
    if(GPIOA->IDR & 0x1){
      if (i > 3) i = 0;
      switch(i){
        case 0:
          GPIOD->ODR = PORTD_12;
          break;
        case 1:
          GPIOD->ODR = PORTD_13;
          break;
        case 2:
          GPIOD->ODR = PORTD_14;
          break;
        case 3:
          GPIOD->ODR = PORTD_15;
          break;
        default:
          GPIOD->ODR = PORTD_ALL;
          break;
      }

      i++;
      for (index = 0; index < 5000000; index++);/* Debouncing*/
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
