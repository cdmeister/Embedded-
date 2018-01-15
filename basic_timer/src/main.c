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
#define PORTD_15 0x00008000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  /*Enable Clock*/
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  GPIOD->MODER &=~(GPIO_MODER_MODE15);
  //GPIOD->MODER |=(GPIO_MODER_MODE15_0); //output
  GPIOD->MODER |=(GPIO_MODER_MODE15_1); //alternate function

  GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL15);
  GPIOD->AFR[1] |= GPIO_AFRH_AFSEL15_1;

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT15);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED15); /* Configure as high speed */
  GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED15);

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD15); /*no pul-up, no pull-down*/

  // Enable Timer 4 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  // Counting Direction: 0 = up-counting, 1 = down-counting
  TIM4->CR1 &=~(TIM_CR1_DIR);

  //Clock Prescaler
  TIM4->PSC = 62499;

  //Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0)
  TIM4->ARR = 1344;

  // Can be any value between 0 and 1344
  // Used to compare CNT with CCR4
  TIM4->CCR4 = 1344;

  // Clear output compare mode bits for channel 4
  TIM4->CCMR2 &= ~TIM_CCMR2_OC4M;

  // Select toggle mode(0011)
  TIM4->CCMR2 |= (TIM_CCMR2_OC4M_0|TIM_CCMR2_OC4M_1);

  // Select Preload Enable to be disable, allow to update CCR4 register
  // to be updated at anytime
  TIM4->CCMR2 &=~(TIM_CCMR2_OC4PE);

  // Select Output polarity: 0 = active high, 1 = active low
  TIM4->CCER &= ~(TIM_CCER_CC4P);
  TIM4->CCER &= ~(TIM_CCER_CC4NP);

  // Enable Output for channel 4
  TIM4->CCER |=TIM_CCER_CC4E;

  // Enable Timer 4
  TIM4->CR1 |= TIM_CR1_CEN;


  //unsigned int delay = 0;
  //GPIOD->ODR |= PORTD_15;
  //for(delay= 0; delay < 1066667; delay++);
  //GPIOD->ODR &=~PORTD_15;

  while(1);
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
