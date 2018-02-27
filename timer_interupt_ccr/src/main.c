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
#define PORTD_14 0x00004000
#define PORTD_15 0x00008000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t TimeDelay;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void TIM4_IRQHandler(void){

  // Check whether an overflow event has taken place
  if((TIM4->SR & TIM_SR_UIF) != 0){


    GPIOD->ODR ^=(PORTD_15);

    TIM4->SR &= ~TIM_SR_UIF;
  }


}

// SysTick System Handler
void SysTick_Handler (void){ // SysTick interrupt service routine
  // TimeDelay is a global variable delcared as volatile
  if (TimeDelay >0)         // Prevent it from being negative
    TimeDelay--;            // TimeDelay is global volatile variable
}

void Delay(uint32_t nTime){
  // nTime: specifies the delay time Length
  TimeDelay = nTime;        // Time Delay must be declared as volatile
  while(TimeDelay != 0);    // Busy wait
}

// Input: ticks = muber of ticks between two interrupts
void SysTick_Init (uint32_t ticks){

  //Disable SysTick IRQ and SysTick counter
  SysTick->CTRL = 0;

  // Set reload register
  SysTick->LOAD = ticks - 1;

  // Set interrupt priority of SysTick
  // Make SysTick Least urgent(i.ie., highest priority number)
  // __NVIC_PRIO_BITS: number of bits for priority levels, defined in CMSIS
  NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) -1);

  // Reset the SysTick counter value
  SysTick->VAL = 0;

  // Select Processor Cycle
  // 1 = processor clock; 0 = external clock
  SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

  // Enables SysTick exception request
  // 1 = counting down to zero asserts the SysTick exception request
  // 0 = counting down to zero does not assert the SysTick exception request
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

  // Enable SysTick Timer
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;


}

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
  GPIOD->MODER &=~(GPIO_MODER_MODE15|GPIO_MODER_MODE14);
  GPIOD->MODER |=(GPIO_MODER_MODE15_0|GPIO_MODER_MODE14_0); //output
  //GPIOD->MODER |=(GPIO_MODER_MODE15_1); //alternate function

  //GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL15);
  //GPIOD->AFR[1] |= GPIO_AFRH_AFSEL15_1;

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT15|GPIO_OTYPER_OT14);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED15|GPIO_OSPEEDR_OSPEED14); /* Configure as high speed */
  GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED15|GPIO_OSPEEDR_OSPEED14);

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD15|GPIO_PUPDR_PUPD14); /*no pul-up, no pull-down*/

  // Enable Timer 4 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  // Disable Timer 4
  TIM4->CR1 &= ~TIM_CR1_CEN;

  // Counting Direction: 0 = up-counting, 1 = down-counting
  TIM4->CR1 &=~(TIM_CR1_DIR);

  //Clock Prescaler
  TIM4->PSC = 62499;

  //Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0)
  TIM4->ARR = 1344;

  //Enable interrupts
  TIM4->DIER |= TIM_DIER_UIE;

  // Set TIM4 priority to 1
  NVIC_SetPriority(TIM4_IRQn,1);

  // Enable TIM4 interrupt
  NVIC_EnableIRQ(TIM4_IRQn);

  // Enable Timer 4
  TIM4->CR1 |= TIM_CR1_CEN;

  // Generate and interrupt every 1ms
  // http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html
  // If the clock is at 168MHz, then that is 168 000 000 ticks per second
  // but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead
  // you can generate an interupt every 1ms so that would be 168 000 ticks per
  // ms and you can fit 168 000 ticks into the LOAD register
  SysTick_Init(SystemCoreClock/1000);

  //unsigned int delay = 0;
  //GPIOD->ODR |= PORTD_15;
  //for(delay= 0; delay < 1066667; delay++);
  //GPIOD->ODR &=~PORTD_15;

  while(1){
    GPIOD->ODR ^=PORTD_14;
    Delay(500);

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
