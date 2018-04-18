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
#define PORTD_12 0x00001000 //Green
#define PORTD_13 0x00002000 //Orange
#define PORTD_14 0x00004000 //Red
#define PORTD_15 0x00008000 //Blue

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t TimeDelay;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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

void timer_init(void){

  // Enable Timer 4 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  // Disable Timer 4
  TIM4->CR1 &= ~TIM_CR1_CEN;

  // Counting Direction: 0 = up-counting, 1 = down-counting
  TIM4->CR1 &=~(TIM_CR1_DIR);

  // Auto-reload preload enable
  TIM4->CR1 &=~(TIM_CR1_ARPE);
  TIM4->CR1 |=(TIM_CR1_ARPE);

  //Clock Prescaler
  uint32_t TIM4COUNTER_Frequency = 100000; //Desired Frequency
  TIM4->PSC = (84000000/TIM4COUNTER_Frequency)-1;

  //Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0)
  TIM4->ARR = .01 * 100000-1;

  // ------------------Channel 3 Setup ----------------------------------

  // Disable Input/Output for Channel 3
  // This must be disable in order to set the channel as
  // Input or Output
  TIM4->CCER &= ~TIM_CCER_CC3E;

  // Set Channel 4 as output channel
  TIM4->CCMR2 &= ~(TIM_CCMR2_CC3S);

  // Set the first value to compare against
  // 50% duty cycle
  //TIM4->CCR3=.5*TIM4->ARR;
  TIM4->CCR3 = 0;

  // Clear Output compare mode bits for channel 3
  TIM4->CCMR2 &= ~TIM_CCMR2_OC3M;

  // Select Pulse Width Modulation Mode 1
  TIM4->CCMR2 |= (TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1);

  // Select Preload Enable to be enable for PWM, allow to update CCR4 register
  // to be updated at overflow/underflow events
  TIM4->CCMR2 &=~(TIM_CCMR2_OC3PE);
  TIM4->CCMR2 |= (TIM_CCMR2_OC3PE);

  // Select Ouput polarity: 0 = active high, 1 = active low
  TIM4->CCER &= ~(TIM_CCER_CC3P);

  // Compare/Caputre output Complementary Polarity
  // Must be kept at reset for channel if configured as output
  TIM4->CCER &=~(TIM_CCER_CC3NP);

  // Enable Output for channel 4
  TIM4->CCER |= TIM_CCER_CC3E;

  // ------------------Channel 4 Setup ----------------------------------

  // Disable Input/Output for Channel 4
  // This must be disable in order to set the channel as
  // Input or Output
  TIM4->CCER &= ~TIM_CCER_CC4E;

  // Set Channel 4 as output channel
  TIM4->CCMR2 &= ~(TIM_CCMR2_CC4S);

  // Set the first value to compare against
  TIM4->CCR4=0;

  // Clear Output compare mode bits for channel 1
  TIM4->CCMR2 &= ~TIM_CCMR2_OC4M;

  // Select Pulse Width Modulation Mode 1
  TIM4->CCMR2 |= (TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4M_1);

  // Select Preload Enable to be enable for PWM, allow to update CCR4 register
  // to be updated at overflow/underflow events
  TIM4->CCMR2 &=~(TIM_CCMR2_OC4PE);
  TIM4->CCMR2 |= (TIM_CCMR2_OC4PE);

  // Select Ouput polarity: 0 = active high, 1 = active low
  TIM4->CCER &= ~(TIM_CCER_CC4P);

  // Compare/Caputre output Complementary Polarity
  // Must be kept at reset for channel if configured as output
  TIM4->CCER &=~(TIM_CCER_CC4NP);

  // Enable Output for channel 4
  TIM4->CCER |= TIM_CCER_CC4E;

  // --------------------------------------------------------------

  // Enable Update Generation
  TIM4->EGR &= ~TIM_EGR_UG;
  TIM4->EGR |= TIM_EGR_UG;

  // Center Align Mode Selection
  TIM4->CR1 &=~(TIM_CR1_CMS);



  // Enable Timer 4 after all of the initialization
  TIM4->CR1 |= TIM_CR1_CEN;

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
  GPIOD->MODER &=~(GPIO_MODER_MODE15|GPIO_MODER_MODE14
                  |GPIO_MODER_MODE12|GPIO_MODER_MODE13);
  GPIOD->MODER |=(GPIO_MODER_MODE13_0|GPIO_MODER_MODE12_0);
  //                GPIO_MODER_MODE12_0|GPIO_MODER_MODE13_0); //output
  GPIOD->MODER |=(GPIO_MODER_MODE15_1 | GPIO_MODER_MODE14_1);
                 // GPIO_MODER_MODE13_1 | GPIO_MODER_MODE12_1); //alternate function

  GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL15|GPIO_AFRH_AFSEL14);
  GPIOD->AFR[1] |= (GPIO_AFRH_AFSEL15_1|GPIO_AFRH_AFSEL14_1);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT15|GPIO_OTYPER_OT14
                    |GPIO_OTYPER_OT13|GPIO_OTYPER_OT12);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED15|GPIO_OSPEEDR_OSPEED14
                    |GPIO_OSPEEDR_OSPEED13|GPIO_OSPEEDR_OSPEED12); /* Configure as high speed */
  GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED15|GPIO_OSPEEDR_OSPEED14
                    |GPIO_OSPEEDR_OSPEED13|GPIO_OSPEEDR_OSPEED12); /* Configure as high speed */

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD15|GPIO_PUPDR_PUPD14 /*no pul-up, no pull-down*/
                    |GPIO_PUPDR_PUPD13|GPIO_PUPDR_PUPD12);

  // Generate and interrupt every 1ms
  // http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html
  // If the clock is at 168MHz, then that is 168 000 000 ticks per second
  // but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead
  // you can generate an interupt every 1ms so that would be 168 000 ticks per
  // ms and you can fit 168 000 ticks into the LOAD register
  SysTick_Init(SystemCoreClock/1000);

  timer_init();

  const int max_brightness = TIM4->ARR;
  while(1){
    GPIOD->ODR ^=PORTD_12;
    GPIOD->ODR ^=PORTD_13;
   // GPIOD->ODR ^=PORTD_14;
   // GPIOD->ODR ^=PORTD_15;
    //Delay(500);
    int i=0;
    for(;i<max_brightness;i+=2){
      TIM4->CCR3=i;
      TIM4->CCR4=i;
      Delay(5);
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
