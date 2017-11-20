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
#include "stdio.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t TimeDelay;

/* Private function prototypes -----------------------------------------------*/

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
  //Disable STDOUT buffering. Otherwise nothing will be printed before
  //a newline character or when the buffer is flushed. This MUST be done
  //before any writes to STDOUT to have any effect...
  setbuf(stdout, NULL);

  /* Enable Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* for external  button 1*/

  // Set mode of all pins as digital output
  // 00 = digital input       01 = digital output
  // 10 = alternate function  11 = analog (default)
  GPIOD->MODER &= ~(0xFF<<24); /* Clear mode bits */
  GPIOD->MODER |= 85UL<<24;/* LED 5-8 are on GPIOD Pins 12-15 */

  // Set output type of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~(0xF<<12); /*Configure as output open-drain */

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(0xFF<<24); /* Configure as high speed */
  GPIOD->OSPEEDR |= (0xFF<<24);

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(0xFF<<24); /*no pul-up, no pull-down*/

  /* Setup the Pin for GPIOA - Push button */
  GPIOA->MODER &= ~0x3UL; /* Input Mode */
  GPIOA->OTYPER |= 0x00000; /*Configure as output as default push-pull*/
  GPIOA->OSPEEDR |=0x00000002; /*Configure as high speed*/
  GPIOA->PUPDR &= ~0x3UL; /*Configure as no pullup or no pulldown*/

  // Generate interupt every 1ms
  // Since clock is 1068 MHz, then 168MHz/1000 is
  // 168000 clock ticks inorder to generate an interupt every 1ms
  SysTick_Init(SystemCoreClock/1000);

  /* Infinite loop */
  while (1)
  {
    GPIOD->ODR |=PORTD_12;
    printf("PORT12 is ON\n");
    Delay(1000);
    GPIOD->ODR &=~PORTD_12;
    Delay(1000);
    GPIOD->ODR |=PORTD_13;
    Delay(1000);
    GPIOD->ODR &=~PORTD_13;
    Delay(1000);
    GPIOD->ODR |=PORTD_14;
    Delay(1000);
    GPIOD->ODR &=~PORTD_14;
    Delay(1000);
    GPIOD->ODR |=PORTD_15;
    Delay(1000);
    GPIOD->ODR &=~PORTD_15;
    Delay(1000);
    GPIOD->ODR |=PORTD_ALL;

    Delay(1000);
    GPIOD->ODR &=~PORTD_ALL;
    Delay(1000);

    printf("Hello world\r\n");
    printf("Press return to continue\r\n");

    //Wait for a return key press in the OpenOCD/GDB server window
    getchar();

    printf("Goodbye world\r\n");

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
