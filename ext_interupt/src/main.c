/**
  ******************************************************************************
  * @file    main.c
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    31-December-2017
  * @brief   Main program body
  ******************************************************************************

*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PORTD_15 0x00008000
#define PORTD_14 0x00004000
#define PORTD_13 0x00002000
#define PORTD_12 0x00001000
#define PORTD_ALL 0x0000F000


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t TimeDelay;

/* Private function prototypes -----------------------------------------------*/

void EXTI_Init(void){

// Enable SYSCFG clock
RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

// Select PC11 as the trigger source of EXTI 11
SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI11; //clear EXTI 11
SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PC; // set EXTI11 to map ext interrupt
                                               //  to PC11
// Enable rising edge trigger for EXTI 11
// Rising edge trigger selection register (RSTR)
// 0 = disable  1 = enable
EXTI->RTSR &= ~EXTI_RTSR_TR11;
EXTI->RTSR |= EXTI_RTSR_TR11;

// Disable Falling edge trigger EXTI 11
// Falling trigger selection
// 0 = disable  1 = enable
EXTI->FTSR &= ~EXTI_FTSR_TR11;

// Enable EXTI 11 Interrupt
// Interrupt mask register: 0 = masked, 1 = unmasked
// "Masked" means that processor ignores corresponding interupt
EXTI->IMR &= ~EXTI_IMR_MR11;
EXTI->IMR |= EXTI_IMR_MR11;


// Set EXTI 11 priority to 1
NVIC_SetPriority(EXTI15_10_IRQn, 1);

// Enable EXT 11 interrupt
NVIC_EnableIRQ(EXTI15_10_IRQn);


}

void EXTI15_10_IRQHandler(void){

  // Check for EXTI 11 flag
  if((EXTI->PR & EXTI_PR_PR11_Msk) == EXTI_PR_PR11){

    // Toggle all LED on board
    GPIOD->ODR ^= PORTD_ALL;

    // Clear interupt pending request
    EXTI->PR |= EXTI_PR_PR11;
  }
}

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
  //Disable STDOUT buffering. Otherwise nothing will be printed before
  //a newline character or when the buffer is flushed. This MUST be done
  //before any writes to STDOUT to have any effect...
  setbuf(stdout, NULL);

  /* Enable Clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; /* for external  button 1*/

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

  /* Setup the Pin for GPIOC - Push button */
  GPIOC->MODER &=~GPIO_MODER_MODE11; /* Input Mode */
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT11; /*Configure as output as default push-pull*/
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED11; /*Configure as high speed*/
  GPIOC->OSPEEDR |=GPIO_OSPEEDR_OSPEED11_1; /*Configure as high speed*/
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD11; /*Configure as no pullup or no pulldown*/
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD11_1; /*Configure as pulldown*/

  EXTI_Init();
  /* Infinite loop */
  while (1)
  {

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
