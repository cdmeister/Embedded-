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
#include "systick.h"
#include "lcd.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC_TEMPERATURE_V25       760  /* mV */
#define ADC_TEMPERATURE_AVG_SLOPE 2500 /* mV/C */
uint16_t * temp_30 = (uint16_t *) ((uint32_t)0x1FFF7A2C);
uint16_t * temp_110 =(uint16_t *) ((uint32_t)0x1FFF7A2E);
uint16_t * vref_cal =(uint16_t *) 0x1FFF7A2A;
uint8_t counter =0;
uint16_t vref_value = 0;
uint16_t temp_value = 0;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
float adc_value_to_temp(const uint16_t value) {
 /* convert reading to millivolts */
  float conv_value=value;
  conv_value *= 3;
  conv_value /= 4096; //Reading in mV
  //conv_value /= 1000.0; //Reading in Volts
  conv_value -= 0.760; // Subtract the reference voltage at 25°C
  conv_value /= .0025; // Divide by slope 2.5mV
  conv_value += 25.0; // Add the 25°C
  //conv_value -= 11.0; // Add the 25°C
return conv_value;
}

float adc_steps_per_volt(const uint16_t vref_value) {
 return 3.0*(*vref_cal/(float)vref_value);
}

void ADC_IRQHandler(void){
  if((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC){
    /* acknowledge interrupt */
    uint16_t value;

    value = ADC1->DR;
    if(counter % 2 == 0) {
      temp_value = value;
    } else {
      vref_value = value;
    }
    counter++;

  }

}



void ADCx_Init(ADC_TypeDef * ADCx){

  // Enable ADCx
  if(ADCx == ADC1) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  else if(ADCx == ADC2) RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
  else RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;


  /*
   * ADC Mode Selection
   *
   * Note:
   *  00000 : Independent Mode, ADC operate independently
   */
  ADC123_COMMON->CCR &= ~(ADC_CCR_MULTI);


  /*
   * Set and cleared by software to select the frequency of the clock
   *  to the ADC. The clock is common for all the ADCs.
   *
   * Note:
   *  00: PCLK2 divided by 2
   *  01: PCLK2 divided by 4
   *  10: PCLK2 divided by 6
   *  11: PCLK2 divided by 8
  */
  ADC123_COMMON->CCR &= ~(ADC_CCR_ADCPRE);  // Clear
  ADC123_COMMON->CCR |= (ADC_CCR_ADCPRE_0); // DIV4


  // Disable DMA
  ADC123_COMMON->CCR &= ~(ADC_CCR_DMA);

  //Configurable delay between conversions in Dual/Triple interleaved mode
  ADC123_COMMON->CCR &= ~(ADC_CCR_DELAY);

  // Resolution ot 12-bits
  ADCx->CR1 &= ~(ADC_CR1_RES);

  // Scan Mode for this example
  ADCx->CR1 |= ADC_CR1_SCAN;

  // Disable Continuos Mode
  ADCx->CR2 &= ~(ADC_CR2_CONT);

  // External Trigger on rising edge
  ADCx->CR2 &= ~(ADC_CR2_EXTEN);

  // Timer 2 Trigger to drive ADC conversion
  ADCx->CR2 &= ~ADC_CR2_EXTSEL;

  // Data Alignment
  ADCx->CR2 &= ~(ADC_CR2_ALIGN);

  // Number of Conversions
  ADCx->SQR1 &= ~(ADC_SQR1_L);
  ADCx->SQR1 |= (ADC_SQR1_L_0); // 2 conversion


  // Enable Temperature/Vref
  ADC123_COMMON->CCR |=ADC_CCR_TSVREFE;


  /* Configure Channel For Temp Sensor */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
  // Channel 16 for temp sensor on stm32f4 disc
  ADCx->SQR3 |= ADC_SQR3_SQ1_4;
  // Sample Time is 480 cycles
  ADCx->SMPR1 |= ADC_SMPR1_SMP16;

  /* Configure Channel For Vref*/
  ADCx->SQR3 &= ~(ADC_SQR3_SQ2);
  // Channel 17 for vref on stm32f4 disc
  ADCx->SQR3 |= (ADC_SQR3_SQ2_4|ADC_SQR3_SQ2_0);
  // Sample Time is 480 cycles
  ADCx->SMPR1 |= ADC_SMPR1_SMP17;

  // This call enables the end-of-conversion flag after each channel,
  // which triggers the end-of-conversion interrupt every time this flag is set.
  ADCx->CR2 |= ADC_CR2_EOCS;

  // Enable Regular channel Interrupt
  ADCx->CR1 |= ADC_CR1_EOCIE;


  // Set ADCx priority to 1
  NVIC_SetPriority(ADC_IRQn,1);

  // Enable ADCx interrupt
  NVIC_EnableIRQ(ADC_IRQn);


  // Turn on the ADC
  ADCx->CR2 |= ADC_CR2_ADON;


}

void DMAx_init(DMA_Stream_TypeDef * DMAx){

  // Enable DMA Clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  // Disable DMA2 so that we can configure it
  DMAx->CR &= ~(DMA_SxCR_EN);

  // Initialize the Channel Member
  // ADC on stream 0 channel 0 of DMA2
  DMAx->CR &= ~(DMA_SxCR_CHSEL);

  // Initialize number of transactions to perform,
  // transaction can be thought of number of sources you need to transfer
  // data from. this is decremented after each transfer.
  DMAx->NDTR &= ~(DMA_SxNDT);
  DMAx->NDTR |= (DMA_SxNDT_0|DMA_SxNDT_1);

  // Direction, Periphery to Memory
  DMAx->CR &= ~(DMA_SxCR_DIR);

  // No Fifo mode. Direct mode
  DMAx->FCR &= ~(DMA_SxFCR_DMDIS);

  // Fifo Threshold, since using direct mode, just set this to default value
  // Not used in Direct mode
  DMAx->FCR &= ~(DMA_SxFCR_FTH);
  DMAx->FCR |= ~(DMA_SxFCR_FTH_0);

  // Memory Burst Mode
  // In direct mode, these bits are forced to 0x0
  // by hardware as soon as bit EN= '1'.


}

uint16_t adc_read(ADC_TypeDef * ADCx){

  /* Configure Channel For requested channel */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
  ADCx->SQR3 |= (ADC_SQR3_SQ1_3|ADC_SQR3_SQ1_1|ADC_SQR3_SQ1_0); // Channel 16 for temp sensor on stm32f4 disc
  // Sample Time is 480 cycles
  ADCx->SMPR1 |= ADC_SMPR1_SMP11;


  /* Enable the selected ADC conversion for regular group */
  ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;

  /* wait for end of conversion */
  while((ADCx->SR & ADC_SR_EOC) == 0);

 return (uint16_t) ADCx->DR ;



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

  // Enable GPIO clock and configure the Tx pin and the Rx pin as
  // Alternating functions, High Speed, Push-pull, Pull-up

  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOCEN);

 // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  // LCD
  GPIOD->MODER &=~(GPIO_MODER_MODE6|GPIO_MODER_MODE4
                  | GPIO_MODER_MODE3|GPIO_MODER_MODE2
                  | GPIO_MODER_MODE1|GPIO_MODER_MODE0);

  GPIOD->MODER |=(GPIO_MODER_MODE6_0|GPIO_MODER_MODE4_0
                  | GPIO_MODER_MODE3_0|GPIO_MODER_MODE2_0 //output
                  | GPIO_MODER_MODE1_0|GPIO_MODER_MODE0_0); //output

  // Temp sensor
  GPIOC->MODER &=~(GPIO_MODER_MODE1);
  GPIOC->MODER |= (GPIO_MODER_MODE1_0|GPIO_MODER_MODE1_1);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~( GPIO_OTYPER_OT6 | GPIO_OTYPER_OT4
                    | GPIO_OTYPER_OT3 | GPIO_OTYPER_OT2
                    | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT0);
  GPIOC->OTYPER &= ~( GPIO_OTYPER_OT1);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED6|GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3|GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1|GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */

  GPIOD->OSPEEDR |=(GPIO_OSPEEDR_OSPEED6|GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3|GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1|GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */

  GPIOC->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED1); /* Configure as high speed */

  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1); /* Configure as high speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD6|GPIO_PUPDR_PUPD4 /*no pul-up, no pull-down*/
                    |GPIO_PUPDR_PUPD3|GPIO_PUPDR_PUPD2
                    |GPIO_PUPDR_PUPD1|GPIO_PUPDR_PUPD0);

  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1);

    // Generate and interrupt every 1ms
  // http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html
  // If the clock is at 168MHz, then that is 168 000 000 ticks per second
  // but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead
  // you can generate an interupt every 1ms so that would be 168 000 ticks per
  // ms and you can fit 168 000 ticks into the LOAD register
  SysTick_Init(SystemCoreClock/1000);

  ADCx_Init(ADC1);

  LCD rgb_lcd;
  LCD_init(&rgb_lcd,GPIOD,0,0,1,0,0,0,0,2,3,4,6,4,20,LCD_4BITMODE,LCD_5x8DOTS);
  LCD_setRowOffsets(&rgb_lcd,0x00,0x40,0x14,0x54);
  LCD_clear(&rgb_lcd);
  LCD_print(&rgb_lcd, "I AM HERE");
  Delay(5000);

  LCD_clear(&rgb_lcd);
  //LCD_home(&rgb_lcd);
  LCD_setCursor(&rgb_lcd, 0,0);
  LCD_noCursor(&rgb_lcd);
  LCD_noBlink(&rgb_lcd);

  while(1){
    /* Enable the selected ADC conversion for regular group */
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
    while(counter <= 1); // Wait till conversion is done
    counter = 0;
    float temp = adc_value_to_temp(temp_value);
    float vref = adc_steps_per_volt(vref_value);
    LCD_print(&rgb_lcd, "ADC TEMP: %d", temp_value);
    LCD_setCursor(&rgb_lcd, 0,1);
    LCD_print(&rgb_lcd,"TEMP: %4.2f\xDF%c", temp,'C');
    LCD_setCursor(&rgb_lcd, 0,2);
    LCD_print(&rgb_lcd,"VREF ADC: %d", vref_value);
    LCD_setCursor(&rgb_lcd, 0,3);
    LCD_print(&rgb_lcd,"Voltage: %4.2fV", vref);

    LCD_home(&rgb_lcd);

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
