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
#define PORTD_12 0x00001000 //Green
#define PORTD_13 0x00002000 //Orange
#define PORTD_14 0x00004000 //Red
#define PORTD_15 0x00008000 //Blue
uint16_t * temp_30 = (uint16_t *) ((uint32_t)0x1FFF7A2C);
uint16_t * temp_110 =(uint16_t *) ((uint32_t)0x1FFF7A2E);
uint16_t * vref_cal =(uint16_t *) 0x1FFF7A2A;
uint8_t counter =0;
uint16_t vref_value = 0;
uint16_t temp_value = 0;
uint16_t thermistor_value = 0;
volatile uint16_t sample_buffer0[3];
volatile uint16_t sample_buffer1[3];
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
    GPIOD->ODR ^=PORTD_15;
    ADC1->SR &= ~(ADC_SR_EOC);
  }

}

void DMA2_Stream0_IRQHandler(void)
{
	/* transmission complete interrupt */
	if (DMA2->LISR & DMA_LISR_TCIF0)
	{
    GPIOD->ODR ^=(PORTD_12);
		DMA2->LIFCR |= (DMA_LIFCR_CTCIF0);  // acknowledge interrupt

    uint16_t *p;
	  if ((DMA2_Stream0->CR & DMA_SxCR_CT) == 0)  // current target buffer 0 (read buffer 1)
		  p = (uint16_t*) &sample_buffer0[0];
	  else                                        // current target buffer 1 (read buffer 0)
		  p = (uint16_t*) &sample_buffer1[0];

    temp_value = p[0];
    vref_value = p[1];
    thermistor_value = p[2];
    counter = 1;
	}

  if (DMA2->LISR & DMA_LISR_TEIF0)
	{
    GPIOD->ODR =(PORTD_13);
		DMA2->LIFCR |= (DMA_LIFCR_CTEIF0);  // acknowledge interrupt

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
  //ADC123_COMMON->CCR |= (ADC_CCR_ADCPRE); // DIV4


  // Disable DMA for Dual/Triple modes
  ADC123_COMMON->CCR &= ~(ADC_CCR_DMA);

  //Configurable delay between conversions in Dual/Triple interleaved mode
  ADC123_COMMON->CCR &= ~(ADC_CCR_DELAY);

  // Resolution ot 12-bits
  ADCx->CR1 &= ~(ADC_CR1_RES);

  // Scan Mode for this example
  ADCx->CR1 |= ADC_CR1_SCAN;

  // Enable Continuos Mode
  ADCx->CR2 |= (ADC_CR2_CONT);

  // External Trigger on rising edge
  ADCx->CR2 &= ~(ADC_CR2_EXTEN);

  // Timer 2 Trigger to drive ADC conversion
  ADCx->CR2 &= ~ADC_CR2_EXTSEL;

  // Data Alignment
  ADCx->CR2 &= ~(ADC_CR2_ALIGN);

  // Number of Conversions
  ADCx->SQR1 &= ~(ADC_SQR1_L);
  // 3 conversion, this is offset by 1.
  ADCx->SQR1 |= (ADC_SQR1_L_1);

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

  /* Configure Channel For requested channel */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ3);
  // PC1 is connected to ADC channel 11
  ADCx->SQR3 |= (ADC_SQR3_SQ3_3|ADC_SQR3_SQ3_1|ADC_SQR3_SQ3_0);
  // Sample Time is 480 cycles
  ADCx->SMPR1 |= ADC_SMPR1_SMP11;


  // This call enables the end-of-conversion flag after each channel,
  // which triggers the end-of-conversion interrupt every time this flag is set.
  ADCx->CR2 &= ~(ADC_CR2_EOCS);

  // Enable Regular channel Interrupt
  ADCx->CR1 |= ADC_CR1_EOCIE;

  // For Double-Circular mode for DMA
  // you can continue to generate requests
  ADCx->CR2 |= ADC_CR2_DDS;

  // Enable DMA mode for ADC
  ADCx->CR2 |= ADC_CR2_DMA;


  // Set ADCx priority to 1
  NVIC_SetPriority(ADC_IRQn,1);

  // Enable ADCx interrupt
  NVIC_EnableIRQ(ADC_IRQn);


  // Turn on the ADC
  ADCx->CR2 |= ADC_CR2_ADON;

  /* Enable the selected ADC conversion for regular group */
  ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;



}

void DMAx_Init(DMA_Stream_TypeDef * DMAx, ADC_TypeDef * ADCx){

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
  DMAx->NDTR |= (DMA_SxNDT_0|DMA_SxNDT_1); //3

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
  DMAx->CR &= ~(DMA_SxCR_MBURST);

  // Periphery Burst Mode
  // In direct mode, these bits are forced to 0x0
  // by hardware as soon as bit EN= '1'.
  DMAx->CR &= ~(DMA_SxCR_PBURST);

  // Circular Buffer
  DMAx->CR |= (DMA_SxCR_CIRC);

  // Use Double buffering
  DMAx->CR |= (DMA_SxCR_DBM);

  // Set the Priority
  DMAx->CR |= (DMA_SxCR_PL); // Highest

  /* Periphery Source configuration */
  DMAx->PAR = (uint32_t)&ADCx->DR; // Source of the Data to grab
  DMAx->CR &= ~(DMA_SxCR_PSIZE);
  DMAx->CR |= (DMA_SxCR_PSIZE_0);
  // Keep the pointer incremenent constant
  DMAx->CR &= ~(DMA_SxCR_PINC);

  /* Memory Destination Configuration */
  DMAx->M0AR =(uint32_t) &sample_buffer0;
  DMAx->M1AR =(uint32_t) &sample_buffer1;
  // In direct mode, MSIZE is forced by hardware to the
  // same value as PSIZE as soon as bit EN= '1'.
  DMAx->CR &= ~(DMA_SxCR_MSIZE);
  DMAx->CR |= (DMA_SxCR_MSIZE_0);
  // Increment the pointer
  DMAx->CR |= (DMA_SxCR_MINC);

  // Set the DMA as the flow controller
  DMAx->CR &= ~(DMA_SxCR_PFCTRL);

  // Enable the DMA transfer complete interrupt
  DMAx->CR |= DMA_SxCR_TCIE;
  DMAx->CR |= DMA_SxCR_TEIE;

  // Set DMAx priority to 1
  NVIC_SetPriority(DMA2_Stream0_IRQn,1);

  // Enable DMAx interrupt
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  // Enable the DMA
  DMAx->CR  |= DMA_SxCR_EN;



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

  GPIOD->MODER &=~( GPIO_MODER_MODE15 | GPIO_MODER_MODE14
                  | GPIO_MODER_MODE13 | GPIO_MODER_MODE12
                  | GPIO_MODER_MODE6  | GPIO_MODER_MODE4
                  | GPIO_MODER_MODE3  | GPIO_MODER_MODE2
                  | GPIO_MODER_MODE1  | GPIO_MODER_MODE0);

  GPIOD->MODER |= ( GPIO_MODER_MODE15_0 |  GPIO_MODER_MODE14_0
                  | GPIO_MODER_MODE13_0 |  GPIO_MODER_MODE12_0
                  | GPIO_MODER_MODE6_0  |  GPIO_MODER_MODE4_0
                  | GPIO_MODER_MODE3_0  |  GPIO_MODER_MODE2_0 //output
									| GPIO_MODER_MODE1_0 | GPIO_MODER_MODE0_0); //output

	  // Temp sensor
  GPIOC->MODER &=~(GPIO_MODER_MODE1);
  GPIOC->MODER |= (GPIO_MODER_MODE1_0|GPIO_MODER_MODE1_1);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~( GPIO_OTYPER_OT15 | GPIO_OTYPER_OT14
                    | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT12
                    | GPIO_OTYPER_OT6  | GPIO_OTYPER_OT4
                    | GPIO_OTYPER_OT3  | GPIO_OTYPER_OT2
                    | GPIO_OTYPER_OT1  | GPIO_OTYPER_OT0);

  GPIOC->OTYPER &= ~( GPIO_OTYPER_OT1);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~( GPIO_OSPEEDR_OSPEED15 | GPIO_OSPEEDR_OSPEED14
                    | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED12
                    | GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */

  GPIOD->OSPEEDR |= ( GPIO_OSPEEDR_OSPEED15 | GPIO_OSPEEDR_OSPEED14
                    | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED12
                    | GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */


  GPIOC->OSPEEDR &=~(GPIO_OSPEEDR_OSPEED1); /* Configure as high speed */

  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1); /* Configure as high speed */


  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved

  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD15 | GPIO_PUPDR_PUPD14
                  | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD12
                  | GPIO_PUPDR_PUPD6  | GPIO_PUPDR_PUPD4 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD3  | GPIO_PUPDR_PUPD2
                  | GPIO_PUPDR_PUPD1  | GPIO_PUPDR_PUPD0);

  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1);

    // Generate and interrupt every 1ms
  // http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html
  // If the clock is at 168MHz, then that is 168 000 000 ticks per second
  // but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead
  // you can generate an interupt every 1ms so that would be 168 000 ticks per
  // ms and you can fit 168 000 ticks into the LOAD register
  SysTick_Init(SystemCoreClock/1000);
  DMAx_Init(DMA2_Stream0,ADC1);

  ADCx_Init(ADC1);

  LCD rgb_lcd;
  LCD_init(&rgb_lcd,GPIOD,0,0,1,0,0,0,0,2,3,4,6,4,20,LCD_4BITMODE,LCD_5x8DOTS);
  LCD_setRowOffsets(&rgb_lcd,0x00,0x40,0x14,0x54);
  LCD_clear(&rgb_lcd);
  LCD_print(&rgb_lcd, "I AM HERE");
  Delay(1000);

  LCD_clear(&rgb_lcd);
  //LCD_home(&rgb_lcd);
  LCD_setCursor(&rgb_lcd, 0,0);
  LCD_noCursor(&rgb_lcd);
  LCD_noBlink(&rgb_lcd);

  while(1){

    while(counter == 0); // Wait till conversion is done
    counter = 0;
    float temp = adc_value_to_temp(temp_value);
    float vref = adc_steps_per_volt(vref_value);
    float thermistor_res =10000/((4095.0/thermistor_value)-1.0);

    LCD_print(&rgb_lcd, "ADC: %d %4.2f\xDF%c", temp_value, temp, 'C');
    Delay(200);
    LCD_setCursor(&rgb_lcd, 0,1);
    LCD_print(&rgb_lcd,"VREF: %d %4.2fV", vref_value,vref);
    Delay(200);
    LCD_setCursor(&rgb_lcd, 0,2);
    LCD_print(&rgb_lcd,"THERM: %4d %4.2f", thermistor_value,thermistor_res );
    Delay(200);
    LCD_setCursor(&rgb_lcd, 0,3);
    float steinhart;
    steinhart = thermistor_res / 10000;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= 3522;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (25 + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    LCD_print(&rgb_lcd,"TTEMP: %4.2f", steinhart);

  LCD_home(&rgb_lcd);
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
