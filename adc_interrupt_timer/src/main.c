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
#include "usart.h"

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
uint8_t counter2 =0;
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

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  // if input is smaller/bigger than expected return the min/max out ranges value
  if (x < in_min)
    return out_min;
  else if (x > in_max)
    return out_max;

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // map the input to the output range.
  // round up if mapping bigger ranges to smaller ranges
  //else  if ((in_max - in_min) > (out_max - out_min))
   // return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
  // round down if mapping smaller ranges to bigger ranges
  //else
   // return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float adc_steps_per_volt(const uint16_t vref_value) {
 return 3.0*(*vref_cal/(float)vref_value);
}

void ADC_IRQHandler(void){
if((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC){
  	/* acknowledge interrupt */
    GPIOD->ODR ^=PORTD_13;
		uint16_t value;

		temp_value = ADC1->DR;
		counter=0;
  	//ADC1->SR &= ~(ADC_SR_EOC);
	}

}
void TIM4_IRQHandler(void){

  // Check whether an overflow event has taken place
  if((TIM4->SR & TIM_SR_CC2IF) != 0){
    TIM4->SR &= ~TIM_SR_CC2IF;
  }
  /*if((TIM4->SR & TIM_SR_CC3IF) != 0){
    TIM4->SR &= ~TIM_SR_CC3IF;
  }
  if((TIM4->SR & TIM_SR_CC4IF) != 0){
    TIM4->SR &= ~TIM_SR_CC4IF;
  }*/
  // Check whether an overflow event has taken place
  /*if((TIM4->SR & TIM_SR_UIF) != 0){
    TIM4->SR &= ~TIM_SR_UIF;
  }*/

}
void timer4_init(void){

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
  //TIM4->PSC = 62499;

  //Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0)
  TIM4->ARR = .01 * 100000-1;
  //TIM4->ARR = 1343;

  // ------------------Channel 2 Setup ----------------------------------

  // Disable Input/Output for Channel 3
  // This must be disable in order to set the channel as
  // Input or Output
  //TIM4->CCER &= ~TIM_CCER_CC2E;

  // Set Channel 4 as output channel
  //TIM4->CCMR1 &= ~(TIM_CCMR1_CC2S);

  // Set the first value to compare against
  // 50% duty cycle
  //TIM4->CCR3=.5*TIM4->ARR;
  //TIM4->CCR2 = TIM4->ARR;

  // Clear Output compare mode bits for channel 3
  //TIM4->CCMR1 &= ~TIM_CCMR1_OC2M;

  // Select Pulse Width Modulation Mode 1
  //TIM4->CCMR1 |= (TIM_CCMR1_OC2M_1);

  // Select Preload Enable to be enable for PWM, allow to update CCR4 register
  // to be updated at overflow/underflow events
  //TIM4->CCMR1 &=~(TIM_CCMR1_OC2PE);

  // Select Ouput polarity: 0 = active high, 1 = active low
  //TIM4->CCER &= ~(TIM_CCER_CC2P);

  // Compare/Caputre output Complementary Polarity
  // Must be kept at reset for channel if configured as output
  //TIM4->CCER &=~(TIM_CCER_CC2NP);

  // Enable Output for channel 4
  //TIM4->CCER |= TIM_CCER_CC2E;


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

  //Clear interrupt status only on channel 3 and 4

  TIM4->SR &= ~(TIM_SR_CC2IF);

  //Enable interrupts only on channel 3 and 4

  //TIM4->DIER |= (TIM_DIER_CC2IE);


  // Set TIM4 priority to 1
  //NVIC_SetPriority(TIM4_IRQn,3);

  // Enable TIM4 interrupt
  //NVIC_EnableIRQ(TIM4_IRQn);


  // Enable Timer 4 after all of the initialization
  TIM4->CR1 |= TIM_CR1_CEN;

}
void DMA2_Stream0_IRQHandler(void)
{
	/* transmission complete interrupt */
	if (DMA2->LISR & DMA_LISR_TCIF0)
	{
    //GPIOD->ODR ^=(PORTD_12);
		DMA2->LIFCR |= (DMA_LIFCR_CTCIF0);  // acknowledge interrupt

    uint16_t *p;
	  if ((DMA2_Stream0->CR & DMA_SxCR_CT) == 0)  // current target buffer 0 (read buffer 1)
		  p = (uint16_t*) &sample_buffer0[0];
	  else                                        // current target buffer 1 (read buffer 0)
		  p = (uint16_t*) &sample_buffer1[0];

    //temp_value = p[0];
    //vref_value = p[1];
    thermistor_value = p[0];
    counter = 1;
	}

  if (DMA2->LISR & DMA_LISR_TEIF0)
	{
    //GPIOD->ODR ^=(PORTD_13);
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
// ADC123_COMMON->CCR &= ~(ADC_CCR_ADCPRE);  // Clear
// ADC123_COMMON->CCR |= (ADC_CCR_ADCPRE_0); // DIV2
// //ADC123_COMMON->CCR |= (ADC_CCR_ADCPRE); // DIV4
//
//
// // Disable DMA for Dual/Triple modes
// ADC123_COMMON->CCR &= ~(ADC_CCR_DMA);
//
// //Configurable delay between conversions in Dual/Triple interleaved mode
// ADC123_COMMON->CCR &= ~(ADC_CCR_DELAY);
//
// // Resolution ot 12-bits
// ADCx->CR1 &= ~(ADC_CR1_RES);
//
// // Scan Mode for this example
// ADCx->CR1 &= ~(ADC_CR1_SCAN);
//
// // Disable Continuos Mode
// ADCx->CR2 &= ~(ADC_CR2_CONT);
//
// // External Trigger on rising edge
// ADCx->CR2 &= ~(ADC_CR2_EXTEN);
// ADCx->CR2 |= ADC_CR2_EXTEN_0;
//
// // Timer 3 TRGO to drive ADC conversion
// ADCx->CR2 &= ~ADC_CR2_EXTSEL;
// ADCx->CR2 |= ADC_CR2_EXTSEL_3;
//
// // Data Alignment
// ADCx->CR2 &= ~(ADC_CR2_ALIGN);
//
// // Number of Conversions
// ADCx->SQR1 &= ~(ADC_SQR1_L);
// // 3 conversion, this is offset by 1.
// //ADCx->SQR1 |= (ADC_SQR1_L_1);
//
// // Enable Temperature/Vref
// ADC123_COMMON->CCR |=ADC_CCR_TSVREFE;
//
//
// /* Configure Channel For Temp Sensor */
// ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
// // Channel 16 for temp sensor on stm32f4 disc
// ADCx->SQR3 |= ADC_SQR3_SQ1_4;
// // Sample Time is 480 cycles
// //ADCx->SMPR1 |= ADC_SMPR1_SMP16;
// ADCx->SMPR1 &= ~(ADC_SMPR1_SMP16);
// /* Configure Channel For Vref*/
// //ADCx->SQR3 &= ~(ADC_SQR3_SQ2);
// // Channel 17 for vref on stm32f4 disc
// //ADCx->SQR3 |= (ADC_SQR3_SQ2_4|ADC_SQR3_SQ2_0);
// // Sample Time is 480 cycles
// //ADCx->SMPR1 |= ADC_SMPR1_SMP17;
//
// /* Configure Channel For requested channel */
// //ADCx->SQR3 &= ~(ADC_SQR3_SQ3);
// // PC1 is connected to ADC channel 11
// //ADCx->SQR3 |= (ADC_SQR3_SQ3_3|ADC_SQR3_SQ3_1|ADC_SQR3_SQ3_0);
// // Sample Time is 480 cycles
// //ADCx->SMPR1 |= ADC_SMPR1_SMP11;
//
//
// // This call enables the end-of-conversion flag after each channel,
// // which triggers the end-of-conversion interrupt every time this flag is set.
// //ADCx->CR2 &= ~(ADC_CR2_EOCS);
// ADCx->CR2 |= (ADC_CR2_EOCS);
//
// // Enable Regular channel Interrupt
// ADCx->CR1 |= ADC_CR1_EOCIE;
//
// // For Double-Circular mode for DMA
// // you can continue to generate requests
// //ADCx->CR2 |= ADC_CR2_DDS;
//
// // Enable DMA mode for ADC
// //ADCx->CR2 |= ADC_CR2_DMA;
//
//
// // Set ADCx priority to 1
// NVIC_SetPriority(ADC_IRQn,1);
//
// // Enable ADCx interrupt
// NVIC_EnableIRQ(ADC_IRQn);
//
//
// // Turn on the ADC
// ADCx->CR2 |= ADC_CR2_ADON;
//
// /* Enable the selected ADC conversion for regular group */
// //ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;

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
 ADCx->CR2 |= ADC_CR2_EXTEN_0;

 // Timer 3 TRGO to drive ADC conversion
 ADCx->CR2 &= ~ADC_CR2_EXTSEL;
 ADCx->CR2 |= ADC_CR2_EXTSEL_3;


  // Data Alignment
  ADCx->CR2 &= ~(ADC_CR2_ALIGN);

  // Number of Conversions
  ADCx->SQR1 &= ~(ADC_SQR1_L);
  //ADCx->SQR1 |= (ADC_SQR1_L_0); // 2 conversion


  // Enable Temperature/Vref
  ADC123_COMMON->CCR |=ADC_CCR_TSVREFE;


  /* Configure Channel For Temp Sensor */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
  // Channel 16 for temp sensor on stm32f4 disc
  ADCx->SQR3 |= ADC_SQR3_SQ1_4;
  // Sample Time is 480 cycles
	ADCx->SMPR1 &= ~(ADC_SMPR1_SMP16);
  //ADCx->SMPR1 |= ADC_SMPR1_SMP16;

  /* Configure Channel For Vref*/
  //ADCx->SQR3 &= ~(ADC_SQR3_SQ2);
  // Channel 17 for vref on stm32f4 disc
  //ADCx->SQR3 |= (ADC_SQR3_SQ2_4|ADC_SQR3_SQ2_0);
  // Sample Time is 480 cycles
  //ADCx->SMPR1 |= ADC_SMPR1_SMP17;

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

void timer_init(void){

  // Enable Timer 3 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Disable Timer 3
  TIM3->CR1 &= ~TIM_CR1_CEN;

  // Counting Direction: 0 = up-counting, 1 = down-counting
  TIM3->CR1 &=~(TIM_CR1_DIR);

  // Clock Division - same as input clock
  TIM3->CR1 &=~(TIM_CR1_CKD);

  //Clock Prescaler
  //uint32_t TIM3COUNTER_Frequency = 100000; //Desired Frequency
  TIM3->PSC =420;// (84000000/TIM3COUNTER_Frequency)-1;

  // Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0)
  // In PWM, the ARR controls the period
  //uint32_t PWM_Freq = 10000;
  TIM3->ARR = 49;//(TIM3COUNTER_Frequency/PWM_Freq)-1;

  // ------------------Channel 3 Setup ----------------------------------

  // Disable Input/Output for Channel 3
  // This must be disable in order to set the channel as
  // Input or Output
  TIM3->CCER &= ~TIM_CCER_CC3E;

  // Set Channel 3 as output channel
//  TIM3->CCMR2 &= ~(TIM_CCMR2_CC3S);

  // In PWM, when you fix ARR, CCR3 controls the duty cycle
  // Set the first value to compare against
 // TIM3->CCR3=1;//(TIM3->ARR+1)/2;

  // Clear Output compare mode bits for channel 3
 // TIM3->CCMR2 &= ~TIM_CCMR2_OC3M;

  // Select PWM Mode 1
  //TIM3->CCMR2 |= (TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2);
 // TIM3->CCMR2 |= (TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2);

  // Select Preload Enable to be disable, allow to update CCR4 register
  // to be updated at anytime
//  TIM3->CCMR2 &=~(TIM_CCMR2_OC3PE);

  // Select Ouput polarity: 0 = active high, 1 = active low
//  TIM3->CCER &= ~(TIM_CCER_CC3P);

  // Compare/Caputre output Complementary Polarity
  // Must be kept at reset for channel if configured as output
//  TIM3->CCER &=~(TIM_CCER_CC3NP);

  // Master Mode Selection
  // Use OC3REF as the trigger output (TRGO)
  TIM3->CR2 &= ~(TIM_CR2_MMS);
  TIM3->CR2 |= (TIM_CR2_MMS_1);
  //TIM3->CR2 |= (TIM_CR2_MMS_2|TIM_CR2_MMS_1);

  // Enable Output for channel 3
  //TIM3->CCER |= TIM_CCER_CC3E;

  // --------------------------------------------------------------
  //Clear interrupt status only on channel 3 and 4
  TIM3->SR &= ~(TIM_SR_CC3IF | TIM_SR_CC4IF);

  //Disable interrupts only on channel 3 and 4
  TIM3->DIER &= ~(TIM_DIER_CC3IE);

  // Enable Timer 3 after all of the initialization
  TIM3->CR1 |= TIM_CR1_CEN;

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
  DMAx->NDTR |= (DMA_SxNDT_0); //3
  //DMAx->NDTR |= (DMA_SxNDT_0|DMA_SxNDT_1); //3

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
                  | GPIO_MODER_MODE10 | GPIO_MODER_MODE9
                  | GPIO_MODER_MODE8  | GPIO_MODER_MODE7
                  | GPIO_MODER_MODE6  | GPIO_MODER_MODE4
                  | GPIO_MODER_MODE3  | GPIO_MODER_MODE2
                  | GPIO_MODER_MODE1  | GPIO_MODER_MODE0);

  GPIOD->MODER |= //( GPIO_MODER_MODE15_0 |  GPIO_MODER_MODE14_0
                  //| GPIO_MODER_MODE13_0 |  GPIO_MODER_MODE12_0
                  ( GPIO_MODER_MODE13_0 |  GPIO_MODER_MODE12_0
                  | GPIO_MODER_MODE10_0 | GPIO_MODER_MODE9_0
                  | GPIO_MODER_MODE8_0  | GPIO_MODER_MODE7_0
                  | GPIO_MODER_MODE6_0  |  GPIO_MODER_MODE4_0
                  | GPIO_MODER_MODE3_0  |  GPIO_MODER_MODE2_0 //output
									| GPIO_MODER_MODE1_0 | GPIO_MODER_MODE0_0); //output

	  // Temp sensor
  GPIOC->MODER &=~(GPIO_MODER_MODE1);
  GPIOC->MODER |= (GPIO_MODER_MODE1_0|GPIO_MODER_MODE1_1);

  GPIOD->MODER |=(GPIO_MODER_MODE15_1 | GPIO_MODER_MODE14_1);

    GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL15|GPIO_AFRH_AFSEL14);
GPIOD->AFR[1] |= (GPIO_AFRH_AFSEL15_1|GPIO_AFRH_AFSEL14_1);

  // Set output tupe of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOD->OTYPER &= ~( GPIO_OTYPER_OT15 | GPIO_OTYPER_OT14
                    | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT12
                    | GPIO_OTYPER_OT10 | GPIO_OTYPER_OT9
                    | GPIO_OTYPER_OT8  | GPIO_OTYPER_OT7
                    | GPIO_OTYPER_OT6  | GPIO_OTYPER_OT4
                    | GPIO_OTYPER_OT3  | GPIO_OTYPER_OT2
                    | GPIO_OTYPER_OT1  | GPIO_OTYPER_OT0);

  GPIOC->OTYPER &= ~( GPIO_OTYPER_OT1);

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOD->OSPEEDR &=~( GPIO_OSPEEDR_OSPEED15 | GPIO_OSPEEDR_OSPEED14
                    | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED12
                    | GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED9
                    | GPIO_OSPEEDR_OSPEED8  | GPIO_OSPEEDR_OSPEED7
                    | GPIO_OSPEEDR_OSPEED6  | GPIO_OSPEEDR_OSPEED4
                    | GPIO_OSPEEDR_OSPEED3  | GPIO_OSPEEDR_OSPEED2
                    | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED0); /* Configure as high speed */

  GPIOD->OSPEEDR |= ( GPIO_OSPEEDR_OSPEED15 | GPIO_OSPEEDR_OSPEED14
                    | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED12
                    | GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED9
                    | GPIO_OSPEEDR_OSPEED8  | GPIO_OSPEEDR_OSPEED7
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
                  | GPIO_PUPDR_PUPD10  | GPIO_PUPDR_PUPD9 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD8  | GPIO_PUPDR_PUPD7 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD6  | GPIO_PUPDR_PUPD4 /*no pul-up, no pull-down*/
                  | GPIO_PUPDR_PUPD3  | GPIO_PUPDR_PUPD2
                  | GPIO_PUPDR_PUPD1  | GPIO_PUPDR_PUPD0);

  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1);

    // Enable GPIO clock and configure the Tx pin and the Rx pin as
  // Alternating functions, High Speed, Push-pull, Pull-up

  // GPIO Initialization for USART 2
  // PA2 (Tx) PA3 (Rx)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // Set mode of all pins as digital output
  // 00 = digital input       01 = digital output
  // 10 = alternate function  11 = analog (default)
  GPIOA->MODER &= ~(0xF<<4); /* Clear mode bits */
  GPIOA->MODER |= 0xA<<4;/* PA2 and PA3  are on GPIOA */

  // Alternate Function 7 = USART 2
  GPIOA->AFR[0] = (0x77 << 8); //Set alternate function for Pins 2 and 3

  // Set output type of all pins as push-pull
  // 0 = push-pull (default)
  // 1 = open-drain
  GPIOA->OTYPER &= ~(0x3<<2); /*Configure as output open-drain */

  // Set output speed of all pins as high
  // 00 = Low speed           01 = Medium speed
  // 10 = Fast speed          11 = High speed
  GPIOA->OSPEEDR &=~(0xF<<4); /* Configure as high speed */
  GPIOA->OSPEEDR |= (0xF<<4);

  // Set all pins as no pull-up, no pull-down
  // 00 = no pull-up, no pull-down    01 = pull-up
  // 10 = pull-down,                  11 = reserved
  GPIOA->PUPDR &= ~(0xF<<4); /*no pul-up, no pull-down*/
GPIOA->PUPDR |= (0x5<<4); /*no pul-up, no pull-down*/



    // Generate and interrupt every 1ms
  // http://www.electronics-homemade.com/STM32F4-LED-Toggle-Systick.html
  // If the clock is at 168MHz, then that is 168 000 000 ticks per second
  // but the LOAD register is only 24-bit so you can't fit 168 000 000. Instead
  // you can generate an interupt every 1ms so that would be 168 000 ticks per
  // ms and you can fit 168 000 ticks into the LOAD register
  SysTick_Init(SystemCoreClock/1000);
   LCD rgb_lcd;
  LCD_init(&rgb_lcd,GPIOD,0,0,1,7,8,9,10,2,3,4,6,4,20,LCD_8BITMODE,LCD_5x8DOTS);
  LCD_setRowOffsets(&rgb_lcd,0x00,0x40,0x14,0x54);
  LCD_clear(&rgb_lcd);
  //GPIOD->ODR |=PORTD_15;
  LCD_print(&rgb_lcd, "I AM HERE");
  Delay(1000);
  ADCx_Init(ADC1);

  LCD_clear(&rgb_lcd);
  LCD_print(&rgb_lcd, "I AM GROOT");
  Delay(1000);
  //timer4_init();
  //DMAx_Init(DMA2_Stream0,ADC1);

  LCD_clear(&rgb_lcd);
  timer_init();
  LCD_print(&rgb_lcd, "I AM HERE AGAIN LOLOL");
  Delay(1000);
  USARTx_Init(USART2);

   LCD_clear(&rgb_lcd);
  //LCD_home(&rgb_lcd);
  LCD_setCursor(&rgb_lcd, 0,0);
  LCD_noCursor(&rgb_lcd);
  LCD_noBlink(&rgb_lcd);
  float setpoint = 3071;
  //1float kp = 1;
  while(1){

    while(counter == 0); // Wait till conversion is done
    counter = 1;
    //float temp = adc_value_to_temp(temp_value);
    //float vref = adc_steps_per_volt(vref_value);
    //float thermistor_res =10000/((4095.0/thermistor_value)-1.0);

    //LCD_print(&rgb_lcd, "ADC: %d %4.2f\xDF%c", temp_value, temp, 'C');
    //Delay(200);
    LCD_setCursor(&rgb_lcd, 0,1);
    //LCD_print(&rgb_lcd,"VREF: %d %4.2fV", vref_value,vref);
    //Delay(200);
    //LCD_setCursor(&rgb_lcd, 0,2);
    LCD_print(&rgb_lcd,"THERM: %4d", thermistor_value);
    Delay(200);
    LCD_setCursor(&rgb_lcd, 0,2);
    USART_print(USART2,"VALUE: %4d\r\n",thermistor_value);
    LCD_print(&rgb_lcd,"THERM: %4d", thermistor_value);
    //LCD_setCursor(&rgb_lcd, 0,3);
   /* float steinhart;
    steinhart = thermistor_res / 10000;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= 3522;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (25 + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    LCD_print(&rgb_lcd,"TTEMP: %4.2f", steinhart);*/
    float error = setpoint-thermistor_value;
    USART_print(USART2,"ERROR: %4.2f VALUE: %d\r\n",error,thermistor_value);
   // float pTerm = kp * error;
 //   if(pTerm > 4095) pTerm = 4095;
 //   else if (pTerm < 0) pTerm = 0;
   // long new_pTerm = map(pTerm,0,4095,0,999);

    //long new_pTerm = map((4095-thermistor_value),0,4095,0,999);
   // USART_print(USART2,"PTERM: %4d VALUE: %4d ERROR: \r\n",
    //                  pTerm, thermistor_value, error);

    //TIM4->CCR3=thermistor_value;
    TIM4->CCR4=thermistor_value;
    LCD_home(&rgb_lcd);
    //  Delay(100);
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
