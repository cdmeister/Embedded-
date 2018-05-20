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
volatile uint32_t counter_toggle;
const uint32_t one_sec =50;


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void TIM4_IRQHandler(void){

  // Check whether an overflow event has taken place
  if((TIM4->SR & TIM_SR_CC2IF) != 0){
    if(counter_toggle == one_sec){
      GPIOD->ODR ^=(PORTD_12);
      counter_toggle =0;
    }
    else{
      counter_toggle++;
    }
    TIM4->SR &= ~TIM_SR_CC2IF;
  }
  if((TIM4->SR & TIM_SR_CC4IF) != 0){
    GPIOD->ODR ^=(PORTD_13);
    if(TIM4->CCR4==TIM4->ARR)
      TIM4->CCR4 =0;
    else
      TIM4->CCR4+=1;

    TIM4->SR &= ~TIM_SR_CC4IF;
  }
  // Check whether an overflow event has taken place
  /*if((TIM4->SR & TIM_SR_UIF) != 0){

    TIM4->SR &= ~TIM_SR_UIF;

  }*/

}

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

uint16_t adc_steps_per_volt(const uint16_t vref_value) {
 return (vref_value * 10) / 12; /* assume 1.2V internal voltage */
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

  // Delay (only used in double or triple mode
  ADC123_COMMON->CCR &= ~(ADC_CCR_DELAY);

  // Resolution ot 12-bits
  ADCx->CR1 &= ~(ADC_CR1_RES);

  // Scan Mode
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
  ADCx->SQR1 &= ~(ADC_SQR1_L); // 1 conversion


     // Enable Temperature/Vref
  ADC123_COMMON->CCR |=ADC_CCR_TSVREFE;

  // Turn on the ADC
  ADCx->CR2 |= ADC_CR2_ADON;


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

uint16_t adc_read_temp(ADC_TypeDef* ADCx) {

  /* Configure Channel For Temp Sensor */
  ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
  ADCx->SQR3 |= ADC_SQR3_SQ1_4; // Channel 16 for temp sensor on stm32f4 disc
  // Sample Time is 480 cycles
  ADCx->SMPR1 |= ADC_SMPR1_SMP16;


  /* Enable the selected ADC conversion for regular group */
  ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;

  /* wait for end of conversion */
  while((ADCx->SR & ADC_SR_EOC) == 0);

 return (uint16_t) ADCx->DR ;
}

uint16_t adc_read_vref(ADC_TypeDef* ADCx) {
  /* Configure Channel For Vref*/
  ADCx->SQR3 &= ~(ADC_SQR3_SQ1);
  // Channel 17 for vref on stm32f4 disc
  ADCx->SQR3 |= (ADC_SQR3_SQ1_4|ADC_SQR3_SQ1_0);
  // Sample Time is 480 cycles
  ADCx->SMPR1 |= ADC_SMPR1_SMP17;



  /* Enable the selected ADC conversion for regular group */
  ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;

  /* wait for end of conversion */
  while((ADCx->SR & ADC_SR_EOC) == 0);

 return (uint16_t) ADCx->DR ;
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
  //TIM4->PSC = 62499;

  //Auto Reload: up-counting (0-> ARR), down-counting (ARR -> 0)
  TIM4->ARR = (.02*TIM4COUNTER_Frequency)-1;
  //TIM4->ARR = 1343;

  // ------------------Channel 2 Setup ----------------------------------

  // Disable Input/Output for Channel 3
  // This must be disable in order to set the channel as
  // Input or Output
  TIM4->CCER &= ~TIM_CCER_CC2E;

  // Set Channel 4 as output channel
  TIM4->CCMR1 &= ~(TIM_CCMR1_CC2S);

  // Set the first value to compare against
  // 50% duty cycle
  //TIM4->CCR3=.5*TIM4->ARR;
  TIM4->CCR2 = TIM4->ARR;

  // Clear Output compare mode bits for channel 2
  TIM4->CCMR1 &= ~TIM_CCMR1_OC2M;

  // Select Pulse Width Modulation Mode 1
  TIM4->CCMR1 |= (TIM_CCMR1_OC2M_1);

  // Select Preload Enable to be enable for PWM, allow to update CCR4 register
  // to be updated at overflow/underflow events
  TIM4->CCMR1 &=~(TIM_CCMR1_OC2PE);

  // Select Ouput polarity: 0 = active high, 1 = active low
  TIM4->CCER &= ~(TIM_CCER_CC2P);

  // Compare/Caputre output Complementary Polarity
  // Must be kept at reset for channel if configured as output
  TIM4->CCER &=~(TIM_CCER_CC2NP);

  // Enable Output for channel 4
  TIM4->CCER |= TIM_CCER_CC2E;


  // ------------------Channel 3 Setup ----------------------------------

  // Disable Input/Output for Channel 3
  // This must be disable in order to set the channel as
  // Input or Output
  TIM4->CCER &= ~TIM_CCER_CC3E;

  // Set Channel 4 as output channel
  TIM4->CCMR2 &= ~(TIM_CCMR2_CC3S);

  // Set the first value to compare against
  // 50% duty cycle
  TIM4->CCR3=.01*TIM4->ARR;
  //TIM4->CCR3 = 0;

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
  TIM4->CCR4 = TIM4->ARR;
  //TIM4->CCR4=0;

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

  //Clear interrupt status only on channel 2 and 4

  TIM4->SR &= ~(TIM_SR_CC2IF|TIM_SR_CC4IF);

  //Enable interrupts only on channel 2 and 4

  TIM4->DIER |= (TIM_DIER_CC2IE|TIM_DIER_CC4IE);


  // Set TIM4 priority to 1
  NVIC_SetPriority(TIM4_IRQn,3);

  // Enable TIM4 interrupt
  NVIC_EnableIRQ(TIM4_IRQn);


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

  // Enable GPIO clock and configure the Tx pin and the Rx pin as
  // Alternating functions, High Speed, Push-pull, Pull-up

  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOCEN);

 // Set mode of all pins as digital output
  // 00 = digital input         01 = digital output
  // 10 = alternate function    11 = analog (default)
  GPIOD->MODER &=~( GPIO_MODER_MODE15 | GPIO_MODER_MODE14
                  | GPIO_MODER_MODE13 | GPIO_MODER_MODE12
                  | GPIO_MODER_MODE6  | GPIO_MODER_MODE4
                  | GPIO_MODER_MODE3  | GPIO_MODER_MODE2
                  | GPIO_MODER_MODE1  | GPIO_MODER_MODE0);

  GPIOD->MODER |= ( GPIO_MODER_MODE15_1 |  GPIO_MODER_MODE14_1
                  | GPIO_MODER_MODE13_0 |  GPIO_MODER_MODE12_0
                  | GPIO_MODER_MODE6_0  |  GPIO_MODER_MODE4_0
                  | GPIO_MODER_MODE3_0  |  GPIO_MODER_MODE2_0 //output
                  | GPIO_MODER_MODE1_0  |  GPIO_MODER_MODE0_0); //output

  GPIOD->AFR[1] &= ~(GPIO_AFRH_AFSEL15|GPIO_AFRH_AFSEL14);
  GPIOD->AFR[1] |= (GPIO_AFRH_AFSEL15_1|GPIO_AFRH_AFSEL14_1);

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

  ADCx_Init(ADC1);
  timer_init();

  LCD rgb_lcd;
  LCD_init(&rgb_lcd,GPIOD,0,0,1,0,0,0,0,2,3,4,6,4,20,LCD_4BITMODE,LCD_5x8DOTS);
  LCD_setRowOffsets(&rgb_lcd,0x00,0x40,0x14,0x54);
  LCD_clear(&rgb_lcd);
  /*char * str;
  str = "Lebron is GOAT";
  while(*str !=0) LCD_write(&rgb_lcd,*str++);
  Delay(1000);

  LCD_setCursor(&rgb_lcd,0,1);
  str = "I am legend";
  while(*str !=0) LCD_write(&rgb_lcd,*str++);
  Delay(1000);
  int game = 4;
  LCD_clearRow(&rgb_lcd,1);
  LCD_print(&rgb_lcd,"Cavs in %d", game);
  Delay(1000);
  LCD_setCursor(&rgb_lcd,0,2);
  LCD_print(&rgb_lcd,"Boston is toast");
  Delay(1000);
  LCD_setCursor(&rgb_lcd,0,3);
  LCD_print(&rgb_lcd,"LeBron is my Hero");
  Delay(1000);
  LCD_clearRow(&rgb_lcd,3);
  LCD_print(&rgb_lcd, "I want the \x23%d pick",1);
  Delay(1000);
*/
  LCD_clear(&rgb_lcd);
  //LCD_home(&rgb_lcd);
  LCD_setCursor(&rgb_lcd, 0,0);
  LCD_noCursor(&rgb_lcd);
  LCD_noBlink(&rgb_lcd);

  /*float slope =-1 *((float)(110-30))/((float)(*temp_30-*temp_110));
  LCD_print(&rgb_lcd, "Slope: %3.5f", slope);
  LCD_setCursor(&rgb_lcd,0,1);
  LCD_print(&rgb_lcd, "30: %d", *temp_30);
  LCD_setCursor(&rgb_lcd,0,2);
  LCD_print(&rgb_lcd, "100: %d", *temp_110);

  Delay(5000);
  LCD_clear(&rgb_lcd);
  */
  //const int max_brightness = TIM4->ARR;
  while(1){
    uint16_t adc_value_temp = adc_read_temp(ADC1);
    float temp = adc_value_to_temp(adc_value_temp);
    LCD_print(&rgb_lcd, "ADC TEMP: %d", adc_value_temp);
    LCD_setCursor(&rgb_lcd, 0,1);
    Delay(20);
    LCD_print(&rgb_lcd,"TEMP: %4.2f\xDF%c", temp,'C');
    Delay(20);
    LCD_setCursor(&rgb_lcd, 0,2);
    uint16_t adc_pot = adc_read(ADC1);
    //float pot_volt = (3.0/4095)*adc_pot;
    //LCD_print(&rgb_lcd,"ADC: %-4d Vol: %1.3f", adc_pot, pot_volt);
    float thermistor_res =10000/((4095.0/adc_pot)-1.0);
    LCD_print(&rgb_lcd,"ADC %4d %4.2f", adc_pot,thermistor_res );
    Delay(20);
    LCD_setCursor(&rgb_lcd, 0,3);
    //uint16_t adc_vref = adc_read_vref(ADC1);
    //LCD_print(&rgb_lcd,"VREF %-4d ADC %-4d ", adc_vref, *vref_cal);
    float steinhart;
    steinhart = thermistor_res / 10000;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= 3522;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (25 + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    LCD_print(&rgb_lcd,"TEMP: %4.2f", steinhart);
    LCD_home(&rgb_lcd);

   Delay(50);
   // GPIOD->ODR ^=PORTD_14;
   // GPIOD->ODR ^=PORTD_15;
    //Delay(500);
    /*int i=0;
    for(;i<max_brightness;i+=1){
      TIM4->CCR4=i;
      TIM4->CCR3=i;
      Delay(1);
    }
    for(;i>0;i-=1){
      TIM4->CCR4=i;
      TIM4->CCR3=i;
      Delay(1);
    }*/

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
