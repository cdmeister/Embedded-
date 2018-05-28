#include "clock.h"


static uint8_t APBxPrescTable[4]={2,4,8,16};

/* Always use HSE instead of HSI for PLL
 * Use this function if HSI or HSE is on and you want to achieve different
 * clock via PLL
 */
void SystemPLLClockEnable(uint16_t ahb_prescaler, uint8_t  apb1_prescaler,
                          uint8_t apb2_prescaler, uint8_t pll_m, uint16_t pll_n,
                          uint8_t pll_p, uint8_t pll_q){


  /* Enable HSE clock */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready */
	while (!(RCC->CR & RCC_CR_HSERDY)){};

  /* Select HSE clock as main clock, so we can turn off HSI/PLL and make updates
   * for the PLL
   */
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_HSE;
  /* wait til HSE is selected as main clock */
  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE){}

  /* Turn off HSI since its no longer needed for operation */
  RCC->CR &=~(RCC_CR_HSION);

  /* Make sure PLL is off before applying updates for prescalers */
  RCC->CR &=~(RCC_CR_PLLON);

  /* Apply System Prescalers */
  SystemClockPrescaler(ahb_prescaler,apb1_prescaler,apb2_prescaler);
  SystemPLLPrescaler(pll_m,pll_n,pll_p,pll_q);

  /* Select PLL Source to be HSE */
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
  RCC->PLLCFGR |=RCC_PLLCFGR_PLLSRC_HSE;

  SetFlashWaitState(ahb_prescaler,pll_m,pll_n,pll_p);

  /* Turn the PLL ON after applying all of the settings */
  RCC->CR |= RCC_CR_PLLON;

  /* Wait till the main PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0){}

  /* Select the main PLL as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  /* Wait till the main PLL is used as system clock source */
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL){}


  SystemCoreClockUpdate();
}


//void SystemPLLClockUpdate(uint16_t ahb_prescaler, uint8_t  apb1_prescaler,
  //                    uint8_t apb2_prescaler){

  /* Read the SWS to know what is the current clock selected */
 // uint32_t current_clock = RCC->CFGR & RCC_CFGR_SWS;




//}
void SystemHSIenable(uint16_t ahb_prescaler, uint8_t  apb1_prescaler,
                      uint8_t apb2_prescaler){

  /* Read the SWS to know what is the current clock selected */
  uint32_t current_clock = RCC->CFGR & RCC_CFGR_SWS;

  /* Enable HSI clock */
	RCC->CR |= RCC_CR_HSION;

	/* Wait till HSI is ready */
	while (!(RCC->CR & RCC_CR_HSIRDY)){};

  /* Follow Procedure for increasing cpu frequency, page 81 ref manual*/
  if(current_clock == RCC_CFGR_SWS_HSE){

    /* Program the wait states */
    FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY)) |FLASH_ACR_LATENCY_0WS;

    /* Select HSI clock as main clock */
	  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_HSI;
    /* wait til HSI is selected as main clock */
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI){}

    /* Set the prescalers accordingly for HSI */
    SystemClockPrescaler(ahb_prescaler,apb1_prescaler,apb2_prescaler);

    RCC->CR &=~(RCC_CR_HSEON);

  }
  /* Follow Procedure for decreasing cpu frequency, page 81 ref manual */
  else{

    /* Select HSI clock as main clock */
	  RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_HSI;
    /* wait til HSI is selected as main clock */
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI){}

    /* Set the prescalers accordingly for HSI */
    SystemClockPrescaler(ahb_prescaler,apb1_prescaler,apb2_prescaler);

    /* Program the wait states */
    FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY)) |FLASH_ACR_LATENCY_0WS;

    RCC->CR &=~(RCC_CR_PLLON);
  }
  SystemCoreClockUpdate();
}

void SystemHSEenable(uint16_t ahb_prescaler, uint8_t  apb1_prescaler,
                      uint8_t apb2_prescaler){

  /* Read the SWS to know what is the current clock selected */
  uint32_t current_clock = RCC->CFGR & RCC_CFGR_SWS;

  /* Enable HSI clock */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait till HSE is ready */
	while (!(RCC->CR & RCC_CR_HSERDY)){};

  /* Select HSE clock as main clock */
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW)) | RCC_CFGR_SW_HSE;
  /* wait til HSE is selected as main clock */
  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE){}

  /* Set the prescalers accordingly for HSI */
  SystemClockPrescaler(ahb_prescaler,apb1_prescaler,apb2_prescaler);

  /* Program the wait states */
  FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY)) |FLASH_ACR_LATENCY_0WS;


  if(current_clock == RCC_CFGR_SWS_HSI){
    RCC->CR &=~(RCC_CR_HSION);

  }
  else{
    RCC->CR &=~(RCC_CR_PLLON);
  }
  SystemCoreClockUpdate();
}


void SystemClockPrescaler(uint16_t ahb_prescaler, uint8_t  apb1_prescaler,
                      uint8_t apb2_prescaler){

    /* AHB Clock Divider */
    RCC->CFGR &= ~(RCC_CFGR_HPRE);

    /* APB1 Clock Divider */
    RCC->CFGR &= ~(RCC_CFGR_PPRE1);

    /* APB2 Clock Divider */
    RCC->CFGR &= ~(RCC_CFGR_PPRE2);


  /* HPRE( aka AHB) prescaler setting*/
  switch ( ahb_prescaler){
    case 0:
      /* Do not change the current prescaler if 0 */
      break;
    case 1:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
      break;
    case 2:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV2;
      break;
    case 4:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV4;
      break;
    case 8:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV8;
      break;
    case 16:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV16;
      break;
    case 64:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV64;
      break;
    case 128:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV128;
      break;
    case 256:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV256;
      break;
    case 512:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV512;
      break;
    default:
      RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
      break;
  }
  /* PPRE1(APB1) prescaler setting*/
  switch (apb1_prescaler){
    case 0:
      break;
    case 1:
      RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
      break;
    case 2:
      RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
      break;
    case 4:
      RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
      break;
    case 8:
      RCC->CFGR |= RCC_CFGR_PPRE1_DIV8;
      break;
    case 16:
      RCC->CFGR |= RCC_CFGR_PPRE1_DIV16;
      break;
    default:
      RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
      break;

  }

  /* PPRE2(APB2) prescaler setting*/
  switch (apb2_prescaler){
    case 0:
      break;
    case 1:
      RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
      break;
    case 2:
      RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
      break;
    case 4:
      RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;
      break;
    case 8:
      RCC->CFGR |= RCC_CFGR_PPRE2_DIV8;
      break;
    case 16:
      RCC->CFGR |= RCC_CFGR_PPRE2_DIV16;
      break;
    default:
      RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
      break;

  }

}

void SystemPLLPrescaler(uint8_t pll_m, uint16_t pll_n,
                        uint8_t pll_p, uint8_t pll_q){

  /*Configre the multiplier and division factors to the RCC_PLLCFGR Register*/
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
  RCC->PLLCFGR |= (pll_m << RCC_PLLCFGR_PLLM_Pos); /*PLL_M*/

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
  RCC->PLLCFGR |= (pll_n << RCC_PLLCFGR_PLLN_Pos); /*PLL_N*/

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);
  RCC->PLLCFGR |= (((pll_p>> 1)-1) << RCC_PLLCFGR_PLLP_Pos); /*PLL_P*/

  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ);
  RCC->PLLCFGR |= (pll_q << RCC_PLLCFGR_PLLQ_Pos); /*PLL_Q*/

}

uint16_t GetAHBPrescaler(void){

  uint16_t ahb_prescaler = ((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos);
  if (ahb_prescaler < 0x8) return 1;
  else return ahb_prescaler;

}

uint8_t GetAPB1Prescaler(void){

  uint8_t apb1_prescaler = ((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos);
  if (apb1_prescaler < 0x4) return 1;
  else return APBxPrescTable[apb1_prescaler&0x3];

}

uint8_t GetAPB2Prescaler(void){

  uint8_t apb2_prescaler = ((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos);
  if (apb2_prescaler < 0x4) return 1;
  else return APBxPrescTable[apb2_prescaler&0x3];

}
/* HCLK is the core clock */
uint32_t GetHCLK(void){

  uint16_t hpre_prescaler = GetAHBPrescaler();
  return SystemCoreClock/hpre_prescaler;

}

uint32_t GetPCLK1(void){
  uint32_t HCLK = GetHCLK();
  uint8_t apb1_prescaler = GetAPB1Prescaler();
  return HCLK/apb1_prescaler;

}

uint32_t GetPCLK2(void){
  uint32_t HCLK = GetHCLK();
  uint8_t apb2_prescaler = GetAPB2Prescaler();
  return HCLK/apb2_prescaler;

}

