#include "flash.h"


uint32_t GetFlashWaitState(uint32_t projected_hclk){

  if (projected_hclk > 0 && projected_hclk <= 30){
    return FLASH_ACR_LATENCY_0WS;
  }
  else if (projected_hclk > 30 && projected_hclk <= 60){
    return FLASH_ACR_LATENCY_1WS;
  }
  else if (projected_hclk > 60 && projected_hclk <= 90){
    return FLASH_ACR_LATENCY_2WS;
  }
  else if (projected_hclk > 90 && projected_hclk <= 120){
    return FLASH_ACR_LATENCY_3WS;
  }
  else if (projected_hclk > 120 && projected_hclk <= 150){
    return FLASH_ACR_LATENCY_4WS;
  }
  else {//(projected_hclk > 150 && projected_hclk <= 168)
    return FLASH_ACR_LATENCY_5WS;
  }



}

void SetFlashWaitState(uint16_t ahb_prescaler,
                      uint8_t pll_m,uint16_t pll_n, uint8_t pll_p){

  uint32_t pll_vco = (HSE_VALUE/pll_m)*pll_n;
  uint32_t projected_sysclk = pll_vco/pll_p;
  uint32_t projected_hclk = projected_sysclk/ahb_prescaler;
  uint32_t flash_wait_state = GetFlashWaitState(projected_hclk);

  /* Program the wait states here */
  FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY)) | flash_wait_state;


}
