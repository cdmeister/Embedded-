Various Projects for STM32F4

- assembly: low level code,arm assembly to blink led
- blinky: simple project to get a led to blink on the board
- blinky_interupt: use systick(interupt) to blink led
  - GBD session: monitor arm semihosting enable
  - telnet session: arm semihosting enable
- led_switch: use switch on board to turn led on/off
- ext_led_switch: external push button connect on the GPIO to turn led on/off
- ext_interupt: using interrupts to light led on off using push button
- usart: use "polling" usart technique to print to serial console
- basic_timer: use general purpose timer to toggle led on pin PD15 via alternate function
- timer_interrupt_arr: use general purpose timer and ARR to generate interrupt
