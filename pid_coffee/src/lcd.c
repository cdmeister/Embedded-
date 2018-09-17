#include "lcd.h"

void LCD_init(LCD * LCDx, GPIO_TypeDef * GPIOx,
         uint8_t rs, uint8_t rw, uint8_t en,
         uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
         uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
         uint8_t num_rows, uint8_t num_cols,
         uint8_t bitmode, uint8_t charsize){
  LCDx->GPIOx=GPIOx;

  LCDx->_rs_pin=rs;
  LCDx->_rw_pin=rw;
  LCDx->_enable_pin=en;

  LCDx->bitmode=bitmode;

  LCDx->_data_pins[0]=d0;
  LCDx->_data_pins[1]=d1;
  LCDx->_data_pins[2]=d2;
  LCDx->_data_pins[3]=d3;
  LCDx->_data_pins[4]=d4;
  LCDx->_data_pins[5]=d5;
  LCDx->_data_pins[6]=d6;
  LCDx->_data_pins[7]=d7;

  LCDx->_num_rows = num_rows;
  LCDx->_num_cols = num_cols;
  LCDx->_displayFunction = 0x0;

  if ( LCDx->_num_rows > 1)
    LCDx->_displayFunction |= LCD_2LINE;
  else
    LCDx->_displayFunction |= LCD_1LINE;

  if (bitmode == LCD_4BITMODE)
    LCDx->_displayFunction |= LCD_4BITMODE | LCD_5x8DOTS;
  else
    LCDx->_displayFunction |= LCD_8BITMODE | LCD_5x8DOTS;

  LCD_setRowOffsets(LCDx, 0x00, 0x40,
                    0x00 + LCDx->_num_cols, 0x40 + LCDx->_num_cols);

  // for some 1 line displays you can select a 10 pixel high font
  if ((charsize != LCD_5x8DOTS) && (LCDx->_num_rows == 1)) {
    LCDx->_displayFunction |= LCD_5x10DOTS;
  }

   // Initialize to default text direction (for romance languages)
  LCDx->_displayMode = 0x0;
  LCDx->_displayMode |= LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;


  // Initialization

  // Now we pull both RS and R/W low to begin commands
  LCDx->GPIOx->BSRR = (1 << (LCDx->_enable_pin+0x10));
  LCDx->GPIOx->BSRR = (1 <<(LCDx->_rs_pin+0x10));


  // 8-bit mode
  if(LCDx->bitmode == LCD_8BITMODE){
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    write8bits(LCDx,0x30);
    Delay(5);

    write8bits(LCDx,0x30);
    Delay(5);

    write8bits(LCDx,0x30);
    Delay(5);

    write8bits(LCDx, LCDx->_displayFunction|LCD_FUNCTIONSET);
    write8bits(LCDx, 0x08); // Turn off display
    write8bits(LCDx, 0x01); // clear display
    write8bits(LCDx, LCD_ENTRYMODESET|LCDx->_displayMode); // set entry mode

    // turn the display on with no cursor or blinking default
    LCDx->_displayControl = 0x0;
    LCDx->_displayControl |= LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON;
    write8bits(LCDx, LCDx->_displayControl | LCD_DISPLAYCONTROL);

  }
  else{// 4 bit mode
    // this is according to the hitachi HD44780 datasheet
    // page 46 figure 24

    write4bits(LCDx,0x3);
    Delay(5);

    write4bits(LCDx,0x3);
    Delay(5);

    write4bits(LCDx,0x3);
    Delay(5);


    write4bits(LCDx,0x2);


    write4bits(LCDx, (LCDx->_displayFunction|LCD_FUNCTIONSET)>>4);
    write4bits(LCDx, (LCDx->_displayFunction|LCD_FUNCTIONSET)  & 0xF);

    write4bits(LCDx, 0x08 >> 4); // Turn off display
    write4bits(LCDx, 0x08 & 0xF); // Turn off display

    write4bits(LCDx, 0x01>> 4); // clear display
    write4bits(LCDx, 0x01 & 0xF); // clear display

    write4bits(LCDx, (LCD_ENTRYMODESET|LCDx->_displayMode)>>4); // set entry mode
    write4bits(LCDx, (LCD_ENTRYMODESET|LCDx->_displayMode) & 0xF); // set entry mode

    // turn the display on with no cursor or blinking default
    LCDx->_displayControl = 0x0;
    LCDx->_displayControl |= LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON;
    write4bits(LCDx, (LCDx->_displayControl | LCD_DISPLAYCONTROL)>>4);
    write4bits(LCDx, (LCDx->_displayControl | LCD_DISPLAYCONTROL)&0xF);
  }




/* // clear it off LCD_command(LCDx, 0x01);
  Delay(5);
  // Initialize to default text direction (for romance languages)
  LCDx->_displayMode = 0x0;
  LCDx->_displayMode |= LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  LCD_command(LCDx, 0x06);
  Delay(5);

  // turn the display on with no cursor or blinking default
  LCDx->_displayControl = 0x0;
  LCDx->_displayControl |= LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON;
  LCD_command(LCDx,0x0F);
  Delay(5);
*/




}

void LCD_setRowOffsets(LCD * LCDx, int row1, int row2, int row3, int row4){

  LCDx->_row_offsets[0]=row1;
  LCDx->_row_offsets[1]=row2;
  LCDx->_row_offsets[2]=row3;
  LCDx->_row_offsets[3]=row4;
  return;

}

void LCD_home(LCD * LCDx){
  LCD_command(LCDx, LCD_RETURNHOME);
  // Delay
  Delay(5);
  return;
}

void LCD_clear(LCD * LCDx){
  LCD_command(LCDx, LCD_CLEARDISPLAY);
  //Delay
  Delay(5); // this command takes a long time!
  return;
}

void LCD_clearRow(LCD * LCDx, uint8_t row){
  LCD_setCursor(LCDx,0,row);
  int index=0;
  for(;index<LCDx->_num_cols;index++) LCD_write(LCDx,0x20);
  LCD_setCursor(LCDx,0,row);
  return;
}

void LCD_noDisplay(LCD * LCDx){

  LCDx->_displayControl &= ~(LCD_DISPLAYON);
  LCD_command(LCDx,LCD_DISPLAYCONTROL|LCDx->_displayControl);
  return;

}

void LCD_display(LCD * LCDx){

  LCDx->_displayControl |= LCD_DISPLAYON;
  LCD_command(LCDx, LCD_DISPLAYCONTROL | LCDx->_displayControl);
  return;
}

// Turns the underline cursor on/off
void LCD_noCursor(LCD * LCDx) {
  LCDx->_displayControl &= ~LCD_CURSORON;
  LCD_command(LCDx, LCD_DISPLAYCONTROL | LCDx->_displayControl);
}
void LCD_cursor(LCD * LCDx) {
  LCDx->_displayControl |= LCD_CURSORON;
  LCD_command(LCDx, LCD_DISPLAYCONTROL | LCDx->_displayControl);
}

// Turn on and off the blinking cursor
void LCD_noBlink(LCD * LCDx) {
  LCDx->_displayControl &= ~LCD_BLINKON;
  LCD_command(LCDx, LCD_DISPLAYCONTROL | LCDx->_displayControl);
}
void LCD_blink(LCD * LCDx) {
  LCDx->_displayControl |= LCD_BLINKON;
  LCD_command(LCDx, LCD_DISPLAYCONTROL | LCDx->_displayControl);
}


void write4bits(LCD * LCDx ,uint8_t data){

/*  if ( data & 0x8 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[7];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[7]);

  if ( data & 0x4 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[6];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[6]);

  if ( data & 0x2 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[5];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[5]);

  if ( data & 0x1 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[4];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[4]);

*/

  int pin=0;
  for(;pin<4;pin++){
    LCDx->GPIOx->BSRR = ((data>>pin)&0x1)? (1 << LCDx->_data_pins[pin+4])  :
                                          (1 << (LCDx->_data_pins[pin+4]+0x10));
  }

  pulseEnable(LCDx);

}

void write8bits(LCD * LCDx, uint8_t data){

  int pin=0;
  for(;pin<8;pin++){
    LCDx->GPIOx->BSRR = ((data>>pin)&0x1)? (1 << LCDx->_data_pins[pin])  :
                                          (1 << (LCDx->_data_pins[pin]+0x10));
  }

/*  if ( data & 0x80 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[7];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[7]);

  if ( data & 0x40 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[6];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[6]);

  if ( data & 0x20 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[5];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[5]);

  if ( data & 0x10 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[4];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[4]);

  if ( data & 0x08 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[3];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[3]);

  if ( data & 0x04 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[2];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[2]);

  if ( data & 0x02 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[1];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[1]);

  if ( data & 0x01 ) LCDx->GPIOx->ODR |= 1 << LCDx->_data_pins[0];
  else LCDx->GPIOx->ODR &= ~(1 << LCDx->_data_pins[0]);
*/
  pulseEnable(LCDx);

  return;

}

void pulseEnable(LCD * LCDx){

  LCDx->GPIOx->BSRR = (1 << LCDx->_enable_pin);
  //LCDx->GPIOx->ODR |= (1 << LCDx->_enable_pin);
  Delay(2);
  //LCDx->GPIOx->ODR &= ~(1 << LCDx->_enable_pin);
  LCDx->GPIOx->BSRR = (1 << (LCDx->_enable_pin+0x10));
  Delay(2);

}

void send(LCD * LCDx, uint8_t data, uint8_t state){
  // write to the rs_pin
  // 0 for cmd, 1 for data
  if(state == LOW ) LCDx->GPIOx->BSRR = ( 1 <<(LCDx->_rs_pin+0x10));
  else LCDx->GPIOx->BSRR=(1 << LCDx->_rs_pin);
  //if(state == LOW) LCDx->GPIOx->ODR &= ~(1 << LCDx->_rs_pin);
  //else LCDx->GPIOx->ODR |= (1 << LCDx->_rs_pin);


  /* Figure out RW pin handling */


  if (LCDx->bitmode ==  LCD_8BITMODE) {
    write8bits(LCDx,data);
  } else {
    write4bits(LCDx,data>>4);
    write4bits(LCDx,data & 0xF);
  }

}


void LCD_command(LCD * LCDx, uint8_t data){
  send(LCDx, data, LOW);
  return;
}
void LCD_write(LCD * LCDx, uint8_t data){
  send(LCDx, data, HIGH);
  return;
}

void LCD_print(LCD * LCDx,const char * fmt,...){

  //char * buffer = NULL;
  char buffer[LCDx->_num_cols];
  va_list argp;
  va_start(argp, fmt);
  int num = vsprintf(buffer,fmt,argp);
  va_end(argp);

  //int buffersize = sizeof(buffer)/sizeof(buffer[0]);
  int index =0;
  for(;index<num;index++){LCD_write(LCDx, buffer[index]);}

  return;
}

void LCD_setCursor(LCD * LCDx, uint8_t col, uint8_t row)
{
  const size_t max_lines = sizeof(LCDx->_row_offsets) / sizeof(LCDx->_row_offsets[0]);
  if ( row >= max_lines ) {
    row = max_lines - 1;    // we count rows starting w/0
  }
  if ( row >= LCDx->_num_rows ) {
    row = LCDx->_num_rows - 1;    // we count rows starting w/0
  }

  LCD_command(LCDx,LCD_SETDDRAMADDR | (col + LCDx->_row_offsets[row]));
}
