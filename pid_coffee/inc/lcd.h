/**
  ******************************************************************************
  * @file    lcd.h
  * @author  Moeiz Riaz
  * @version V1.0.0
  * @date    30-April-2018
  * @brief   Header for lcd.c module, compatitable with HD44780
             tested on JHD 162A
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "systick.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"
/* Exported types ------------------------------------------------------------*/

typedef struct LCD{

  GPIO_TypeDef * GPIOx;

  /* Pins on LCD display */
  volatile uint8_t _rs_pin; // LOW: command.  HIGH: character.
  volatile uint8_t _rw_pin; // LOW: write to LCD.  HIGH: read from LCD.
  volatile uint8_t _enable_pin; // activated by a HIGH pulse.
  volatile uint8_t _data_pins[8];

  /* Variables hold state */
  volatile uint8_t _displayFunction;
  volatile uint8_t _displayMode; // Shift/ I/D for entry mode tracking
  volatile uint8_t _displayControl; //Display ON/OFF, blinking, cursor


  volatile uint8_t _num_cols; // cols on lcd screen
  volatile uint8_t _num_rows; // lines/rows on lcd screen
  volatile uint8_t _row_offsets[4]; // row addressing.  0x0-0x7 and 0x40-0x47

  volatile uint8_t bitmode;

}LCD;

/* Exported constants --------------------------------------------------------*/
enum state{
  HIGH = 1,
  LOW = 0
};

/* Exported macro ------------------------------------------------------------*/

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* Exported functions ------------------------------------------------------- */

void LCD_init(LCD * LCDx, GPIO_TypeDef * GPIOx,
              uint8_t rs, uint8_t rw, uint8_t en,
              uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
              uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
              uint8_t num_rows, uint8_t num_cols,
              uint8_t bitmode, uint8_t charsize);

void LCD_home(LCD * LCDx);
void LCD_clear(LCD * LCDx);
void LCD_clearRow(LCD * LCDx, uint8_t row);

void LCD_noDisplay(LCD * LCDx);
void LCD_display(LCD * LCDx);
void LCD_noBlink(LCD * LCDx);
void LCD_blink(LCD * LCDx);
void LCD_noCursor(LCD * LCDx);
void LCD_cursor(LCD * LCDx);

void LCD_scrollDisplayLeft(LCD * LCDx);
void LCD_scrollDisplayRight(LCD * LCDx);
void LCD_leftToRight(LCD * LCDx);
void LCD_rightToLeft(LCD * LCDx);
void LCD_autoscroll(LCD * LCDx);
void LCD_noAutoscroll(LCD * LCDx);
void LCD_setCursor(LCD * LCDx, uint8_t col, uint8_t row);


inline void LCD_setRowOffsets(LCD * LCDx, int row1,
                              int row2, int row3, int row4);
//void LCD_createChar(uint8_t, uint8_t[]);
void LCD_write(LCD * LCDx, uint8_t data);
void LCD_command(LCD * LCDx, uint8_t data);

void send(LCD * LCDx , uint8_t data, uint8_t state);
void write4bits(LCD * LCDx, uint8_t data);
void write8bits(LCD * LCDx, uint8_t data);
void pulseEnable(LCD * LCDx);

void LCD_print(LCD *LCDx, const char * fmt, ...);

#endif /* __LCD_H */
