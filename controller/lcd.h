#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "board_config.h"

void Timer4us(unsigned char us);
void waitms(unsigned int ms);
void LCD_pulse(void);
void LCD_byte(unsigned char x);
void WriteData(unsigned char x);
void WriteCommand(unsigned char x);
void LCD_4BIT(void);
void LCDprint(const char *string, unsigned char line, unsigned char clear);

#endif // LCD_H
