#include <xc.h>
#include <sys/attribs.h>
#include "lcd.h"

// Uses Timer4 to delay <us> microseconds
void Timer4us(unsigned char t) 
{
	T4CON = 0x8000; // enable Timer4, source PBCLK, 1:1 prescaler
 
    // delay 100us per loop until less than 100us remain
    while( t >= 100){
        t-=100;
        TMR4=0;
        while(TMR4 < SYSCLK/10000L);
    }
 
    // delay 10us per loop until less than 10us remain
    while( t >= 10){
        t-=10;
        TMR4=0;
        while(TMR4 < SYSCLK/100000L);
    }
 
    // delay 1us per loop until finished
    while( t > 0)
    {
        t--;
        TMR4=0;
        while(TMR4 < SYSCLK/1000000L);
    }
    // turn off Timer4 so function is self-contained
    T4CONCLR=0x8000;
}

void waitms(unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0;j<ms;j++)
		for(k=0;k<4;k++)
			Timer4us(250);
}

void LCD_pulse(void)
{
	LCD_E = 1;
	Timer4us(40);
	LCD_E = 0;
}

void LCD_byte(unsigned char x)
{
	LCD_D7=(x&0x80)?1:0;
	LCD_D6=(x&0x40)?1:0;
	LCD_D5=(x&0x20)?1:0;
	LCD_D4=(x&0x10)?1:0;
	LCD_pulse();
	Timer4us(40);
	LCD_D7=(x&0x08)?1:0;
	LCD_D6=(x&0x04)?1:0;
	LCD_D5=(x&0x02)?1:0;
	LCD_D4=(x&0x01)?1:0;
	LCD_pulse();
}

void WriteData(unsigned char x)
{
	LCD_RS = 1;
	LCD_byte(x);
	waitms(2);
}

void WriteCommand(unsigned char x)
{
	LCD_RS = 0;
	LCD_byte(x);
	waitms(5);
}

void LCD_4BIT(void)
{
    // Configure the pins used to communicate with the LCD as digital outputs.
    ANSELA_DisableAnalog(LCD_E_MASK_A | LCD_D4_MASK_A | LCD_D6_MASK_A);
    ANSELB_DisableAnalog(LCD_RS_MASK_B | LCD_D5_MASK_B | LCD_D7_MASK_B);

    LCD_RS_ENABLE = 0;
    LCD_E_ENABLE = 0;
    LCD_D4_ENABLE = 0;
    LCD_D5_ENABLE = 0;
    LCD_D6_ENABLE = 0;
    LCD_D7_ENABLE = 0;

    LCD_E = 0; // Resting state of LCD enable is low.
    waitms(20);

    // Ensure LCD starts in 8-bit mode, then switch to 4-bit.
    WriteCommand(0x33);
    WriteCommand(0x33);
    WriteCommand(0x32);

    WriteCommand(0x28); // 4-bit, 2 lines, 5x8 font
    WriteCommand(0x0c); // Display ON, cursor OFF
    WriteCommand(0x01); // Clear display
    waitms(20);
}

void LCDprint(const char *string, unsigned char line, unsigned char clear)
{
    int j;

    WriteCommand(line == 2 ? 0xc0 : 0x80);
    waitms(5);
    for (j = 0; string[j] != 0; j++)
        WriteData((unsigned char)string[j]);
    if (clear)
        for (; j < CHARS_PER_LINE; j++)
            WriteData(' ');
}