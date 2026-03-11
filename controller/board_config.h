#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <xc.h>
#include <stdint.h>

// Clock configuration
#define SYSCLK 40000000UL
#define PBCLK  SYSCLK

/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/

// PPS configuration values
// UART2: RXD<->RB8, TXD<->RB9, RTS<->RB7, CTS<->RB11
#define UART2_RX_PPS_INPUT_SEL   4U   // U2RX <- RB8 (RPB8 = 4)
#define UART2_TX_PPS_OUTPUT_SEL  2U   // RB9 -> U2TX (RPB9R = 2)
#define UART2_RTS_PPS_OUTPUT_SEL 2U   // RB7 -> U2RTS (RPB7R = 2)
#define UART2_CTS_PPS_INPUT_SEL  3U   // U2CTS <- RB11 (RPB11 = 3)

// LCD pin numbers on PORTA/PORTB
#define PIN_LCD_E_A          2U
#define PIN_LCD_D4_A         3U
#define PIN_LCD_D6_A         4U
#define PIN_LCD_RS_B         3U
#define PIN_LCD_D5_B         4U
#define PIN_LCD_D7_B         5U

// Pin numbers on PORTA
#define PIN_JOYSTICK_VRX_A   0U  // AN0
#define PIN_JOYSTICK_VRY_A   1U  // AN1

// Pin numbers on PORTB
#define PIN_JOYSTICK_SW_B    0U  // Digital input with pull-up
#define PIN_IR_LED_B         6U  // IR LED transmitter output

#define CHARS_PER_LINE       16U

#define PIN_MASK_A(pin)      (1UL << (pin))
#define PIN_MASK_B(pin)      (1UL << (pin))

#define LCD_E_MASK_A         PIN_MASK_A(PIN_LCD_E_A)
#define LCD_D4_MASK_A        PIN_MASK_A(PIN_LCD_D4_A)
#define LCD_D6_MASK_A        PIN_MASK_A(PIN_LCD_D6_A)
#define LCD_RS_MASK_B        PIN_MASK_B(PIN_LCD_RS_B)
#define LCD_D5_MASK_B        PIN_MASK_B(PIN_LCD_D5_B)
#define LCD_D7_MASK_B        PIN_MASK_B(PIN_LCD_D7_B)

#define JOYSTICK_VRX_MASK_A  PIN_MASK_A(PIN_JOYSTICK_VRX_A)
#define JOYSTICK_VRY_MASK_A  PIN_MASK_A(PIN_JOYSTICK_VRY_A)
#define JOYSTICK_SW_MASK_B   PIN_MASK_B(PIN_JOYSTICK_SW_B)
#define IR_LED_MASK_B        PIN_MASK_B(PIN_IR_LED_B)

// ADC channel numbers for joystick analog inputs
#define JOYSTICK_VRX_ADC_CHANNEL 0U  // AN0
#define JOYSTICK_VRY_ADC_CHANNEL 1U  // AN1

// LCD signal aliases used by lcd.c
#define LCD_RS               LATBbits.LATB3
#define LCD_E                LATAbits.LATA2
#define LCD_D4               LATAbits.LATA3
#define LCD_D5               LATBbits.LATB4
#define LCD_D6               LATAbits.LATA4
#define LCD_D7               LATBbits.LATB5

#define LCD_RS_ENABLE        TRISBbits.TRISB3
#define LCD_E_ENABLE         TRISAbits.TRISA2
#define LCD_D4_ENABLE        TRISAbits.TRISA3
#define LCD_D5_ENABLE        TRISBbits.TRISB4
#define LCD_D6_ENABLE        TRISAbits.TRISA4
#define LCD_D7_ENABLE        TRISBbits.TRISB5

static void GPIOA_SetOutput(const uint32_t mask) { TRISACLR = mask; }
static void GPIOA_SetInput(const uint32_t mask)  { TRISASET = mask; }
static void GPIOA_SetHigh(const uint32_t mask)   { LATASET = mask; }
static void GPIOA_SetLow(const uint32_t mask)    { LATACLR = mask; }
static uint8_t GPIOA_Read(const uint32_t mask)   { return (PORTA & mask) ? 1U : 0U; }

static void GPIOB_SetOutput(const uint32_t mask) { TRISBCLR = mask; }
static void GPIOB_SetInput(const uint32_t mask)  { TRISBSET = mask; }
static void GPIOB_SetHigh(const uint32_t mask)   { LATBSET = mask; }
static void GPIOB_SetLow(const uint32_t mask)    { LATBCLR = mask; }
static uint8_t GPIOB_Read(const uint32_t mask)   { return (PORTB & mask) ? 1U : 0U; }

static void ANSELA_EnableAnalog(const uint32_t mask)  { ANSELASET = mask; }
static void ANSELA_DisableAnalog(const uint32_t mask) { ANSELACLR = mask; }

static void ANSELB_EnableAnalog(const uint32_t mask)  { ANSELBSET = mask; }
static void ANSELB_DisableAnalog(const uint32_t mask) { ANSELBCLR = mask; }

#endif // BOARD_CONFIG_H

