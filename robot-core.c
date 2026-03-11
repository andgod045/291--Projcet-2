#include <stdio.h>
#include <EFM8LB1.h>
#include <stdint.h>

#define SYSCLK 72000000L // 72 MHz System Clock

//==================== Calibration ====================
#define RIGHT_WHEEL_TRIM 90 // Scales the right motor down to 85% of target speed

//==================== LCD Pins ====================
#define LCD_RS  P1_6
#define LCD_E   P1_5
#define LCD_D4  P1_4
#define LCD_D5  P1_3
#define LCD_D6  P1_2
#define LCD_D7  P1_1

//==================== Button, Motor & IR Pins ====================
#define BTN_L       P2_6  
#define BTN_R       P3_1  

#define MOTOR_L_FWD P2_4 
#define MOTOR_L_REV P2_3  
#define MOTOR_R_FWD P2_1 
#define MOTOR_R_REV P2_2    

#define IR_PIN      P0_0  // TSOP33338 Output

// =========================================================================
//                          RC5 PROTOCOL LIBRARY
// =========================================================================

#define IR_RC5_ADDRESS          0x00U
#define IR_CARRIER_HZ           38000UL
#define IR_HALF_BIT_US          889U
#define IR_FULL_BIT_US          1778U
#define IR_TIMING_TOLERANCE_US  200U
#define IR_RC5_FRAME_BITS       14U
#define IR_MAGNITUDE_MAX        7U
#define IR_COMMAND_MAX          63U

typedef enum {
    IR_CMD_STOP       = 0x0U,
    IR_CMD_FORWARD    = 0x1U,
    IR_CMD_BACKWARD   = 0x2U,
    IR_CMD_LEFT       = 0x3U,
    IR_CMD_RIGHT      = 0x4U,
    IR_CMD_ROTATE_180 = 0x5U,
    IR_CMD_PATH_1     = 0x6U,
    IR_CMD_PATH_2     = 0x7U,
    IR_CMD_PATH_3     = 0x8U,
    IR_CMD_AUTO_START = 0x9U,
    IR_CMD_AUTO_STOP  = 0xAU,
    IR_CMD_CUSTOM_1   = 0xBU,
    IR_CMD_CUSTOM_2   = 0xCU,
    IR_CMD_CUSTOM_3   = 0xDU,
    IR_CMD_CUSTOM_4   = 0xEU
} IrCommand;

typedef struct {
    uint8_t  bits[IR_RC5_FRAME_BITS]; 
    IrCommand command;
    uint8_t  toggle;
    uint8_t  magnitude;               
} IrRC5Frame;

typedef enum {
    IR_DECODE_BUSY = 0, 
    IR_DECODE_DONE,     
    IR_DECODE_ERROR     
} IrDecodeStatus;

typedef struct {
    IrRC5Frame frame;        
    uint8_t    bit_index;    
    uint8_t    half_count;   
    uint8_t    last_level;   
    uint8_t    toggle;       
} IrRC5Decoder;

static uint8_t is_half_bit(uint32_t duration_us) {
    return (duration_us >= (IR_HALF_BIT_US - IR_TIMING_TOLERANCE_US)) &&
           (duration_us <= (IR_HALF_BIT_US + IR_TIMING_TOLERANCE_US));
}

static uint8_t is_full_bit(uint32_t duration_us) {
    return (duration_us >= (IR_FULL_BIT_US - IR_TIMING_TOLERANCE_US)) &&
           (duration_us <= (IR_FULL_BIT_US + IR_TIMING_TOLERANCE_US));
}

int IrRC5_BuildFrame(IrCommand cmd, uint8_t toggle, uint8_t magnitude, IrRC5Frame *frame) {
    uint8_t i;
    uint8_t address = IR_RC5_ADDRESS;

    if (frame == (void *)0) return 0;
    if (toggle > 1U) return 0;
    if ((uint8_t)cmd > IR_COMMAND_MAX) return 0;
    if (magnitude > IR_MAGNITUDE_MAX) return 0;

    frame->bits[0] = 1U;
    frame->bits[1] = 1U;
    frame->bits[2] = toggle;

    for (i = 0U; i < 2U; i++) frame->bits[3U + i] = (uint8_t)((address >> (1U - i)) & 0x1U);
    for (i = 0U; i < 6U; i++) frame->bits[5U + i] = (uint8_t)(((uint8_t)cmd >> (5U - i)) & 0x1U);
    for (i = 0U; i < 3U; i++) frame->bits[11U + i] = (uint8_t)((magnitude >> (2U - i)) & 0x1U);

    frame->toggle    = toggle;
    frame->command   = cmd;
    frame->magnitude = magnitude;

    return 1;
}

void IrRC5_DecoderReset(IrRC5Decoder *dec) {
    uint8_t i;
    if (dec == (void *)0) return;

    dec->bit_index  = 0U;
    dec->half_count = 0U;
    dec->last_level = 0U;
    dec->toggle     = 0U;

    for (i = 0U; i < IR_RC5_FRAME_BITS; i++) dec->frame.bits[i] = 0U;

    dec->frame.command   = IR_CMD_STOP;
    dec->frame.toggle    = 0U;
    dec->frame.magnitude = 0U;
}

IrDecodeStatus IrRC5_DecoderFeed(IrRC5Decoder *dec, uint8_t level, uint32_t duration_us) {
    uint8_t half_count;
    uint8_t bit_val;

    if (dec == (void *)0) return IR_DECODE_ERROR;
    if (dec->bit_index >= IR_RC5_FRAME_BITS) return IR_DECODE_ERROR;

    if (is_half_bit(duration_us)) half_count = 1U;
    else if (is_full_bit(duration_us)) half_count = 2U;
    else return IR_DECODE_ERROR;

    while (half_count > 0U) {
        half_count--;

        if (dec->half_count == 0U) {
            dec->last_level = level;
            dec->half_count = 1U;
        } else {
            if (dec->last_level == 1U && level == 0U) bit_val = 1U;
            else if (dec->last_level == 0U && level == 1U) bit_val = 0U;
            else return IR_DECODE_ERROR; 

            dec->frame.bits[dec->bit_index++] = bit_val;
            dec->half_count = 0U;

            if (dec->bit_index == IR_RC5_FRAME_BITS) {
                uint8_t i;
                uint8_t address_rx = 0U;
                uint8_t command_rx = 0U;
                uint8_t magnitude_rx = 0U;

                if (dec->frame.bits[0] != 1U || dec->frame.bits[1] != 1U) return IR_DECODE_ERROR;

                for (i = 0U; i < 2U; i++) address_rx = (uint8_t)((address_rx << 1U) | dec->frame.bits[3U + i]);
                if (address_rx != IR_RC5_ADDRESS) return IR_DECODE_ERROR;

                for (i = 0U; i < 6U; i++) command_rx = (uint8_t)((command_rx << 1U) | dec->frame.bits[5U + i]);
                for (i = 0U; i < 3U; i++) magnitude_rx = (uint8_t)((magnitude_rx << 1U) | dec->frame.bits[11U + i]);

                dec->frame.toggle    = dec->frame.bits[2];
                dec->frame.command   = (IrCommand)command_rx;
                dec->frame.magnitude = magnitude_rx;

                return IR_DECODE_DONE;
            }
        }
    }
    return IR_DECODE_BUSY;
}

// =========================================================================
//                          MAIN ROBOT CODE
// =========================================================================

// Global Variables 
volatile unsigned char pwm_L = 0;
volatile unsigned char dir_L = 1; 
volatile unsigned char pwm_R = 0;
volatile unsigned char dir_R = 1; 

IrRC5Decoder ir_decoder;
volatile IrDecodeStatus ir_status = IR_DECODE_BUSY;

// Prototypes
char _c51_external_startup(void);
void Timer3us(unsigned char us);
void waitms(unsigned int ms);
void Timer2_Init(void);
void LCD_Pulse_E(void);
void LCD_Write_Nibble(unsigned char nib);
void LCD_Write_Byte(unsigned char byte, bit isData);
void LCD_Command(unsigned char cmd);
void LCD_Data(unsigned char ch);
void LCD_Print(const char *s);
void LCD_SetCursor(unsigned char row, unsigned char col);
void LCD_Init(void);
void LCD_PrintIRCommand(IrCommand cmd);
void LCD_PrintMotorCompact(unsigned char pwmL, unsigned char pwmR);

// Timer 2 ISR (10kHz)
void Timer2_ISR(void) interrupt 5 using 1 
{
    static unsigned char pwm_counter = 0;
    static unsigned char last_ir_pin = 1;
    static unsigned int ir_ticks_us = 0;
    unsigned char current_ir_pin;
    
    TF2H = 0; 
    
    // --- 1. Motor PWM Logic ---
    pwm_counter++;
    if(pwm_counter >= 100) pwm_counter = 0;

    if (pwm_L == 0) { MOTOR_L_FWD = 0; MOTOR_L_REV = 0; } 
    else if (pwm_counter < pwm_L) {
        if (dir_L == 0) { MOTOR_L_FWD = 1; MOTOR_L_REV = 0; } 
        else            { MOTOR_L_FWD = 0; MOTOR_L_REV = 1; }
    } else { MOTOR_L_FWD = 0; MOTOR_L_REV = 0; }

    if (pwm_R == 0) { MOTOR_R_FWD = 0; MOTOR_R_REV = 0; } 
    else if (pwm_counter < pwm_R) {
        if (dir_R == 0) { MOTOR_R_FWD = 1; MOTOR_R_REV = 0; } 
        else            { MOTOR_R_FWD = 0; MOTOR_R_REV = 1; }
    } else { MOTOR_R_FWD = 0; MOTOR_R_REV = 0; }

    // --- 2. IR RC5 Decoder Polling ---
    ir_ticks_us += 100; 
    current_ir_pin = IR_PIN;
    
    if (current_ir_pin != last_ir_pin)
    {
        unsigned char ended_level = (last_ir_pin == 0) ? 1 : 0;
        
        if (ir_status != IR_DECODE_DONE) 
        {
            ir_status = IrRC5_DecoderFeed(&ir_decoder, ended_level, ir_ticks_us);
        }
        
        ir_ticks_us = 0;
        last_ir_pin = current_ir_pin;
    }
    else if (ir_ticks_us > 10000) 
    {
        if (ir_status != IR_DECODE_DONE) 
        {
            IrRC5_DecoderReset(&ir_decoder);
            ir_status = IR_DECODE_BUSY;
        }
        ir_ticks_us = 10000; 
    }
}

void main(void)
{
    unsigned char btn_L_pressed = 0;
    unsigned char btn_R_pressed = 0;

    _c51_external_startup();

    SFRPAGE = 0x20;
    P1MDOUT |= 0x7E;  
    P2MDOUT |= 0x1E;  
    
    P2MDIN  |= 0x40;   
    P2MDOUT &= ~0x40; 
    P3MDIN  |= 0x02;
    P3MDOUT &= ~0x02; 

    P0MDIN  |= 0x01;
    P0MDOUT &= ~0x01;
    P0SKIP  |= 0x01;  
    
    SFRPAGE = 0x00;
    P2 |= 0x40; 
    P3 |= 0x02; 
    P0 |= 0x01; 

    MOTOR_L_FWD = 0; MOTOR_L_REV = 0;
    MOTOR_R_FWD = 0; MOTOR_R_REV = 0;
    
    IrRC5_DecoderReset(&ir_decoder);
    Timer2_Init();
    
    LCD_Init();
    LCD_SetCursor(0, 0); LCD_Print("WAITING FOR IR  ");
    LCD_SetCursor(1, 0); LCD_Print("L:  0%   R:  0% ");

    while(1)
    {
        if (ir_status == IR_DECODE_DONE)
        {
            IrCommand cmd = ir_decoder.frame.command;
            unsigned char mag = ir_decoder.frame.magnitude;
            
            unsigned char target_pwm = (mag * 100) / 7;
            
            // Trim the right wheel speed based on the #define at the top
            unsigned char trimmed_pwm_R = ((unsigned int)target_pwm * RIGHT_WHEEL_TRIM) / 100;

            // Display the exact command received on the top row
            LCD_PrintIRCommand(cmd);

            switch (cmd)
            {
                case IR_CMD_FORWARD:
                    dir_L = 0; dir_R = 0; 
                    pwm_L = target_pwm; pwm_R = trimmed_pwm_R; 
                    break;
                case IR_CMD_BACKWARD:
                    dir_L = 1; dir_R = 1; 
                    pwm_L = target_pwm; pwm_R = trimmed_pwm_R; 
                    break;
                case IR_CMD_LEFT:
                    dir_L = 1; dir_R = 0; 
                    pwm_L = target_pwm; pwm_R = trimmed_pwm_R; 
                    break; 
                case IR_CMD_RIGHT:
                    dir_L = 0; dir_R = 1; 
                    pwm_L = target_pwm; pwm_R = trimmed_pwm_R; 
                    break; 
                case IR_CMD_STOP:
                    pwm_L = 0; pwm_R = 0; 
                    break;
                default:
                    break; 
            }

            EA = 0;
            IrRC5_DecoderReset(&ir_decoder);
            ir_status = IR_DECODE_BUSY;
            EA = 1;
        }

        if (BTN_L == 0) 
        { 
            if (btn_L_pressed == 0) 
            {
                LCD_SetCursor(0, 0); LCD_Print("CMD: L-BTN OVRD ");
                dir_L = !dir_L;
                pwm_L = 0;
                btn_L_pressed = 1;
            }
            if (pwm_L < 100) pwm_L++;
        } 
        else btn_L_pressed = 0;

        if (BTN_R == 0) 
        { 
            if (btn_R_pressed == 0) 
            {
                LCD_SetCursor(0, 0); LCD_Print("CMD: R-BTN OVRD ");
                dir_R = !dir_R;
                pwm_R = 0;
                btn_R_pressed = 1;
            }
            if (pwm_R < RIGHT_WHEEL_TRIM) pwm_R++; // Stop ramping at the trim limit
        } 
        else btn_R_pressed = 0;

        LCD_PrintMotorCompact(pwm_L, pwm_R); 
        
        waitms(30); 
    }
}

void LCD_PrintIRCommand(IrCommand cmd)
{
    LCD_SetCursor(0, 0);
    switch (cmd)
    {
        case IR_CMD_FORWARD:  LCD_Print("CMD: FORWARD    "); break;
        case IR_CMD_BACKWARD: LCD_Print("CMD: BACKWARD   "); break;
        case IR_CMD_LEFT:     LCD_Print("CMD: PIVOT LEFT "); break;
        case IR_CMD_RIGHT:    LCD_Print("CMD: PIVOT RIGH "); break;
        case IR_CMD_STOP:     LCD_Print("CMD: STOPPED    "); break;
        default:              LCD_Print("CMD: UNKNOWN    "); break;
    }
}

void LCD_PrintMotorCompact(unsigned char pwmL, unsigned char pwmR)
{
    LCD_SetCursor(1, 0);
    
    LCD_Print("L:");
    if (pwmL == 100) { LCD_Print("100%"); }
    else if (pwmL >= 10) { LCD_Data(' '); LCD_Data('0' + (pwmL/10)); LCD_Data('0' + (pwmL%10)); LCD_Data('%'); }
    else { LCD_Print("  "); LCD_Data('0' + pwmL); LCD_Data('%'); }
    
    LCD_Print("  R:");
    if (pwmR >= 100) { LCD_Print("100%"); } // Prevent overflow display
    else if (pwmR >= 10) { LCD_Data(' '); LCD_Data('0' + (pwmR/10)); LCD_Data('0' + (pwmR%10)); LCD_Data('%'); }
    else { LCD_Print("  "); LCD_Data('0' + pwmR); LCD_Data('%'); }
    LCD_Print(" ");
}

void Timer2_Init(void)
{
    TMR2CN0 = 0x00; 
    CKCON0 |= 0x10; 
    TMR2RL = 65536 - 7200; 
    TMR2 = TMR2RL;
    
    IE |= 0x20; 
    EA = 1;     
    TR2 = 1;    
}

char _c51_external_startup(void)
{
    SFRPAGE = 0x00; WDTCN = 0xDE; WDTCN = 0xAD; 
    VDM0CN=0x80; RSTSRC=0x02|0x04;
    SFRPAGE = 0x10; PFE0CN  = 0x20; SFRPAGE = 0x00;
    CLKSEL = 0x00; CLKSEL = 0x00; while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03; CLKSEL = 0x03; while ((CLKSEL & 0x80) == 0); 
    P0MDOUT |= 0x10; XBR0 = 0x01; XBR1 = 0X00; XBR2 = 0x40; 
    return 0;
}

void Timer3us(unsigned char us)
{
    unsigned char i; CKCON0|=0b_0100_0000;
    TMR3RL = (-(SYSCLK)/1000000L); TMR3 = TMR3RL; TMR3CN0 = 0x04;
    for (i = 0; i < us; i++) { while (!(TMR3CN0 & 0x80)); TMR3CN0 &= ~(0x80); }
    TMR3CN0 = 0 ;
}

void waitms(unsigned int ms) 
{ 
    unsigned int j; unsigned char k; 
    for(j=0; j<ms; j++) for (k=0; k<4; k++) Timer3us(250); 
}

void LCD_Pulse_E(void) { LCD_E = 1; Timer3us(2); LCD_E = 0; Timer3us(50); }

void LCD_Write_Nibble(unsigned char nib)
{
    LCD_D4 = (nib & 0x01) ? 1 : 0; 
    LCD_D5 = (nib & 0x02) ? 1 : 0; 
    LCD_D6 = (nib & 0x04) ? 1 : 0; 
    LCD_D7 = (nib & 0x08) ? 1 : 0; 
    LCD_Pulse_E();
}

void LCD_Write_Byte(unsigned char byte, bit isData) 
{ 
    LCD_RS = isData; 
    LCD_Write_Nibble(byte >> 4); 
    LCD_Write_Nibble(byte & 0x0F); 
}

void LCD_Command(unsigned char cmd) 
{ 
    LCD_Write_Byte(cmd, 0); 
    if(cmd == 0x01 || cmd == 0x02) waitms(2); 
}

void LCD_Data(unsigned char ch) { LCD_Write_Byte(ch, 1); }

void LCD_Print(const char *s) { while(*s) LCD_Data(*s++); }

void LCD_SetCursor(unsigned char row, unsigned char col) 
{ 
    unsigned char addr = (row==0) ? 0x00 : 0x40; 
    LCD_Command(0x80 | (addr + col)); 
}

void LCD_Init(void)
{
    LCD_RS = 0; LCD_E  = 0; waitms(20);
    LCD_Write_Nibble(0x03); waitms(5); 
    LCD_Write_Nibble(0x03); Timer3us(150); 
    LCD_Write_Nibble(0x03); Timer3us(150); 
    LCD_Write_Nibble(0x02);
    LCD_Command(0x28); LCD_Command(0x0C); 
    LCD_Command(0x06); LCD_Command(0x01); waitms(2);
}
