#include <stdio.h>
#include <EFM8LB1.h>
#include <stdint.h>

#define SYSCLK 72000000L // 72 MHz System Clock

//==================== Calibration ====================
#define RIGHT_WHEEL_TRIM 90 // Scales the right motor down to 85% of target speed

// Autopilot Tuning Constants
#define TURN_180_TIME_MS 850 // TUNE THIS: Milliseconds to spin exactly 180 degrees
#define CURVE_FAST       80  // The outside wheel speed during a curve
#define CURVE_SLOW       35  // The inside wheel speed during a curve
#define FIG8_TIME_MS     3000 // Milliseconds to drive one half of the figure-8

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
//                          NEW RC5 MOVEMENT PROTOCOL LIBRARY
// =========================================================================

#define IR_RC5_ADDRESS            0x00U
#define IR_CARRIER_HZ             38000UL
#define IR_HALF_BIT_US            889U
#define IR_FULL_BIT_US            1778U
#define IR_TIMING_TOLERANCE_US    200U
#define IR_RC5_FRAME_BITS         14U

#define IR_MOVEMENT_AXIS_MIN      (-7)
#define IR_MOVEMENT_AXIS_MAX      (7)
#define IR_MOVEMENT_MAGNITUDE_MAX 7U
#define IR_MOVEMENT_NIBBLE_INVALID 0x08U

#define IR_MISC_SELECT_PATH_BASE  16U
#define IR_MISC_SELECT_PATH_COUNT 16U
#define IR_MISC_SELECT_PATH_MAX   (IR_MISC_SELECT_PATH_BASE + IR_MISC_SELECT_PATH_COUNT - 1U)

typedef enum {
    IR_DATA_MISC = 0,
    IR_DATA_MOVEMENT = 1
} IrDataType;

typedef enum {
    IR_MISC_STOP = 0,
    IR_MISC_FORWARD = 1,
    IR_MISC_BACKWARD = 2,
    IR_MISC_LEFT = 3,
    IR_MISC_RIGHT = 4,
    IR_MISC_ROTATE_180 = 5,
    IR_MISC_START_PATH = 6,
    IR_MISC_STOP_PATH = 7,
    IR_MISC_EDIT_PATH = 8,
    IR_MISC_SAVE_PATH = 9,
    IR_MISC_EDIT_NEXT_STEP = 10,
    IR_MISC_ESTOP = 11,
    IR_MISC_ESTOP_CLEAR = 12,
    IR_MISC_SELECT_PATH_1 = IR_MISC_SELECT_PATH_BASE
} IrMiscCode;

typedef struct {
    uint8_t bits[IR_RC5_FRAME_BITS];
    uint8_t toggle;
    uint8_t address;
    uint8_t data_type;
    uint8_t payload; 
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

static uint8_t axis_to_nibble(const int8_t axis) {
    return (uint8_t)axis & 0x0FU;
}

static int nibble_to_axis(uint8_t nibble, int8_t *axis) {
    nibble &= 0x0FU;
    if (axis == (void *)0) return 0;
    if (nibble == IR_MOVEMENT_NIBBLE_INVALID) return 0;
    if ((nibble & 0x08U) != 0U) *axis = (int8_t)(nibble | 0xF0U);
    else *axis = (int8_t)nibble;
    if (*axis < IR_MOVEMENT_AXIS_MIN || *axis > IR_MOVEMENT_AXIS_MAX) return 0;
    return 1;
}

int IrRC5_IsValidDataType(const uint8_t data_type) {
    return (data_type == (uint8_t)IR_DATA_MISC || data_type == (uint8_t)IR_DATA_MOVEMENT) ? 1 : 0;
}

int IrRC5_IsValidMiscCode(const uint8_t misc_code) {
    if (misc_code <= (uint8_t)IR_MISC_ESTOP_CLEAR) return 1;
    if (misc_code >= IR_MISC_SELECT_PATH_BASE && misc_code <= IR_MISC_SELECT_PATH_MAX) return 1;
    return 0;
}

int IrRC5_DecodeMovement(const uint8_t payload, int8_t *x, int8_t *y) {
    int8_t x_decoded;
    int8_t y_decoded;
    if (x == (void *)0 || y == (void *)0) return 0;
    if (nibble_to_axis((uint8_t)(payload & 0x0FU), &x_decoded) == 0) return 0;
    if (nibble_to_axis((uint8_t)((payload >> 4U) & 0x0FU), &y_decoded) == 0) return 0;
    *x = x_decoded;
    *y = y_decoded;
    return 1;
}

void IrRC5_DecoderReset(IrRC5Decoder *dec) {
    uint8_t i;
    if (dec == (void *)0) return;
    dec->bit_index  = 0U; dec->half_count = 0U; dec->last_level = 0U; dec->toggle = 0U;
    for (i = 0U; i < IR_RC5_FRAME_BITS; i++) dec->frame.bits[i] = 0U;
    dec->frame.toggle = 0U; dec->frame.address = IR_RC5_ADDRESS;
    dec->frame.data_type = (uint8_t)IR_DATA_MISC; dec->frame.payload = (uint8_t)IR_MISC_STOP;
}

IrDecodeStatus IrRC5_DecoderFeed(IrRC5Decoder *dec, uint8_t level, uint32_t duration_us) {
    uint8_t half_count, bit_val;

    if (dec == (void *)0) return IR_DECODE_ERROR;
    if (dec->bit_index >= IR_RC5_FRAME_BITS) return IR_DECODE_ERROR;

    if (is_half_bit(duration_us)) half_count = 1U;
    else if (is_full_bit(duration_us)) half_count = 2U;
    else return IR_DECODE_ERROR;

    while (half_count > 0U) {
        half_count--;
        if (dec->half_count == 0U) {
            dec->last_level = level; dec->half_count = 1U;
        } else {
            if (dec->last_level == 1U && level == 0U) bit_val = 1U;
            else if (dec->last_level == 0U && level == 1U) bit_val = 0U;
            else return IR_DECODE_ERROR; 

            dec->frame.bits[dec->bit_index++] = bit_val;
            dec->half_count = 0U;

            if (dec->bit_index == IR_RC5_FRAME_BITS) {
                uint8_t i, address_rx = 0U, data_type_rx, payload_rx = 0U;

                if (dec->frame.bits[0] != 1U || dec->frame.bits[1] != 1U) return IR_DECODE_ERROR;
                for (i = 0U; i < 2U; i++) address_rx = (uint8_t)((address_rx << 1U) | dec->frame.bits[3U + i]);
                if (address_rx != IR_RC5_ADDRESS) return IR_DECODE_ERROR;

                data_type_rx = dec->frame.bits[5U];
                if (IrRC5_IsValidDataType(data_type_rx) == 0) return IR_DECODE_ERROR;

                for (i = 0U; i < 8U; i++) payload_rx = (uint8_t)((payload_rx << 1U) | dec->frame.bits[6U + i]);

                if (data_type_rx == (uint8_t)IR_DATA_MISC) {
                    if (IrRC5_IsValidMiscCode(payload_rx) == 0) return IR_DECODE_ERROR;
                } else {
                    int8_t x, y;
                    if (IrRC5_DecodeMovement(payload_rx, &x, &y) == 0) return IR_DECODE_ERROR;
                }

                dec->frame.toggle = dec->frame.bits[2U];
                dec->frame.address = address_rx;
                dec->frame.data_type = data_type_rx;
                dec->frame.payload = payload_rx;
                return IR_DECODE_DONE;
            }
        }
    }
    return IR_DECODE_BUSY;
}

int IrRC5_FrameToMisc(const IrRC5Frame *frame, uint8_t *misc_code) {
    if (frame == (void *)0 || misc_code == (void *)0) return 0;
    if (frame->data_type != (uint8_t)IR_DATA_MISC) return 0;
    if (IrRC5_IsValidMiscCode(frame->payload) == 0) return 0;
    *misc_code = frame->payload;
    return 1;
}

int IrRC5_FrameToMovement(const IrRC5Frame *frame, int8_t *x, int8_t *y) {
    if (frame == (void *)0) return 0;
    if (frame->data_type != (uint8_t)IR_DATA_MOVEMENT) return 0;
    return IrRC5_DecodeMovement(frame->payload, x, y);
}

// =========================================================================
//                          GLOBAL VARIABLES
// =========================================================================

volatile unsigned char pwm_L = 0;
volatile unsigned char dir_L = 1; 
volatile unsigned char pwm_R = 0;
volatile unsigned char dir_R = 1; 

IrRC5Decoder ir_decoder;
volatile IrDecodeStatus ir_status = IR_DECODE_BUSY;

// =========================================================================
//                          PROTOTYPES & HARDWARE
// =========================================================================

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
void LCD_PrintMotorCompact(unsigned char pwmL, unsigned char pwmR);

// State Machine Enums
typedef enum {
    MODE_MANUAL,
    MODE_CIRCLE,
    MODE_FIGURE_8,
    MODE_ROTATE_180 // <-- NEW STATE ADDED
} RobotMode;

// Helper function to print XY axes neatly
void LCD_PrintAxis(int8_t val)
{
    if (val < 0) { LCD_Data('-'); val = -val; }
    else { LCD_Data(' '); }
    LCD_Data('0' + val);
}

// Helper to print Misc codes
void LCD_PrintIRMisc(IrMiscCode cmd)
{
    LCD_SetCursor(0, 0);
    switch (cmd)
    {
        case IR_MISC_FORWARD:    LCD_Print("CMD: FORWARD    "); break;
        case IR_MISC_BACKWARD:   LCD_Print("CMD: BACKWARD   "); break;
        case IR_MISC_LEFT:       LCD_Print("CMD: PIVOT LEFT "); break;
        case IR_MISC_RIGHT:      LCD_Print("CMD: PIVOT RIGH "); break;
        case IR_MISC_STOP:       LCD_Print("CMD: STOPPED    "); break;
        case IR_MISC_ROTATE_180: LCD_Print("CMD: ROTATE 180 "); break; // <-- ADDED DISPLAY STRING
        default:                 LCD_Print("CMD: MISC CMD   "); break;
    }
}

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

// =========================================================================
//                          MAIN LOOP
// =========================================================================

void main(void)
{
    unsigned char btn_L_pressed = 0;
    unsigned char btn_R_pressed = 0;
    
    RobotMode current_mode = MODE_MANUAL;
    unsigned int mode_timer_ms = 0;
    unsigned char fig8_state = 0;

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
        // --- 1. Autopilot State Machine ---
        if (current_mode == MODE_CIRCLE)
        {
            dir_L = 0; dir_R = 0; 
            pwm_L = CURVE_FAST;
            pwm_R = ((unsigned int)CURVE_SLOW * RIGHT_WHEEL_TRIM) / 100;
            LCD_SetCursor(0, 0); LCD_Print("AUTOPLT: CIRCLE ");
        }
        else if (current_mode == MODE_FIGURE_8)
        {
            dir_L = 0; dir_R = 0; 
            mode_timer_ms += 30;  
            
            if (fig8_state == 0) 
            {
                pwm_L = CURVE_FAST; 
                pwm_R = ((unsigned int)CURVE_SLOW * RIGHT_WHEEL_TRIM) / 100;
                if (mode_timer_ms >= FIG8_TIME_MS) { mode_timer_ms = 0; fig8_state = 1; }
            } 
            else 
            {
                pwm_L = CURVE_SLOW; 
                pwm_R = ((unsigned int)CURVE_FAST * RIGHT_WHEEL_TRIM) / 100;
                if (mode_timer_ms >= FIG8_TIME_MS) { mode_timer_ms = 0; fig8_state = 0; }
            }
            LCD_SetCursor(0, 0); LCD_Print("AUTOPLT: FIG 8  ");
        }
        else if (current_mode == MODE_ROTATE_180) // <-- TIMED 180 SPIN LOGIC
        {
            // Spin in place (Left wheel Reverse, Right wheel Forward)
            dir_L = 1; 
            dir_R = 0; 
            pwm_L = 100; // Max speed
            pwm_R = RIGHT_WHEEL_TRIM; // Max trimmed speed
            
            mode_timer_ms += 30; // Increment timer by the loop delay (30ms)
            
            // When timer hits target, drop back to manual mode and stop
            if (mode_timer_ms >= TURN_180_TIME_MS) 
            {
                current_mode = MODE_MANUAL;
                pwm_L = 0; 
                pwm_R = 0; 
                LCD_SetCursor(0, 0); LCD_Print("CMD: STOPPED    ");
            }
        }

        // --- 2. Process IR Input ---
        if (ir_status == IR_DECODE_DONE)
        {
            if (ir_decoder.frame.data_type == IR_DATA_MISC)
            {
                uint8_t misc_code;
                if (IrRC5_FrameToMisc((IrRC5Frame *)&ir_decoder.frame, &misc_code))
                {
                    unsigned char default_pwm = 80; 
                    unsigned char trimmed_R = ((unsigned int)default_pwm * RIGHT_WHEEL_TRIM) / 100;

                    // FILTER: Ignore "Key Release" STOP commands if we are spinning
                    if (current_mode == MODE_ROTATE_180 && misc_code == IR_MISC_STOP)
                    {
                        // Do absolutely nothing! Let the spin timer finish.
                    }
                    else
                    {
                        LCD_PrintIRMisc(misc_code);

                        switch (misc_code)
                        {
                            case IR_MISC_START_PATH: current_mode = MODE_CIRCLE; break;
                            case IR_MISC_EDIT_PATH:  current_mode = MODE_FIGURE_8; mode_timer_ms = 0; fig8_state = 0; break;
                            case IR_MISC_FORWARD:    current_mode = MODE_MANUAL; dir_L = 0; dir_R = 0; pwm_L = default_pwm; pwm_R = trimmed_R; break;
                            case IR_MISC_BACKWARD:   current_mode = MODE_MANUAL; dir_L = 1; dir_R = 1; pwm_L = default_pwm; pwm_R = trimmed_R; break;
                            case IR_MISC_LEFT:       current_mode = MODE_MANUAL; dir_L = 1; dir_R = 0; pwm_L = default_pwm; pwm_R = trimmed_R; break; 
                            case IR_MISC_RIGHT:      current_mode = MODE_MANUAL; dir_L = 0; dir_R = 1; pwm_L = default_pwm; pwm_R = trimmed_R; break; 
                            case IR_MISC_STOP:       current_mode = MODE_MANUAL; pwm_L = 0; pwm_R = 0; break;
                            case IR_MISC_ROTATE_180: current_mode = MODE_ROTATE_180; mode_timer_ms = 0; break;
                            default: break; 
                        }
                    }
                }
            }
            else if (ir_decoder.frame.data_type == IR_DATA_MOVEMENT)
            {
                int8_t x, y;
                if (IrRC5_FrameToMovement((IrRC5Frame *)&ir_decoder.frame, &x, &y))
                {
                    // FILTER: Ignore "Neutral Joystick" release commands if we are spinning
                    if (current_mode == MODE_ROTATE_180 && x == 0 && y == 0)
                    {
                        // Do absolutely nothing! Let the spin timer finish.
                    }
                    else
                    {
                        int left_val, right_val;
                        
                        // User pushed the joystick! Cancel autopilot and take manual control.
                        current_mode = MODE_MANUAL; 

                        LCD_SetCursor(0, 0); 
                        LCD_Print("CMD: X:");
                        LCD_PrintAxis(x);
                        LCD_Print(" Y:");
                        LCD_PrintAxis(y);
                        LCD_Print(" ");

                        // Arcade Drive Mixer
                        left_val = ((int)y * 100) / 7 + ((int)x * 100) / 7;
                        right_val = ((int)y * 100) / 7 - ((int)x * 100) / 7;

                        if (left_val > 100) left_val = 100;
                        if (left_val < -100) left_val = -100;
                        if (right_val > 100) right_val = 100;
                        if (right_val < -100) right_val = -100;

                        if (left_val >= 0) { dir_L = 0; pwm_L = (unsigned char)left_val; }
                        else               { dir_L = 1; pwm_L = (unsigned char)(-left_val); }

                        if (right_val >= 0) { dir_R = 0; pwm_R = ((unsigned int)right_val * RIGHT_WHEEL_TRIM) / 100; }
                        else                { dir_R = 1; pwm_R = ((unsigned int)(-right_val) * RIGHT_WHEEL_TRIM) / 100; }
                    }
                }
            }
            
            EA = 0; IrRC5_DecoderReset((IrRC5Decoder *)&ir_decoder); ir_status = IR_DECODE_BUSY; EA = 1;
        }

        // --- Hardware Override Buttons ---
        if (BTN_L == 0) 
        { 
            if (btn_L_pressed == 0) 
            {
                current_mode = MODE_MANUAL;
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
                current_mode = MODE_MANUAL;
                LCD_SetCursor(0, 0); LCD_Print("CMD: R-BTN OVRD ");
                dir_R = !dir_R;
                pwm_R = 0;
                btn_R_pressed = 1;
            }
            if (pwm_R < RIGHT_WHEEL_TRIM) pwm_R++; 
        } 
        else btn_R_pressed = 0;

        LCD_PrintMotorCompact(pwm_L, pwm_R); 
        
        waitms(30); 
    }
}

// =========================================================================
//                          HARDWARE IMPLEMENTATION
// =========================================================================

void LCD_PrintMotorCompact(unsigned char pwmL, unsigned char pwmR)
{
    LCD_SetCursor(1, 0);
    
    LCD_Print("L:");
    if (pwmL == 100) { LCD_Print("100%"); }
    else if (pwmL >= 10) { LCD_Data(' '); LCD_Data('0' + (pwmL/10)); LCD_Data('0' + (pwmL%10)); LCD_Data('%'); }
    else { LCD_Print("  "); LCD_Data('0' + pwmL); LCD_Data('%'); }
    
    LCD_Print("  R:");
    if (pwmR >= 100) { LCD_Print("100%"); } 
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
