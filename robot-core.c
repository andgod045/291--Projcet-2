#include <stdio.h>
#include <EFM8LB1.h>

#define SYSCLK 72000000L // 72 MHz System Clock

//==================== LCD Pins ====================
#define LCD_RS  P1_6
#define LCD_E   P1_5
#define LCD_D4  P1_4
#define LCD_D5  P1_3
#define LCD_D6  P1_2
#define LCD_D7  P1_1

//==================== Button & Motor Pins ====================
#define BTN_L       P2_6  // Left Motor Button
#define BTN_R       P3_1  // Right Motor Button

#define MOTOR_L_FWD P2_1  // Left Motor Positive
#define MOTOR_L_REV P2_2  // Left Motor Negative
#define MOTOR_R_FWD P2_3  // Right Motor Positive
#define MOTOR_R_REV P2_4  // Right Motor Negative

//==================== Global PWM Variables ====================
// Declared volatile to prevent optimization issues inside the ISR
volatile unsigned char pwm_L = 0;
volatile unsigned char dir_L = 1; 

volatile unsigned char pwm_R = 0;
volatile unsigned char dir_R = 1; 

//==================== Prototypes ====================
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
void LCD_PrintMotorStatus(unsigned char is_right, unsigned char dir, unsigned char pwm);
void LCD_Init(void);

//==================== PWM Interrupt Service Routine ====================
// Runs 10,000 times a second to manage the duty cycle of both motors
void Timer2_ISR(void) interrupt 5 using 1 
{
    static unsigned char pwm_counter = 0;
    
    TF2H = 0; // Clear Timer 2 interrupt flag
    
    pwm_counter++;
    if(pwm_counter >= 100) pwm_counter = 0;

    // --- Left Motor Logic ---
    if (pwm_L == 0) 
    {
        MOTOR_L_FWD = 0; MOTOR_L_REV = 0;
    } 
    else if (pwm_counter < pwm_L) 
    {
        if (dir_L == 0) { MOTOR_L_FWD = 1; MOTOR_L_REV = 0; } 
        else            { MOTOR_L_FWD = 0; MOTOR_L_REV = 1; }
    } 
    else 
    {
        MOTOR_L_FWD = 0; MOTOR_L_REV = 0;
    }

    // --- Right Motor Logic ---
    if (pwm_R == 0) 
    {
        MOTOR_R_FWD = 0; MOTOR_R_REV = 0;
    } 
    else if (pwm_counter < pwm_R) 
    {
        if (dir_R == 0) { MOTOR_R_FWD = 1; MOTOR_R_REV = 0; } 
        else            { MOTOR_R_FWD = 0; MOTOR_R_REV = 1; }
    } 
    else 
    {
        MOTOR_R_FWD = 0; MOTOR_R_REV = 0;
    }
}

//==================== Main Program ====================
void main(void)
{
    unsigned char btn_L_pressed = 0;
    unsigned char btn_R_pressed = 0;

    _c51_external_startup();

    // --- Port Configuration ---
    SFRPAGE = 0x20;
    
    // Set LCD pins as Push-Pull output
    P1MDOUT |= 0x7E; 
    
    // Set Motor pins (P2.1 to P2.4) as Push-Pull output
    P2MDOUT |= 0x1E; 
    
    // Set P2.6 and P3.1 (Buttons) as Digital Inputs
    P2MDIN  |= 0x40;   
    P2MDOUT &= ~0x40; // Open-drain
    
    P3MDIN  |= 0x02;
    P3MDOUT &= ~0x02; // Open-drain
    
    SFRPAGE = 0x00;
    P2 |= 0x40; // Enable pull-up for P2.6
    P3 |= 0x02; // Enable pull-up for P3.1

    // --- System Initialization ---
    MOTOR_L_FWD = 0; MOTOR_L_REV = 0;
    MOTOR_R_FWD = 0; MOTOR_R_REV = 0;
    Timer2_Init();
    
    LCD_Init();
    LCD_SetCursor(0, 0);
    LCD_Print("L: STOP      0% ");
    LCD_SetCursor(1, 0);
    LCD_Print("R: STOP      0% ");

    // --- Main Control Loop ---
    while(1)
    {
        // --- Process Left Button ---
        if (BTN_L == 0) 
        { 
            if (btn_L_pressed == 0) 
            {
                dir_L = !dir_L;
                pwm_L = 0;
                btn_L_pressed = 1;
            }
            if (pwm_L < 100) pwm_L++; // Ramp up
        } 
        else 
        {
            pwm_L = 0; // Immediate stop
            btn_L_pressed = 0;
        }

        // --- Process Right Button ---
        if (BTN_R == 0) 
        { 
            if (btn_R_pressed == 0) 
            {
                dir_R = !dir_R;
                pwm_R = 0;
                btn_R_pressed = 1;
            }
            if (pwm_R < 100) pwm_R++; // Ramp up
        } 
        else 
        {
            pwm_R = 0; // Immediate stop
            btn_R_pressed = 0;
        }

        // --- Update LCD Display ---
        LCD_PrintMotorStatus(0, dir_L, pwm_L); // Update Left
        LCD_PrintMotorStatus(1, dir_R, pwm_R); // Update Right
        
        // Loop delay dictates the speed of the ramp-up (30ms * 100 steps = 3 seconds)
        waitms(30); 
    }
}

//==================== System Setup & LCD Functions ====================
// Helper function to format the motor status beautifully on the LCD
void LCD_PrintMotorStatus(unsigned char is_right, unsigned char dir, unsigned char pwm)
{
    LCD_SetCursor(is_right, 0); // Row 0 for Left, Row 1 for Right
    
    if (is_right) LCD_Data('R');
    else          LCD_Data('L');
    LCD_Print(": ");

    if (pwm == 0)      LCD_Print("STOP  ");
    else if (dir == 0) LCD_Print("FWD   ");
    else               LCD_Print("REV   ");

    // Print percentage right-aligned
    if (pwm == 100) { LCD_Data('1'); LCD_Data('0'); LCD_Data('0'); }
    else if (pwm >= 10) { LCD_Data(' '); LCD_Data('0' + (pwm/10)); LCD_Data('0' + (pwm%10)); }
    else { LCD_Data(' '); LCD_Data(' '); LCD_Data('0' + pwm); }
    LCD_Print("% ");
}

void Timer2_Init(void)
{
    TMR2CN0 = 0x00; 
    CKCON0 |= 0x10; 
    TMR2RL = 65536 - 7200; 
    TMR2 = TMR2RL;
    
    IE |= 0x20; // ET2 = 1
    EA = 1;     // Enable Global Interrupts
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
