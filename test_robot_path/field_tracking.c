// ~C51~

#include <stdlib.h>
#include <EFM8LB1.h>
#include <stdint.h>

// ---------------- System ----------------
#define SYSCLK    72000000L
#define BAUDRATE  115200L
#define SARCLK    18000000L
#define VDD       3.3035

// ---------------- Pins ----------------
#define MOTOR_L_FWD P2_4
#define MOTOR_L_REV P2_3
#define MOTOR_R_FWD P2_1
#define MOTOR_R_REV P2_2

#define LEFT_COIL_PIN   QFP32_MUX_P1_0
#define RIGHT_COIL_PIN  QFP32_MUX_P2_6
#define CENTER_COIL_PIN QFP32_MUX_P0_1

#define LCD_RS  P1_6
#define LCD_E   P1_5
#define LCD_D4  P1_4
#define LCD_D5  P1_3
#define LCD_D6  P1_2
#define LCD_D7  P1_1

// ---------------- Manual Path Select ----------------
// 0 = Path 1, 1 = Path 2, 2 = Path 3
#define MANUAL_PATH_INDEX 1
#if (MANUAL_PATH_INDEX > 2)
#error MANUAL_PATH_INDEX must be 0, 1, or 2
#endif

// ---------------- Wire Follow Tuning ----------------
#define RIGHT_WHEEL_TRIM      90
#define DEFAULT_ROBOT_SPEED   40 // Base PWM for both wheels during straight-line driving
#define STEER_GAIN            34 // Multiplier on (left-right) coil difference to compute steering correction
#define FOLLOW_TURN_SPEED     12 // Inner wheel PWM during sharp pivot corrections in follow_wire mode
#define PIVOT_THRESHOLD       38 // Steer value above which follow_wire does a full pivot instead of differential speed
#define COIL_DIFF_OFFSET_V    0.12f // // Added to (left-right) diff to compensate for physical coil imbalance

// ---------------- Intersection Detect ----------------
//following INTERSECTION_THRESHOLD_HIGH constant work for the following settings: 16.466,667KHz, 19Vpp on fn generator
#define INTERSECTION_THRESHOLD_HIGH   0.75 // Center coil voltage above this = intersection detected
#define INTERSECTION_THRESHOLD_LOW    0.25f // Center coil must drop below this to trigger the detector for center coil reading spike
#define INTERSECTION_DEBOUNCE_COUNT   2  // Consecutive samples above high (or below low) needed to confirm
#define INTERSECTION_COOLDOWN_MS      7000 // After detecting an intersection, ignore center spikes for this long
#define STARTUP_INTERSECTION_LOCKOUT_MS 2000 // Ignore center spikes for this long after power-on

// ---------------- Intersection Action ----------------
/*3 for turn-exit behavior:

TURN_CENTER_LOW_THRESHOLD
TURN_MIN_MS
TURN_TIMEOUT_MS

*/
#define ACTION_SETTLE_STOP_MS         50 // Pause after stopping before starting a turn, lets momentum settle
#define BEFORE_ACTION_FORWARD_MS      80 // Drive forward this long to center robot on intersection before turning
#define AFTER_INTERSECTION_MS         120 // Drive forward this long after completing a turn to clear the intersection
#define AFTER_INTERSECTION_SPEED      ((DEFAULT_ROBOT_SPEED > 30) ? (DEFAULT_ROBOT_SPEED - 30) : 10) // Slower forward speed after turn to avoid overshooting the wire

//following intersection constants work for the following settings: 16.466,667KHz, 19Vpp on fn generator
// Diagonal-turn controls (pivot -> forward nudge -> settle -> sample)
#define TURN_SPEED_INTERSECTION       30//40   // Pivot strength per cycle.
#define TURN_STEP_MS                  650   // Pivot duration per cycle.
#define TURN_FORWARD_NUDGE_SPEED      25//40   // Forward nudge speed per cycle.
#define TURN_FORWARD_NUDGE_MS         350   // Forward nudge duration per cycle.
#define TURN_SAMPLE_SETTLE_MS         30   // Settle interval before center read.

#define TURN_MIN_MS                   300 // Minimum turn time before sensor exit is allowed
#define TURN_TIMEOUT_MS               5000 // Maximum turn time before failure
#define TURN_CENTER_LOW_THRESHOLD     0.03f //During turn: center voltage threshold to stop turning (alignment exit).
#define TURN_EXIT_DEBOUNCE_COUNT      2 // Consecutive low-center samples required before exiting turn.

// ---------------- UART Debug Log ----------------
#define DEBUG_UART_ENABLE             1    // 1=enable UART log output, 0=disable.
#define DEBUG_LOG_EVERY_LOOPS         4    // 4 * 30ms = ~120ms log period.

#define LOOP_PERIOD_MS                30

#define NUM_PATHS                     3
#define NUM_INTERSECTIONS             8

#define GO_FORWARD                    0
#define GO_LEFT                       1
#define GO_RIGHT                      2
#define STOP                          3

#define CENTER_READY                  0
#define CENTER_TRIGGERED                  1

volatile unsigned char pwm_L = 0;
volatile unsigned char pwm_R = 0;
volatile unsigned char dir_L = 0; // 0=forward, 1=reverse
volatile unsigned char dir_R = 0; // 0=forward, 1=reverse

unsigned char current_path = 0;
unsigned char intersection_count = 0;
unsigned char path_done = 0;

unsigned int cooldown_ms = 0; // Remaining lockout time (ms) before next intersection can be detected
unsigned char center_state = CENTER_READY; // Intersection detector state: READY = looking for spike, TRIGGERED = waiting for drop
unsigned char high_debounce = 0; // Consecutive loops center_v has been above HIGH threshold
unsigned char low_debounce = 0;  // Consecutive loops center_v has been below LOW threshold
unsigned char debug_log_div = 0;

void UpdateDisplayStatus(float left_v, float right_v, float center_v);

#if DEBUG_UART_ENABLE
void UART_SendChar(char c)
{
    while (!TI);
    TI = 0;
    SBUF0 = c;
}

void UART_Send2(unsigned char v)
{
    UART_SendChar((char)('0' + ((v / 10) % 10)));
    UART_SendChar((char)('0' + (v % 10)));
}

void UART_Send4(unsigned int v)
{
    UART_SendChar((char)('0' + ((v / 1000) % 10)));
    UART_SendChar((char)('0' + ((v / 100) % 10)));
    UART_SendChar((char)('0' + ((v / 10) % 10)));
    UART_SendChar((char)('0' + (v % 10)));
}
#endif

unsigned char code pre_configured_path_table[NUM_PATHS][NUM_INTERSECTIONS] =
{
    //   Int1         Int2       Int3       Int4         Int5        Int6         Int7        Int8
    { GO_FORWARD, GO_LEFT,   GO_LEFT,   GO_FORWARD,  GO_RIGHT,  GO_LEFT,    GO_RIGHT,   STOP }, // Path 1
    { GO_LEFT,    GO_RIGHT,  GO_LEFT,   GO_RIGHT,    GO_FORWARD, GO_FORWARD, STOP,       STOP }, // Path 2
    { GO_RIGHT,   GO_FORWARD,GO_RIGHT,  GO_LEFT,     GO_RIGHT,  GO_LEFT,    GO_FORWARD, STOP }  // Path 3
};

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE;
	WDTCN = 0xAD;

	VDM0CN=0x80;
	RSTSRC=0x02|0x04;

	#if (SYSCLK == 48000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x10;
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20;
		SFRPAGE = 0x00;
	#endif

	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif

	P0MDOUT |= 0x10;
	XBR0     = 0x01;
	XBR1     = 0X00;
	XBR2     = 0x40;

	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;
	TMOD &= ~0xf0;
	TMOD |=  0x20;
	TR1 = 1;
	TI = 1;

	return 0;
}

void Timer3us(unsigned char us)
{
	unsigned char i;

	CKCON0|=0b_0100_0000;

	TMR3RL = (-(SYSCLK)/1000000L);
	TMR3 = TMR3RL;

	TMR3CN0 = 0x04;
	for (i = 0; i < us; i++)
	{
		while (!(TMR3CN0 & 0x80));
		TMR3CN0 &= ~(0x80);
	}
	TMR3CN0 = 0;
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
	{
		for (k=0; k<4; k++) Timer3us(250);
	}
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

void Timer2_Init(void)
{
    TMR2CN0 = 0x00;
    CKCON0 |= 0x10;
    TMR2RL = 65536 - 7200; // 10 kHz ISR rate
    TMR2 = TMR2RL;
    IE |= 0x20;
    EA = 1;
    TR2 = 1;
}

void Timer2_ISR(void) interrupt 5 using 1
{
    static unsigned char pwm_counter = 0;

    TF2H = 0;

    pwm_counter++;
    if (pwm_counter >= 100) pwm_counter = 0;

    if (pwm_L == 0)
    {
        MOTOR_L_FWD = 0;
        MOTOR_L_REV = 0;
    }
    else if (pwm_counter < pwm_L)
    {
        if (dir_L == 0) { MOTOR_L_FWD = 1; MOTOR_L_REV = 0; }
        else            { MOTOR_L_FWD = 0; MOTOR_L_REV = 1; }
    }
    else
    {
        MOTOR_L_FWD = 0;
        MOTOR_L_REV = 0;
    }

    if (pwm_R == 0)
    {
        MOTOR_R_FWD = 0;
        MOTOR_R_REV = 0;
    }
    else if (pwm_counter < pwm_R)
    {
        if (dir_R == 0) { MOTOR_R_FWD = 1; MOTOR_R_REV = 0; }
        else            { MOTOR_R_FWD = 0; MOTOR_R_REV = 1; }
    }
    else
    {
        MOTOR_R_FWD = 0;
        MOTOR_R_REV = 0;
    }
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0;

	ADC0CN1=
		(0x2 << 6) |
        (0x0 << 3) |
		(0x0 << 0) ;

	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) |
		(0x0 << 2);

	ADC0CF1=
		(0 << 7)   |
		(0x1E << 0);

	ADC0CN0 =
		(0x0 << 7) |
		(0x0 << 6) |
		(0x0 << 5) |
		(0x0 << 4) |
		(0x0 << 3) |
		(0x0 << 2) |
		(0x0 << 0) ;

	ADC0CF2=
		(0x0 << 7) |
		(0x1 << 5) |
		(0x1F << 0);

	ADC0CN2 =
		(0x0 << 7) |
		(0x0 << 0) ;

	ADEN=1;
}

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask);
			P0SKIP |= mask;
		break;
		case 1:
			P1MDIN &= (~mask);
			P1SKIP |= mask;
		break;
		case 2:
			P2MDIN &= (~mask);
			P2SKIP |= mask;
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;
	ADINT = 0;
	ADBUSY = 1;
	while (!ADINT);
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

unsigned char trim_right_pwm(unsigned char pwm)
{
    unsigned int scaled;
    scaled = ((unsigned int)pwm * RIGHT_WHEEL_TRIM) / 100;
    if (scaled > 100) scaled = 100;
    return (unsigned char)scaled;
}

void stop_motors(void)
{
    pwm_L = 0;
    pwm_R = 0;
    dir_L = 0;
    dir_R = 0;
}

void set_forward(unsigned char left_pwm, unsigned char right_pwm)
{
    dir_L = 0;
    dir_R = 0;
    pwm_L = left_pwm;
    pwm_R = trim_right_pwm(right_pwm);
}

void pivot_left(unsigned char speed)
{
    dir_L = 1;
    dir_R = 0;
    pwm_L = speed;
    pwm_R = trim_right_pwm(speed);
}

void pivot_right(unsigned char speed)
{
    dir_L = 0;
    dir_R = 1;
    pwm_L = speed;
    pwm_R = trim_right_pwm(speed);
}

void follow_wire(float left_v, float right_v)
{
    float diff;
    int steer;
    int pwm_left;
    int pwm_right;

    diff = (left_v - right_v) + COIL_DIFF_OFFSET_V;

    steer = (int)(diff * STEER_GAIN);
    if (steer > STEER_GAIN) steer = STEER_GAIN;
    if (steer < -STEER_GAIN) steer = -STEER_GAIN;

    if (steer > PIVOT_THRESHOLD)
    {
        // Sharp left correction
        dir_L = 1;
        dir_R = 0;
        pwm_L = FOLLOW_TURN_SPEED;
        pwm_R = DEFAULT_ROBOT_SPEED;
    }
    else if (steer < -PIVOT_THRESHOLD)
    {
        // Sharp right correction
        dir_L = 0;
        dir_R = 1;
        pwm_L = DEFAULT_ROBOT_SPEED;
        pwm_R = FOLLOW_TURN_SPEED;
    }
    else
    {
        dir_L = 0;
        dir_R = 0;

        pwm_left  = DEFAULT_ROBOT_SPEED - steer;
        pwm_right = DEFAULT_ROBOT_SPEED + steer;

        if (pwm_left > 100) pwm_left = 100;
        if (pwm_left < 0)   pwm_left = 0;
        if (pwm_right > 100) pwm_right = 100;
        if (pwm_right < 0)   pwm_right = 0;

        pwm_L = (unsigned char)pwm_left;
        pwm_R = trim_right_pwm((unsigned char)pwm_right);
    }
}

unsigned char execute_turn(unsigned char turn_left)
{
    unsigned int elapsed_ms;
    unsigned int pivot_elapsed_ms;
    unsigned char low_confirm_count;
    float center_v;

    stop_motors();
    waitms(ACTION_SETTLE_STOP_MS);

    set_forward(DEFAULT_ROBOT_SPEED, DEFAULT_ROBOT_SPEED);
    waitms(BEFORE_ACTION_FORWARD_MS);

    elapsed_ms = 0;
    pivot_elapsed_ms = 0;
    low_confirm_count = 0;
    while (elapsed_ms < TURN_TIMEOUT_MS)
    {
        // Pivot a little in commanded direction
        if (turn_left) pivot_left(TURN_SPEED_INTERSECTION);
        else           pivot_right(TURN_SPEED_INTERSECTION);

        waitms(TURN_STEP_MS);
        elapsed_ms += TURN_STEP_MS;
        pivot_elapsed_ms += TURN_STEP_MS;

        // Move forward a little so robot progresses diagonally off the cross center
        set_forward(TURN_FORWARD_NUDGE_SPEED, TURN_FORWARD_NUDGE_SPEED);
        waitms(TURN_FORWARD_NUDGE_MS);
        elapsed_ms += TURN_FORWARD_NUDGE_MS;

        // Stop, settle, sample center
        stop_motors();
        waitms(TURN_SAMPLE_SETTLE_MS);
        elapsed_ms += TURN_SAMPLE_SETTLE_MS;
        center_v = Volts_at_Pin(CENTER_COIL_PIN);

        // Exit only after minimum pivot time + consecutive low-center confirmations
        if ((pivot_elapsed_ms >= TURN_MIN_MS) && (center_v < TURN_CENTER_LOW_THRESHOLD))
        {
            if (low_confirm_count < 255) low_confirm_count++;
            if (low_confirm_count >= TURN_EXIT_DEBOUNCE_COUNT)
            {
                stop_motors();
                return 1;
            }
        }
        else
        {
            low_confirm_count = 0;
        }
    }

    stop_motors();
    return 0;
}

unsigned char current_action(void)
{
    if (intersection_count >= NUM_INTERSECTIONS) return STOP;
    return pre_configured_path_table[current_path][intersection_count];
}

void UpdateDisplayStatus(float left_v, float right_v, float center_v)
{
    unsigned int mv_l, mv_r, mv_c;
    unsigned char d1, d2, d3;
    unsigned char i_show;
    char line1[17] = "L:0.00 R:0.00   ";
    char line2[17] = "C:0.00 I00      ";

    mv_l = (unsigned int)(left_v * 1000.0f);
    mv_r = (unsigned int)(right_v * 1000.0f);
    mv_c = (unsigned int)(center_v * 1000.0f);
    if (mv_l > 3304) mv_l = 3304;
    if (mv_r > 3304) mv_r = 3304;
    if (mv_c > 3304) mv_c = 3304;

    d1 = mv_l / 1000;
    d2 = (mv_l % 1000) / 100;
    d3 = (mv_l % 100) / 10;
    line1[2] = '0' + d1;
    line1[4] = '0' + d2;
    line1[5] = '0' + d3;

    d1 = mv_r / 1000;
    d2 = (mv_r % 1000) / 100;
    d3 = (mv_r % 100) / 10;
    line1[9]  = '0' + d1;
    line1[11] = '0' + d2;
    line1[12] = '0' + d3;

    d1 = mv_c / 1000;
    d2 = (mv_c % 1000) / 100;
    d3 = (mv_c % 100) / 10;
    line2[2] = '0' + d1;
    line2[4] = '0' + d2;
    line2[5] = '0' + d3;

    i_show = intersection_count;
    if (i_show > 99) i_show = 99;
    line2[8] = '0' + (i_show / 10);
    line2[9] = '0' + (i_show % 10);

    LCD_SetCursor(0, 0);
    LCD_Print(line1);
    LCD_SetCursor(1, 0);
    LCD_Print(line2);
}

unsigned char intersection_action(void)
{
    unsigned char action;
    unsigned char turn_ok;

    if (path_done) return 0;

    action = current_action();

    switch(action)
    {
        case GO_FORWARD:
            set_forward(AFTER_INTERSECTION_SPEED, AFTER_INTERSECTION_SPEED);
            waitms(AFTER_INTERSECTION_MS);
            return 1;

        case GO_LEFT:
            turn_ok = execute_turn(1);
            if (turn_ok)
            {
                set_forward(AFTER_INTERSECTION_SPEED, AFTER_INTERSECTION_SPEED);
                waitms(AFTER_INTERSECTION_MS);
                return 1;
            }
            return 0;

        case GO_RIGHT:
            turn_ok = execute_turn(0);
            if (turn_ok)
            {
                set_forward(AFTER_INTERSECTION_SPEED, AFTER_INTERSECTION_SPEED);
                waitms(AFTER_INTERSECTION_MS);
                return 1;
            }
            return 0;

        case STOP:
        default:
            stop_motors();
            path_done = 1;
            return 1;
    }
}

void main(void)
{
    float left_v;
    float right_v;
    float center_v;
    unsigned char action_ok;
    unsigned char triggered;

    // ---- INITIALIZATION ----

    waitms(500); // Let power supply settle before doing anything

    current_path = MANUAL_PATH_INDEX; // Select which path to execute (hardcoded for now)

    // Configure GPIO pins as push-pull outputs
    SFRPAGE = 0x20;
    P1MDOUT |= 0x7E; // LCD pins P1.1..P1.6
    P2MDOUT |= 0x1E; // Motor pins P2.1..P2.4
    SFRPAGE = 0x00;

    stop_motors(); // Make sure motors are off at startup

    // Configure ADC input pins for the 3 pickup coils
    InitPinADC(1, 0); // Left coil on P1.0
    InitPinADC(2, 6); // Right coil on P2.6
    InitPinADC(0, 1); // Center coil on P0.1
    InitADC();

    Timer2_Init(); // Start 10kHz PWM timer for motor control
    LCD_Init();    // LCD init (robot-core format)

    // Show startup splash screen
    LCD_SetCursor(0, 0); LCD_Print("FIELD TRACKER   ");
    LCD_SetCursor(1, 0); LCD_Print("INIT PATH       ");
    waitms(300);

    // Prevent false first-turn on power-up while ADC/magnetics settle.
    cooldown_ms = STARTUP_INTERSECTION_LOCKOUT_MS;
    high_debounce = 0;
    low_debounce = 0;

    // ---- MAIN LOOP (runs every 30ms) ----

    while(1)
    {
        triggered = 0; // Reset intersection trigger flag each iteration

        // Tick down the cooldown timer (prevents double-detecting same intersection)
        if (cooldown_ms > LOOP_PERIOD_MS) cooldown_ms -= LOOP_PERIOD_MS;
        else if (cooldown_ms > 0) cooldown_ms = 0;

        // Read all 3 pickup coils
        left_v   = Volts_at_Pin(LEFT_COIL_PIN);
        right_v  = Volts_at_Pin(RIGHT_COIL_PIN);
        center_v = Volts_at_Pin(CENTER_COIL_PIN);

        if (!path_done) // Robot is still executing a path
        {
            if (center_state == CENTER_READY) // Detector is triggered, looking for a spike
            {
				// Only check if cooldown has expired and center voltage exceeds the high threshold
                if ((cooldown_ms == 0) && (center_v > INTERSECTION_THRESHOLD_HIGH))
                {
                    // Increment debounce counter (require multiple consecutive high readings to confirm)
                    if (high_debounce < 255) high_debounce++;
                    low_debounce = 0; // Reset low counter since we're seeing high values

                    // If enough consecutive high readings, confirm this is a real intersection
                    if (high_debounce >= INTERSECTION_DEBOUNCE_COUNT)
                    {
                        center_state = CENTER_TRIGGERED;  // Lock detector, won't fire again until voltage drops
                        high_debounce = 0; // Reset for next intersection
                        triggered = 1;  // Flag so follow_wire is skipped this loop

                        // Run the path table action (turn left/right, go forward, or stop)
                        // This call blocks until the maneuver finishes
                        action_ok = intersection_action();

                        // Always advance intersection count regardless of turn success/failure
                        if ((!path_done) && (intersection_count < 255))
                        {
                            intersection_count++;
                        }

                        // Start cooldown, ignore all center spikes for 7 seconds
						// so the same intersection isn't detected twice
                        cooldown_ms = INTERSECTION_COOLDOWN_MS;
                    }
                }
                else
                {
                    high_debounce = 0; // Reset debounce if voltage dropped before confirming
                }
            }
            else // center_state == CENTER_TRIGGERED, waiting for voltage to drop
            {
                if (center_v < INTERSECTION_THRESHOLD_LOW) // Check if center voltage has fallen below the low threshold
                {
                    if (low_debounce < 255) low_debounce++; // Count consecutive low readings
                    if (low_debounce >= INTERSECTION_DEBOUNCE_COUNT) // Confirmed stable low
                    {
                        center_state = CENTER_READY; // Turn detector back on for next intersection
                        low_debounce = 0;
                    }
                }
                else
                {
                    low_debounce = 0; // Reset if voltage bounced back up
                }
            }

            // ---- WIRE FOLLOWING MODE ----
            if ((!triggered) && (!path_done))
            {
                follow_wire(left_v, right_v); // Normal steering using left/right coils
            }
        }
        else // Path is complete (hit a STOP action)
        {
            stop_motors(); // Keep motors off
        }

        UpdateDisplayStatus(left_v, right_v, center_v); // L:, R:, C:, Ixx

#if DEBUG_UART_ENABLE
        debug_log_div++;
        if (debug_log_div >= DEBUG_LOG_EVERY_LOOPS)
        {
            unsigned int adc_l;
            unsigned int adc_r;
            unsigned int adc_c;
            unsigned int mv_l;
            unsigned int mv_r;
            unsigned int mv_c;

            debug_log_div = 0;
            adc_l = ADC_at_Pin(LEFT_COIL_PIN);
            adc_r = ADC_at_Pin(RIGHT_COIL_PIN);
            adc_c = ADC_at_Pin(CENTER_COIL_PIN);

            // Normalize possible left-justified 14-bit ADC values before mV conversion.
            if (adc_l > 16383U) adc_l >>= 2;
            if (adc_r > 16383U) adc_r >>= 2;
            if (adc_c > 16383U) adc_c >>= 2;

            mv_l = (unsigned int)(((unsigned long)adc_l * 3304UL) / 16383UL);
            mv_r = (unsigned int)(((unsigned long)adc_r * 3304UL) / 16383UL);
            mv_c = (unsigned int)(((unsigned long)adc_c * 3304UL) / 16383UL);

            UART_SendChar('L'); UART_SendChar('=');
            UART_Send4(mv_l);
            UART_SendChar(' ');
            UART_SendChar('R'); UART_SendChar('=');
            UART_Send4(mv_r);
            UART_SendChar(' ');
            UART_SendChar('C'); UART_SendChar('=');
            UART_Send4(mv_c);
            UART_SendChar(' ');
            UART_SendChar('P'); UART_SendChar('L'); UART_SendChar('=');
            UART_Send2(pwm_L);
            UART_SendChar(' ');
            UART_SendChar('P'); UART_SendChar('R'); UART_SendChar('=');
            UART_Send2(pwm_R);
            UART_SendChar(' ');
            UART_SendChar('D'); UART_SendChar('L'); UART_SendChar('=');
            UART_SendChar((char)('0' + dir_L));
            UART_SendChar(' ');
            UART_SendChar('D'); UART_SendChar('R'); UART_SendChar('=');
            UART_SendChar((char)('0' + dir_R));
            UART_SendChar('\r');
            UART_SendChar('\n');
        }
#endif

        waitms(LOOP_PERIOD_MS); // Wait 30ms before next iteration
    }
}
