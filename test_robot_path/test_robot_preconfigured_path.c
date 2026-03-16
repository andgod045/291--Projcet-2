// ~C51~

#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>
#include <stdint.h>

// Robot automatically follows the guide wire using 3 pickup coils.
// Added intersection feature. Need to change path manually.

#define SYSCLK    72000000L
#define BAUDRATE  115200L
#define SARCLK    18000000L
#define VDD       3.3035

// Pin definitions
// Motor pins
#define MOTOR_L_FWD P2_4
#define MOTOR_L_REV P2_3
#define MOTOR_R_FWD P2_1
#define MOTOR_R_REV P2_2

// ADC coil pins
#define LEFT_COIL_PIN   QFP32_MUX_P1_0  // Left pickup coil on P1.0
#define RIGHT_COIL_PIN  QFP32_MUX_P1_7  // Right pickup coil on P1.7
#define CENTER_COIL_PIN QFP32_MUX_P2_5  // Center pickup coil on P2.5 for intersection

// Require calibration while testing
#define RIGHT_WHEEL_TRIM   90   // Right motor runs faster, scale it to 90%
#define DEFAULT_ROBOT_SPEED      60
#define STEER_GAIN         40   // How much to steer

// Intersection detection
#define INTERSECTION_THRESHOLD  1.5  // If center coil voltage above this = intersection (calibrate later)
#define TURN_SPEED              60   // PWM during pivot turns at intersections
#define TURN_TIME_MS            500  // How long to pivot at intersection (calibrate later)
#define AFTER_INTERSECTION_MS        300  // Drive forward briefly after turn to clear intersection

// Number of intersections and paths for project 2 requirement 9
#define NUM_INTERSECTIONS  8
#define NUM_PATHS          3

// Turn actions for path table
#define GO_FORWARD  0
#define GO_LEFT     1
#define GO_RIGHT    2
#define STOP     3

// pre_configured_path_table[path][intersection] = what to do at that intersection
unsigned char code pre_configured_path_table[NUM_PATHS][NUM_INTERSECTIONS] = {
//   Int1         Int2       Int3       Int4         Int5        Int6         Int7        Int8
    {GO_FORWARD, GO_LEFT,   GO_LEFT,   GO_FORWARD,  GO_RIGHT,  GO_LEFT,    GO_RIGHT,   STOP},        // Path 1
    {GO_LEFT,    GO_RIGHT,  GO_LEFT,   GO_RIGHT,    GO_FORWARD, GO_FORWARD, STOP,       STOP},        // Path 2
    {GO_RIGHT,   GO_FORWARD,GO_RIGHT,  GO_LEFT,     GO_RIGHT,  GO_LEFT,    GO_FORWARD, STOP}          // Path 3
};

// Which path to follow (0 = Path 1, 1 = Path 2, 2 = Path 3)
// change this manually to select a different path
unsigned char current_path = 0;

// Keeps track of which intersection we're at (0 = first intersection)
unsigned char intersection_count = 0;

// speed between 0 and 100
volatile unsigned char pwm_L = 0;   // Left motor speed
volatile unsigned char pwm_R = 0;   // Right motor speed
volatile unsigned char dir_L = 0;   // Left motor direction: 0=forward, 1=reverse
volatile unsigned char dir_R = 0;   // Right motor direction: 0=forward, 1=reverse

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key

	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
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
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif

	P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)
	XBR1     = 0X00;
	XBR2     = 0x40; // Enable crossbar and weak pull-ups

	// Configure Uart 0
	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	SCON0 = 0x10;
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	return 0;
}

// Uses Timer3 to delay <us> micro-seconds.
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter

	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;

	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow

	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
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

// To control the motors, adjust pwm_L, pwm_R, dir_L, dir_R in main.
// This ISR handles the actual pin toggling automatically.
// Timer 2 ISR (10kHz)
void Timer2_ISR(void) interrupt 5 using 1
{
    static unsigned char pwm_counter = 0;

    TF2H = 0;  // Clear Timer2 interrupt flag

    pwm_counter++;
    if (pwm_counter >= 100) pwm_counter = 0;

    // Left motor PWM
    if (pwm_L == 0) { MOTOR_L_FWD = 0; MOTOR_L_REV = 0; }
    else if (pwm_counter < pwm_L) {
        if (dir_L == 0) { MOTOR_L_FWD = 1; MOTOR_L_REV = 0; }  // Forward
        else             { MOTOR_L_FWD = 0; MOTOR_L_REV = 1; }  // Reverse
    } else { MOTOR_L_FWD = 0; MOTOR_L_REV = 0; }

    // Right motor PWM
    if (pwm_R == 0) { MOTOR_R_FWD = 0; MOTOR_R_REV = 0; }
    else if (pwm_counter < pwm_R) {
        if (dir_R == 0) { MOTOR_R_FWD = 1; MOTOR_R_REV = 0; }  // Forward
        else             { MOTOR_R_FWD = 0; MOTOR_R_REV = 1; }  // Reverse
    } else { MOTOR_R_FWD = 0; MOTOR_R_REV = 0; }
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC

	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32

	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.

	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)

	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2=
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)

	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;

	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

// If d1 > d2, move to one side.
// If d2 > d1, move to the other side.
// If d1 = d2, move straight.
// d1 and d2 are left and right coil voltages.
// Stronger voltage means coil is closer to the wire.
// The bigger the difference between left and right coil voltages,
// the sharper the turn the robot will make.
void follow_wire(float left_v, float right_v)
{
    float diff;
    int steer;
    int pwm_left, pwm_right;

    // Positive diff means left coil is stronger and robot is right of wire, so steer left
    // Negative diff means right coil is stronger and robot is left of wire, so steer right
    diff = left_v - right_v;

    // Scale diff to a steering value
    // Multiply by STEER_GAIN to control how much to correct robot direction to stay aligned with the wire.
    // Limit to +/- STEER_GAIN to prevent overcorrect.
    // +ve steer means wire is to the left, -ve steer means wire is to the right of the robot.
    // If steer is 0, wire is centered.
    steer = (int)(diff * STEER_GAIN);
    if (steer > STEER_GAIN) steer = STEER_GAIN;
    if (steer < -STEER_GAIN) steer = -STEER_GAIN;

    // Set both motors spinning the forward direction.
    dir_L = 0;
    dir_R = 0;

    // Adjust motor speeds
    // If detects left wire (+ve steer), slows down left motor
    // and speeds up right motor.
    // If detects right wire (-ve steer) slows down right motor
    // and speeds up left motor.
    pwm_left  = DEFAULT_ROBOT_SPEED - steer;
    pwm_right = DEFAULT_ROBOT_SPEED + steer;

    // Limit to valid range, 0 and 100.
    if (pwm_left > 100) pwm_left = 100;
    if (pwm_left < 0)   pwm_left = 0;
    if (pwm_right > 100) pwm_right = 100;
    if (pwm_right < 0)   pwm_right = 0;

    // Apply right wheel trim.
    pwm_right = (pwm_right * RIGHT_WHEEL_TRIM) / 100;

    // Set the global variables that the ISR uses to drive the motors
    pwm_L = pwm_left;
    pwm_R = pwm_right;
}

// Called when center pickup coil detects an intersection.
// Looks up path table and executes the action (forward, left, right, or stop).
// Returns 1 if path is complete, 0 else.
unsigned char intersection(void)
{
    unsigned char action;

    // don't go past 8 intersections
    if (intersection_count >= NUM_INTERSECTIONS)
        return 1;

    // Look up what to do at this intersection
    action = pre_configured_path_table[current_path][intersection_count];

    // Stop motors briefly before executing the action
    pwm_L = 0;
    pwm_R = 0;
    waitms(100);

    switch (action)
    {
        case GO_FORWARD:
            // Drive forward to pass through the intersection
            dir_L = 0; dir_R = 0;
            pwm_L = DEFAULT_ROBOT_SPEED;
            pwm_R = (DEFAULT_ROBOT_SPEED * RIGHT_WHEEL_TRIM) / 100;
            waitms(AFTER_INTERSECTION_MS);
            break;

        case GO_LEFT:
            // Turn left by having left wheel backward, right wheel forward
            dir_L = 1; dir_R = 0;
            pwm_L = TURN_SPEED; // Both motors running at turning speed, for duration of TURN_TIME_MS
            pwm_R = TURN_SPEED;
            waitms(TURN_TIME_MS);
            // Drive forward to clear intersection
            dir_L = 0; dir_R = 0;
            pwm_L = DEFAULT_ROBOT_SPEED; // Both motors now running at default speed
            pwm_R = (DEFAULT_ROBOT_SPEED * RIGHT_WHEEL_TRIM) / 100;
            waitms(AFTER_INTERSECTION_MS);
            break;

        case GO_RIGHT:
            // Turn right by having left wheel forward, right wheel backward
            dir_L = 0; dir_R = 1;
            pwm_L = TURN_SPEED; // Both motors running at turning speed, for duration of TURN_TIME_MS
            pwm_R = TURN_SPEED;
            waitms(TURN_TIME_MS);
            // Drive forward to clear the intersection
            dir_L = 0; dir_R = 0;
            pwm_L = DEFAULT_ROBOT_SPEED; // Both motors now running at default speed
            pwm_R = (DEFAULT_ROBOT_SPEED * RIGHT_WHEEL_TRIM) / 100;
            waitms(AFTER_INTERSECTION_MS);
            break;

        case STOP:
            // Path complete, stop motors
            pwm_L = 0;
            pwm_R = 0;
            return 1;
    }

    intersection_count++;
    return 0;
}

// Reads pickup coil voltages to follow the wire and handle intersection.
// Left and right pickup coils to guide the robot to follow the wire.
// Center pickup coil detects intersection.
void main(void)
{
    float left_v, right_v, center_v; // pickup coil voltages
    unsigned char path_done = 0;     // 1 = path complete, robot stops
    unsigned char previous_center_flag = 0; // Prevents calling intersection() multiple times when robot reaches the intersection

    waitms(500);

    // Configure motor pins
    SFRPAGE = 0x20;
    P2MDOUT |= 0x1E;  // P2.1, P2.2, P2.3, P2.4 = motor pins
    SFRPAGE = 0x00;

    // Motors off
    MOTOR_L_FWD = 0; MOTOR_L_REV = 0;
    MOTOR_R_FWD = 0; MOTOR_R_REV = 0;

    // Configure ADC for the 3 pickup coils
    InitPinADC(1, 0);  // Left coil on P1.0
    InitPinADC(1, 7);  // Right coil on P1.7
    InitPinADC(2, 5);  // Center coil on P2.5
    InitADC();

    // Start motor PWM timer
    Timer2_Init();

    // read coils -> check intersection -> steer -> loop
    while(1)
    {
        // Read voltages from the 3 peak detectors
        left_v   = Volts_at_Pin(LEFT_COIL_PIN);
        right_v  = Volts_at_Pin(RIGHT_COIL_PIN);
        center_v = Volts_at_Pin(CENTER_COIL_PIN);

        // If path is done, stop the robot
        if (path_done)
        {
            pwm_L = 0;
            pwm_R = 0;
        }
        // If center coil is strong, robot has hit an intersection
        else if (center_v > INTERSECTION_THRESHOLD)
        {   
            // To trigger intersection() only once while at intersection
            if(!previous_center_flag)
            {
                previous_center_flag = 1;
                path_done = intersection();
            }
        }
        // Else follow wire
        else
        {   
            previous_center_flag = 0; // resets the flag when robot is not at intersection
            follow_wire(left_v, right_v);
        }

        // Debug output via UART
        printf("\rL=%5.3fV R=%5.3fV C=%5.3fV  PWM: L=%3d R=%3d  Int:%d",
               left_v, right_v, center_v, (int)pwm_L, (int)pwm_R, (int)intersection_count);

        waitms(30);
    }
}