//gain of ~67.66, Gain of 1+(100k/1.5k)
//300 200 1k 100k(left to right)
// fn gen settings:
// vpp = 17Vpp,freq=16.67kHz
// 900 microhenry, cap across is 0.1 microfarads
// when gain used is ~11, 10k/1k, gives ~1.2V readings at ADC pins
// fn gen settings:
// 20Vpp,16.666kHz
#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>

#define SYSCLK 72000000L
#define BAUDRATE 115200L
#define SARCLK 18000000L

#define VDD 3.3035

#define LEFT_COIL_PIN   QFP32_MUX_P1_0
#define RIGHT_COIL_PIN  QFP32_MUX_P1_7
#define CENTER_COIL_PIN QFP32_MUX_P2_5

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

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC

	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift.
		(0x0 << 0) ; // Accumulate 1 conversion

	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz.
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK.

	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode.
		(0x1E << 0); // Conversion Tracking Time.

	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on.
		(0x0 << 5) | // ADINT.
		(0x0 << 4) | // ADBUSY.
		(0x0 << 3) | // ADWINT.
		(0x0 << 2) | // ADGN. 0x0: PGA gain=1.
		(0x0 << 0) ; // TEMPE. 0: Disable Temperature Sensor.

	ADC0CF2=
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin.
		(0x1 << 5) | // REFSL. 0x1: VDD pin.
		(0x1F << 0); // ADPWR. Power Up Delay Time.

	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: ADC accumulator is over-written.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY trigger.

	ADEN=1; // Enable ADC
}

// Uses Timer3 to delay <us> micro-seconds.
void Timer3us(unsigned char us)
{
	unsigned char i;

	CKCON0|=0b_0100_0000;

	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;

	TMR3CN0 = 0x04;                 // Start Timer3 and clear overflow flag
	for (i = 0; i < us; i++)
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

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;

	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask;    // Skip Crossbar decoding for this pin
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

void main(void){
	float left_v, right_v, center_v;
	//float diff;

    waitms(500);
	
	InitPinADC(1, 0); // Left coil on P1.0
	InitPinADC(1, 7); // Right coil on P1.7
	InitPinADC(2, 5); // Center coil on P2.5
    InitADC();
	
	while(1)
	{
		
		// Read DC voltage from each peak detector
		left_v   = Volts_at_Pin(LEFT_COIL_PIN);
		right_v  = Volts_at_Pin(RIGHT_COIL_PIN);
		center_v = Volts_at_Pin(CENTER_COIL_PIN);
		
		printf("\rL=%5.3fV  R=%5.3fV  C=%5.3fV",left_v, right_v, center_v);
			   
		waitms(200);

	}
}




