/*
		Joshua Malek, Bryce Northrop, Caroline Zaw
		11/2/18
		Section 04
		Side B
		Lab 4
		Description: This program controls the steering servo and drive motor of a smart car, driving the
		car in a specified direction, stopping for obstacles, and backing up when it detects a bright light.	*/

#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

// Function Prototypes

// Drive Motor Control
void speed_control(void); // adjusts PW_FWD_SC based on potentiometer reading
unsigned int read_ranger(void); // reads the range from the ranger
unsigned int pw_modify(unsigned int range); // chooses a pulsewidth based on ranger data and PW_FWD_SC
void Drive_Motor(unsigned int pw_run); // sets the drive motor to a specified pulsewidth
// unsigned int read_revision(void);
void faster();
void slower();
void left();
void right();

// Steering Servo Control
unsigned int read_compass(void); // read the current heading in tenths of a degree
void steer(unsigned int ch, unsigned int th); // steer toward the target heading, based on the current heading
signed char direct(unsigned int ch, unsigned int th); // choose the easiest direction to turn based on ch and th

// Light Sensor Reading
unsigned char light_check(void); // check if there is a bright light

// Initializations / Helper Functions
void XBR0_Init(void);
void Port_Init(void);
void Interrupt_Init(void);
void Serial_Init(void);
void PCA_Init(void);
void PCA_ISR (void) __interrupt 9;
void ADC_Init(void);							// Initialize the AD Converter
unsigned char read_AD_input(unsigned char n);	// read an analog input on Port 1 Pin n
void ping_ranger(void); // instructs the ultasonic ranger to take a new measurement
void delay(unsigned char c); // delay for a specified number of counts

// Global Variables

// PCA0-PWM / I2C-Serial Communication
#define PCA_start		28672
unsigned int counts = 0; // generic time counter
unsigned char c_counts = 0; // electric compass time counter - also used for data output
unsigned char r_counts = 0; // ranger time counter - must reach 4 to show 80ms
unsigned char l_counts = 0; // LCD screen time counter - must reach 20 to show 400ms
unsigned char i2c_data[2]; // helper array used for reading/writing to serial devices

// Drive Motor Control
unsigned int Kpr = 2456; // drive motor gain
unsigned int RNG_val = 0; // 0cm - 100cm
signed int DMPW_val = 0; // -100% - 100%
#define ranger_addr		0xE0 // serial address of ultrasonic ranger
#define PW_REV			2027 // full speed reverse
#define PW_NEUT			2765 // neutral - car is stationary
#define PW_FWD			3502 // full speed forward
unsigned int PW_FWD_SC = 3502; // user adjustable max speed - always <= PW_FWD
unsigned int PW_MOTOR = 2765; // current PW for the drive motor

// Steering Servo Control
unsigned int Kps = 6144; // steering gain
unsigned int target_heading = 2700; // default: west
signed int CPHE_val = 0; // -100% - 100%
signed int SSPW_val = 0; // -100% - 100%
unsigned int current_heading = 0; // current heading (0-3599)
unsigned char reverse = 0; // boolean flag for direction of travel
#define target_heading_rev 2700 // target heading when backing up - car faces west, moves east
#define compass_addr	0xC0 // serial address of the electric compass
#define PW_LEFT			1659 // steering is all the way left
#define PW_CENTER		2765 // steering is centered (straight)
#define PW_RIGHT		3871 // steering is all the way right
unsigned int PW_STEER = 2765;

// Light Sensor Reading
unsigned char LS_val = 0; // 0% - 100%

__sbit __at 0xB7 RCTRL; // Run Control on P3.7

void main(void)
{
	unsigned int temp = 0;
	char input;
	Sys_Init();
	putchar(' ');
	XBR0_Init();
	Port_Init();
	Interrupt_Init();
	Serial_Init();
	PCA_Init();
	ADC_Init();
	
	printf("Ready");
	Drive_Motor(PW_NEUT);
	steer(0, 0);
	
	while(1)
	{
		
		input = getchar();
		if(input == 'w'){
			faster();
		}
		else if(input == 's'){
			slower();
		}
		else if(input == 'd'){
			right();
		}
		else if(input == 'a'){
			left();
		}
		else{
			printf("invalid");
		}
		Drive_Motor(PW_MOTOR);
		PCA0CP0 = 0xFFFF - PW_STEER;
	}

}
void faster(){
	if(PW_MOTOR < PW_FWD){
		PW_MOTOR += 20;
	}
	else{
		PW_MOTOR = PW_FWD;
	}
}
void slower(){
	if(PW_MOTOR > PW_REV){
		PW_MOTOR -= 20;
	}
	else{
		PW_MOTOR = PW_REV;
	}
}
void right(){
	if(PW_STEER < PW_RIGHT){
		PW_STEER += 30;
	}
	else{
		PW_STEER = PW_RIGHT;
	}
}
void left(){
	if(PW_STEER > PW_LEFT){
		PW_STEER -= 30;
	}
	else{
		PW_STEER = PW_LEFT;
	}
}


void speed_control() // modifies maximum forward speed (PW_FWD) based on potentiometer reading
{
	unsigned char adc_val = read_AD_input(4);
	PW_FWD_SC = PW_NEUT + (unsigned int)(2.89 * (unsigned int)adc_val);
}

unsigned int read_ranger() //return range value
{
	i2c_read_data(ranger_addr, 2, i2c_data, 2); // read 2 bytes from register 2 of the ranger
	RNG_val = ((unsigned int)i2c_data[0] << 8) | i2c_data[1]; // merge i2c_data to find the range in cm
	if (RNG_val > 100) // adjust distances greater than 100cm for data viewability purposes
		RNG_val = 100;
	return RNG_val;
}

unsigned int pw_modify(unsigned int range) //change PW based on distance
{
	unsigned int scaled_value = PW_NEUT + (Kpr * (range-30) / 100);
	if (range <= 30)
		return PW_NEUT;
	if (range >= 60)
		return PW_FWD_SC;
	if (scaled_value > PW_FWD_SC)
		return PW_FWD_SC;
	return scaled_value; // decelerate when an object is within 60 cm
}

void Drive_Motor(unsigned int pw_run) //run drive motor
{
	PW_MOTOR = pw_run;
	if (PW_MOTOR < PW_REV) // check that PW_MOTOR is between full reverse and full forward
		PW_MOTOR = PW_REV;
	if (PW_MOTOR > PW_FWD)
		PW_MOTOR = PW_FWD;
	PCA0CP2 = 0xFFFF - PW_MOTOR;
	DMPW_val = (PW_MOTOR-PW_NEUT)*100/(PW_FWD-PW_NEUT);
}

/*unsigned int read_revision()
{
	i2c_read_data(ranger_addr, 0, i2c_data, 1); // reads the revision number from register 0
	return i2c_data[0];
}*/

unsigned int read_compass()
{
	i2c_read_data(compass_addr, 2, i2c_data, 2); // read 2 bytes of data from register 2 of the electic compass
	current_heading = ((unsigned int)i2c_data[0] << 8 ) | i2c_data[1]; // merge two bytes into a single unsigned int
	return current_heading;
}

void steer(unsigned int ch, unsigned int th)
{
	unsigned int error = abs(ch - th); // determine absolute heading error
	if (error > 1800)
		error = 3600 - error; // adjust for angles > 180deg.
	if (reverse)
		PW_STEER = PW_CENTER - (signed int)(direct(ch, th)) * (signed int)((float)Kps * error / 1000.0); // new steering setting
	else
		PW_STEER = PW_CENTER + (signed int)(direct(ch, th)) * (signed int)((float)Kps * error / 1000.0); // new steering setting
	if (PW_STEER > PW_RIGHT) // check if greater than pulsewidth maximum
		PW_STEER = PW_RIGHT; // set PW to the maximum value
	if (PW_STEER < PW_LEFT) // check if less than pulsewidth minimum
		PW_STEER = PW_LEFT; // set PW to the minimum value
	CPHE_val = error * 100 / 1800;
	if (PW_STEER > PW_CENTER)
		CPHE_val *= -1;
	PCA0CP0 = 0xFFFF - PW_STEER;
	//printf("%d\r\n", PW_STEER);
	SSPW_val = (PW_STEER-PW_CENTER)*100/(PW_RIGHT-PW_CENTER);
}

signed char direct(unsigned int ch, unsigned int th) // determine the direction that the car should turn
{
	// using th to divide the circle in two, check which half ch falls in and use this to determine the turning direction
	if (th > 1799)
	{
		if(ch < th && ch > th - 1800)
			return 1;
		return -1;
	}
	if (ch > th + 1800 || ch < th)
		return 1;
	return -1;
}

unsigned char light_check() // check if there is a bright light
{
	i2c_read_data(ranger_addr, 1, i2c_data, 1); // read 1 byte from register 1 (light sensor)
	LS_val = i2c_data[0] * 100 / 250;
	return i2c_data[0] > 225;
}

void XBR0_Init()
{
	XBR0 = 0x27;
}

void Port_Init(void)
{
	P0MDOUT &= 0x3F;					// set SDA and SCL to open drain mode

	P1MDIN &= ~0x10;					// Set P1.4 for analog input
	P1MDOUT &= ~0x10;					// Set P1.4 to open drain
	P1 |= 0x10;							// Set input pin P1.4 to high impedance
	P1MDOUT |= 0x05;					// set output pins for CEX0 and CEX2 in push-pull mode

	P3MDOUT &= ~0x80;					// P3.7 in open drain mode
	P3 |= 0x80;							// P3.7 in high impedance state
}

void Interrupt_Init()
{
	EIE1 |= 0x08;						// enable PCA interrupts
	EA = 1;								// master interrupt enable
}

void Serial_Init()
{
	SMB0CR = 0x93;						// SMB0 at 100kHz
	ENSMB = 1;							// Enable SMB0
}

void PCA_Init(void)
{
	PCA0MD = 0x81;						// SYSCLK/12, enable CF interrupts, suspend when idle
	PCA0CPM0 = 0xC2;					// CCM0 in 16 bit mode, enable compare, enable PWM on CEX0
	PCA0CPM2 = 0xC2;					// CCM2 in 16 bit mode, enable compare, enable PWM on CEX2
	PCA0CN |= 0x40;						// enable PCA
	PCA0CN &= 0xFE;						// clear capture/compare flag
	PCA0 = PCA_start;					// set appropriate value for 20 ms period
}

void PCA_ISR (void) __interrupt 9
{
	if (CF)
	{
		counts++;
		c_counts++;
		r_counts++;
		l_counts++;
		PCA0 = PCA_start;				// set PCA0 to produce a 20 ms period
		CF = 0;							// clear overflow flag

	}
	else
		PCA0CN &= 0xC0;
}

void ADC_Init(void)
{
	REF0CN = 0x03;						// Set Vref to use internal reference voltage (2.4 V)
	ADC1CN = 0x80;						// Enable A/D converter (ADC1)
	ADC1CF |= 0x01;						// Set A/D converter gain to 1
}

unsigned char read_AD_input(unsigned char n) // read an analog input on Port 1 Pin n
{
	AMX1SL = n;							// Set P1.n as the analog input for ADC1
	ADC1CN &= ~0x20;					// Clear the Conversion Completed flag
	ADC1CN |= 0x10;						// Initiate A/D conversion
	while ((ADC1CN & 0x20) == 0x00);	// Wait for conversion to complete
	return ADC1;						// Return digital value in ADC1 register
}

void ping_ranger() //ping_ranger for both light and ranger
{
	i2c_data[0] = 0x51; // 0x51 indicates a new measurement in cm
	i2c_write_data(ranger_addr, 0, i2c_data, 1); // write to the ranger to start a measurement
}

void delay(unsigned char c) // wait for c counts.  each count is 20 ms
{
	unsigned int c_marker = counts;
	while(counts < c_marker + c);
}
