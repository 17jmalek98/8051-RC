/*
		Joshua Malek
		11/2/18
		Description: This program controls the steering servo and drive motor of a smart car, driving the
		car in a specified direction, stopping for obstacles, and backing up when it detects a bright light.	*/

#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

// Function Prototypes

// Drive Motor Control
void Drive_Motor(unsigned int pw_run); // sets the drive motor to a specified pulsewidth
// unsigned int read_revision(void);
void faster();
void slower();
void left();
void right();

// Initializations / Helper Functions
void XBR0_Init(void);
void Port_Init(void);
void Interrupt_Init(void);
void Serial_Init(void);
void PCA_Init(void);
void PCA_ISR (void) __interrupt 9;
void ADC_Init(void);							// Initialize the AD Converter

// Global Variables

// PCA0-PWM / I2C-Serial Communication
#define PCA_start		28672
unsigned int counts = 0; // generic time counter

// Drive Motor Control
#define PW_REV			2027 // full speed reverse
#define PW_NEUT			2765 // neutral - car is stationary
#define PW_FWD			3502 // full speed forward
unsigned int PW_MOTOR = 2765; // current PW for the drive motor

// Steering Servo Control
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

void Drive_Motor(unsigned int pw_run) //run drive motor
{
	PW_MOTOR = pw_run;
	if (PW_MOTOR < PW_REV) // check that PW_MOTOR is between full reverse and full forward
		PW_MOTOR = PW_REV;
	if (PW_MOTOR > PW_FWD)
		PW_MOTOR = PW_FWD;
	PCA0CP2 = 0xFFFF - PW_MOTOR;
	//DMPW_val = (PW_MOTOR-PW_NEUT)*100/(PW_FWD-PW_NEUT);
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
