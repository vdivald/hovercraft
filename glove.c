/*
 * glove.c
 *
 *Measure pitch and roll with MPU-6050 6DOF Inertial Measurement Unit
 *and send to serial port
 *
 * Created: 2014.01.28. 22:00:00
 * Author: Viktor Divald
 */ 

#define F_CPU 16000000UL
#define UART_BAUDRATE 57600
#define UBRR_VALUE ((F_CPU/(UART_BAUDRATE * 16UL))-1)

#define CODE_CASE1	0x85
#define CODE_CASE2	0x90

#include "mpu6050.h"
#include "twi.h"
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>

const float PI=3.14159265358;

//global variables
uint32_t current_time, previous_time=0;	//timestamps

volatile int state = 1;		//for glove functions
volatile int isr_flag = 0;	//timer1 interrupt happened
volatile uint32_t millisec_count = 0;	//count milliseconds from start
float Xangle=0, Yangle=0;	//measured angles in degrees around X and Y axis

//prototypes
void init(void);
void calculate_pitchroll(int8_t* pitch, int8_t* roll);
void print_data(int8_t* pitch, int8_t* roll);
void config_timer1();
void config_timer0();
void mpu6050_init();
void uart_config();
void uart_sendbyte(int8_t data);

static FILE mystdout = FDEV_SETUP_STREAM(uart_sendbyte, NULL, _FDEV_SETUP_WRITE);	//to use printf() with UART


int main(void)
{	
	int8_t pitch, roll;		//pitch and roll in degrees
	
	init();
	
	printf("szia\n");

	while(1)
	{
		switch(state)
		{
			case 1:		//hovercraft movement control
			{
//				uart_sendbyte(CODE_CASE1);	//code for case1
				if(isr_flag)	//true in every 10ms
				{
					isr_flag=0;
					calculate_pitchroll(&pitch, &roll);
					print_data(&pitch, &roll);
//					uart_sendbyte(pitch);
//					uart_sendbyte(roll);
//					uart_sendbyte('\n');
				}
				break;
			}

			case 2:		//robot arm movement control
			{
//				uart_sendbyte(CODE_CASE2)
				if(isr_flag)
				{
					isr_flag=0;
					calculate_pitchroll(&pitch, &roll);
					print_data(&pitch, &roll);
//					uart_sendbyte(pitch);
//					uart_sendbyte(roll);
//					uart_sendbyte('\n');
				}
				break;
			}
		}	
	}

	return 0;
}


void init(void)
{
	DDRB |= (1<<DDB7);
	PORTB = 0x00;
	int ok = 0;

	uart_config();
	stdout = &mystdout; //Required for printf init
	
	printf("Initializing timers...\n");
	config_timer1();
	config_timer0();
	
	printf("Initializing I2C...\n");
	TWIInit();

	printf("Initializing MPU-6050...\n");
	mpu6050_init();
	
	ok = mpu6050_testConnection();
	if(ok)	
		printf("Connection successful!\n");
	else
		printf("Connection failed\n");
		
	_delay_ms(50);
	
	sei();
}

void config_timer1()	//for reading sensor
{
	TCCR1A=0;
	TCCR1B |= (1<<CS12) | (1<<CS10) | (1<<WGM12);  //prescaler: clk/1024, CTC mode
	OCR1A = 156;  			// 1/(16000000Hz/1024) * 156 = ~0.01s -> interrupt in every 10ms
	TIMSK1 = (1<<OCIE1A);
	TIFR1 &= ~(1<<OCF1A);	//clear interrupt flag
}

void config_timer0()	//for count elapsed time
{
	TCCR0A = (1<<WGM01);
	TCCR0B |= (1<<CS01) | (1<<CS00);  //prescaler: clk/64, CTC mode
	OCR0A = 250;  			// 1/(16000000Hz/64) * 250 = 1ms -> interrupt in every 1ms
	TIMSK0 = (1<<OCIE0A);
	TIFR1 &= ~(1<<OCF0A);	//clear interrupt flag
}

void calculate_pitchroll(int8_t* pitch, int8_t* roll)
{
	int16_t ax,ay,az,gx,gy,gz;
	float accXangle, accYangle;
	float dt;	//time between 2 sample
	const float tau=0.075;	//complementary filter time constant, must be bigger, than dt
	float a;
	
	//Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
	mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
	current_time=millisec_count;
	
	accXangle = (atan2(ay,az))*180/PI;
	accYangle = (atan2(ax,az))*180/PI;
	
	dt=((float)(current_time-previous_time))/1000;
	
	a=tau/(tau+dt);
	Xangle=a*(Xangle + ((float)gx/131 * dt)) + (1-a)*accXangle;    //Complementary filter
	Yangle=a*(Yangle + ((float)gy/131 * dt)) + (1-a)*accYangle;
	previous_time = current_time;
	
	*pitch = (int8_t)Xangle;
	*roll = (int8_t)Yangle;
}

void print_data(int8_t* pitch, int8_t* roll)
{
	printf("pitch: %d\t roll: %d\n",*pitch,*roll);
}

ISR(TIMER1_COMPA_vect)
{
	isr_flag = 1;
}

ISR(TIMER0_COMPA_vect)
{
	millisec_count++;
}

void uart_config()
{
	UBRR0 = UBRR_VALUE;
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);	//8bit, no parity, 1 stopbit
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
}

void uart_sendbyte(int8_t data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0=data;	
}
