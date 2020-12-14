/*
Justin Vuong
Controller uses 8-bit counter for PWM to drive motors
ATmega328p datasheet
https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Holds a value from 0-255 which is set by the ADC
double dutyCycleLeftWheels = 0;
double dutyCycleRightWheels = 0;
double steer = 0;

//To maintain control of car when the controller disconnects, if there are no new commands 
//within a second the motors will be turned off (note 1 sec = 100 overflows of counter)
int count = 0;

void updateMotorPWM(int X, int Y)
{
	//New instruction arrived, refresh timeout timer
	count = 0;

	//The joystick's control of the PWM must be mirror around the center 
	//as the direction of the wheels is controlled by setting pins. 

	//Subtract 127 so that reading is centered around 0 (-127 to 128)
	dutyCycleRightWheels = Y  - 127 ;
	
	if (dutyCycleRightWheels > 0) {
		//Driving forwards
		PORTC = (1 << PORTC0) | (1 << PORTC2);
		PORTB = (1 << PORTB0) | (1 << PORTB2);
	
	} else {
		//Driving backwards
		//Duty cycle must always be > 0
		dutyCycleRightWheels *= -1;
		
		//Drive backwards and flip all driving bits
		PORTC = (1 << PORTC1) | (1 << PORTC3);
		PORTB = (1 << PORTB1) | (1 << PORTB3);
	}

	dutyCycleRightWheels *= 2;

	//Both wheels are set to the same duty cycle and X direction will be adjusted shortly
	dutyCycleLeftWheels = dutyCycleRightWheels;

	//Subtract 127 so that reading is centered around 0 (-127 to 128)
	steer = X - 127;

	//Allow dutyCycle to change by at most 20 which is subtracted from 
	//one side of the motors and added to the other
	int adj = steer;
	
	if (steer > 0) {
		dutyCycleRightWheels += adj;
		dutyCycleLeftWheels -= adj;
	} else {
		dutyCycleRightWheels += adj;
		dutyCycleLeftWheels -= adj;
	}

	//Make sure 0 <= dutyCycle <= 255
	if (dutyCycleRightWheels < 0){
		dutyCycleRightWheels = 0;
	}
	if (dutyCycleLeftWheels < 0){
		dutyCycleLeftWheels = 0;
	}
	if (dutyCycleRightWheels > 255){
		dutyCycleRightWheels = 255;
	}
	if (dutyCycleLeftWheels > 255){
		dutyCycleLeftWheels = 255;
	}
}

int parsePayload(uint8_t* payload, int payloadSize)
{
	//Byte 3 and 6 are the X and Y positions respectively

	//If the payload has an unexpected length then return error
	if(payloadSize < 6)
	{
		return -1;
	}

	updateMotorPWM(payload[2], payload[5]);
}

int main(void)
{
	DDRC = (1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3);
	DDRB = (1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3);
	PORTC = (1 << PORTC0) | (1 << PORTC2);
	PORTB = (1 << PORTB0) | (1 << PORTB2);

	//Setting pin D6 as output
	DDRD |= (1 << PORTD5) | (1 << PORTD6);
	//Setting timer to clear OC0A and OC0B pin on compare match and set pin after overflow			
	TCCR0A = (1 << COM0A1) | (1 << COM0B1);
	//Set Fast PWM with timer overflow at 0xFF
	TCCR0A |= (1 << WGM00) | (1 << WGM01);
	//Trigger interrupt upon timer overflow
	TIMSK0 = (1 << TOIE0);

	//Enable external interrupts
	sei();

	//Setting prescalar to 1024. Setting this bit starts the timer
	TCCR0B = (1 << CS00) | (1 << CS02);

	uint8_t message1[6] = {'X',':',100,'Y',':',250};
	uint8_t message2[6] = {'X',':',150,'Y',':',150};
	uint8_t message3[6] = {'X',':',250,'Y',':',0};
	
	while (1)
	{
		parsePayload(message1,6);
		_delay_ms(2500);
		parsePayload(message2,6);
		_delay_ms(2500);
		parsePayload(message3,6);
		_delay_ms(2500);
	}

	return 0;
}

//Configure external interupt to update duty cycle upon overflow
ISR(TIMER0_OVF_vect){
	count += 1;
	if (count > 100)
	{
		dutyCycleRightWheels = 0;
		dutyCycleLeftWheels = 0;
	}

	//Set compare registers values
	OCR0A = dutyCycleRightWheels; 
	OCR0B = dutyCycleLeftWheels;
}
