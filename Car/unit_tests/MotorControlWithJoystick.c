/*
Justin Vuong
Controller uses 8-bit counter for PWM to drive motors
ATmega328p datasheet
https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

Unit test for motors
Attach Joysick to C4 and C5 analog pins

*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Holds a value from 0-255 which is set by the ADC
double dutyCycleLeftWheels = 0;
double dutyCycleRightWheels = 0;
double steer = 0;

void start_conversion(){

	ADCSRA |= (1 << ADSC); 		//Start a read on the ADC

}

void ADC_setup(){
	//Using a 10 bit ADC
	//Use external voltage as reference, use ADC5, left adjust register to read only 8 MSB
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX0) | (1 << MUX2);
	//Enable ADC, Enable interrupts, set prescalar
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); 	
	//Disable digital input buffer to avoid reading wrong value
	DIDR0 |=  (1 << ADC4D) | (1 << ADC5D);  
	start_conversion();
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

	ADC_setup();

	//Enable external interrupts
	sei();

	//Setting prescalar to 1024. Setting this bit starts the timer
	TCCR0B = (1 << CS00) | (1 << CS02);

	while (1)
	{
		//Application code
	}

	return 0;
}

//Configure external interupt to update duty cycle upon overflow
ISR(TIMER0_OVF_vect){
	//Set compare registers values
	OCR0A = dutyCycleRightWheels; 
	OCR0B = dutyCycleLeftWheels;
}

ISR(ADC_vect)
{
	
	if ((ADMUX & 0b1111) == 0b0100) {
		//ADC 4 selected
		//Subtract 127 so that reading is centered around 0 (-127 to 128)
		steer = ADCH - 127;

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

		//Set the interrupt to read from ADC5 next time
		ADMUX |= 0b00000001; 
	} else {
		
		//ADC 5 selected
		//The joystick's control of the PWM must be mirror around the center 
		//as the direction of the wheels is controlled by setting pins. 

		//Subtract 127 so that reading is centered around 0 (-127 to 128)
		dutyCycleRightWheels = ADCH  - 127 ;
		
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
		//dutyCycleRightWheels = 255 - dutyCycleRightWheels;
		dutyCycleLeftWheels = dutyCycleRightWheels;
		
		ADMUX &= 0b11111110; //Set the interrupt to read from ADC4 next time

	}
	
	start_conversion();
}