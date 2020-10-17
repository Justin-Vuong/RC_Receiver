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
double dutyCycle = 0;

void start_conversion(){

	ADCSRA |= (1 << ADSC); 		//Start a read on the ADC

}

void ADC_setup(){
	//Using a 10 bit ADC
	ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX2) | (1 << ADLAR); 			//Use external voltage as reference, use ADC5, left adjust register to read only 8 MSB
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); 	//Enable ADC, Enable interrupts, set prescalar
	DIDR0 = (1 << ADC5D); 									//Disable digital input buffer to avoid reading wrong value 
	start_conversion();
}

int main(void)
{
	DDRC = (1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3);
	DDRB = (1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3);
	PORTC = (1 << PORTC0) | (1 << PORTC2);
	PORTB = (1 << PORTB0) | (1 << PORTB2);

	DDRD |= (1 << PORTD5) | (1 << PORTD6);			//Setting pin D6 as output
	TCCR0A = (1 << COM0A1) | (1 << COM0B1);			//Setting timer to clear OC0A and OC0B pin on compare match and set pin after overflow
	TCCR0A |= (1 << WGM00) | (1 << WGM01);	 		//Set Fast PWM with timer overflow at 0xFF
	TIMSK0 = (1 << TOIE0);					//Trigger interrupt upon timer overflow

	ADC_setup();

	sei(); 		//Enable external interrupts

	TCCR0B = (1 << CS00) | (1 << CS02); 		//Setting prescalar to 1024. Setting this bit starts the timer
	
	while (1)
	{
		//Application code
	}

	return 0;
}

//Configure external interupt to update duty cycle upon overflow
ISR(TIMER0_OVF_vect){
	//Set compare registers values
	OCR0A = dutyCycle; 
	OCR0B = dutyCycle;
}

ISR(ADC_vect)
{
	//The joystick's control of the PWM must be mirror around the center 
	//as the direction of the wheels is controlled by setting pins. 

	dutyCycle = ADCH;
	
	if (dutyCycle - 127 > 0) {
		dutyCycle -= 127;
	}
	dutyCycle *= 2;
	dutyCycle = 255 - dutyCycle;

	start_conversion();
}
