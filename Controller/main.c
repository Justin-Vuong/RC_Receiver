//Code meant for controller
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serialPort.h"
#include "nRFL01.h"
#include "spi.h"

#define CE          PINB1
#define CS          PINB2
#define MOSI        PINB3
#define MISO        PINB4
#define SCK         PINB5

//Used to debounce input signal from antenna
uint8_t ISR_Running = 0;

uint8_t joyX = 0;
uint8_t joyY = 0;

void send_update(uint8_t X, uint8_t Y)
{
    uint8_t message[6] = {'X',':',X,'Y',':',Y};
    USART0Send(message,6);
}

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

int main (void)
{
    USART0Init();
	ADC_setup();

	//Enable external interrupts
	sei();

    while (1)
    {
        _delay_ms(1000);
        start_conversion();
    }
}

ISR(ADC_vect)
{
	
	if ((ADMUX & 0b1111) == 0b0100) {
		//ADC 4 selected (X dir)
		//Subtract 127 so that reading is centered around 0 (-127 to 128)
		joyX = ADCH;
        
		//Set the interrupt to read from ADC5 next time
		ADMUX |= 0b00000001; 
	} else {
		
		//ADC 5 selected (Y dir)
		//The joystick's control of the PWM must be mirror around the center 
		//as the direction of the wheels is controlled by setting pins. 
        joyY = ADCH;

        send_update(joyX, joyY);
		
		ADMUX &= 0b11111110; //Set the interrupt to read from ADC4 next time
	}
}
