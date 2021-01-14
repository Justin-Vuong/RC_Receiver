/*
Justin Vuong
Controller uses 8-bit counter for PWM to drive motors
ATmega328p datasheet
https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h> 
#include "serialPort.h"
#include "nRFL01.h"
#include "spi.h"

//Pins for transceiver on ATmega328p
#define CE          PINB1
#define CS          PINB2
#define MOSI        PINB3
#define MISO        PINB4
#define SCK         PINB5

//SPI commands for transceiver
#define R_REGISTER(ADDR)        (0x00 | ADDR)
#define W_REGISTER(ADDR)        (0x20 | ADDR)
#define R_RX_PAYLOAD            0x61
#define W_TX_PAYLOAD            0xA0
#define FLUSH_TX                0xE1
#define FLUSH_RX                0xE2
#define REUSE_TX_PL             0xE3
#define R_RX_PL_WID             0x60
#define NOP                     0xFF 

//Holds a value from 0-255 which is set by the ADC
double dutyCycleLeftWheels = 0;
double dutyCycleRightWheels = 0;
double steer = 0;

//To maintain control of car when the controller disconnects, if there are no new commands 
//within a second the motors will be turned off (note 1 sec = 100 overflows of counter)
int count = 0;

//Used to debounce input signal from antenna
uint8_t 	    ISR_Running = 0;

//Variable used to store the incoming payload
unsigned char   recv_Message[] = {'F', 'I', 'L', 'L', 'E', 'R'};

uint8_t         fifo_status = 0;
uint8_t         isDone = 0;
uint8_t         heartbeat[] = {5,5};

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
	//Initialize USART0 serial port for debugging
    USART0Init();

	//Initialize pins used for motors
	DDRC |= (1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3);
	DDRB |= (1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3);
	PORTC |= (1 << PORTC0) | (1 << PORTC2);
	PORTB |= (1 << PORTB0) | (1 << PORTB2);

	//Initialize pins used for tranceiver
	DDRB |= (1 << CE) | (1 << CS) | (1 << MOSI) | (1 <<  SCK);

	//Setting pin D6 as output
	DDRD |= (1 << PORTD5) | (1 << PORTD6);
	//Setting timer to clear OC0A and OC0B pin on compare match and set pin after overflow			
	TCCR0A = (1 << COM0A1) | (1 << COM0B1);
	//Set Fast PWM with timer overflow at 0xFF
	TCCR0A |= (1 << WGM00) | (1 << WGM01);
	//Trigger interrupt upon timer overflow
	TIMSK0 = (1 << TOIE0);

	SPI_init_master();

    nRFL01_RX_Init();

	//Verify registers were initialized with the correct values
    uint8_t ret = 0;
    uint8_t regValues[22];

    if((ret = nRFL01_check_registers(regValues, 1)) != 0 )
    {
        USART0SendByte('E');
        USART0SendByte(ret);

        for(int a = 0; a < 22; a++)
        {
            USART0SendByte(regValues[a]);
        }
        return 0;
    }

	//Enable external interrupts
	sei();

	//Setting timer prescalar to 1024. Setting this bit starts the timer
	TCCR0B = (1 << CS00) | (1 << CS02);

    //Set pin INT0 which is used for transceiver IRQ
    EIMSK = (1 << INT0);

    //Trigger interrupt on falling edge of pin INT0 which is used for IRQ on transceiver
    EICRA = (1 << ISC01);

	USART0SendByte('S');
	while (1)
	{
		nRFL01_heartbeat(heartbeat);
		USART0SendByte('H');
		USART0Send(heartbeat, 2);
		_delay_ms(1000);
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

//Software interrupt for transceiver when it receives data
ISR(INT0_vect)
{
    if (!ISR_Running)
    {
        ISR_Running = 1;
        USART0SendByte('I');

        isDone = 0;
        while (isDone == 0)
        {
            //Read from FIFO, data is ready    
            nRFL01_RX_Read_Payload(recv_Message, 6);
            USART0SendByte('R');
            USART0Send(recv_Message, 6);
			parsePayload(recv_Message, 6);

            //Read from the FIFO_STATUS register to see if there are any other packets in RX FIFO
            PORTB &= ~(1 << CS);
            SPI_Send(0x17);
            fifo_status = SPI_Read();
            PORTB |= (1 << CS);

            USART0SendByte('F');
            USART0SendByte(fifo_status);

            if((fifo_status & 0x01) == 0x01)
            {
                isDone = 1;
            }
        }
        ISR_Running = 0;
    }
}