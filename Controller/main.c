//Code used for controller
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

//Used to debounce input signal from antenna
uint8_t ISR_Running = 0;

uint8_t joyX = 0;
uint8_t joyY = 0;
uint8_t payload[6] = {'X',':','0','Y',':','0'};
uint8_t fifo_status = 0;
uint8_t heartbeat[] = {5,5};

void send_update(uint8_t X, uint8_t Y)
{
	payload[2] = X;
	payload[5] = Y;
    USART0Send(payload,6);
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
	//Initialize USART0 serial port for debugging
    USART0Init();

	//ADC used to read values from joystick
	ADC_setup();

	//Enable pins for SPI
    DDRB |= (1 << CE) | (1 << CS) | (1 << MOSI) | (1 <<  SCK);
	SPI_init_master();

	nRFL01_TX_Init();
	
	//Enable external interrupts used for reading ADC
	sei();

	_delay_ms(2500);

	//Set pin INT0 used for transceiver IRQ
    EIMSK = (1 << INT0);

    //Trigger interrupt on falling edge of pin INT0 used for transceiver IRQ
    EICRA = (1 << ISC01);

    uint8_t ret = 0;
    uint8_t regValues[22];

    if((ret = nRFL01_check_registers(regValues, 0)) != 0)
    {
        USART0SendByte('E');
        USART0SendByte(ret);
        for(int a = 0; a < 22; a++)
        {
            USART0SendByte(regValues[a]);
        }
        return 0;
    }

	uint8_t status_reg = 0;
	USART0SendByte('D');

    while (1)
    {
		//Update packet values and transmit it
        start_conversion();

		//Make sure RX FIFO is ready for new payload
		PORTB &= ~(1 << CS);
        SPI_Send(R_REGISTER(0x17));
        fifo_status = SPI_Read();
        PORTB |= (1 << CS);

        if((fifo_status & 0x10) == 0x00)
        {
            //Send FLUSH_TX to clear TX FIFO
            PORTB &= ~(1 << CS);
            SPI_Send(FLUSH_TX);
            PORTB |= (1 << CS);
        }
        else if (fifo_status == 0x11)
        {
			//Send data
            nRFL01_Write_Tx_Payload(payload, 6);
            nRFL01_heartbeat(heartbeat);
            USART0SendByte('S');
            USART0Send(heartbeat, 2);
        }

        nRFL01_heartbeat(heartbeat);
		if ((heartbeat[1] & (1 << 5)) == 1 << 5)
        {
			//TX_DS was asserted meaning data was sucessfully sent
            USART0SendByte(heartbeat[1]);

            //Reset IRQ Pin
            PORTB &= ~(1 << CS);
            status_reg = SPI_RW_Byte(NOP);
            PORTB |= (1 << CS);

            //Write 1 to TX_DS in Status register to reset IRQ
            PORTB &= ~(1 << CS);
            SPI_Send(W_REGISTER(0x07));
            SPI_Send(status_reg | (1 << 5));
            PORTB |= (1 << CS);

        }
        //Looking in the MAX_RT bit in Status register
        if (heartbeat[0] & (1 << 4))
        {
            USART0SendByte('B');
            //Print ARC_Cnt to see how many retransmits have passed
            USART0SendByte(0x0F & nRFL01_Read_Reg(0x08));
            nRFL01_clear_max_retries();
        }
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
