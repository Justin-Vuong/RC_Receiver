/*
To compile move into Car directory and change Makefile as follows:
    SRC = $(TARGET).c serialPort.c spi.c nRFL01.c
    TARGET = rx
*/

//Code used to test tranceiver which will be integrated into car
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h> 
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
uint8_t 	    ISR_Running = 0;
unsigned char   recv_Message[] = {'F', 'I', 'L', 'L', 'E', 'R'};
uint8_t         count = 0;
uint8_t         slowReadyMessage = 0;
uint8_t         pipe = 0;
uint8_t         fifo_status = 0;
uint8_t         isDone = 0;
uint8_t         heart[] = {5,5};

int main (void)
{
    //Initialize USART0 serial port for debugging
    USART0Init();

    //Chip enable to 1   
    DDRB |= (1 << CE) | (1 << CS) | (1 << MOSI) | (1 <<  SCK);

    SPI_init_master();

    nRFL01_RX_Init();

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

	//Init external software interrupts
    sei();
    
    //Set pin INT0
    EIMSK = (1 << INT0);

    //Trigger interrupt on falling edge of pin INT0
    EICRA = (1 << ISC01);

    uint8_t RPD = 0;
    USART0SendByte('S');
	while(1)
	{
		nRFL01_heartbeat(heart);

        //Read from the FIFO_STATUS register to see if there are any other packets in RX FIFO
        PORTB &= ~(1 << CS);
        SPI_Send(0x17);
        fifo_status = SPI_Read();
        PORTB |= (1 << CS);

        if (heart[0] == 0x42 && heart[1] == 0x0F) 
        {

        }
        else if(!(fifo_status & 0x01))
        {
            USART0SendByte('F');
            USART0SendByte(fifo_status);
        }
        else if(heart[0] != 0x0E || heart[1] != 0x0F)
        {
            //Something wrong with registers
            USART0SendByte('H');
            USART0Send(heart, 2);
        }

        /*
        //Checking RX_DR in status register
        if (heart[0] & (1 << 6))
        {
            if(slowReadyMessage == 1000)
            {            
                USART0SendByte('D');
                slowReadyMessage = 0;
            }
            else
            {
                slowReadyMessage += 1;
            }
        }
        */

        /*
        //Checking RX_P_NO in status register (000-101 is returned if a pipe has data available)
        if ((pipe = heart[0] & 0x0E) <= 0b101)
        {
            USART0SendByte('D');
            USART0SendByte('A');
            USART0SendByte('T');
            USART0SendByte('A');
        }
        else if (pipe == 0b00000110)
        {
            USART0SendByte('E');
            USART0SendByte('R');
            USART0SendByte('R');
            USART0SendByte(':');
            USART0SendByte('F');
            USART0SendByte('I');
            USART0SendByte('F');
            USART0SendByte('O');
        }
        */
        
        /*
        //Check Received Power Detector register (0 means no signal and 1 means power above -64dBm)
        PORTB &= ~(1 << CS);
        SPI_Send(0x09);
        RPD = SPI_Read();
        PORTB |= (1 << CS);

        if (RPD)
        {
            if (count == 15)
            {
                USART0SendByte('S');  
                USART0SendByte(RPD);
                count = 0;
            }
            else 
            {
                count++;
            }
        }
        */

        /* 
        //Check fifo_status register and trigger if RX_EMPTY is false aka data was received
        PORTB &= ~(1 << CS);
        SPI_Send(0x17);
        fifo_status = SPI_Read();
        PORTB |= (1 << CS);

        if (!(fifo_status & 0x01))
        {
            USART0SendByte('F');
            USART0SendByte('I');
            USART0SendByte('F');
            USART0SendByte('O');
        }
        */
	}
}

//Software interrupt for transceiver when it receives or transmits data
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
            USART0SendByte(';');

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

            nRFL01_heartbeat(heart);
            USART0SendByte('H');
            USART0Send(heart, 2);
        }
        ISR_Running = 0;
    }
}