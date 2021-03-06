#include <avr/io.h>
#include "serialPort.h"

//Baud rate prescalar

#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define USART_BAUDRATE 9600
#define F_CPU 16000000UL

void USART0Init(void)
{
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR_VALUE>>8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    // Set frame format to 8 data bits, no parity, 1 stop bit
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
    //enable transmission and reception
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
}

void USART0Send(uint8_t* u8Data, int size)
{
    for (int a = 0; a < size; a++)
    {
        //wait while previous byte is completed
        while(!(UCSR0A&(1<<UDRE0))){};
        // Transmit data
        UDR0 = u8Data[a];
    }
}

void USART0Read(uint8_t* buff, int size)
{
    for (int a = 0; a < size; a++)
    {
        //wait while previous byte is completed
        while(!(UCSR0A&(1<<UDRE0))){};
        // Transmit data
        buff[a] = UDR0;
    }
}

void USART0SendByte(uint8_t u8Data)
{
    //wait while previous byte is completed
    while(!(UCSR0A&(1<<UDRE0))){};
    // Transmit data
    UDR0 = u8Data;
}

uint8_t USART0ReadByte()
{
    // Wait for byte to be received
    while(!(UCSR0A&(1<<RXC0))){};
    // Return received data
    return UDR0;
}
