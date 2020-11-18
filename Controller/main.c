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

int main (void)

{
    //Initialize USART0 serial port for debugging
    USART0Init();

    //Chip enable to 1   
    DDRB = (1 << CE) | (1 << CS) | (1 << MOSI) | (1 <<  SCK);

    SPI_init_master();

    nRFL01_TX_Init();

    //Init Software interrupt
    sei();
    
    //Set pin INT0
    EIMSK = (1 << INT0);

    //Trigger interrupt on rising edge of pin INT0
    EICRA = (1 << ISC00) | (1 << ISC01);


    while (1)
    {

    }
}

//Software interrupt for transceiver when it receives or transmits data
ISR(INT0_vect)
{
    if (!ISR_Running)
    {
        ISR_Running = 1;
        count +=1;
    
        USART0SendByte(count);
        ISR_Running = 0;
    }
}