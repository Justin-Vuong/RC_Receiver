#include <avr/io.h>
#include <util/delay.h>
#include "serialPort.h"
#include "nRFL01.h"
#include "spi.h"

#define CE          PINB1
#define CS          PINB2
#define MOSI        PINB3
#define MISO        PINB4
#define SCK         PINB5

int main (void)
{
    //Initialize USART0 serial port for debugging
    USART0Init();

    //Chip enable to 1   
    DDRB = (1 << CE) | (1 << CS) | (1 << MOSI) | (1 <<  SCK);

    SPI_init_master();

    nRFL01_TX_Init();

}