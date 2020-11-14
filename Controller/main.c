#include <avr/io.h>
#include <util/delay.h>
#include "serialPort.h"
#include "spi.h"

#define CE      PINB1
#define CS      PINB2
#define MOSI    PINB3
#define MISO    PINB4
#define SCK     PINB5

int main (void)
{
    _delay_ms(5000);

    uint8_t u8TempData;
    //Initialize USART0 serial port for debugging
    USART0Init();

    //Chip enable to 1   
    DDRB = (1 << CE) | (1 << CS) | (1 << MOSI) | (1 <<  SCK);

    SPI_init_master();

    while (1)
    {
        SPI_start_transaction();
        
        SPI_Send_Recv_Byte(0b00100000);
        SPI_Send_Recv_Byte(0b1010);

        //USART0SendByte('-');
        //USART0SendByte(realRet2);
        //USART0SendByte('-');

	SPI_end_transaction();
        
        SPI_start_transaction();
        
        SPI_Send_Recv_Byte(0b00000000);
        uint8_t data = SPI_Send_Recv_Byte(0b1010);

        USART0SendByte(data);
        
	SPI_end_transaction();
    }
}
