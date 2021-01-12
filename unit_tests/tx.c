/*
To compile move into Controller directory and change Makefile as follows:
    SRC = $(TARGET).c serialPort.c spi.c nRFL01.c
    TARGET = tx
*/

//Used to test functionality of transceiver before integrating into message.c
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
uint8_t count = 0;
uint8_t sent_packets = 0;
uint8_t heart[] = {5,5};
uint8_t fifo_status = 0;
uint8_t payload_num = 0;

int main (void)

{
    //Initialize USART0 serial port for debugging
    USART0Init();

    //Chip enable to 1   
    DDRB |= (1 << CE) | (1 << CS) | (1 << MOSI) | (1 <<  SCK);

    SPI_init_master();

    uint8_t chk[]= {0,0};
    
    nRFL01_TX_Init();
    
    _delay_ms(2500);

    //Init Software interrupt
    sei();
    
    //Set pin INT0
    EIMSK = (1 << INT0);

    //Trigger interrupt on falling edge of pin INT0
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

    char payloads[3][6] = {"Hello!", "Justin", "GBye!!"};
    uint8_t status_reg = 0;

    USART0SendByte('D');
    
    while(1)
    {
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
            nRFL01_Write_Tx_Payload(payloads[payload_num % 3], 6);
            nRFL01_heartbeat(heart);
            USART0SendByte('S');
            USART0Send(heart, 2);
        }

        //Reset TX Max Retry Flag in status register
        nRFL01_heartbeat(heart);

        if ((heart[1] & (1 << 5)) == 1 << 5)
        {
            USART0SendByte(heart[1]);
            USART0SendByte('S');
            USART0SendByte('E');
            USART0SendByte('N');
            USART0SendByte('T');

            //Reset IRQ Pin
            PORTB &= ~(1 << CS);
            status_reg = SPI_RW_Byte(NOP);
            PORTB |= (1 << CS);

            //Write 1 to TX_DS in Status register
            PORTB &= ~(1 << CS);
            SPI_Send(W_REGISTER(0x07));
            SPI_Send(status_reg | (1 << 5));
            PORTB |= (1 << CS);

            PORTB &= ~(1 << CS);
            status_reg = SPI_RW_Byte(NOP);
            PORTB |= (1 << CS);

            USART0SendByte(status_reg);

        }
        //Looking in the MAX_RT bit in Status register
        if (heart[0] & (1 << 4))
        {
            USART0SendByte('B');
            //Read ARC_Cnt to see how many retransmits have passed
            USART0SendByte(0x0F & nRFL01_Read_Reg(0x08));
            nRFL01_clear_max_retries();
        }
        payload_num += 1;
    }
}

//Software interrupt for transceiver when it receives or transmits data (or max retries)
ISR(INT0_vect)
{
    if (!ISR_Running)
    {
        ISR_Running = 1;
        count += 1;
        USART0SendByte('I');
        USART0SendByte(count);
        ISR_Running = 0;
    }
}
