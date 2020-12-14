#include <avr/io.h>
#include <stdlib.h> 
#include <stdbool.h>
#include <util/delay.h>
#include "serialPort.h"
#include "spi.h"
#include "nRFL01.h"

//SPI Pins
#define SPI_DDR DDRB
#define CE PINB1
#define CS PINB2
#define MOSI PINB3
#define MISO PINB4
#define SCK PINB5

//SPI commands for transceiver
#define R_REGISTER(ADDR)        (0x00 | ADDR)
#define W_REGISTER(ADDR)        (0x20 | ADDR)
#define R_RX_PAYLOAD            0x61
#define W_TX_PAYLOAD            0xA0
#define FLUSH_TX                0xE1
#define FLUSH_RX                0xE2
#define REUSE_TX_PL             0xE3
#define ACTIVATE                0x50
#define R_RX_PL_WID             0x60
#define NOP                     0xFF 

void nRFL01_Write_Reg(uint8_t addr, uint8_t value)
{
    //Writes one byte to addr
    PORTB &= ~(1 << CS);
    SPI_Send(W_REGISTER(addr));
    SPI_Send(value);
    PORTB |= (1 << CS);
}

uint8_t nRFL01_Read_Reg(uint8_t addr)
{
    uint8_t data = 0;

    //Writes one byte to addr
    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(addr));
    data = SPI_Read();
    PORTB |= (1 << CS);
    return data;
}

void nRFL01_Write_Regs(uint8_t addr, uint8_t* values, int size)
{
    //Writes multiple byte to addr
    PORTB &= ~(1 << CS);
    SPI_Send(W_REGISTER(addr));
    SPI_Send_Bytes(values, size);
    PORTB |= (1 << CS);

}

void nRFL01_TX_Init(void)
{
    //Configure setup register
    nRFL01_Write_Reg(0x00, 0x0E);

    //Set RX address length to 3 bytes
    nRFL01_Write_Reg(0x03, 0x01);

    //Set RX_ADDR_P0 (3 Bytes)
    uint8_t RX_Addr[] = {0x78, 0x78, 0x78};
    nRFL01_Write_Regs(0x0A, RX_Addr, 3);

    //Set TX_ADDR same as RX_ADDR_P0
    nRFL01_Write_Regs(0x10, RX_Addr, 3);
    /*
    //Send activate command
    PORTB &= ~(1 << CS);
    SPI_Send(ACTIVATE);
    SPI_Send(0x73);
    PORTB |= (1 << CS);
    */
    //Enable Auto_ACK and dynamic payload length
    nRFL01_Write_Reg(0x1D, 0x06);

    //Send FLUSH_TX to clear TX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(FLUSH_TX);
    PORTB |= (1 << CS);

    //Set enable pin to enter standby mode 2
    PORTB |= (1<<CE);
}

//Loads up to 32 bytes in TX FIFO
void nRFL01_Write_Tx_Payload(uint8_t* values, int size)
{
    //Writes multiple byte to addr
    PORTB &= ~(1 << CS);
    SPI_Send(W_TX_PAYLOAD);
    SPI_Send_Bytes(values, size);
    PORTB |= (1 << CS);

}

void nRFL01_RX_Init(void)
{
    //Configure setup register
    nRFL01_Write_Reg(0x00, 0x0F);

    //Set RX address length to 3 bytes
    nRFL01_Write_Reg(0x03, 0x01);

    //Set RX_ADDR_P0 (3 Bytes)
    uint8_t RX_Addr[] = {0x78, 0x78, 0x78};
    nRFL01_Write_Regs(0x0A, RX_Addr, 3);

    //Set TX_ADDR same as RX_ADDR_P0
    nRFL01_Write_Reg(0x1C, 0x1);

    /*
    //Send activate command
    PORTB &= ~(1 << CS);
    SPI_Send(ACTIVATE);
    SPI_Send(0x73);
    PORTB |= (1 << CS);
    */

    //Enable Auto_ACK and dynamic payload length
    nRFL01_Write_Reg(0x1D, 0x06);

    //Send FLUSH_RX to clear RX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(FLUSH_RX);
    PORTB |= (1 << CS);

    //Set enable pin to enter standby mode 2
    PORTB |= (1<<CE);
}


void nRFL01_RX_Read_Payload(uint8_t* data, int dataSz)
{   
    //Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(R_RX_PL_WID);
    dataSz = SPI_Read();
    PORTB |= (1 << CS);

    //Read RX payload through SPI interface
    data = (uint8_t*) calloc(dataSz, sizeof(uint8_t));
    PORTB &= ~(1 << CS);
    SPI_Send(R_RX_PAYLOAD);
    SPI_Read_Bytes(data, dataSz);
    PORTB |= (1 << CS);

    //char debugMsg[7] = "RX Size";
    //USART0Send(debugMsg, 7);

}
void nRFL01_heartbeat(uint8_t* data)
{
    //Read status register
    PORTB &= ~(1 << CS);
    *data = SPI_RW_Byte(NOP);
    PORTB |= (1 << CS);
}

uint8_t nRFL01_check_registers(uint8_t* data, bool isRX)
{
    //data for TX is 9 and RX is 7
    //Checks registers against expected values for RX/TX (isRX ? RX:TX)
    //Return 1 else negative of index with incorrect register value

    /*
    For RX you should expect data to have:
        0-2 -> reg 0A -> 0x787878
        3   -> reg 1D -> 110
        4   -> reg 00 -> 00001111
        5   -> reg 03 -> 0x1
        6   -> reg 1C -> 0x1    

    For TX you should expect indexes to have:
        0-2 -> reg 0A   -> 0x787878
        3   -> reg 1D   -> 110
        4   -> reg 00   -> 00001110
        5   -> reg 03   -> 01
        6-8 -> reg 10   -> 0x787878
    */

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x0A));
    SPI_Read_Bytes(data, 3);
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x1D));
    data[3] = SPI_Read();
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x00));
    data[4] = SPI_Read();
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x03));
    data[5] = SPI_Read();
    PORTB |= (1 << CS);

    if(isRX)
    {
        PORTB &= ~(1 << CS);
        SPI_Send(R_REGISTER(0x1C));
        data[6] = SPI_Read();
        PORTB |= (1 << CS);

        if (data[0] != 0x78)
        {
            return 1;
        }
        else if (data[1] != 0x78)
        {
            return 2;
        }
        else if (data[2] != 0x78)
        {
            return 3;
        }
        else if (data[3] != 0b110)
        {
            return 4;
        }
        else if (data[4] != 0b1111)
        {
            return 5;
        }
        else if (data[5] != 0b1)
        {
            return 6;
        }
        else if (data[6] != 0b1)
        {
            return 7;
        }

        return 0;
    }
    else
    {
        PORTB &= ~(1 << CS);
        SPI_Send(R_REGISTER(0x10));
        SPI_Read_Bytes(&data[6], 3);
        PORTB |= (1 << CS);

        if (data[0] != 0x78)
        {
            return 1;
        }
        else if (data[1] != 0x78)
        {
            return 2;
        }
        else if (data[2] != 0x78)
        {
            return 3;
        }
        else if (data[3] != 0b110)
        {
            return 4;
        }
        else if (data[4] != 0b1110)
        {
            return 5;
        }
        else if (data[5] != 0b1)
        {
            return 6;
        }
        else if (data[6] != 0x78)
        {
            return 7;
        }
        else if (data[7] != 0x78)
        {
            return 8;
        }
        else if (data[8] != 0x78)
        {
            return 9;
        }
        return 0;
    }
    
}