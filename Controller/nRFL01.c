#include <avr/io.h>
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
    //Send activate command
    PORTB &= ~(1 << CS);
    SPI_Send(ACTIVATE);
    SPI_Send(0x73);
    PORTB |= (1 << CS);

    //Enable Auto_ACK and dynamic payload length
    nRFL01_Write_Reg(0x1D, 0x06);

    //Configure setup register
    nRFL01_Write_Reg(0x00, 0x0E);

    //Set RX address length to 3 bytes
    nRFL01_Write_Reg(0x03, 0x01);

    //Set RX_ADDR_P0 (3 Bytes)
    uint8_t RX_Addr[] = {0x78, 0x78, 0x78};
    nRFL01_Write_Regs(0x0A, RX_Addr, 3);

    //Set TX_ADDR same as RX_ADDR_P0
    nRFL01_Write_Regs(0x10, RX_Addr, 3);

    //Send FLUSH_TX to clear TX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(FLUSH_TX);
    PORTB |= (1 << CS);

    //Set enable pin to enter standby mode 2
    PORTB |= (1<<CE);
}