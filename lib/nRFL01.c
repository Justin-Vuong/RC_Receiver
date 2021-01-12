#include <avr/io.h>
#include <stdlib.h> 
#include <stdbool.h>
#include <util/delay.h>
#include <string.h>
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
#define R_RX_PL_WID             0x60
#define NOP                     0xFF 

uint8_t tx_addr[] = {0xE8, 0xE8, 0xE8, 0xE8, 0xE8};
uint8_t rx_addr[] = {0xC1, 0xC1, 0xC1, 0xC1, 0xC1};

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
    _delay_ms(100);

    //Configure config register
    nRFL01_Write_Reg(0x00, 0x0E);

    _delay_ms(2);

    //Enable Auto Acknowledge on all pipes
    nRFL01_Write_Reg(0x01, 0x3F);

    //Enable RX_addresses 0 and 1
    nRFL01_Write_Reg(0x02, 0x01);

    //Set RX address length to 5 bytes
    nRFL01_Write_Reg(0x03, 0x03);

    //Set auto retransmit delay to 4000 us
    nRFL01_Write_Reg(0x04, 0xFF);

    //Set frequency channel
    //Might need to change based on other RF devices nearby
    nRFL01_Write_Reg(0x05, 0xF0);

    //Set data rate to 2Mbps
    nRFL01_Write_Reg(0x06, 0x0E);

    //Reset IRQ Flags in Status register
    nRFL01_Write_Reg(0x07, 0x7E);

    //Set TX_ADDR
    nRFL01_Write_Regs(0x10, tx_addr, 5);

    //Set RX_ADDR_P0  (Acknowledgement packet pipe)
    nRFL01_Write_Regs(0x0A, tx_addr, 5);

    //Set RX_ADDR_P1  (Data pipe)
    nRFL01_Write_Regs(0x0B, rx_addr, 5);

    //Set RX_PW_P0 to receive payloads of length 6
    nRFL01_Write_Reg(0x11, 0x06);

    //Set RX_PW_P1 to receive payloads of length 6
    nRFL01_Write_Reg(0x12, 0x06);

    //Reset TX Max Retry Flag in status register
    nRFL01_clear_max_retries();

    //Send FLUSH_TX to clear TX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(FLUSH_TX);
    PORTB |= (1 << CS);

    //Send FLUSH_RX to clear RX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(FLUSH_RX);
    PORTB |= (1 << CS);
    
    _delay_ms(100);
}

//Loads up to 32 bytes in TX FIFO
void nRFL01_Write_Tx_Payload(uint8_t* values, int size)
{
    //Send FLUSH_TX to clear TX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(FLUSH_TX);
    PORTB |= (1 << CS);

    //Writes multiple byte to addr
    PORTB &= ~(1 << CS);
    SPI_Send(W_TX_PAYLOAD);
    SPI_Send_Bytes(values, size);
    PORTB |= (1 << CS);

    _delay_ms(10);
    //Set chip enable pin to enter standby mode 2 (send data)
    PORTB |= (1<<CE); 
    _delay_ms(20);
    PORTB &= ~(1<<CE); 
    _delay_ms(10);

}

void nRFL01_RX_Init(void)
{
    _delay_ms(100);

    //Configure config register
    nRFL01_Write_Reg(0x00, 0x0F);

    _delay_ms(2);

    //Enable auto acknowledgement packets for all pipes
    nRFL01_Write_Reg(0x01, 0x3F);

    //Enable RX pipe 1
    nRFL01_Write_Reg(0x02, 0x02);

    //Set RX address length to 5 bytes
    nRFL01_Write_Reg(0x03, 0x03);

    //Set auto retransmit delay to 4000 us alnd Auto Retransmit Count to 15
    nRFL01_Write_Reg(0x04, 0xFF);

    //Set frequency channel
    //Might need to change based on other RF devices nearby
    nRFL01_Write_Reg(0x05, 0xF0);

    //Set data rate to 2Mbps
    nRFL01_Write_Reg(0x06, 0x0E);

    //Reset IRQ Flags in Status register
    nRFL01_Write_Reg(0x07, 0x7E);

    //Set TX_ADDR for the RX device
    nRFL01_Write_Regs(0x10, rx_addr, 5);

    //Set RX_ADDR_P0 (acknowledgement packet pipe)
    nRFL01_Write_Regs(0x0A, rx_addr, 5);

    //Set RX_ADDR_P1 (data pipe)
    nRFL01_Write_Regs(0x0B, tx_addr, 5);

    //Set data length on pipe 0 to 6
    nRFL01_Write_Reg(0x11, 0x06);    

    //Set data length on pipe 1 to 6
    nRFL01_Write_Reg(0x12, 0x06);    

    //Reset TX Max Retry Flag in status register
    nRFL01_clear_max_retries();

    //Send FLUSH_TX to clear TX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(FLUSH_TX);
    PORTB |= (1 << CS);

    //Send FLUSH_RX to clear RX FIFO
    PORTB &= ~(1 << CS);
    SPI_Send(FLUSH_RX);
    PORTB |= (1 << CS);

    //Set enable pin to enter standby mode 2
    PORTB |= (1<<CE);

    _delay_ms(100);
}

void nRFL01_RX_Read_Payload(unsigned char* data, uint8_t dataSz)
{   
    //Read RX payload through SPI interface
    strcpy((unsigned char*) data, "FILLER");
    PORTB &= ~(1 << CS);
    SPI_Send(R_RX_PAYLOAD);
    SPI_Read_Bytes(data, dataSz);
    PORTB |= (1 << CS);

    //Store the status register in status_reg
    uint8_t status_reg = 0;
    PORTB &= ~(1 << CS);
    status_reg = SPI_RW_Byte(NOP);
    PORTB |= (1 << CS);

    //Write a 1 to RX_DR to clear IRQ
    PORTB &= ~(1 << CS);
    SPI_Send(W_REGISTER(0x07));
    SPI_Send(status_reg | (1 << 6));
    PORTB |= (1 << CS);

    _delay_ms(10);
}

void nRFL01_heartbeat(uint8_t* data)
{
    /*
    xxx represents place holders depending where data can be read in RX
        -> 000-101 are data pipe numbers with data
        -> 110 means not used
        -> 111 means RX FIFO Empty

    For TX expect: data[0] = 0x0E and data[1] = 0b0000xxx0
    For RX expect: data[0] = 0x0F and data[1] = 0b0000xxx0
    */

    //Read status register
    PORTB &= ~(1 << CS);
    data[0] = SPI_RW_Byte(NOP);
    PORTB |= (1 << CS);
    
    //Read config register
    data[1] = nRFL01_Read_Reg(0x00);
}

uint8_t nRFL01_check_registers(uint8_t* data, bool isRX)
{
    //PASS IN AN ARRAY OF SIZE 22 AS DATA
    //Checks registers against expected values for RX/TX (isRX ? RX:TX)
    //Return 1 else negative of index with incorrect register value

    /*
    For RX you should expect indexes to have:
        Error Code  | Register  | Value
        1           | 0x00      | 0x0F
        2           | 0x01      | 0x3F
        3           | 0x02      | 0x02
        4           | 0x03      | 0x03
        5           | 0x06      | 0x0E
        6           | 0x10      | 0xC1C1C1C1C1
        7           | 0x11      | 0x06
        8           | 0x12      | 0x06
        9           | 0x0A      | 0xC1C1C1C1C1 
        10          | 0x0B      | 0xE8E8E8E8E8

    For TX you should expect data to have:
        Error Code  | Register  | Value
        1           | 0x00      | 0x0E
        2           | 0x01      | 0x3F
        3           | 0x02      | 0x03 
        4           | 0x03      | 0x03
        5           | 0x06      | 0x0E
        6           | 0x10      | 0xE8E8E8E8E8
        7           | 0x11      | 0x06
        8           | 0x12      | 0x06  
        9           | 0x0A      | 0xE8E8E8E8E8
        //10          | 0x0B      | 0xC1C1C1C1C1
    
    */

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x00));
    data[0] = SPI_Read();
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x01));
    data[1] = SPI_Read();
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x02));
    data[2] = SPI_Read();
    PORTB |= (1 << CS);
    
    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x03));
    data[3] = SPI_Read();
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x06));
    data[4] = SPI_Read();
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x10));
    SPI_Read_Bytes(&data[5], 5);
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x11));
    data[10] = SPI_Read();
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x12));
    data[11] = SPI_Read();
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x0A));
    SPI_Read_Bytes(&data[12], 5);
    PORTB |= (1 << CS);

    PORTB &= ~(1 << CS);
    SPI_Send(R_REGISTER(0x0B));
    SPI_Read_Bytes(&data[17], 5);
    PORTB |= (1 << CS);

    if(isRX)
    {
        if (data[0] != 0x0F)
        {
            return 1;
        }
        else if (data[1] != 0x3F)
        {
            return 2;
        }
        else if (data[2] != 0x02)
        {
            return 3;
        }
        else if (data[3] != 0x03)
        {
            return 4;
        }
        else if (data[4] != 0x0E)
        {
            return 5;
        }
        else if (data[5] != 0xC1)
        {
            return 6;
        }
        else if (data[6] != 0xC1)
        {
            return 6;
        }
        else if (data[7] != 0xC1)
        {
            return 6;
        }
        else if (data[8] != 0xC1)
        {
            return 6;
        }
        else if (data[9] != 0xC1)
        {
            return 6;
        }
        
        else if (data[10] != 0x06)
        {
            return 7;
        }
        else if (data[11] != 0x06)
        {
            return 8;
        }
        
        else if (data[12] != 0xC1)
        {
            return 9;
        }
        else if (data[13] != 0xC1)
        {
            return 9;
        }
        else if (data[14] != 0xC1)
        {
            return 9;
        }
        else if (data[15] != 0xC1)
        {
            return 9;
        }
        else if (data[16] != 0xC1)
        {
            return 9;
        }
        else if (data[17] != 0xE8)
        {
            return 10;
        }
        else if (data[18] != 0xE8)
        {
            return 10;
        }
        else if (data[19] != 0xE8)
        {
            return 10;
        }
        else if (data[20] != 0xE8)
        {
            return 10;
        }
        else if (data[21] != 0xE8)
        {
            return 10;
        }
    }
    else
    {
        if (data[0] != 0x0E)
        {
            return 1;
        }
        else if (data[1] != 0x3F)
        {
            return 2;
        }
        else if (data[2] != 0x01)
        {
            return 3;
        }
        else if (data[3] != 0x03)
        {
            return 4;
        }
        else if (data[4] != 0x0E)
        {
            return 5;
        }
        else if (data[5] != 0xE8)
        {
            return 6;
        }
        else if (data[6] != 0xE8)
        {
            return 6;
        }
        else if (data[7] != 0xE8)
        {
            return 6;
        }
        else if (data[8] != 0xE8)
        {
            return 6;
        }
        else if (data[9] != 0xE8)
        {
            return 6;
        }
        else if (data[10] != 0x06)
        {
            return 7;
        }
        else if (data[11] != 0x06)
        {
            return 8;
        }
        else if (data[12] != 0xE8)
        {
            return 9;
        }
        else if (data[13] != 0xE8)
        {
            return 9;
        }
        else if (data[14] != 0xE8)
        {
            return 9;
        }
        else if (data[15] != 0xE8)
        {
            return 9;
        }
        else if (data[16] != 0xE8)
        {
            return 9;
        }
        else if (data[17] != 0xC1)
        {
            return 10;
        }
        else if (data[18] != 0xC1)
        {
            return 10;
        }
        else if (data[19] != 0xC1)
        {
            return 10;
        }
        else if (data[20] != 0xC1)
        {
            return 10;
        }
        else if (data[21] != 0xC1)
        {
            return 10;
        }
    }
    
    return 0;
}

void nRFL01_clear_max_retries(void)
{
    uint8_t status_reg = 0;
    PORTB &= ~(1 << CS);
    status_reg = SPI_RW_Byte(NOP);
    PORTB |= (1 << CS);

    //Write 1 to MAX_RT in Status register
    PORTB &= ~(1 << CS);
    SPI_Send(W_REGISTER(0x07));
    SPI_Send(status_reg | (1 << 4));
    PORTB |= (1 << CS);
}