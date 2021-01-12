#include <avr/io.h>
#include <stdlib.h>

#define SPI_DDR DDRB
#define CE PINB1
#define CS PINB2
#define MOSI PINB3
#define MISO PINB4
#define SCK PINB5

//Initialize SPI Master Device
void SPI_init_master(void)
{
    DDRB |= (1 << CS) | (1 << MOSI) | (1 << SCK); //Set MOSI, SCK as Output
    SPCR = (1 << SPE) | (1 << MSTR);             //Enable SPI, Set as Master
}

void SPI_init_slave(void)
{
    DDRB |= (1 << MISO); //MISO as OUTPUT
    SPCR = (1 << SPE);  //Enable SPI
}

//Function to send and receive data
//Make sure size is the same to maintain proper shift and timing
void SPI_RW(char *send_buff, int sendSz, char *recv_buff, int recvSz)
{
    int count = 0;
    int bit_count = (sendSz > recvSz) ? sendSz : recvSz;

    while (count < bit_count)
    {
        if (count < sendSz)
        {
            SPDR = send_buff[count]; //Load data into the buffer
        }

        while (!(SPSR & (1 << SPIF)))
            ; //Wait until transmission complete
        if (count < recvSz)
        {
            recv_buff[count] = SPDR; //Read out data
        }
        count++;
    }
}

uint8_t SPI_RW_Byte(uint8_t data)
{
    //Send a byte then read a byte
    SPDR = data; //Load data into the buffer

    while (!(SPSR & (1 << SPIF))); //Wait until transmission complete

    return SPDR; //Read out data
}

void SPI_Send(uint8_t data)
{
    //Send one byte over SPI
    SPDR = data; //Load data into the buffer

    while (!(SPSR & (1 << SPIF))); //Wait until transmission complete
}

void SPI_Send_Bytes(uint8_t* data, int size)
{
    //Send multiples bytes over SPI
    for(int a = 0; a < size; a++)
    {
        SPDR = data[a]; //Load data into the buffer

        while (!(SPSR & (1 << SPIF))); //Wait until transmission complete
    }
}

uint8_t SPI_Read(void)
{
    //Read one byte frpm SPI register
    SPDR = 0xFF; //Load dummy byte into the buffer

    while (!(SPSR & (1 << SPIF))); //Wait until transmission complete

    return SPDR; //Read out data
}

void SPI_Read_Bytes(uint8_t* buffer, int buffSz)
{
    for (int a = 0; a < buffSz; a++)
    {
        //Read one byte frpm SPI register
        SPDR = 0xFF; //Load dummy byte into the buffer

        while (!(SPSR & (1 << SPIF))); //Wait until transmission complete

        buffer[a] = SPDR;
    }
}