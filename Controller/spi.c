#include <avr/io.h>
#include <stdlib.h>

#define SPI_DDR DDRB
#define CS      PINB2
#define MOSI    PINB3
#define MISO    PINB4
#define SCK     PINB5


//Initialize SPI Master Device
void SPI_init_master (void)
{
    DDRB = (1<<CS)|(1<<MOSI)|(1<<SCK);              //Set MOSI, SCK as Output
    SPCR = (1<<SPE)|(1<<MSTR);                      //Enable SPI, Set as Master
}

void SPI_init_slave (void)
{
    DDRB=(1<<MISO);                               //MISO as OUTPUT
    SPCR=(1<<SPE);                                //Enable SPI
}
 
//Function to send and receive data
void SPI_RW(char* out_buff, int out_Sz, char* in_buff, int inSize)
{
    int count = 0;
    int bit_count = (out_Sz > inSize) ? out_Sz : inSize;

    while (count < bit_count)
    {
        if (count < out_Sz)
        {
            SPDR = out_buff[count];         //Load data into the buffer
        }
                      
        while(!(SPSR & (1<<SPIF)));         //Wait until transmission complete
        if (count < inSize) 
        {
            in_buff[count] = SPDR;          //Read out data
        }
        count++;
    }
}