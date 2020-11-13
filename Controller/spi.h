#ifndef SPI_SOURCE
#define SPI_SOURCE

void SPI_init_master(void);
void SPI_init_slave(void);
void SPI_RW(char* out_buff, int out_Sz, char* in_buff, int inSize);

#endif