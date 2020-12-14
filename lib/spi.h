#ifndef SPI_SOURCE
#define SPI_SOURCE

void SPI_init_master(void);
void SPI_init_slave(void);
void SPI_RW(char* out_buff, int out_Sz, char* in_buff, int inSize);
uint8_t SPI_RW_Byte(uint8_t data);
void SPI_Send(uint8_t data);
void SPI_Send_Bytes(uint8_t* data, int size);
uint8_t SPI_Read(void);
void SPI_Read_Bytes(uint8_t* buffer, int buffSz);
#endif
