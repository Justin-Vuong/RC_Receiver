#ifndef SPI_SOURCE
#define SPI_SOURCE

void SPI_init_master(void);
void SPI_init_slave(void);
void SPI_RW(char* out_buff, int out_Sz, char* in_buff, int inSize);
uint8_t SPI_Send_Recv_Byte(uint8_t data);
void SPI_start_transaction(void);
void SPI_end_transaction(void);

#endif
