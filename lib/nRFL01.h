#include <stdbool.h>

#ifndef nRFL01_SRC
#define nRFL01_SRC

void nRFL01_Write_Reg(uint8_t addr, uint8_t value);
uint8_t nRFL01_Read_Reg(uint8_t addr);
void nRFL01_Write_Regs(uint8_t addr, uint8_t* values, int size);
void nRFL01_TX_Init(void);
void nRFL01_Write_Tx_Payload(uint8_t* values, int size);
void nRFL01_RX_Init(void);
void nRFL01_RX_Read_Payload(uint8_t* data, int dataSz);
void nRFL01_heartbeat(uint8_t* data);
uint8_t nRFL01_check_registers(uint8_t* data, bool isRX);

#endif
