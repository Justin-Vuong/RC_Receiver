#ifndef nRFL01_SRC
#define nRFL01_SRC

void nRFL01_Write_Reg(uint8_t addr, uint8_t value);
void nRFL01_Write_Regs(uint8_t addr, uint8_t* values, int size);
void nRFL01_TX_Init(void);

#endif
