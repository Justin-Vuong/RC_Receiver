#ifndef SERIAL_PORT_SOURCE
#define SERIAL_PORT_SOURCE

void USART0Init(void);
void USART0SendByte(uint8_t u8Data);
void USART0Send(uint8_t* u8Data, int size);
void USART0Read(uint8_t* u8Data, int size);
uint8_t USART0ReadByte();

#endif