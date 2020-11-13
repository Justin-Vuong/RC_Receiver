#ifndef SERIAL_PORT_SOURCE
#define SERIAL_PORT_SOURCE

void USART0Init(void);
void USART0SendByte(uint8_t u8Data);
uint8_t USART0ReceiveByte();

#endif