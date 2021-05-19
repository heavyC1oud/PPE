
#ifndef _UART_H_
#define _UART_H_

#define USART_BUF_SIZE 256

typedef struct
{
    uint8_t data[USART_BUF_SIZE];
    uint16_t count;
} uart_buf_t;


void initUART(void);
void uart_putch(char ch);
void uart_puts(char* str);
void uart_putdata(uint8_t* data, uint16_t size);
void uart_putdata(uint8_t* data, uint16_t size);
uint8_t getUartTF(void);

extern uart_buf_t rbuf;

#endif  // _UART_H_
