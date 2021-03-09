#ifndef _FUNCTION_INITIALIZATION_H
#define _FUNCTION_INITIALIZATION_H

void INIT_PINS(void);
void INIT_UART_1(unsigned int baudRate, unsigned int stop_bit, unsigned int parity_mode);
void INIT_UART_2(unsigned int baudRate, unsigned int stop_bit, unsigned int parity_mode);
void INIT_UART_3(unsigned int baudRate, unsigned int stop_bit, unsigned int parity_mode);
void INIT_UART_4(unsigned int baudRate, unsigned int stop_bit, unsigned int parity_mode);

#endif