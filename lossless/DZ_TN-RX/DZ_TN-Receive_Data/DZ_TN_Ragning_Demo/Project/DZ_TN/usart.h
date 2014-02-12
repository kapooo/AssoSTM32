

#ifndef __USART_H

#define __USART_H



#ifdef __cplusplus

 extern "C" {

#endif

#include "stdint.h"

   

void usart_init(u32 baud);

void usart2_init(u32 baud); // Aggiunto da AC per abilitare USART2

int myprintf(const char *fmt, ...);

int myprintf2(const char *fmt, ...); // Aggiunto da AC per stampare su USART2

int myprintf_CMD(const char *fmt, ...);

uint8_t SerialGetChar1(void);

void SerialGetString1(u8 *string, u8 len );

#ifdef __cplusplus

}

#endif



#endif /* __USART_H */