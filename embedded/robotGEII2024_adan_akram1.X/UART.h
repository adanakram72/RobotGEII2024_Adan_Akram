 /* 
 * File:   UART.h
 * Author: Table 9
 *
 * Created on 13 novembre 2024, 17:31
 */

#ifndef UART_H
#define	UART_H

void InitUART(void);
void SendMessageDirect(unsigned char* message, int length);
void InitUART2(void);


#endif	/* UART_H */
