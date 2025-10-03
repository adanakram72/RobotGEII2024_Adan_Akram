#include <xc.h>
#include "UART.h"
#include "ChipConfig.h"
#define FCY 60000000
#define BAUDRATE 115200
#define BRGVAL ((FCY/BAUDRATE)/4)-1

void InitUART(void) {
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 1; // Low Speed mode
    U1BRG = BRGVAL; // BAUD Rate Setting

    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    IFS0bits.U1TXIF = 0; // clear TX interrupt flag   
    IEC0bits.U1TXIE = 1; // Enable UART Tx interrupt

    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received;
    IFS0bits.U1RXIF = 0; // Enable clear RX interrupt flag
    IEC0bits.U1RXIE = 1; //  UART Rx interrupt
    
    U1MODEbits.UARTEN = 1; // Enable UART
    IPC2bits.U1RXIP = 7;
    U1STAbits.UTXEN = 1; // Enable UART Tx
}

//void InitUART2(void) {
//    U2MODEbits.STSEL = 0; // 1-stop bit
//    U2MODEbits.PDSEL = 0; // No Parity, 8-data bits
//    U2MODEbits.ABAUD = 0; // Auto-Baud Disabled
//    U2MODEbits.BRGH = 1; // Low Speed mode
//    U2BRG = BRGVAL; // BAUD Rate Setting
//
//    U2STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
//    U2STAbits.UTXISEL1 = 0;
//    IFS1bits.U2TXIF = 0; // clear TX interrupt flag   
//    IEC1bits.U2TXIE = 1; // Enable UART Tx interrupt
//
//    U2STAbits.URXISEL = 0; // Interrupt after one RX character is received;
//    IFS1bits.U2RXIF = 0; // Enable clear RX interrupt flag
//    IEC1bits.U2RXIE = 1; //  UART Rx interrupt
//    
//    U2MODEbits.UARTEN = 1; // Enable UART
//    U2STAbits.UTXEN = 1; // Enable UART Tx
//}


void SendMessageDirect(unsigned char* message, int length) {
    unsigned char i = 0;
    for (i = 0; i < length; i++) {
        while (U1STAbits.UTXBF); // wait while Tx buffer full
        U1TXREG = *(message)++; // Transmit one character
    }
}

