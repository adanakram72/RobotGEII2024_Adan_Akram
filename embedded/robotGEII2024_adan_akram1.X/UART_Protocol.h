#include "asservissement.h"
#include "robot.h"


#ifndef UART_Protocol_H
#define	UART_Protocol_H

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
void UartDecodeMessage(unsigned char c);
void UartProcessDecodedMessage(int function, int payloadLength, unsigned char* payload);
void EnvoieDistanceTelemetre();
void EvoieMoteurInfo();
void sendled();

#define PidXConf 0x0091  //linéaire
#define PidThetaConf 0x0092  //angulaire
#define PidThetaXConf 0x0093
#define PosClickGhost 0x0095 


typedef enum {
    Waiting,
    FunctionMSB,
    FunctionLSB,
    PayloadLengthMSB,
    PayloadLengthLSB,
    Payload,
    CheckSum
} StateReception;

#endif	/* UART_H */