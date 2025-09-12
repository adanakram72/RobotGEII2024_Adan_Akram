
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
#define AssX 0x0093 //erreur et corr lin
#define AssTheta 0x0094 //erreur et corr an
#define Consignes 0x0095  //consignes
#define CmdErreur 0x0096 //commandes


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