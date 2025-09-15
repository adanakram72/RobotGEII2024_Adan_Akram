#include <stdio.h>
#include <xc.h>
#include "UART_Protocol.h"
#include "CB_TX1.h"
#include "IO.h"
#include "Utilities.h"
#include "robot.h"
#include "asservissement.h"
#include <math.h>

// Fonction pour envoyer les valeurs des télémètres via UART

void EnvoieDistanceTelemetre() {
    unsigned char payload[10];
    // int val_ExG = (int) robotState.distanceTelemetreExGauche;
    payload[0] = (unsigned char) ((int) robotState.distanceTelemetreExGauche);
    payload[1] = (unsigned char) (((int) robotState.distanceTelemetreExGauche) >> 8);
    payload[2] = (unsigned char) ((int) robotState.distanceTelemetreGauche);
    payload[3] = (unsigned char) (((int) robotState.distanceTelemetreGauche) >> 8);
    payload[4] = (unsigned char) ((int) robotState.distanceTelemetreCentre);
    payload[5] = (unsigned char) (((int) robotState.distanceTelemetreCentre) >> 8);
    payload[6] = (unsigned char) ((int) robotState.distanceTelemetreDroit);
    payload[7] = (unsigned char) (((int) robotState.distanceTelemetreDroit) >> 8);
    payload[8] = (unsigned char) ((int) robotState.distanceTelemetreExDroite);
    payload[9] = (unsigned char) (((int) robotState.distanceTelemetreExDroite) >> 8);
    UartEncodeAndSendMessage(0x0030, 10, payload);
}

/*void EvoieMoteurInfo(){
    
}*/

void sendled(void) {
    unsigned char led[5];
    led[0] = LED_VERTE_2;
    led[1] = LED_BLEUE_2;
    led[2] = LED_BLANCHE_2;
    led[3] = LED_ORANGE_2;
    led[4] = LED_ROUGE_2;
    UartEncodeAndSendMessage(0x0020, 5, led);
}

void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* payload) {
    unsigned char message [6 + msgPayloadLength];
    int pos = 0;
    message[pos++] = 0xFE;
    message[pos++] = (unsigned char) (msgFunction >> 8);
    message[pos++] = (unsigned char) (msgFunction);
    message[pos++] = (unsigned char) (msgPayloadLength >> 8);
    message[pos++] = (unsigned char) (msgPayloadLength);
    for (int i = 0; i < msgPayloadLength; i++) {
        message[pos++] = payload[i];
    }
    char c = UartCalculateChecksum(msgFunction, msgPayloadLength, payload);
    message[pos++] = c;
    SendMessage(message, pos);
}

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* payload) {
    unsigned char c = 0;
    c ^= 0xFE;
    c ^= (char) (msgFunction >> 8);
    c ^= (char) (msgFunction);
    c ^= (char) (msgPayloadLength >> 8);
    c ^= (char) (msgPayloadLength);
    for (int i = 0; i < msgPayloadLength; i++) {
        c ^= payload[i];
    }
    return c;
}

int msgDecodedFunction = 0;
int msgDecodedPayloadLength = 0;
unsigned char msgDecodedPayload[128];
int msgDecodedPayloadIndex = 0;
StateReception rcvState = Waiting;

void UartDecodeMessage(unsigned char c) {
    unsigned char receivedChecksum;
    unsigned char calculatedChecksum;
    switch (rcvState) {
        case Waiting:
            if (c == 0xFE) {
                rcvState = FunctionMSB;
            }
            break;
        case FunctionMSB:
            msgDecodedFunction = c << 8;
            rcvState = FunctionLSB;
            break;
        case FunctionLSB:
            msgDecodedFunction |= c;
            rcvState = PayloadLengthMSB;
            break;
        case PayloadLengthMSB:
            msgDecodedPayloadLength = c << 8;
            rcvState = PayloadLengthLSB;
            break;
        case PayloadLengthLSB:
            msgDecodedPayloadLength |= c;
            if (msgDecodedPayloadLength > 128) {
                rcvState = Waiting; // Payload trop grand
            } else if (msgDecodedPayloadLength > 0) {
                msgDecodedPayloadIndex = 0;
                rcvState = Payload;
            } else {
                rcvState = CheckSum;
            }
            break;
        case Payload:
            msgDecodedPayload[msgDecodedPayloadIndex++] = c;
            if (msgDecodedPayloadIndex >= msgDecodedPayloadLength) {
                rcvState = CheckSum;
            }
            break;
        case CheckSum:
            receivedChecksum = c;
            calculatedChecksum = UartCalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            if (receivedChecksum == calculatedChecksum) {
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            }
            rcvState = Waiting;
            break;
        default:
            rcvState = Waiting;
            break;
    }
}

float correcteurKp, correcteurKd, correcteurKi, consigneLineaire, limitPX, limitIX, limitDX;

float correcteurThetaKp, correcteurThetaKd, correcteurThetaKi, consigneAngulaire, limitPTheta, limitITheta, limitDTheta;

void UartProcessDecodedMessage(int function, int payloadLength, unsigned char* payload) {
    int etatLed;
    switch (function) {
        case 0x0030:
            break;
        case 0x0040:

            break;
        case 0x0020:
            etatLed = payload[0];
            if (etatLed == 0) {
                LED_VERTE_2 = payload[1];
            } else if (etatLed == 1) {
                LED_BLEUE_2 = payload[1];
            } else if (etatLed == 2) {
                LED_BLANCHE_2 = payload[1];
            } else if (etatLed == 3) {
                LED_ORANGE_2 = payload[1];
            } else if (etatLed == 4) {
                LED_ROUGE_2 = payload[1];
            }
            break;

        case PidXConf:
            correcteurKp = getFloat(payload, 0);
            correcteurKi = getFloat(payload, 4);
            correcteurKd = getFloat(payload, 8);
            limitPX = getFloat(payload, 12);
            limitIX = getFloat(payload, 16);
            limitDX = getFloat(payload, 20);

            SetupPidAsservissement(&robotState.PidX,
                    (double) correcteurKp,
                    (double) correcteurKi,
                    (double) correcteurKd,
                    (double) limitPX,
                    (double) limitIX,
                    (double) limitDX);

            Correcteur(&robotState.PidX, 0.5);

            getBytesFromFloat(robotState.correcteursXPayload, 0, (float) correcteurKp);
            getBytesFromFloat(robotState.correcteursXPayload, 4,(float)correcteurKi);
            getBytesFromFloat(robotState.correcteursXPayload, 8, (float)correcteurKd);
            getBytesFromFloat(robotState.correcteursXPayload, 12, (float)limitPX);
            getBytesFromFloat(robotState.correcteursXPayload, 16, (float)limitIX);
            getBytesFromFloat(robotState.correcteursXPayload, 20, (float)limitDX);

            getBytesFromFloat(robotState.correcteursXPayload, 24, (float)robotState.PidX.corrP);
            getBytesFromFloat(robotState.correcteursXPayload, 28, (float)robotState.PidX.erreurProportionelleMax);
            getBytesFromFloat(robotState.correcteursXPayload, 32, (float)robotState.PidX.corrI);
            getBytesFromFloat(robotState.correcteursXPayload, 36, (float)robotState.PidX.erreurIntegraleMax);
            getBytesFromFloat(robotState.correcteursXPayload, 40, (float)robotState.PidX.corrD);
            getBytesFromFloat(robotState.correcteursXPayload, 44, (float)robotState.PidX.erreurDeriveeMax);

            getBytesFromFloat(robotState.correcteursXPayload, 48, (float)robotState.PidX.erreur);
            getBytesFromFloat(robotState.correcteursXPayload, 52, (float)robotState.xCorrectionVitesse);
            getBytesFromFloat(robotState.correcteursXPayload, 56, (float)robotState.vitesseLineaireFromOdometry);

            UartEncodeAndSendMessage(PidXConf, 60, robotState.correcteursXPayload);
            break;

        case PidThetaConf:
            correcteurThetaKp = getFloat(payload, 0);
            correcteurThetaKi = getFloat(payload, 4);
            correcteurThetaKd = getFloat(payload, 8);
            limitPTheta = getFloat(payload, 12);
            limitITheta = getFloat(payload, 16);
            limitDTheta = getFloat(payload, 20);

            SetupPidAsservissement(&robotState.PidTheta,
                    (double) correcteurThetaKp,
                    (double) correcteurThetaKi,
                    (double) correcteurThetaKd,
                    (double) limitPTheta,
                    (double) limitITheta,
                    (double) limitDTheta);

            Correcteur(&robotState.PidTheta, 1);

            getBytesFromFloat(robotState.correcteursThetaPayload, 0, correcteurThetaKp);
            getBytesFromFloat(robotState.correcteursThetaPayload, 4, correcteurThetaKi);
            getBytesFromFloat(robotState.correcteursThetaPayload, 8, correcteurThetaKd);
            getBytesFromFloat(robotState.correcteursThetaPayload, 12, limitPTheta);
            getBytesFromFloat(robotState.correcteursThetaPayload, 16, limitITheta);
            getBytesFromFloat(robotState.correcteursThetaPayload, 20, limitDTheta);

            getBytesFromFloat(robotState.correcteursThetaPayload, 24, robotState.PidTheta.corrP);
            getBytesFromFloat(robotState.correcteursThetaPayload, 28, robotState.PidTheta.erreurProportionelleMax);
            getBytesFromFloat(robotState.correcteursThetaPayload, 32, robotState.PidTheta.corrI);
            getBytesFromFloat(robotState.correcteursThetaPayload, 36, robotState.PidTheta.erreurIntegraleMax);
            getBytesFromFloat(robotState.correcteursThetaPayload, 40, robotState.PidTheta.corrD);
            getBytesFromFloat(robotState.correcteursThetaPayload, 44, robotState.PidTheta.erreurDeriveeMax);

            getBytesFromFloat(robotState.correcteursThetaPayload, 48, robotState.PidTheta.erreur);
            getBytesFromFloat(robotState.correcteursThetaPayload, 52, robotState.thetaCorrectionVitesse);
            getBytesFromFloat(robotState.correcteursThetaPayload, 56, robotState.vitesseAngulaireFromOdometry);

            UartEncodeAndSendMessage(PidThetaConf, 60, robotState.correcteursThetaPayload);
            break;

        default:
            break;
    }
}