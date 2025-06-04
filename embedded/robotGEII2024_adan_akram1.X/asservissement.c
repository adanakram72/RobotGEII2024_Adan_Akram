
#include "asservissement.h"

#include "QEI.h"
#include "timer.h"
#include "IO.h"
#include "UART_Protocol.h"
#include "UART.h"
#include <math.h>
#include "Utilities.h"
#include "robot.h"
#include <xc.h>

void SetupPidAsservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, double proportionelleMax, double integralMax, double deriveeMax) {
    PidCorr->Kp = Kp;
    PidCorr->erreurProportionelleMax = proportionelleMax; //On limite la correction due au Kp
    PidCorr->Ki = Ki;
    PidCorr->erreurIntegraleMax = integralMax; //On limite la correction due au Ki
    PidCorr->Kd = Kd;
    PidCorr->erreurDeriveeMax = deriveeMax;
}

#define PID_Data 0x0040

void SendPositionData() {
    unsigned char positionPayload[108];
    getBytesFromDouble(positionPayload, 0, timestamp);
    getBytesFromDouble(positionPayload, 8, (double) (PidCorrector->Kp));
    getBytesFromDouble(positionPayload, 16, (double) (PidCorrector->Ki));
    getBytesFromDouble(positionPayload, 24, (double) (PidCorrector->corrD));
    getBytesFromDouble(positionPayload, 32, (double) (PidCorrector->corrI));
    getBytesFromDouble(positionPayload, 40, (double) (PidCorrector->corrP));
    getBytesFromDouble(positionPayload, 48, (double) (PidCorrector->epsilon_1));
    getBytesFromDouble(positionPayload, 56, (double) (PidCorrector->erreur));
    getBytesFromDouble(positionPayload, 64, (double) (PidCorrector->erreurDeriveeMax));
    getBytesFromDouble(positionPayload, 72, (double) (PidCorrector->erreurIntegrale));
    UartEncodeAndSendMessage(PID_Data, 80, positionPayload);
}