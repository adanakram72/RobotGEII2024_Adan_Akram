
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
    PidCorr->erreurIntegrale = 0;
    PidCorr->epsilon_1 = 0;
}

//void SendPidX() {
//    unsigned char asservissementXPayload[24];
//    getBytesFromFloat((unsigned char*)asservissementXPayload, 0, (float) (robotState.PidX.corrP));
//    getBytesFromFloat((unsigned char*)asservissementXPayload, 4, (float) (robotState.PidX.erreurProportionelleMax));
//    getBytesFromFloat((unsigned char*)asservissementXPayload, 8, (float) (robotState.PidX.corrI));
//    getBytesFromFloat((unsigned char*)asservissementXPayload, 12, (float) (robotState.PidX.erreurIntegraleMax));
//    getBytesFromFloat((unsigned char*)asservissementXPayload, 16, (float) (robotState.PidX.corrD));
//    getBytesFromFloat((unsigned char*)asservissementXPayload, 20, (float) (robotState.PidX.erreurDeriveeMax));
//    
//    UartEncodeAndSendMessage(ASSERVISSEMENTX, 24, (unsigned char*)asservissementXPayload);        
//}
//
//void SendPidTheta() {
//    unsigned char asservissementThetaPayload[24];
//    getBytesFromFloat((unsigned char*)asservissementThetaPayload, 0, (float) (robotState.PidTheta.corrP));
//    getBytesFromFloat((unsigned char*)asservissementThetaPayload, 4, (float) (robotState.PidTheta.erreurProportionelleMax));
//    getBytesFromFloat((unsigned char*)asservissementThetaPayload, 8, (float) (robotState.PidTheta.corrI));
//    getBytesFromFloat((unsigned char*)asservissementThetaPayload, 12, (float) (robotState.PidTheta.erreurIntegraleMax));
//    getBytesFromFloat((unsigned char*)asservissementThetaPayload, 16, (float) (robotState.PidTheta.corrD));
//    getBytesFromFloat((unsigned char*)asservissementThetaPayload, 20, (float) (robotState.PidTheta.erreurDeriveeMax));
//    
//    UartEncodeAndSendMessage(ASSERVISSEMENTTHETA, 24, (unsigned char*)asservissementThetaPayload);
//}
//
//void SendCommandeErreur() {
//    unsigned char CommandeErreurPayload[16];
//    getBytesFromFloat((unsigned char*)CommandeErreurPayload, 0, (float) (robotState.xCorrectionVitesse));
//    getBytesFromFloat((unsigned char*)CommandeErreurPayload, 4, (float) (robotState.thetaCorrectionVitesse));
//    getBytesFromFloat((unsigned char*)CommandeErreurPayload, 8, (float) (robotState.PidX.erreur));
//    getBytesFromFloat((unsigned char*)CommandeErreurPayload, 12, (float) (robotState.PidTheta.erreur));
//    UartEncodeAndSendMessage(ASSERVISSEMENTERREUR, 16, (unsigned char*)CommandeErreurPayload);
//}