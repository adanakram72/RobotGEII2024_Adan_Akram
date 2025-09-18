#include "asservissement.h"
#include "QEI.h"
#include "timer.h"
#include "IO.h"
#include "UART_Protocol.h"
#include "UART.h"
#include "Utilities.h"
#include "robot.h"
#include <xc.h>
#include "PWM.h"

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

double Correcteur(volatile PidCorrector* PidCorr, double erreur) {
    PidCorr->erreur = erreur;
    double erreurProportionnelle = LimitToIntervalBis(erreur, -PidCorr->erreurProportionelleMax / PidCorr->Kp, PidCorr->erreurProportionelleMax / PidCorr->Kp);
    PidCorr->corrP = PidCorr->Kp * erreurProportionnelle;

    PidCorr->erreurIntegrale += erreur / FREQ_ECH_QEI;
    PidCorr->erreurIntegrale = LimitToIntervalBis(PidCorr->erreurIntegrale, -PidCorr->erreurIntegraleMax / PidCorr->Ki, PidCorr->erreurIntegraleMax / PidCorr->Ki);
    PidCorr->corrI = PidCorr->Ki * PidCorr->erreurIntegrale;

    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
    double deriveeBornee = LimitToIntervalBis(erreurDerivee, -PidCorr->erreurDeriveeMax / PidCorr->Kd, PidCorr->erreurDeriveeMax / PidCorr->Kd);
    PidCorr->epsilon_1 = erreur;
    PidCorr->corrD = deriveeBornee * PidCorr->Kd;

    return PidCorr->corrP + PidCorr->corrI + PidCorr->corrD;
}

void UpdateAsservissement() {
    robotState.PidX.erreur = robotState.consigneVitesseLineaire - robotState.vitesseLineaireFromOdometry;
    robotState.PidTheta.erreur = robotState.consigneVitesseAngulaire - robotState.vitesseAngulaireFromOdometry;
    robotState.xCorrectionVitesse = Correcteur(&robotState.PidX, robotState.PidX.erreur);
    robotState.thetaCorrectionVitesse = Correcteur(&robotState.PidTheta, robotState.PidTheta.erreur);
    PWMSetSpeedConsignePolaire(robotState.xCorrectionVitesse, robotState.thetaCorrectionVitesse);

}

void sendPidDonnees() {

    getBytesFromFloat(robotState.correcteursThetaXPayload, 0, robotState.PidX.erreur);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 4, robotState.xCorrectionVitesse);

    getBytesFromFloat(robotState.correcteursThetaXPayload, 8, robotState.PidX.Kp);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 12, robotState.PidX.corrP);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 16, robotState.PidX.erreurProportionelleMax);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 20, robotState.PidX.Ki);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 24, robotState.PidX.corrI);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 28, robotState.PidX.erreurIntegraleMax);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 32, robotState.PidX.Kd);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 36, robotState.PidX.corrD);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 40, robotState.PidX.erreurDeriveeMax);


    getBytesFromFloat(robotState.correcteursThetaXPayload, 44, robotState.PidTheta.erreur);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 48, robotState.thetaCorrectionVitesse);

    getBytesFromFloat(robotState.correcteursThetaXPayload, 52, robotState.PidTheta.Kp);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 56, robotState.PidTheta.corrP);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 60, robotState.PidTheta.erreurProportionelleMax);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 64, robotState.PidTheta.Ki);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 68, robotState.PidTheta.corrI);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 72, robotState.PidTheta.erreurIntegraleMax);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 76, robotState.PidTheta.Kd);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 80, robotState.PidTheta.corrD);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 84, robotState.PidTheta.erreurDeriveeMax);

    UartEncodeAndSendMessage(PidThetaXConf, 88, robotState.correcteursThetaXPayload);
}

float LimitToIntervalBis(float value, float lowLimit, float highLimit) {
    if (value > highLimit)
        value = highLimit;
    else if (value < lowLimit)
        value = lowLimit;
    return value;
}