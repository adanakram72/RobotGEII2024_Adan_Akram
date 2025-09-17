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
    double erreurProportionnelle = LimitToInterval(erreur, -PidCorr->erreurProportionelleMax / PidCorr->Kp, PidCorr->erreurProportionelleMax / PidCorr->Kp);
    PidCorr->corrP = PidCorr->Kp * erreurProportionnelle;

    PidCorr->erreurIntegrale += erreur / FREQ_ECH_QEI;
    PidCorr->erreurIntegrale = LimitToInterval(PidCorr->erreurIntegrale, -PidCorr->erreurIntegraleMax / PidCorr->Ki, PidCorr->erreurIntegraleMax / PidCorr->Ki);
    PidCorr->corrI = PidCorr->Ki * PidCorr->erreurIntegrale;

    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
    double deriveeBornee = LimitToInterval(erreurDerivee, -PidCorr->erreurDeriveeMax / PidCorr->Kd, PidCorr->erreurDeriveeMax / PidCorr->Kd);
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
