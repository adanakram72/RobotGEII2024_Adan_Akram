#include <xc.h>
#include "IO.h"
#include "PWM.h"
#include "robot.h"
#include "ToolBox.h"
#include "timer.h"
#include "main.h"
#include "QEI.h"


#define PWMPER 24.0
#define SPEED_TO_PERCENT 32.0

float acceleration = 5;
int modeAuto = 0;

void InitPWM(void) {
    PTCON2bits.PCLKDIV = 0b000; //Divide by 1
    PTPER = 100 * PWMPER; //ÈPriode en pourcentage
    //ÈRglage PWM moteur 1 sur hacheur 1
    IOCON1bits.PMOD = 0b11; //PWM I/O pin pair is in the True Independent Output mode
    IOCON1bits.PENL = 1;
    IOCON1bits.PENH = 1;
    FCLCON1 = 0x0003; //ÈDsactive la gestion des faults
    IOCON2bits.PMOD = 0b11; //PWM I/O pin pair is in the True Independent Output mode
    IOCON2bits.PENL = 1;
    IOCON2bits.PENH = 1;
    FCLCON2 = 0x0003; //ÈDsactive la gestion des faults
    /* Enable PWM Module */
    PTCONbits.PTEN = 1;
}

double talon = 50;

/*void PWMSetSpeed(float vitesseEnPourcents, int MOTEUR) {
    if (MOTEUR == MOTEUR_GAUCHE) {
        PDC1 = vitesseEnPourcents * PWMPER + talon;
        SDC1 = talon;
    } else {
        SDC2 = vitesseEnPourcents * PWMPER + talon;
        PDC2 = talon;
    }
}*/


void PWMUpdateSpeed() {
    // Cette fonction est appelee sur timer et permet de suivre des rampes d acceleration
    if (robotState.vitesseDroiteCommandeCourante < robotState.vitesseDroiteConsigne) {
        robotState.vitesseDroiteCommandeCourante = Min(
                robotState.vitesseDroiteCommandeCourante + acceleration,
                robotState.vitesseDroiteConsigne);
    }
    if (robotState.vitesseDroiteCommandeCourante > robotState.vitesseDroiteConsigne) {
        robotState.vitesseDroiteCommandeCourante = Max(
                robotState.vitesseDroiteCommandeCourante - acceleration,
                robotState.vitesseDroiteConsigne);
    }
    if (robotState.vitesseDroiteCommandeCourante >= 0) {
        PDC2 = robotState.vitesseDroiteCommandeCourante * PWMPER + talon;
        SDC2 = talon;
    } else {
        PDC2 = talon;
        SDC2 = -robotState.vitesseDroiteCommandeCourante * PWMPER + talon;
    }
    if (robotState.vitesseGaucheCommandeCourante < robotState.vitesseGaucheConsigne) {
        robotState.vitesseGaucheCommandeCourante = Min(
                robotState.vitesseGaucheCommandeCourante + acceleration,
                robotState.vitesseGaucheConsigne);
    }
    if (robotState.vitesseGaucheCommandeCourante > robotState.vitesseGaucheConsigne) {
        robotState.vitesseGaucheCommandeCourante = Max(
                robotState.vitesseGaucheCommandeCourante - acceleration,
                robotState.vitesseGaucheConsigne);
    }
    if (robotState.vitesseGaucheCommandeCourante > 0) {
        PDC1 = robotState.vitesseGaucheCommandeCourante * PWMPER + talon;
        SDC1 = talon;
    } else {
        PDC1 = talon;
        SDC1 = -robotState.vitesseGaucheCommandeCourante * PWMPER + talon;
    }
}

void PWMSetSpeedConsigne(float vitesseEnPourcents, int MOTEUR) {
    if (MOTEUR == MOTEUR_DROIT) {
        robotState.vitesseDroiteConsigne = -vitesseEnPourcents;
    }
    else if (MOTEUR == MOTEUR_GAUCHE) {
        robotState.vitesseGaucheConsigne = vitesseEnPourcents;
    }
}

void PWMSetSpeedConsignePolaire(double vitesseLineaire, double vitesseAngulaire) {
    if (modeAuto == 0) {
        robotState.vitesseDroiteConsigne = -SPEED_TO_PERCENT * (vitesseLineaire + vitesseAngulaire*DISTROUES/2);
        robotState.vitesseGaucheConsigne = SPEED_TO_PERCENT * (vitesseLineaire - vitesseAngulaire*DISTROUES/2); // divise distroues par 2 si problËme !
        PWMUpdateSpeed();
    }
}