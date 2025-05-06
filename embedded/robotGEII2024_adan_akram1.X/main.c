/* 
 * File:   main.c
 * Author: Table 9
 *
 * Created on 11 septembre 2024, 14:06
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "robot.h"
#include "ToolBox.h"
#include "ADC.h"
#include "main.h"
#include "UART.h"
#include "CB_TX1.h"
#include "CB_RX1.h"
#include "UART_protocol.h"
#include "QEI.h"
#include <libpic30.h>

unsigned char stateRobot;
unsigned int tstart = 0;
float Vitesse;
float boundaryTelemetre = 100;
unsigned char payload;

void updateSensorValues() {
    if (ADCIsConversionFinished() == 1) {
        ADCClearConversionFinishedFlag();
        unsigned int *result = ADCGetResult();
        float volts = ((float) result[0]) * 3.3 / 4096;
        robotState.distanceTelemetreExGauche = Min(34 / volts - 5, boundaryTelemetre);
        volts = ((float) result[1]) * 3.3 / 4096;
        robotState.distanceTelemetreGauche = Min(34 / volts - 5, boundaryTelemetre);
        volts = ((float) result[2]) * 3.3 / 4096;
        robotState.distanceTelemetreCentre = Min(34 / volts - 5, boundaryTelemetre);
        volts = ((float) result[3]) * 3.3 / 4096;
        robotState.distanceTelemetreDroit = Min(34 / volts - 5, boundaryTelemetre);
        volts = ((float) result[4]) * 3.3 / 4096;
        robotState.distanceTelemetreExDroite = Min(34 / volts - 5, boundaryTelemetre);
    }
}


int main(void) {
    /***********************************************************************************************/
    //Initialisation oscillateur
    /***********************************************************************************************/
    InitOscillator();
    /***********************************************************************************************/
    //Configuration des input et output (IO)
    /***********************************************************************************************/
    InitIO();
    InitTimer1();
    InitTimer4();
    InitTimer23();
    InitPWM();
    InitADC1();
    InitUART();
    InitQEI1();    
    InitQEI2();
    SetFreqTimer4(1000);
    
    // BOUCLE PRINCIPALE
    while (1) {
        unsigned char payload[] = {'B', 'o', 'n', 'j', 'o', 'u', 'r'};
        UartEncodeAndSendMessage(0x0080, sizeof(payload), payload);
        __delay32(4000000);
    }

    return 0;

    if (TIME1 == 1) {
        tstart = 1;
        tstop = 0;
    }


    if (Vitesse > 21) {
        LED_BLANCHE_2 = 1;
        LED_BLEUE_2 = 1;
        LED_ORANGE_2 = 1;
        LED_ROUGE_2 = 1;
        LED_VERTE_2 = 1;
    }

    if (robotState.distanceTelemetreCentre < 47 && robotState.distanceTelemetreGauche < 47 && robotState.distanceTelemetreDroit < 47) {
        Vitesse = 14;
    } else {
        Vitesse = 22;
        LED_BLANCHE_2 = 1;
        LED_BLEUE_2 = 1;
        LED_ORANGE_2 = 1;
        LED_ROUGE_2 = 1;
        LED_VERTE_2 = 1;
    }
}


void Cap() {
    if (robotState.distanceTelemetreExDroite < 24) {
        LED_VERTE_1 = 1;
    } else {
        LED_VERTE_1 = 0;
    }
    if (robotState.distanceTelemetreDroit < 30) {
        LED_ROUGE_1 = 1;
    } else {
        LED_ROUGE_1 = 0;
    }
    if (robotState.distanceTelemetreCentre < 38) {
        LED_ORANGE_1 = 1;
    } else {
        LED_ORANGE_1 = 0;
    }
    if (robotState.distanceTelemetreGauche < 30) {
        LED_BLEUE_1 = 1;
    } else {
        LED_BLEUE_1 = 0;
    }
    if (robotState.distanceTelemetreExGauche < 24) {
        LED_BLANCHE_1 = 1;
    } else {
        LED_BLANCHE_1 = 0;
    }
}

/*
unsigned char nextStateRobot = 0;

void SetNextRobotStateInAutomaticMode() {
    unsigned char positionObstacle = PAS_D_OBSTACLE;
    //éDtermination de la position des obstacles en fonction des ééètlmtres
    if (robotState.distanceTelemetreDroit < 30 &&
            robotState.distanceTelemetreCentre > 20 &&
            robotState.distanceTelemetreGauche > 30) //Obstacle àdroite
        positionObstacle = OBSTACLE_A_DROITE;
    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 20 &&
            robotState.distanceTelemetreGauche < 30) //Obstacle àgauche
        positionObstacle = OBSTACLE_A_GAUCHE;
    else if (robotState.distanceTelemetreCentre < 20) //Obstacle en face
        positionObstacle = OBSTACLE_EN_FACE;
    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 20 &&
            robotState.distanceTelemetreGauche > 30) //pas d?obstacle
        positionObstacle = PAS_D_OBSTACLE;

    if (robotState.distanceTelemetreExDroite < 20 &&
            robotState.distanceTelemetreDroit < 25 &&
            robotState.distanceTelemetreCentre > 20 && robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExGauche > 30)
        positionObstacle = OBSTACLE_TRES_A_DROITE;
    else if (robotState.distanceTelemetreExDroite > 30 &&
            robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 20 && robotState.distanceTelemetreGauche < 25 &&
            robotState.distanceTelemetreExGauche < 20)
        positionObstacle = OBSTACLE_TRES_A_GAUCHE;
   
    if (positionObstacle == PAS_D_OBSTACLE)
        nextStateRobot = STATE_AVANCE;
    else if (positionObstacle == OBSTACLE_A_DROITE)
        nextStateRobot = STATE_TOURNE_GAUCHE;
    else if (positionObstacle == OBSTACLE_A_GAUCHE)
        nextStateRobot = STATE_TOURNE_DROITE;
    else if (positionObstacle == OBSTACLE_EN_FACE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;

    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}

unsigned char ConversionBin() {
    unsigned char state = 0;
    if (robotState.distanceTelemetreExGauche < 23) state |= (1 << 4);
    if (robotState.distanceTelemetreGauche < 28) state |= (1 << 3);
    if (robotState.distanceTelemetreCentre < 37) state |= (1 << 2);
    if (robotState.distanceTelemetreDroit < 28) state |= (1 << 1);
    if (robotState.distanceTelemetreExDroite < 23) state |= (1 << 0);
    return state;
}

void OperatingSystemLoop(void) {
    if (tstop <= 59000 && tstart) {
        unsigned char stateRobot = ConversionBin();

        switch (stateRobot) {
            case 0b00000: // Aucun /
                stateRobot = STATE_AVANCE;
                break;

            case 0b00001: // extrême droite/
                stateRobot = STATE_TOURNE_GAUCHE;
                break;

            case 0b00010: // droite/
                stateRobot = STATE_TOURNE_GAUCHE;
                break;

            case 0b00011: // droite et extrême droite/
                stateRobot = STATE_TOURNE_GAUCHE;
                break;

            case 0b00100: // centre/
            case 0b00101: // centre et extrême droite
            case 0b00111: // droite, centre et extrême droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
                break;

            case 0b01000: // gauche/////
                stateRobot = STATE_TOURNE_DROITE;
                break;

            case 0b01001: // gauche et extrême droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
                break;

            case 0b01010: // gauche et droite///
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b01011: // gauche, droite et extrême droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
                break;

            case 0b01100: // gauche et centre//
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b01101: // gauche, centre et extrême droite//////
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b01110: // gauche, centre et droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b10000: // extrême gauche//
                stateRobot = STATE_TOURNE_DROITE;
                break;

            case 0b01111: // gauche, droite, centre, et extrême droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
                break;

            case 0b10001: // extrême gauche et extrême droite//
                stateRobot = STATE_AVANCE;
                break;

            case 0b10010: // extrême gauche et droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b10011: // extrême gauche, droite et extrême droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b10100: // extrême gauche et centre// 
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b10101: // extrême gauche, centre et extrême droite////
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b10110: // extrême gauche, centre et droite
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b10111: // extrême gauche, droite, centre, et extrême droite
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b11000: // gauche et extrême gauche//
                stateRobot = STATE_TOURNE_DROITE;
                break;

            case 0b11001: // gauche, extrême gauche et extrême droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b11010: // gauche, extrême gauche et droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b11011: // gauche, droite, extrême gauche et extrême droite//
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b11100: // gauche, centre et extrême gauche
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b11101: // gauche, centre, extrême gauche et extrême droite
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b11110: // extrême gauche, gauche, centre, et droite
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                break;

            case 0b11111: // partout
                stateRobot = STATE_RECULE;
                break;

            default:
                stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
                break;
        }
        switch (stateRobot) {

            case STATE_AVANCE:
                VitesseDroit();
                VitesseExtremeDroit();
                VitesseGauche();
                VitesseExtremeGauche();
                PWMSetSpeedConsigne(Vitesse, MOTEUR_DROIT);
                PWMSetSpeedConsigne(Vitesse, MOTEUR_GAUCHE);
                break;

            case STATE_TOURNE_GAUCHE:
                VitesseDroit();
                PWMSetSpeedConsigne(14, MOTEUR_DROIT);
                PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
                LED_BLANCHE_2 = 0;
                LED_BLEUE_2 = 0;
                LED_ORANGE_2 = 0;
                LED_ROUGE_2 = 0;
                LED_VERTE_2 = 0;
                break;


            case STATE_TOURNE_DROITE:
                VitesseGauche();
                PWMSetSpeedConsigne(0, MOTEUR_DROIT);
                PWMSetSpeedConsigne(14, MOTEUR_GAUCHE);
                LED_BLANCHE_2 = 0;
                LED_BLEUE_2 = 0;
                LED_ORANGE_2 = 0;
                LED_ROUGE_2 = 0;
                LED_VERTE_2 = 0;
                break;

            case STATE_TOURNE_SUR_PLACE_GAUCHE:
                VitesseDroit();
                VitesseGauche();
                PWMSetSpeedConsigne(12, MOTEUR_DROIT);
                PWMSetSpeedConsigne(-12, MOTEUR_GAUCHE);
                LED_BLANCHE_2 = 0;
                LED_BLEUE_2 = 0;
                LED_ORANGE_2 = 0;
                LED_ROUGE_2 = 0;
                LED_VERTE_2 = 0;
                break;

            case STATE_TOURNE_SUR_PLACE_DROITE:
                VitesseDroit();
                VitesseGauche();
                PWMSetSpeedConsigne(-12, MOTEUR_DROIT);
                PWMSetSpeedConsigne(12, MOTEUR_GAUCHE);
                LED_BLANCHE_2 = 0;
                LED_BLEUE_2 = 0;
                LED_ORANGE_2 = 0;
                LED_ROUGE_2 = 0;
                LED_VERTE_2 = 0;
                break;

            case STATE_RECULE:
                PWMSetSpeedConsigne(-17, MOTEUR_DROIT);
                PWMSetSpeedConsigne(-17, MOTEUR_GAUCHE);
                LED_BLANCHE_2 = 0;
                LED_BLEUE_2 = 0;
                LED_ORANGE_2 = 0;
                LED_ROUGE_2 = 0;
                LED_VERTE_2 = 0;
                break;

            case STATE_RECULE_GAUCHE:
                PWMSetSpeedConsigne(-12, MOTEUR_DROIT);
                PWMSetSpeedConsigne(-7, MOTEUR_GAUCHE);
                LED_BLANCHE_2 = 0;
                LED_BLEUE_2 = 0;
                LED_ORANGE_2 = 0;
                LED_ROUGE_2 = 0;
                LED_VERTE_2 = 0;
                break;

            case STATE_DEMI_TOUR:
                PWMSetSpeedConsigne(-15, MOTEUR_DROIT);
                PWMSetSpeedConsigne(-15, MOTEUR_GAUCHE);
                LED_BLANCHE_2 = 0;
                LED_BLEUE_2 = 0;
                LED_ORANGE_2 = 0;
                LED_ROUGE_2 = 0;
                LED_VERTE_2 = 0;
                break;

            case STATE_ATTENTE:
                PWMSetSpeedConsigne(0, MOTEUR_DROIT);
                PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
                LED_BLANCHE_2 = 0;
                LED_BLEUE_2 = 0;
                LED_ORANGE_2 = 0;
                LED_ROUGE_2 = 0;
                LED_VERTE_2 = 0;
                break;

            default:
                break;
        }
    } else {
        tstart = 0;
        PWMSetSpeedConsigne(0, MOTEUR_DROIT);
        PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
    }
}

void VitesseCentre() {
    float V_BASSE = 30;
    float V_HAUTE = 50;

    if (robotState.distanceTelemetreCentre < V_BASSE) {
        Vitesse = 9;
    } else if (robotState.distanceTelemetreCentre < V_HAUTE) {
        Vitesse = 15 + (robotState.distanceTelemetreCentre - 30) * (10 / 20);
    } else {
        Vitesse = 24;
    }
}

void VitesseExtremeGauche() {
    float V_BASSE = 20;
    float V_HAUTE = 35;

    if (robotState.distanceTelemetreExGauche < V_BASSE) {
        Vitesse = 9;
    } else if (robotState.distanceTelemetreExGauche < V_HAUTE) {
        Vitesse = 9 + (robotState.distanceTelemetreExGauche - 18) * (10 / 15);
    } else {
        Vitesse = 24;
    }
}

void VitesseDroit() {
    float V_BASSE = 20;
    float V_HAUTE = 50;

    if (robotState.distanceTelemetreDroit < V_BASSE) {
        Vitesse = 8;
    } else if (robotState.distanceTelemetreDroit < V_HAUTE) {
        Vitesse = 9 + (robotState.distanceTelemetreDroit - 20) * (15 / 30);
    } else {
        Vitesse = 24;
    }
}

void VitesseExtremeDroit() {
    float V_BASSE = 20;
    float V_HAUTE = 35;

    if (robotState.distanceTelemetreExDroite < V_BASSE) {
        Vitesse = 9;
    } else if (robotState.distanceTelemetreExDroite < V_HAUTE) {
        Vitesse = 9 + (robotState.distanceTelemetreExDroite - 20) * (10 / 15);
    } else {
        Vitesse = 24;
    }
}

void VitesseGauche() {
    float V_BASSE = 20;
    float V_HAUTE = 50;

    if (robotState.distanceTelemetreGauche < V_BASSE) {
        Vitesse = 8;
    } else if (robotState.distanceTelemetreGauche < V_HAUTE) {
        Vitesse = 9 + (robotState.distanceTelemetreGauche - 20) * (15 / 30);
    } else {
        Vitesse = 24;
    }
}*/