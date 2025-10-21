#include "trajectory.h"
#include "robot.h"
#include "Utilities.h"
#include "UART_Protocol.h"
#include "math.h"
#include "timer.h"
#include "QEI.h"


extern unsigned long timestamp;

volatile GhostPosition ghostPosition;

double maxAngularSpeed = 2; 
double angularAccel = 5; 
double maxLinearSpeed = 1; 
double minMaxLinenearSpeed = 0.4;
double linearAccel = 0.5;

int current_state = IDLE;
int index = 0;
#define MAX_POS 8

struct Waypoint {
    double x;
    double y;
    int last_rotate;
};
typedef struct Waypoint Waypoint_t;

Waypoint_t waypoints[MAX_POS] = {
    {0.3, 0.2, 0},     // Départ bas gauche
    {0.3, 1.5, 0},     // Monter en ordonnée au milieu
    {1.0, 1.5, 0},     // Aller au centre (X milieu)
    {0.0, 0.0, 0},     // Coin bas gauche du carré autour du centre
    {0.0, 3.0, 0},     // Coin haut gauche
    {2.0, 3.0, 0},     // Coin haut droit
    {2.0, 0.0, 0},     // Coin bas droit
    {1.0, 1.5, 1}      // Retour au centre + dernière rotation
};

void InitTrajectoryGenerator(void) {
    ghostPosition.x = 0.3;
    ghostPosition.y = 0.2;
    ghostPosition.theta = -PI;
    ghostPosition.linearSpeed = 0.0;
    ghostPosition.angularSpeed = 0.0;
    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;
    ghostPosition.angleToTarget = 0.0;
    ghostPosition.distanceToTarget = 0.0;
}

void UpdateTrajectory()
{
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;
    ghostPosition.angleToTarget = thetaRestant;
    double thetaArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2 * angularAccel);
    double incrementAng = ghostPosition.angularSpeed / FREQ_ECH_QEI;
    double incremntLin = ghostPosition.linearSpeed / FREQ_ECH_QEI;

    double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2 * linearAccel);

    double distanceRestante = sqrt((ghostPosition.targetX - ghostPosition.x) * (ghostPosition.targetX - ghostPosition.x)
            + (ghostPosition.targetY - ghostPosition.y) * (ghostPosition.targetY - ghostPosition.y));
    ghostPosition.distanceToTarget = distanceRestante;

    if (current_state == IDLE) {
        if(index < MAX_POS) {
            Waypoint_t nextWay = waypoints[index++];
            ghostPosition.targetX = nextWay.x;
            ghostPosition.targetY = nextWay.y;
            current_state = (nextWay.last_rotate ? LASTROTATE : ROTATING);
        }
        
    } else if (current_state == ROTATING || current_state == LASTROTATE) {

        if (ghostPosition.angularSpeed < 0) thetaArret = -thetaArret;

        if (((thetaArret >= 0 && thetaRestant >= 0) || (thetaArret <= 0 && thetaRestant <= 0)) && (Abs(thetaRestant) >= Abs(thetaArret))) {
            if (thetaRestant > 0) {
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, maxAngularSpeed);
            } else if (thetaRestant < 0) {
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, -maxAngularSpeed);
            }


        } else {
            if (thetaRestant >= 0 && ghostPosition.angularSpeed > 0) {
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant >= 0 && ghostPosition.angularSpeed < 0) {
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant <= 0 && ghostPosition.angularSpeed > 0) {
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant <= 0 && ghostPosition.angularSpeed < 0) {
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);
            }

            if (Abs(thetaRestant) < Abs(incrementAng)) {
                incrementAng = thetaRestant;
            }
        }

        ghostPosition.theta += incrementAng;
        robotState.consigneVitesseAngulaire = ghostPosition.angularSpeed;

        if (ghostPosition.angularSpeed == 0 && (Abs(thetaRestant) < 0.01)) {
            ghostPosition.theta = thetaTarget;
            if(current_state != LASTROTATE) 
                current_state = ADVANCING;
            else
                current_state = IDLE;
        }

        
    } else if (current_state == ADVANCING) {

        if ((distanceRestante != 0) && (Modulo2PIAngleRadian(thetaRestant) < 0.01)) {
            if (((distanceArret >= 0 && distanceRestante >= 0) || (distanceArret <= 0 && distanceRestante <= 0)) && Abs(distanceRestante) >= Abs(distanceArret)) {
                if (distanceRestante > 0) {
                    ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, maxLinearSpeed);
                } else if (distanceRestante < 0) {
                    ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, -maxLinearSpeed);
                }
            } else {

                if (distanceRestante >= 0 && ghostPosition.linearSpeed > 0) {
                    ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante >= 0 && ghostPosition.linearSpeed < 0) {
                    ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante <= 0 && ghostPosition.linearSpeed > 0) {
                    ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante <= 0 && ghostPosition.linearSpeed < 0) {
                    ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, 0);
                }

                if (Abs(distanceRestante) < Abs(incremntLin)) {
                    incremntLin = distanceRestante;
                }
            }


        }

        if ((Abs(distanceRestante) < 0.0001)) {
            ghostPosition.linearSpeed = 0;
            ghostPosition.x = ghostPosition.targetX;
            ghostPosition.y = ghostPosition.targetY;
            current_state = IDLE;
        }
        
        ghostPosition.x += incremntLin * cos(ghostPosition.theta);
        ghostPosition.y += incremntLin * sin(ghostPosition.theta);
        robotState.consigneVitesseLineaire = ghostPosition.linearSpeed;
    }
    SendGhostData();
}

void SendGhostData() {
    unsigned char ghostPayload[32];
    getBytesFromInt32(ghostPayload, 0, timestamp);
    getBytesFromFloat(ghostPayload, 4, (float) ghostPosition.angleToTarget);
    getBytesFromFloat(ghostPayload, 8, (float) ghostPosition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 12, (float) ghostPosition.theta);
    getBytesFromFloat(ghostPayload, 16, (float) ghostPosition.angularSpeed);
    getBytesFromFloat(ghostPayload, 20, (float) ghostPosition.x);
    getBytesFromFloat(ghostPayload, 24, (float) ghostPosition.y);
    getBytesFromFloat(ghostPayload, 28, (float) ghostPosition.linearSpeed);
    UartEncodeAndSendMessage(GHOST_DATA, 32, ghostPayload);
}