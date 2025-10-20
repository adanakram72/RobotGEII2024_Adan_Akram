#include <math.h>
#include "trajectory.h"
#include "timer.h"
#include "Robot.h"
#include "utilities.h"
#include "UART_Protocol.h"
#include "QEI.h"

extern unsigned long timestamp;


double maxAngularSpeed = 1.5;
double angularAccel = 2.5;
double maxLinearSpeed = 1;
double minMaxLinenearSpeed = 0.5;
double linearAccel = 1;

int current_state = IDLE;
#define MAX_POS 7

struct Waypoint {
    double x;
    double y;
    int last_rotate;
};
typedef struct Waypoint Waypoint_t;

Waypoint_t waypoints[MAX_POS] = {{0, 0, 0}, {0, 0.1, 0}, {-0.1, 0.1, 0}, {-0.1, -0.1, 0}, {0, -0.1, 0.0}, {0.0, 0, 0}, {0.05, 0, 0}};

void InitTrajectoryGenerator(void) {
    ghostposition.x = 0;
    ghostposition.y = 0;
    ghostposition.theta = 0;
    ghostposition.linearSpeed = 0.0f;
    ghostposition.angularSpeed = -PI;
    ghostposition.targetX = 0.0f;
    ghostposition.targetY = 0.0f;
    ghostposition.angleToTarget = 0.0f;
    ghostposition.distanceToTarget = 0.0f;
}

void UpdateTrajectory()
{
    double thetaTarget = atan2(ghostposition.targetY - ghostposition.y, ghostposition.targetX - ghostposition.x);
    double thetaRestant = ModuloByAngle(ghostposition.theta, thetaTarget) - ghostposition.theta;
    ghostposition.angleToTarget = thetaRestant;
    double thetaArret = ghostposition.angularSpeed * ghostposition.angularSpeed / (2 * angularAccel);
    double incrementAng = ghostposition.angularSpeed / FREQ_ECH_QEI;
    double incremntLin = ghostposition.linearSpeed / FREQ_ECH_QEI;

    double distanceArret = ghostposition.linearSpeed * ghostposition.linearSpeed / (2 * linearAccel);

    double distanceRestante = sqrt((ghostposition.targetX - ghostposition.x) * (ghostposition.targetX - ghostposition.x)
            + (ghostposition.targetY - ghostposition.y) * (ghostposition.targetY - ghostposition.y));
    ghostposition.distanceToTarget = distanceRestante;
    int index = 0;

    if (current_state == IDLE) {
        if(index < MAX_POS) {
            Waypoint_t nextWay = waypoints[index++];
            ghostposition.targetX = nextWay.x;
            ghostposition.targetY = nextWay.y;
            if (nextWay.last_rotate) {
                current_state = LASTROTATE;
            } else {
                current_state = ROTATING;
            }
        }

    } else if (current_state == ROTATING || current_state == LASTROTATE) {

        if (ghostposition.angularSpeed < 0) thetaArret = -thetaArret;

        if (((thetaArret >= 0 && thetaRestant >= 0) || (thetaArret <= 0 && thetaRestant <= 0)) && (Abs(thetaRestant) >= Abs(thetaArret))) {
            if (thetaRestant > 0) {
                ghostposition.angularSpeed = Min(ghostposition.angularSpeed + angularAccel / FREQ_ECH_QEI, maxAngularSpeed);
            } else if (thetaRestant < 0) {
                ghostposition.angularSpeed = Max(ghostposition.angularSpeed - angularAccel / FREQ_ECH_QEI, -maxAngularSpeed);
            }
        } else {

            if (thetaRestant >= 0 && ghostposition.angularSpeed > 0) {
                ghostposition.angularSpeed = Max(ghostposition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant >= 0 && ghostposition.angularSpeed < 0) {
                ghostposition.angularSpeed = Min(ghostposition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant <= 0 && ghostposition.angularSpeed > 0) {
                ghostposition.angularSpeed = Max(ghostposition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant <= 0 && ghostposition.angularSpeed < 0) {
                ghostposition.angularSpeed = Min(ghostposition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);
            }

            if (Abs(thetaRestant) < Abs(incrementAng)) {
                incrementAng = thetaRestant;
            }
        }

        ghostposition.theta += incrementAng;
        robotState.consigneVitesseAngulaire = ghostposition.angularSpeed;

        if (ghostposition.angularSpeed == 0 && (Abs(thetaRestant) < 0.01)) {
            ghostposition.theta = thetaTarget;
            robotState.angleRadianFromOdometry = thetaTarget;
            robotState.PidTheta.epsilon_1 = 0;

            if(current_state != LASTROTATE) 
                current_state = ADVANCING;
            else
                current_state = IDLE;
        }

    } else if (current_state == ADVANCING) {

        if ((distanceRestante != 0) && (Modulo2PIAngleRadian(thetaRestant) < 0.01)) {
            if (((distanceArret >= 0 && distanceRestante >= 0) || (distanceArret <= 0 && distanceRestante <= 0)) && Abs(distanceRestante) >= Abs(distanceArret)) {
                if (distanceRestante > 0) {
                    ghostposition.linearSpeed = Min(ghostposition.linearSpeed + linearAccel / FREQ_ECH_QEI, maxLinearSpeed);
                } else if (distanceRestante < 0) {
                    ghostposition.linearSpeed = Max(ghostposition.linearSpeed - linearAccel / FREQ_ECH_QEI, -maxLinearSpeed);
                }
            } else {

                if (distanceRestante >= 0 && ghostposition.linearSpeed > 0) {
                    ghostposition.linearSpeed = Max(ghostposition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante >= 0 && ghostposition.linearSpeed < 0) {
                    ghostposition.linearSpeed = Min(ghostposition.linearSpeed + linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante <= 0 && ghostposition.linearSpeed > 0) {
                    ghostposition.linearSpeed = Max(ghostposition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante <= 0 && ghostposition.linearSpeed < 0) {
                    ghostposition.linearSpeed = Min(ghostposition.linearSpeed + linearAccel / FREQ_ECH_QEI, 0);
                }

                if (Abs(distanceRestante) < Abs(incremntLin)) {
                    incremntLin = distanceRestante;
                }
            }
        }

        if ((Abs(distanceRestante) < 0.0001)) {
            ghostposition.linearSpeed = 0;
            ghostposition.x = ghostposition.targetX;
            ghostposition.y = ghostposition.targetY;
            current_state = IDLE;
        }
        
        ghostposition.x += incremntLin * cos(ghostposition.theta);
        ghostposition.y += incremntLin * sin(ghostposition.theta);
        robotState.consigneVitesseLineaire = ghostposition.linearSpeed;
    }
    SendGhostData();
}

void SendGhostData() {
    unsigned char ghostPayload[32];
    getBytesFromInt32(ghostPayload, 0, timestamp);
    getBytesFromFloat(ghostPayload, 4, (float) ghostposition.angleToTarget);
    getBytesFromFloat(ghostPayload, 8, (float) ghostposition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 12, (float) ghostposition.theta);
    getBytesFromFloat(ghostPayload, 16, (float) ghostposition.angularSpeed);
    getBytesFromFloat(ghostPayload, 20, (float) ghostposition.x);
    getBytesFromFloat(ghostPayload, 24, (float) ghostposition.y);
    getBytesFromFloat(ghostPayload, 28, (float) ghostposition.linearSpeed);
    UartEncodeAndSendMessage(GHOST_DATA, 32, ghostPayload);
}
