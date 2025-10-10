#include "trajectory.h"
#include "robot.h"
#include "Utilities.h"
#include "UART_Protocol.h"
#include <math.h>
#include "timer.h"
#include "QEI.h"
#include "trajectory.h"

extern unsigned long timestamp;

volatile GhostPosition ghostPosition;

double maxAngularSpeed = 2.0;
double angularAccel     = 5.0;
double maxLinearSpeed   = 1.0;
double minMaxLinenearSpeed = 0.4;   
double linearAccel      = 0.5;

TrajectoryState current_state = IDLE;

void HandleIdleState(void);
void HandleRotatingState(void);
void HandleAdvancingState(void);
void SendGhostData(void);


void InitTrajectoryGenerator(void) {
    ghostPosition.x = 1.33;
    ghostPosition.y = 0.0;
    ghostPosition.theta = -PI;

    ghostPosition.linearSpeed = 0.0;
    ghostPosition.angularSpeed = 0.0;

    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;

    ghostPosition.angleToTarget = 0.0;
    ghostPosition.distanceToTarget = 0.0;

    current_state = IDLE;
}

void UpdateTrajectory(void) {
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y,
                               ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;
    ghostPosition.angleToTarget = thetaRestant;

    double thetaArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2.0 * angularAccel);
    if (ghostPosition.angularSpeed < 0) thetaArret = -thetaArret;

    double incrementAng = ghostPosition.angularSpeed / FREQ_ECH_QEI;
    double incremntLin  = ghostPosition.linearSpeed  / FREQ_ECH_QEI;
    double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2.0 * linearAccel);

    double dx = ghostPosition.targetX - ghostPosition.x;
    double dy = ghostPosition.targetY - ghostPosition.y;
    double distanceRestante = sqrt(dx*dx + dy*dy);
    ghostPosition.distanceToTarget = distanceRestante;

    // Machine d?états
    switch (current_state) {
        case IDLE:
            HandleIdleState();
            break;

        case ROTATING:
        case LASTROTATE:     
            HandleRotatingState();
            break;

        case ADVANCING:
            HandleAdvancingState();
            break;
    }

    SendGhostData(); 
}

void HandleIdleState(void) {
    robotState.consigneVitesseLineaire  = 0.0;
    robotState.consigneVitesseAngulaire = 0.0;
}

void HandleRotatingState(void) {
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y,
                               ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;

    double thetaArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2.0 * angularAccel);
    if (ghostPosition.angularSpeed < 0) thetaArret = -thetaArret;

    double incrementAng = ghostPosition.angularSpeed / FREQ_ECH_QEI;

    if (((thetaArret >= 0 && thetaRestant >= 0) || (thetaArret <= 0 && thetaRestant <= 0)) &&
        (Abs(thetaRestant) >= Abs(thetaArret))) {
        if (thetaRestant > 0) {
            ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI,  maxAngularSpeed);
        } else {
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

    if (ghostPosition.angularSpeed == 0 && Abs(thetaRestant) < 0.01) {
        ghostPosition.theta = thetaTarget;
        current_state = ADVANCING;
    }
}

void HandleAdvancingState(void) {
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y,
                               ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;

    double dx = ghostPosition.targetX - ghostPosition.x;
    double dy = ghostPosition.targetY - ghostPosition.y;
    double distanceRestante = sqrt(dx*dx + dy*dy);

    double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2.0 * linearAccel);
    double incremntLin   = ghostPosition.linearSpeed / FREQ_ECH_QEI;

    if (distanceRestante != 0 && Modulo2PIAngleRadian(thetaRestant) < 0.01) {
        if (((distanceArret >= 0 && distanceRestante >= 0) || (distanceArret <= 0 && distanceRestante <= 0)) &&
            Abs(distanceRestante) >= Abs(distanceArret)) {
            if (distanceRestante > 0) {
                ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI,  maxLinearSpeed);
            } else {
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
    } else {
        if (Modulo2PIAngleRadian(thetaRestant) >= 0.01) {
            if (ghostPosition.linearSpeed > 0) {
                ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
            } else if (ghostPosition.linearSpeed < 0) {
                ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, 0);
            }
            current_state = ROTATING;
        }
    }

    if (Abs(distanceRestante) < 0.0001) {
        ghostPosition.linearSpeed = 0.0;
        ghostPosition.x = ghostPosition.targetX;
        ghostPosition.y = ghostPosition.targetY;
        current_state = IDLE;
    }

    ghostPosition.x += incremntLin * cos(ghostPosition.theta);
    ghostPosition.y += incremntLin * sin(ghostPosition.theta);
    robotState.consigneVitesseLineaire = ghostPosition.linearSpeed;
}

double DistancePointToSegment(double px, double py, double ax, double ay, double bx, double by) {
    double dx = bx - ax;
    double dy = by - ay;

    if (dx == 0 && dy == 0) {
        dx = px - ax;
        dy = py - ay;
        return sqrt(dx*dx + dy*dy);
    }

    double t = ((px - ax) * dx + (py - ay) * dy) / (dx*dx + dy*dy);

    if (t < 0) {
        dx = px - ax;
        dy = py - ay;
    } else if (t > 1) {
        dx = px - bx;
        dy = py - by;
    } else {
        double projx = ax + t * dx;
        double projy = ay + t * dy;
        dx = px - projx;
        dy = py - projy;
    }
    return sqrt(dx*dx + dy*dy);
}

void SetGhostTarget(double Click_x, double Click_y) {
    ghostPosition.targetX = Click_x;
    ghostPosition.targetY = Click_y;
    current_state = ROTATING;  

    ghostPosition.angularSpeed = 0.0;
    ghostPosition.linearSpeed  = 0.0;
}

void SendGhostData(void) {
    unsigned char ghostPayload[32];
    getBytesFromInt32(ghostPayload, 0, timestamp);
    getBytesFromFloat(ghostPayload, 4,  (double) ghostPosition.angleToTarget);
    getBytesFromFloat(ghostPayload, 8,  (double) ghostPosition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 12, (double) ghostPosition.theta);
    getBytesFromFloat(ghostPayload, 16, (double) ghostPosition.angularSpeed);
    getBytesFromFloat(ghostPayload, 20, (double) ghostPosition.x);
    getBytesFromFloat(ghostPayload, 24, (double) ghostPosition.y);
    getBytesFromFloat(ghostPayload, 28, (double) ghostPosition.linearSpeed);
    UartEncodeAndSendMessage(GHOST_DATA, 32, ghostPayload);
}
