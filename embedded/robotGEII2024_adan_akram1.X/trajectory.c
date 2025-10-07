#include "trajectory.h"
#include "robot.h"
#include "Utilities.h"
#include "UART_Protocol.h"
#include <math.h>
#include "timer.h"
#include "QEI.h"

extern unsigned long timestamp;

volatile GhostPosition ghostPosition;

double maxAngularSpeed = 2.0;
double angularAccel = 5.0;
double maxLinearSpeed = 1.0;
double minMaxLinenearSpeed = 0.4;
double linearAccel = 0.5;


int index = 0;

#define MAX_POS 7

typedef struct {
    double x;
    double y;
    int last_rotate;
} Waypoint_t;

void SetGhostTarget(double Click_x, double Click_y) {
    ghostPosition.targetX = Click_x;
    ghostPosition.targetY = Click_y;

    ghostPosition.state = ROTATING;  
}

Waypoint_t waypoints[MAX_POS] = {
    {0, 0, 0},
    {0, 0.5, 0},
    {-1, 0.5, 0},
    {-1, -0.5, 0},
    {0, -0.5, 0},
    {0, 0, 0},
    {1.3, 0, 1}
};

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
}
/*
void UpdateTrajectory(void) {
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;
    ghostPosition.angleToTarget = thetaRestant;

    double thetaArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2 * angularAccel);
    double incrementAng = ghostPosition.angularSpeed / FREQ_ECH_QEI;
    double incremntLin = ghostPosition.linearSpeed / FREQ_ECH_QEI;
    double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2 * linearAccel);

    double distanceRestante = sqrt(pow(ghostPosition.targetX - ghostPosition.x, 2) + pow(ghostPosition.targetY - ghostPosition.y, 2));
    ghostPosition.distanceToTarget = distanceRestante;

    if (current_state == IDLE) {
        if (index < MAX_POS) {
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
            if (current_state != LASTROTATE) {
                current_state = ADVANCING;
            } else {
                current_state = IDLE;
            }
        }

    } else if (current_state == ADVANCING) {
        if (distanceRestante != 0 && Modulo2PIAngleRadian(thetaRestant) < 0.01) {
            if (((distanceArret >= 0 && distanceRestante >= 0) || (distanceArret <= 0 && distanceRestante <= 0)) &&
                    Abs(distanceRestante) >= Abs(distanceArret)) {
                if (distanceRestante > 0) {
                    ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, maxLinearSpeed);
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
        }

        if (Abs(distanceRestante) < 0.0001) {
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
 */
TrajectoryState current_state = IDLE;

void HandleIdleState(void);
void HandleRotatingState(void);
void HandleAdvancingState(void);

void UpdateTrajectory(void) {
    // Calculs communs
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;
    ghostPosition.angleToTarget = thetaRestant;

    double thetaArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2 * angularAccel);
    if (ghostPosition.angularSpeed < 0) thetaArret = -thetaArret;

    double incrementAng = ghostPosition.angularSpeed / FREQ_ECH_QEI;
    double incremntLin = ghostPosition.linearSpeed / FREQ_ECH_QEI;
    double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2 * linearAccel);

    double distanceRestante = sqrt(pow(ghostPosition.targetX - ghostPosition.x, 2) + pow(ghostPosition.targetY - ghostPosition.y, 2));
    ghostPosition.distanceToTarget = distanceRestante;

    // Exécution de l'état courant
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

    SendGhostData(); // Données envoyées à la fin, peu importe l'état
}

void HandleIdleState(void) {
    if (index < MAX_POS) {
        Waypoint_t nextWay = waypoints[index++];
        ghostPosition.targetX = nextWay.x;
        ghostPosition.targetY = nextWay.y;

        if (nextWay.last_rotate) {
            current_state = LASTROTATE;
        } else {
            current_state = ROTATING;
        }
    }
}

void HandleRotatingState(void) {
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;
    double thetaArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2 * angularAccel);
    if (ghostPosition.angularSpeed < 0) thetaArret = -thetaArret;
    double incrementAng = ghostPosition.angularSpeed / FREQ_ECH_QEI;

    if (((thetaArret >= 0 && thetaRestant >= 0) || (thetaArret <= 0 && thetaRestant <= 0)) && (Abs(thetaRestant) >= Abs(thetaArret))) {
        if (thetaRestant > 0) {
            ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, maxAngularSpeed);
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

        if (current_state != LASTROTATE) {
            current_state = ADVANCING;
        } else {
            current_state = IDLE;
        }
    }

}

void HandleAdvancingState(void) {
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;
    double distanceRestante = sqrt(pow(ghostPosition.targetX - ghostPosition.x, 2) + pow(ghostPosition.targetY - ghostPosition.y, 2));
    double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2 * linearAccel);
    double incremntLin = ghostPosition.linearSpeed / FREQ_ECH_QEI;

    if (distanceRestante != 0 && Modulo2PIAngleRadian(thetaRestant) < 0.01) {
        if (((distanceArret >= 0 && distanceRestante >= 0) || (distanceArret <= 0 && distanceRestante <= 0)) &&
                Abs(distanceRestante) >= Abs(distanceArret)) {
            if (distanceRestante > 0) {
                ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, maxLinearSpeed);
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
    }

    if (Abs(distanceRestante) < 0.0001) {
        ghostPosition.linearSpeed = 0;
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


void SendGhostData(void) {
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
