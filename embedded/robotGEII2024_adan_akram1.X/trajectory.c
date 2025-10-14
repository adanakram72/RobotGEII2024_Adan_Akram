#include "robot.h"
#include "Utilities.h"
#include "UART_Protocol.h"
#include <math.h>
#include "timer.h"
#include "QEI.h"
#include "trajectory.h"

#define Ts      (1.0 / FREQ_ECH_QEI)

static const double VTHETA_MAX = 2.0;   
static const double ATHETA     = 6.0;   

static const double VLIN_MAX   = 1.0;   
static const double ALIN       = 3.0; 

static const double ANG_EPS    = 1e-2;  
static const double POS_EPS    = 1e-4;  

static inline double AngleDiffShortest(double target, double current) {
    double d = target - current;
    return atan2(sin(d), cos(d));
}

volatile GhostPosition ghostPosition;

void InitTrajectoryGenerator(void)
{
    ghostPosition.x = 0.0;
    ghostPosition.y = 0.0;
    ghostPosition.theta = 0.0;

    ghostPosition.linearSpeed  = 0.0;
    ghostPosition.angularSpeed = 0.0;

    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;

    ghostPosition.angleToTarget    = 0.0;
    ghostPosition.distanceToTarget = 0.0;

    ghostPosition.state = IDLE;
}

void SetGhostTarget(double Click_x, double Click_y) {
    ghostPosition.targetX = Click_x;
    ghostPosition.targetY = Click_y;

    ghostPosition.angularSpeed = 0.0;
    ghostPosition.linearSpeed  = 0.0;

    ghostPosition.state = ADVANCING;
}

static void StepRotating(void)
{
    double dx = ghostPosition.targetX - ghostPosition.x;
    double dy = ghostPosition.targetY - ghostPosition.y;
   double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y,
                           ghostPosition.targetX - ghostPosition.x);


   double thetaRest = ModuloByAngle(thetaTarget - ghostPosition.theta);

    //double thetaRest = AngleDiffShortest(thetaTarget, ghostPosition.theta);
    ghostPosition.angleToTarget = thetaRest;

    double thetaStop = (ghostPosition.angularSpeed * ghostPosition.angularSpeed) / (2.0 * ATHETA);
    if (ghostPosition.angularSpeed < 0.0) thetaStop = -thetaStop;

    int sameSign = ((thetaStop >= 0.0 && thetaRest >= 0.0) || (thetaStop <= 0.0 && thetaRest <= 0.0));
    if (sameSign && (fabs(thetaRest) >= fabs(thetaStop))) {
        if (thetaRest > 0.0)
            ghostPosition.angularSpeed = fmin(ghostPosition.angularSpeed + ATHETA*Ts,  VTHETA_MAX);
        else if (thetaRest < 0.0)
            ghostPosition.angularSpeed = fmax(ghostPosition.angularSpeed - ATHETA*Ts, -VTHETA_MAX);
    } else {
        if (ghostPosition.angularSpeed > 0.0)
            ghostPosition.angularSpeed = fmax(ghostPosition.angularSpeed - ATHETA*Ts, 0.0);
        else if (ghostPosition.angularSpeed < 0.0)
            ghostPosition.angularSpeed = fmin(ghostPosition.angularSpeed + ATHETA*Ts, 0.0);
    }

    double dtheta = ghostPosition.angularSpeed * Ts;
    if (fabs(dtheta) > fabs(thetaRest)) dtheta = thetaRest;
    ghostPosition.theta += dtheta;

    if (fabs(thetaRest) < ANG_EPS && fabs(ghostPosition.angularSpeed) < 0.01) {
        ghostPosition.theta = thetaTarget;
        ghostPosition.angularSpeed = 0.0;
        ghostPosition.state = LINEAR;
    }
}

static void StepLinear(void)
{
    double dx = ghostPosition.targetX - ghostPosition.x;
    double dy = ghostPosition.targetY - ghostPosition.y;
    double dist = sqrt(dx*dx + dy*dy);
    ghostPosition.distanceToTarget = dist;

    double thetaTarget = atan2(dy, dx);
    double thetaErr = AngleDiffShortest(thetaTarget, ghostPosition.theta);
    
    if (ghostPosition.state == LINEAR) {
        ghostPosition.theta = thetaTarget; 
    }
    
    if (fabs(thetaErr) > ANG_EPS) {
        if (ghostPosition.linearSpeed > 0.0)
            ghostPosition.linearSpeed = fmax(ghostPosition.linearSpeed - ALIN*Ts, 0.0);
        ghostPosition.state = ROTATING;
        return;
    }

    double dStop = (ghostPosition.linearSpeed * ghostPosition.linearSpeed) / (2.0 * ALIN);

    if (dist >= dStop) {
        ghostPosition.linearSpeed = fmin(ghostPosition.linearSpeed + ALIN*Ts, VLIN_MAX);
    } else {
        ghostPosition.linearSpeed = fmax(ghostPosition.linearSpeed - ALIN*Ts, 0.0);
    }

    double step = ghostPosition.linearSpeed * Ts;
    double stepX = step * cos(ghostPosition.theta);
    double stepY = step * sin(ghostPosition.theta);
    if (hypot(stepX, stepY) > dist) { stepX = dx; stepY = dy; }

    ghostPosition.x += stepX;
    ghostPosition.y += stepY;

    if (dist < POS_EPS) {
        ghostPosition.linearSpeed = 0.0;
        ghostPosition.x = ghostPosition.targetX;
        ghostPosition.y = ghostPosition.targetY;
        ghostPosition.state = IDLE;
    }
}

void UpdateTrajectory(void)
{
    switch (ghostPosition.state) {
        case IDLE:
            ghostPosition.angularSpeed = 0.0;
            ghostPosition.linearSpeed  = 0.0;
            ghostPosition.angleToTarget    = 0.0;
            ghostPosition.distanceToTarget = hypot(ghostPosition.targetX - ghostPosition.x, ghostPosition.targetY - ghostPosition.y);
            break;

        case ROTATING:
            StepRotating();
            break;

        case LINEAR:
            StepLinear();
            break;
    }

    SendGhostData();
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
