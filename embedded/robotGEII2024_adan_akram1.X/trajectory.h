#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>

#define GHOST_DATA             0x0010
#define MAX_LINEAR_SPEED       1.0
#define MAX_LINEAR_ACCEL       0.2
#define MAX_ANGULAR_SPEED      (2.0 * PI)
#define MAX_ANGULAR_ACCEL      (2.0 * PI)
#define ANGLE_TOLERANCE        0.05
#define DISTANCE_TOLERANCE     0.1

typedef enum {
    IDLE,
    ROTATING,
    ADVANCING,
    LASTROTATE
} TrajectoryState;

typedef struct {
    TrajectoryState state;
    double x;
    double y;
    double theta;
    double linearSpeed;
    double angularSpeed;
    double targetX;
    double targetY;
    double angleToTarget;
    double distanceToTarget;
    double red_target_x;
    double red_target_y;
} GhostPosition;

extern volatile GhostPosition ghostPosition;

void InitTrajectoryGenerator(void);
void UpdateTrajectory(void);
void SendGhostData(void);
//void rotationTarget(double currentTime);

void HandleIdleState(void);
void HandleRotatingState(void);
void HandleAdvancingState(void);
double DistancePointToSegment(double px, double py, double ax, double ay, double bx, double by);
void SetGhostTarget(double Click_x, Click_y);

#endif // TRAJECTORY_H
