
#ifndef TRAJECTORY_H
#define	TRAJECTORY_H


#define GHOST_DATA 0x0010

// Parametres de trajectoire
#define MAX_LINEAR_SPEED 1 // m/s
#define MAX_LINEAR_ACCEL 0.2 // m/s^2

#define MAX_ANGULAR_SPEED 2 * PI // rad/s
#define MAX_ANGULAR_ACCEL 2 * PI // rad/s^2

#define ANGLE_TOLERANCE 0.05 // radians
#define DISTANCE_TOLERANCE 0.1 // metres

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
} GhostPosition;


extern volatile GhostPosition ghostposition;

void UpdateTrajectory();
void SendGhostData();
void InitTrajectoryGenerator(void);
void rotationTarget(double currentTime);

#endif	

