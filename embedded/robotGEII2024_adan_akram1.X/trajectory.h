#ifndef TRAJECTORY_H
#define	TRAJECTORY_H

#define GHOST_DATA 0x0010

#define MAX_LINEAR_SPEED 1 
#define MAX_LINEAR_ACCEL 0.2 

#define MAX_ANGULAR_SPEED (2.0 * M_PI)
#define MAX_ANGULAR_ACCEL (2.0 * M_PI)

#define ANGLE_TOLERANCE 0.05 
#define DISTANCE_TOLERANCE 0.1


// Etat de controle de la trajectoire
typedef enum {
    IDLE,
    ROTATING,
    ADVANCING,
    LASTROTATE
} TrajectoryState;

// Position et vitesse du Ghost
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
    double red_target_x ;
    double red_target_y ;
} GhostPosition;


extern volatile GhostPosition ghostPosition;

void UpdateTrajectory();
void SendGhostData();
void InitTrajectoryGenerator(void);
//void rotationTarget(double currentTime);

#endif	/* TRAJECTORY_H */

