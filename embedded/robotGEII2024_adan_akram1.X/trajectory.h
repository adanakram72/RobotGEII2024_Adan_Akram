#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>
#define GHOST_DATA 0x0010

typedef enum {
    IDLE = 0,
    ROTATING = 1,
    LINEAR = 2
} TrajectoryState;

typedef struct {
    double x;            
    double y;            
    double theta;        

    double linearSpeed;   
    double angularSpeed;  

    double targetX;       
    double targetY;       

    double angleToTarget;     
    double distanceToTarget;  
    
    TrajectoryState state;
} GhostPosition;

extern volatile GhostPosition ghostPosition;

void InitTrajectoryGenerator(void);
void UpdateTrajectory(void);
void SetGhostTarget(double Click_x, double Click_y);
void SendGhostData(void);

#endif // TRAJECTORY_H
