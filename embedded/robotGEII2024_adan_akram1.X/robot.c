#include "robot.h"
#include "main.h"
#include "asservissement.h"

volatile ROBOT_STATE_BITS robotState;
PidCorrector PidX;
PidCorrector PidTheta;