#include "UART_Protocol.h"
#include "UART.h"
#include <math.h>
#include "Utilities.h"
#include "robot.h"
#include <xc.h>
#include "asservissement.h"


PidCorrector PidX;       // Vitesse liné
PidCorrector PidTheta;   // Vitesse angu


