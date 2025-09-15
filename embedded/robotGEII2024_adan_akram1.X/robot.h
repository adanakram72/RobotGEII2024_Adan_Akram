#include "asservissement.h"

#ifndef ROBOT_H
#define ROBOT_H

typedef struct robotStateBITS {

    union {

        struct {
            unsigned char taskEnCours;
            float vitesseGaucheConsigne;
            float vitesseGaucheCommandeCourante;
            float vitesseDroiteConsigne;
            float vitesseDroiteCommandeCourante;
            float distanceTelemetreCentre;
            float distanceTelemetreGauche;
            float distanceTelemetreDroit;
            float distanceTelemetreExDroite;
            float distanceTelemetreExGauche;
            
            float vitesseDroitFromOdometry;
            float vitesseGaucheFromOdometry;
            float vitesseLineaireFromOdometry;
            float vitesseAngulaireFromOdometry;
            float xPosFromOdometry_1;
            float yPosFromOdometry_1;
            float xPosFromOdometry;
            float yPosFromOdometry;
            float angleRadianFromOdometry_1;
            float angleRadianFromOdometry;
            float timeFrom;
           
            char correcteursXPayload[48];
            char correcteursThetaPayload[48];
            char consignes[8];
            
            
            float xCorrectionVitesse ;
            float thetaCorrectionVitesse ;
            float consigneVitesseLineaire ;
            float consigneVitesseAngulaire ;
            
            PidCorrector PidX ;
            PidCorrector PidTheta ;
        };
       
    };
} ROBOT_STATE_BITS;


extern volatile ROBOT_STATE_BITS robotState;
#endif /* ROBOT_H */

