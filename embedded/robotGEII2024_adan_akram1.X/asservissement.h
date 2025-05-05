/* 
 * File:   asservissement.h
 * Author: E306_PC2
 *
 * Created on 5 mai 2025, 16:21
 */

#ifndef ASSERVISSEMENT_H
#define	ASSERVISSEMENT_H

#ifdef	__cplusplus
extern "C" {
#endif

    typedef struct _PidCorrector {
        double Kp;
        double Ki;
        double Kd;
        double erreurProportionelleMax;
        double erreurIntegraleMax;
        double erreurDeriveeMax;
        double erreurIntegrale;
        double epsilon_1;
        double erreur;
        //For Debug only
        double corrP;
        double corrI;
        double corrD;
    } PidCorrector;

void SetupPidAsservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, double pro);
    
#ifdef	__cplusplus
}
#endif

#endif	/* ASSERVISSEMENT_H */

