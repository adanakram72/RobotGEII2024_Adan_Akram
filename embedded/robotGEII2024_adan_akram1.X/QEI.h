/* 
 * File:   QEI.h
 * Author: E306_PC2
 *
 * Created on 6 janvier 2025, 13:50
 */

#ifndef QEI_H
#define	QEI_H

#define COEF_VITESSE 30
#define DISTROUES 0.217
#define FREQ_ECH_QEI 250

void InitQEI1();
void InitQEI2();
void QEIUpdateData();
void SendPositionData();

#endif	/* QEI_H */

