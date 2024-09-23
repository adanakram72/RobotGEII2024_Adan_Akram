#include <xc.h>
#include "IO.h"
#include "main.h"

void InitIO() {
    //****************************************************************************************************/
    // Declaration des pin Analogiques
    //****************************************************************************************************/
    ANSELA = 0; //Desactivation de toutes entree analogique
    ANSELB = 0; //Desactivation de toutes entree analogique
    ANSELD = 0; //Desactivation de toutes entree analogique
    ANSELC = 0; //Desactivation de toutes entree analogique
    ANSELE = 0; //Desactivation de toutes entree analogique
    ANSELG = 0; //Desactivation de toutes entree analogique

    // Configuration des sorties

    //******* LED ***************************
    _TRISJ4 = 0; // LED Orange1
    _TRISJ6 = 0; //LED Blanche1
    _TRISJ5 = 0; // LED Bleue1
    _TRISJ11 = 0; // LED Rouge1
    _TRISH10 = 0; // LED Verte1
    
    _TRISK15 = 0; // LED Orange2
    _TRISA0 = 0; //LED Blanche2
    _TRISA9 = 0; // LED Bleue2
    _TRISA10 = 0; // LED Rouge2
    _TRISH3 = 0; // LED Verte2
    

    //****** Moteurs ************************

    // Configuration des entr�es


    /****************************************************************************************************/
    // Gestion des pin remappables
    /****************************************************************************************************/
    UnlockIO(); // On unlock les registres d'entr�es/sorties, ainsi que les registres des PPS

    //Assignation des remappable pins

    LockIO(); // On lock les registres d'entr�es/sorties, ainsi que les registres des PPS
}

void LockIO() {
    asm volatile ("mov #OSCCON,w1 \n"
            "mov #0x46, w2 \n"
            "mov #0x57, w3 \n"
            "mov.b w2,[w1] \n"
            "mov.b w3,[w1] \n"
            "bset OSCCON, #6":: : "w1", "w2", "w3");
}

void UnlockIO() {
    asm volatile ("mov #OSCCON,w1 \n"
            "mov #0x46, w2 \n"
            "mov #0x57, w3 \n"
            "mov.b w2,[w1] \n"
            "mov.b w3,[w1] \n"
            "bclr OSCCON, #6":: : "w1", "w2", "w3");
}
