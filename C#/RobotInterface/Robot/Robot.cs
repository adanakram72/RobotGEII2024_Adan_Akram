using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Robot_NS
{
    public class Robot
    {
        public Queue<byte> byteListReceived = new Queue<byte>();
        public string receivedText = "";
        public float distanceTelemetreDroit;
        public float distanceTelemetreCentre;
        public float distanceTelemetreGauche;
        public float positionXOdo, positionYOdo, vitesseLinFOdo, vitesseAngFOdo, angleRadFOdo, timeFrom, positionMD, positionMG;


        public float commandeX;
        public float KpX;
        public float KiX;
        public float KdX;
        public float corrLimitPX;
        public float corrLimitIX;
        public float corrLimitDX;

        public float KpTheta;
        public float KiTheta;
        public float KdTheta;
        public float corrLimitPTheta;
        public float corrLimitITheta;
        public float corrLimitDTheta;

        public float corrPX;
        public float erreurPX;
        public float corrIX;
        public float erreurIX;
        public float corrDX;
        public float erreurDX;

        public float commandeTheta;
        public float corrPTheta;
        public float erreurPTheta;
        public float corrITheta;
        public float erreurITheta;
        public float corrDTheta;
        public float erreurDTheta;

        public float erreurX;
        public float consigneX;
        public float vitesseX;
        public float erreurTheta;
        public float consigneTheta;
        public float vitesseTheta;

        public Robot()
        {

        }

    }
}