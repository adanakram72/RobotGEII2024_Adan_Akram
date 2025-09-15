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

        public float correcteurKp;
        public float correcteurKd;
        public float correcteurKi;
        public float corrPmaxX;
        public float corrImaxX;
        public float corrDmaxX;

        public float correcteurThetaKp;
        public float correcteurThetaKd;
        public float correcteurThetaKi;
        public float corrPmaxTheta;
        public float corrImaxTheta;
        public float corrDmaxTheta;


        public Robot()
        {

        }

    }
}