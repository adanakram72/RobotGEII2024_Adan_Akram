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
        public Robot()
        {

        }

    }
}