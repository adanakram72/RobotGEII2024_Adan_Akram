using System.Globalization;
using System.IO.Ports;
using System.Net.NetworkInformation;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;

using ExtendedSerialPort_NS;
using Robot_NS;
using SciChart.Charting.Visuals;
using WpfOscilloscopeControl;

namespace WpfRobotInterface
{

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        Robot robot = new Robot();
        ExtendedSerialPort serialPort1;
        DispatcherTimer timerAffichage;
        public MainWindow()
        {
            // Set this code once in App.xaml.cs or application startup
            SciChartSurface.SetRuntimeLicenseKey("S2qB4UVLJdHvO3yV/v05tDmm7I3R9d7SjDf/G5oOCFVrydaWnUVVj/Pu6gT5lPw5Y7YdLT6DmsYJXuxfR641bajrGX8GBxpvfw893EURdOjPMU8CPyFUB+hfgMQwYCm9LgRd8m1MKwhKABfRbU0h7S8oNdvqSCHx3uV/20rwATE0k3RPv/lUnr+4098Cigp4ZXCc1WlKIVV14c8HelzCifEfHLLwv7u2eSBClOW7pI7kT7d8EfNdlDKXEa7zjGQq3ye8JMCm7g0YVSnNTDNwZIjx/FI3qB3iGPnAvTw870zisDjXNpA6aTDqtKEZqfKnDnqhs9g3frQyrV8c43N1+4Ce5sbsT4nn+zD7uZcra0F7hbkY6vNQ3HItBDRpZ7NGOCFQCVjsQ43Lj7A5GcsSqO+8bbTWbZABhfRt7K0BCtXmqG8V0yURDKn9405fPJF8vIGsxhqav1b+LtgUtvbek9eVsDAo61Sf8jCBRjDLd58xWKdU2LsC6SkDy2oOHCAFgrjJ8e1l");
            InitializeComponent();

            // Setting serialPort
            serialPort1 = new ExtendedSerialPort("COM4", 115200, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived;
            serialPort1.Open();

            // Setting timer
            timerAffichage = new DispatcherTimer();
            timerAffichage.Interval = new TimeSpan(0, 0, 0, 0, 50);
            timerAffichage.Tick += TimerAffichage_Tick;
            timerAffichage.Start();

            // Setting oscillo 

            oscilloSpeed.isDisplayActivated = true;
            oscilloSpeed.AddOrUpdateLine(1, 200, "Vitesse Lineaire");
            oscilloSpeed.ChangeLineColor(1, Colors.Blue);
            oscilloSpeed.AddOrUpdateLine(2, 200, "Vitesse Angulaire");
            oscilloSpeed.ChangeLineColor(2, Colors.Green);
        }

        private void TimerAffichage_Tick(object? sender, EventArgs e)
        {
            asservSpeedDisplay.UpdateIndependantOdometry(robot.positionMD, robot.positionMG);
            asservSpeedDisplay.UpdatePolarOdometry(robot.vitesseLinFOdo, robot.vitesseAngFOdo);
            //asservSpeedDisplay.UpdatePolarCommandValues(robot.correcteurKd, robot.correcteurKp);
            //worldMap.UpdatePosRobot(robot.positionXOdo * 100 + 50, robot.positionYOdo * 100 + 50);
            oscilloSpeed.AddPointToLine(1, robot.timeFrom, robot.vitesseAngFOdo);
            oscilloSpeed.AddPointToLine(2, robot.timeFrom, robot.vitesseLinFOdo);
            while (robot.byteListReceived.Count > 0)
            {
                byte messageR = robot.byteListReceived.Dequeue();
                DecodeMessage(messageR);
            }
        }

        public void SerialPort1_DataReceived(object sender, DataReceivedArgs e)
        {
            for (int i = 0; i < e.Data.Length; i++)
            {
                robot.byteListReceived.Enqueue(e.Data[i]);
            }

        }
        private void buttonSend_Click(object sender, RoutedEventArgs e)
        {

        }

        private void ButtonSend_Click(object sender, RoutedEventArgs e)
        {
            buttonSend.Background = Brushes.Beige;
            SendMessage();
        }

        private void TextBoxEmission_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                SendMessage();
            }
        }

        private void SendMessage()
        {
            serialPort1.WriteLine(textboxEmission.Text);
            textboxEmission.Text = "";
        }

        private void buttonClear_Click(object sender, RoutedEventArgs e)
        {
            textboxReception.Text = "";
        }

        private void ButtonTest_Click(object sender, RoutedEventArgs e)
        {
            //byte[] byteList = new byte[20];
            //for (int i = 0; i < byteList.Length; i++)
            //{
            //    byteList[i] = (byte)(2 * i);
            //}
            //byteList[byteList.Length - 1] = (byte)'\n';
            //serialPort1.Write(byteList, 0, byteList.Length);
            //string messageStr = "Bonjour";
            //byte[] msgPayload = Encoding.ASCII.GetBytes(messageStr);
            //int msgPayloadLength = msgPayload.Length;
            //int msgFunction = 0x0080;
            //UartEncodeAndSendMessage(msgFunction, msgPayloadLength, msgPayload);
            SendPIDValues();
        }

        private void SendPIDValues()
        {
            string[] pidXValues = TextBoxPidX.Text.Split(new[] { ' ', ':', ',', ';', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (pidXValues.Length == 6)
            {
                float KpX = float.Parse(pidXValues[0], CultureInfo.InvariantCulture);
                float KiX = float.Parse(pidXValues[1], CultureInfo.InvariantCulture);
                float KdX = float.Parse(pidXValues[2], CultureInfo.InvariantCulture);
                float limitPX = float.Parse(pidXValues[3], CultureInfo.InvariantCulture);
                float limitIX = float.Parse(pidXValues[4], CultureInfo.InvariantCulture);
                float limitDX = float.Parse(pidXValues[5], CultureInfo.InvariantCulture);

                byte[] pidXPayload = new byte[24];
                BitConverter.GetBytes(KpX).CopyTo(pidXPayload, 0);
                BitConverter.GetBytes(KdX).CopyTo(pidXPayload, 4);
                BitConverter.GetBytes(KiX).CopyTo(pidXPayload, 8);
                BitConverter.GetBytes(limitPX).CopyTo(pidXPayload, 12);
                BitConverter.GetBytes(limitIX).CopyTo(pidXPayload, 16);
                BitConverter.GetBytes(limitDX).CopyTo(pidXPayload, 20);

                UartEncodeAndSendMessage(0x0091, pidXPayload.Length, pidXPayload);
            }

            string[] pidThetaValues = TextBoxPidTheta.Text.Split(new[] { ' ', ':', ',', ';', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (pidThetaValues.Length == 6)
            {
                float KpTheta = float.Parse(pidThetaValues[0], CultureInfo.InvariantCulture);
                float KiTheta = float.Parse(pidThetaValues[1], CultureInfo.InvariantCulture);
                float KdTheta = float.Parse(pidThetaValues[2], CultureInfo.InvariantCulture);
                float limitPTheta = float.Parse(pidThetaValues[3], CultureInfo.InvariantCulture);
                float limitITheta = float.Parse(pidThetaValues[4], CultureInfo.InvariantCulture);
                float limitDTheta = float.Parse(pidThetaValues[5], CultureInfo.InvariantCulture);

                byte[] pidThetaPayload = new byte[24];
                BitConverter.GetBytes(KpTheta).CopyTo(pidThetaPayload, 0);
                BitConverter.GetBytes(KdTheta).CopyTo(pidThetaPayload, 4);
                BitConverter.GetBytes(KiTheta).CopyTo(pidThetaPayload, 8);
                BitConverter.GetBytes(limitPTheta).CopyTo(pidThetaPayload, 12);
                BitConverter.GetBytes(limitITheta).CopyTo(pidThetaPayload, 16);
                BitConverter.GetBytes(limitDTheta).CopyTo(pidThetaPayload, 20);

                UartEncodeAndSendMessage(0x0092, pidThetaPayload.Length, pidThetaPayload);
            }
        }

        void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte[] message = new byte[6 + msgPayloadLength];
            int pos = 0;
            message[pos++] = 0xFE;
            message[pos++] = (byte)(msgFunction >> 8);
            message[pos++] = (byte)(msgFunction);
            message[pos++] = (byte)(msgPayloadLength >> 8);
            message[pos++] = (byte)(msgPayloadLength);
            for (int i = 0; i < msgPayloadLength; i++)
            {
                message[pos++] = msgPayload[i];
            }
            byte c = CalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
            message[pos++] = c;
            serialPort1.Write(message, 0, pos);
        }
        public enum StateReception
        {
            Waiting,
            FunctionMSB,
            FunctionLSB,
            PayloadLengthMSB,
            PayloadLengthLSB,
            Payload,
            CheckSum
        }
        StateReception rcvState = StateReception.Waiting;
        int msgDecodedFunction = 0;
        int msgDecodedPayloadLength = 0;
        byte[] msgDecodedPayload;
        int msgDecodedPayloadIndex = 0;
        private void DecodeMessage(byte c)
        {
            switch (rcvState)
            {
                case StateReception.Waiting:
                    if (c == 0xFE)
                        rcvState = StateReception.FunctionMSB;
                    break;
                case StateReception.FunctionMSB:
                    msgDecodedFunction = c << 8;
                    rcvState = StateReception.FunctionLSB;
                    break;
                case StateReception.FunctionLSB:
                    msgDecodedFunction |= c;
                    rcvState = StateReception.PayloadLengthMSB;
                    break;
                case StateReception.PayloadLengthMSB:
                    msgDecodedPayloadLength = c << 8;
                    rcvState = StateReception.PayloadLengthLSB;
                    break;
                case StateReception.PayloadLengthLSB:
                    msgDecodedPayloadLength |= c;

                    if (msgDecodedPayloadLength > 1024)
                    {
                        rcvState = StateReception.Waiting;
                    }
                    else if (msgDecodedPayloadLength > 0)
                    {
                        msgDecodedPayload = new byte[msgDecodedPayloadLength];
                        msgDecodedPayloadIndex = 0;
                        rcvState = StateReception.Payload;
                    }
                    else
                    {
                        rcvState = StateReception.CheckSum;
                    }
                    break;
                case StateReception.Payload:
                    msgDecodedPayload[msgDecodedPayloadIndex++] = c;
                    if (msgDecodedPayloadIndex >= msgDecodedPayloadLength)
                    {
                        rcvState = StateReception.CheckSum;
                    }
                    break;
                case StateReception.CheckSum:
                    byte receivedChecksum = c;
                    byte calculatedChecksum = CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);

                    if (receivedChecksum == calculatedChecksum)
                    {
                        ProcessDecodeMessage(msgDecodedFunction, msgDecodedPayload);
                    }
                    else
                    {
                        textboxReception.Text += "Erreur de checksum ! Message rejeté.\n";
                    }
                    rcvState = StateReception.Waiting;
                    break;
                default:
                    rcvState = StateReception.Waiting;
                    break;
            }
        }

        bool ELVerte = false;
        bool ELRouge = false;
        bool ELOrange = false;
        bool ELBleue = false;
        bool ELBlanche = false;

        private void LEDRouge(object sender, RoutedEventArgs e)
        {
            ELRouge = !ELRouge;
            byte[] array = new byte[2];
            array[0] = 0;
            array[1] = Convert.ToByte(ELRouge);
            UartEncodeAndSendMessage(0x0020, 2, array);
        }

        private void LEDOrange(object sender, RoutedEventArgs e)
        {
            ELOrange = !ELOrange;
            byte[] array = new byte[2];
            array[0] = 1;
            array[1] = Convert.ToByte(ELOrange);
            UartEncodeAndSendMessage(0x0020, 2, array);
        }

        private void LEDBlanche(object sender, RoutedEventArgs e)
        {
            ELBlanche = !ELBlanche;
            byte[] array = new byte[2];
            array[0] = 2;
            array[1] = Convert.ToByte(ELBlanche);
            UartEncodeAndSendMessage(0x0020, 2, array);
        }

        private void LEDBleue(object sender, RoutedEventArgs e)
        {
            ELBleue = !ELBleue;
            byte[] array = new byte[2];
            array[0] = 3;
            array[1] = Convert.ToByte(ELBleue);
            UartEncodeAndSendMessage(0x0020, 2, array);
        }

        private void LEDVerte(object sender, RoutedEventArgs e)
        {
            ELVerte = !ELVerte;
            byte[] array = new byte[2];
            array[0] = 4;
            array[1] = Convert.ToByte(ELVerte);
            UartEncodeAndSendMessage(0x0020, 2, array);
        }

        public enum StateRobot
        {
            STATE_ATTENTE = 0,
            STATE_ATTENTE_EN_COURS = 1,
            STATE_AVANCE = 2,
            STATE_AVANCE_EN_COURS = 3,
            STATE_TOURNE_GAUCHE = 4,
            STATE_TOURNE_GAUCHE_EN_COURS = 5,
            STATE_TOURNE_DROITE = 6,
            STATE_TOURNE_DROITE_EN_COURS = 7,
            STATE_TOURNE_SUR_PLACE_GAUCHE = 8,
            STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS = 9,
            STATE_TOURNE_SUR_PLACE_DROITE = 10,
            STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS = 11,
            STATE_ARRET = 12,
            STATE_ARRET_EN_COURS = 13,
            STATE_RECULE = 14,
            STATE_RECULE_EN_COURS = 15
        }

        byte CalculateChecksum(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            ;
            byte c = 0;
            c ^= 0xFE;
            c ^= (byte)(msgFunction >> 8);
            c ^= (byte)(msgFunction);
            c ^= (byte)(msgPayloadLength >> 8);
            c ^= (byte)(msgPayloadLength);
            foreach (byte b in msgPayload)
            {
                c ^= b;
            }
            return c;
        }

        private void ProcessDecodeMessage(int msgFunction, byte[] msgPayload)
        {
            switch (msgFunction)
            {
                case 0x0020:
                    int Led = (int)msgPayload[0];
                    if (Led == 0)
                    {
                        ELVerte = Convert.ToBoolean(msgPayload[1]);
                    }
                    else if (Led == 1)
                    {
                        ELBleue = Convert.ToBoolean(msgPayload[1]);
                    }
                    else if (Led == 2)
                    {
                        ELBlanche = Convert.ToBoolean(msgPayload[1]);
                    }
                    else if (Led == 3)
                    {
                        ELOrange = Convert.ToBoolean(msgPayload[1]);
                    }
                    else if (Led == 4)
                    {
                        ELRouge = Convert.ToBoolean(msgPayload[1]);
                    }
                    break;

                case 0x0061:
                    robot.positionXOdo = BitConverter.ToSingle(msgPayload, 4);
                    robot.positionYOdo = BitConverter.ToSingle(msgPayload, 8);
                    robot.angleRadFOdo = BitConverter.ToSingle(msgPayload, 12);
                    robot.vitesseLinFOdo = BitConverter.ToSingle(msgPayload, 16);
                    robot.vitesseAngFOdo = BitConverter.ToSingle(msgPayload, 20);
                    robot.timeFrom = BitConverter.ToSingle(msgPayload, 24) / 1000;
                    robot.positionMD = BitConverter.ToSingle(msgPayload, 28);
                    robot.positionMG = BitConverter.ToSingle(msgPayload, 32);

                    ValXPos.Content = robot.positionXOdo.ToString("F5");
                    ValYPos.Content = robot.positionYOdo.ToString("F5");
                    ValAngle.Content = robot.angleRadFOdo.ToString("F2");
                    ValVitLin.Content = robot.vitesseLinFOdo.ToString("F2");
                    ValVitAng.Content = robot.vitesseAngFOdo.ToString("F2");
                    VMotDroit.Content = robot.positionMD.ToString("F2");
                    MotGauche.Content = robot.positionMG.ToString("F2");

                    break;

                //case 0x0091: // pid x
                //    robot.correcteurKp = BitConverter.ToSingle(msgPayload, 0);
                //    robot.correcteurKd = BitConverter.ToSingle(msgPayload, 4);
                //    robot.correcteurKi = BitConverter.ToSingle(msgPayload, 8);
                //    robot.corrPmaxX = BitConverter.ToSingle(msgPayload, 12);
                //    robot.corrImaxX = BitConverter.ToSingle(msgPayload, 16);
                //    robot.corrDmaxX = BitConverter.ToSingle(msgPayload, 20);

                //    myAsservissementDisplayControl.UpdatePolarCorrectionGains(robot.correcteurKp, 0,     robot.correcteurKi, 0,      robot.correcteurKd, 0);

                //    myAsservissementDisplayControl.UpdatePolarCorrectionMax(robot.corrPmaxX, 0,   robot.corrImaxX, 0,   robot.corrDmaxX, 0);

                //    break;

                //case 0x0092: // theta
                //    robot.correcteurThetaKp = BitConverter.ToSingle(msgPayload, 0);
                //    robot.correcteurThetaKd = BitConverter.ToSingle(msgPayload, 4);
                //    robot.correcteurThetaKi = BitConverter.ToSingle(msgPayload, 8);
                //    robot.corrPmaxTheta = BitConverter.ToSingle(msgPayload, 12);
                //    robot.corrImaxTheta = BitConverter.ToSingle(msgPayload, 16);
                //    robot.corrDmaxTheta = BitConverter.ToSingle(msgPayload, 20);

                //    myAsservissementDisplayControl.UpdatePolarCorrectionGains(0, robot.correcteurThetaKp, 0, robot.correcteurThetaKi, 0, robot.correcteurThetaKd);
                //    myAsservissementDisplayControl.UpdatePolarCorrectionMax(0, robot.corrPmaxTheta,   0, robot.corrImaxTheta,    0, robot.corrDmaxTheta);

                //    break;

                case 0x0030:
                    byte[] value = new byte[2];

                    Buffer.BlockCopy(msgPayload, 0, value, 0, 2);
                    ValueIRExGauche.Content = BitConverter.ToInt16(value, 0);

                    Buffer.BlockCopy(msgPayload, 2, value, 0, 2);
                    ValueIRGauche.Content = BitConverter.ToInt16(value, 0);

                    Buffer.BlockCopy(msgPayload, 4, value, 0, 2);
                    ValueIRCentre.Content = BitConverter.ToInt16(value, 0);

                    Buffer.BlockCopy(msgPayload, 6, value, 0, 2);
                    ValueIRDroit.Content = BitConverter.ToInt16(value, 0);

                    Buffer.BlockCopy(msgPayload, 8, value, 0, 2);
                    ValueIRExDroit.Content = BitConverter.ToInt16(value, 0);
                    break;
                default:
                    break;
            }
        }

        private void CheckBox_Checked(object sender, RoutedEventArgs e)
        {
            string messageStr = "Bonjour";
            byte[] msgPayload = Encoding.ASCII.GetBytes(messageStr);
            int msgPayloadLength = msgPayload.Length;
            int msgFunction = 0x0080;
            UartEncodeAndSendMessage(msgFunction, msgPayloadLength, msgPayload);
        }

        private void CheckBox_Checked_1(object sender, RoutedEventArgs e)
        {

        }

        private void CheckBox_Checked_2(object sender, RoutedEventArgs e)
        {

        }

        private void CheckBox_Checked_3(object sender, RoutedEventArgs e)
        {

        }

        private void CheckBox_Unchecked(object sender, RoutedEventArgs e)
        {

        }

        private void Mot_KeyUp(object sender, KeyEventArgs e)
        {

        }

        private void oscilloSpeed_Loaded(object sender, RoutedEventArgs e)
        {

        }

        private void textboxEmission_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void worldMap_Loaded(object sender, RoutedEventArgs e)
        {

        }
    }
}