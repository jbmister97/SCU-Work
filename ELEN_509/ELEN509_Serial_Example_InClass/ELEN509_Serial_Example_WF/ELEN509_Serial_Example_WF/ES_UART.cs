using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Management;

namespace ELEN509_Serial_Example_WF
{
    internal class ES_UART
    {
        const Int32 RX_BUFFER_SIZE = 100;
        byte[] rxBuffer = new byte[RX_BUFFER_SIZE];
        Int32 nextBufferInputPoint = 0;
        Int32 nextByteToProcess = 0;
        Int32 noOfCommErrors = 0;


        bool comLogging = false;
        static SerialPort esLink = new SerialPort();
        static System.Windows.Forms.Timer tmrCommBuffer = new System.Windows.Forms.Timer();

        public ES_UART()
        {
            tmrCommBuffer.Interval = 10;
            tmrCommBuffer.Enabled = false;
            tmrCommBuffer.Tick += tmrCommBuffer_Tick;

        }


        public string[] ListComPorts()
        {
            List<String> tList = new List<String>();

            foreach (string s in SerialPort.GetPortNames())
            {
                if (!tList.Contains(s))
                    tList.Add(s);
            }

            tList.Sort((a, b) => (Convert.ToInt32(a.Replace("COM", string.Empty))).CompareTo(Convert.ToInt32(b.Replace("COM", string.Empty))));

            return tList.ToArray();
        }

        #region WMI_Port_Discovery
        public string[] FindAllPorts()
        {
            List<string> ports = new List<string>();

            foreach (ManagementObject obj in FindPorts())
            {
                try
                {
                    if (obj["Caption"].ToString().Contains("(COM"))
                    {
                        string ComName = ParseCommName(obj);
                        if (ComName != null)
                        {
                            ports.Add(ComName);
                        }
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }
            }

            return ports.ToArray();
        }


        public string FindPortByDescription(string description)
        {
            foreach (ManagementObject obj in FindPorts())
            {
                try
                {
                    if (obj["Description"].ToString().ToLower().Equals(description.ToLower()))
                    {
                        string comName = ParseCommName(obj);
                        if (comName != null)
                            return comName;
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }
            }
            return null;
        }


        public List<string> ListPortsByDescription()
        {
            List<string> descList = new List<string>();

            foreach (ManagementObject obj in FindPorts())
            {
                try
                {
                    if (obj["Caption"].ToString().Contains("(COM"))
                        descList.Add(obj["Description"].ToString());
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                    //                    return null;
                }
            }
            return descList;
        }


        public List<string> ListPortsByManufacturer()
        {
            List<string> mfrList = new List<string>();

            foreach (ManagementObject obj in FindPorts())
            {
                try
                {
                    if (obj["Caption"].ToString().Contains("(COM"))
                        mfrList.Add(obj["Manufacturer"].ToString());
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }
            }
            return mfrList;
        }


        public List<string> ListPortsByName()
        {
            List<string> nameList = new List<string>();

            foreach (ManagementObject obj in FindPorts())
            {
                try
                {
                    if (obj["Caption"].ToString().Contains("(COM"))
                        nameList.Add(obj["Name"].ToString());
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }
            }
            return nameList;
        }


        public string FindPortByManufacturer(string manufacturer)
        {
            foreach (ManagementObject obj in FindPorts())
            {
                try
                {
                    if (obj["Manufacturer"].ToString().ToLower().Equals(manufacturer.ToLower()))
                    {
                        string comName = ParseCommName(obj);
                        if (comName != null)
                            return comName;
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                }
            }
            return null;
        }


        static ManagementObject[] FindPorts()
        {
            try
            {
                ManagementObjectSearcher searcher = new ManagementObjectSearcher("root\\CIMV2", "SELECT * FROM Win32_PnPEntity");
                List<ManagementObject> objects = new List<ManagementObject>();

                foreach (ManagementObject obj in searcher.Get())
                {
                    objects.Add(obj);
                }

                return objects.ToArray();
            }
            catch (Exception e)
            {
                Console.WriteLine(e);
                return new ManagementObject[] { };
            }
        }


        static string ParseCommName(ManagementObject obj)
        {
            string name = obj["Name"].ToString();
            int startIndex = name.LastIndexOf("(");
            int endIndex = name.LastIndexOf(")");

            if ((startIndex != -1) && (endIndex != -1))
            {
                name = name.Substring(startIndex + 1, endIndex - startIndex - 1);
                return name;
            }

            return null;
        }


        #endregion WMI_Port_Discovery


        public bool PortIsOpen
        {
            get { return esLink.IsOpen; }
        }

        public void OpenPort()
        {
            if (!esLink.IsOpen)
            {
                esLink.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(Recieve);
                esLink.ErrorReceived += new System.IO.Ports.SerialErrorReceivedEventHandler(CommErrorHandler);
                try
                {
                    esLink.Open();
                }
                catch (System.Exception e)
                {
                    MessageBox.Show(e.Message);
                }
                tmrCommBuffer.Enabled = true;
            }
        }

        void CommErrorHandler(object sender, System.IO.Ports.SerialErrorReceivedEventArgs e)
        {
            noOfCommErrors++;
        }

        public void ClosePort()
        {
            if (esLink.IsOpen)
            {
                esLink.Close();
                tmrCommBuffer.Enabled = false;
            }
        }

        public string PortName
        {
            get { return esLink.PortName; }

            set { esLink.PortName = value; }
        }

        public int BaudRate
        {
            get { return esLink.BaudRate; }
            set { esLink.BaudRate = value; }
        }

        public Parity Parity
        {
            get { return esLink.Parity; }
            set { esLink.Parity = value; }
        }

        public int DataBits
        {
            get { return esLink.DataBits; }
            set { esLink.DataBits = value; }
        }

        public int ReceivedBytesThreshold
        {
            get { return esLink.ReceivedBytesThreshold; }
            set { esLink.ReceivedBytesThreshold = value; }
        }

        public int NoOfReceivedCharInBuffer
        {
            get { return esLink.BytesToWrite; }
            set { }
        }

        private delegate void UpdateUiTextDelegate(byte text, Int32 ptr);
        private void Recieve(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            byte recieved_data;

            // Collecting the characters received to our 'buffer' (string).
            while (esLink.BytesToRead > 0)
            {
                recieved_data = (byte)esLink.ReadByte();

                // add it to the circular buffer
                rxBuffer[nextBufferInputPoint] = recieved_data;

                // Then move the pointer
                if (++nextBufferInputPoint >= RX_BUFFER_SIZE)
                    nextBufferInputPoint = 0;

                // then put it in the text box for logging
                if (comLogging == true)
                {
                    RecievedByteEventArgs args = new RecievedByteEventArgs();
                    args.nextByteToProcess = nextByteToProcess;
                    args.nextBufferInputPoint = nextBufferInputPoint;
                    args.receivedByte = recieved_data;

                    OnReceivedByte(args);
                }
            }
        }
        public void TurnOnLogging()
        {
            comLogging = true;
        }

        public void SendData(string data2send)
        {
            esLink.Write(data2send);
        }

        public void SendData(byte[] data2send)
        {
            esLink.Write(data2send, 0, 1);
        }


        private void tmrCommBuffer_Tick(object sender, EventArgs e)
        {
            byte[] dataByte = new byte[1];

            if (nextBufferInputPoint != nextByteToProcess)
            {
                dataByte[0] = rxBuffer[nextByteToProcess];
                if (++nextByteToProcess >= RX_BUFFER_SIZE)
                    nextByteToProcess = 0;

                if (comLogging)
                {
                    RecievedByteEventArgs args = new RecievedByteEventArgs();
                    args.nextByteToProcess = nextByteToProcess;
                    args.nextBufferInputPoint = nextBufferInputPoint;
                    args.receivedByte = 0;

                    OnReceivedByte(args);
                }
                //                esLink.Write(dataByte, 0, 1);

                OnRxBufferContents(dataByte[0]);
            }
        }


        #region RECEIVE_BYTE_LOG_HANDLER
        protected virtual void OnReceivedByte(RecievedByteEventArgs e)
        {
            ByteRecievedEventHandler handler = ByteRecieved;
            if (handler != null)
            {
                handler(this, e);
            }
        }


        public event ByteRecievedEventHandler ByteRecieved;

        public delegate void ByteRecievedEventHandler(Object sender, RecievedByteEventArgs e);

        public class RecievedByteEventArgs : EventArgs
        {
            public byte receivedByte { get; set; }
            public Int32 nextBufferInputPoint { get; set; }
            public Int32 nextByteToProcess { get; set; }

        }
        #endregion RECEIVE_BYTE_LOG_HANDLER

        #region RX_BUFFER_UNLOAD_HANDLER
        protected virtual void OnRxBufferContents(byte comByte)
        {
            RxBufferRemoveByteEventHandler handler = GrabAByteFromRx;
            if (handler != null)
            {
                handler(this, comByte);
            }
        }

        public event RxBufferRemoveByteEventHandler GrabAByteFromRx;

        public delegate void RxBufferRemoveByteEventHandler(object sender, byte comByte);

        #endregion RX_BUFFER_UNLOAD_HANDLER

    }


}
