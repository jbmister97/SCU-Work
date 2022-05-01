using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Management;



namespace SerialExample_CLS
{
    internal class ES_UART
    {
        const Int32 RX_BUFFER_SIZE = 100;
        byte[] rxBuffer = new byte[RX_BUFFER_SIZE];
        Int32 nextBufferInputPoint = 0;
        Int32 nextByteToProcess = 0;


        bool comLogging = false;
        System.Windows.Forms.TextBox logBox;
        System.Windows.Forms.ToolStripStatusLabel inPtrLbl;
        //Object inPtrLbl;
        System.Windows.Forms.ToolStripStatusLabel outPtrLbl;

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
                esLink.Open();
                tmrCommBuffer.Enabled = true;
            }
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
                if (++nextBufferInputPoint >= RX_BUFFER_SIZE)
                    nextBufferInputPoint = 0;

                // then put it in the text box for logging
                logBox.Invoke(new UpdateUiTextDelegate(WriteData), recieved_data, nextBufferInputPoint);
            }
        }
        private void WriteData(byte text, Int32 ptr)
        {
            // Assign the value of the recieved_data to the RichTextBox.
            if (comLogging)
            {
                logBox.Text += (char)text;
                inPtrLbl.Text = ptr.ToString();
            }
        }

        public void TurnOnLogging(System.Windows.Forms.TextBox location, System.Windows.Forms.ToolStripStatusLabel inLbl, System.Windows.Forms.ToolStripStatusLabel outLbl)
        {
            comLogging = true;
            logBox = location;
            inPtrLbl = new System.Windows.Forms.ToolStripStatusLabel();
            inPtrLbl = inLbl;
            outPtrLbl = outLbl;
        }

        public void SendData(string data2send)
        {
            esLink.Write(data2send);
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
                    outPtrLbl.Text = nextByteToProcess.ToString();
                }
                esLink.Write(dataByte, 0, 1);
            }
        }
    }

}

