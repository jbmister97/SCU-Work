using System.IO.Ports;

namespace ELEN509_Serial_Example_WF
{
    public partial class Form1 : Form
    {
        ES_UART myES = new ES_UART();
        Boolean packetComplete = false;

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            cboCommPort.Items.Add("Select COM Port...");
            cboCommPort.Items.AddRange(myES.ListComPorts());
            cboCommPort.SelectedIndex = 0;

            cboBaudRate.SelectedIndex = 0;
        }

        private void btnOpenClose_Click(object sender, EventArgs e)
        {
            if ((cboCommPort.SelectedIndex == 0) || (cboBaudRate.SelectedIndex == 0))
            {
                MessageBox.Show("Please ensure you've selected a BAUD rate and a valid comm port");
            }
            else
            {
                if (myES.PortIsOpen == false)
                {
                    myES.PortName = cboCommPort.Text;
                    myES.BaudRate = Convert.ToInt32(cboBaudRate.Text);
                    myES.Parity = Parity.None;
                    myES.DataBits = 8;
                    myES.ReceivedBytesThreshold = 1;

                    myES.ByteRecieved += ES_ComLoggingUI_Callback;
                    myES.GrabAByteFromRx += ProcessReceivedByte_Callback;

                    myES.OpenPort();
                    if (myES.PortIsOpen == true)
                    {
                        btnOpenClose.Text = "Close Port";
                        tslPortStatus.Text = "Open";
                        tslPortStatus.BackColor = Color.Green;
                    }
                        
                }
                else
                {
                    if (myES.PortIsOpen == true)
                    {
                        myES.ClosePort();
                        btnOpenClose.Text = "Open Port";
                        tslPortStatus.Text = "Closed";
                        tslPortStatus.BackColor = Color.Red;
                    }
                }
            }

            myES.TurnOnLogging();
        }

        void ProcessReceivedByte_Callback(object sender, byte rcvdByte)
        {
            this.BeginInvoke(new ES_UART.RxBufferRemoveByteEventHandler(ES_RxBufferProcessor), sender, rcvdByte);
        }

        void ES_RxBufferProcessor(object sender, byte e)
        {
            byte[] daByte = new byte[1];
            daByte[0] = e;
            if (daByte[0] != 0)
            {
                
                // Makes sure the data is an ASCII character
                if (((daByte[0] >= 'a') && daByte[0] <= 'z') || (daByte[0] == ' ') ||
                ((daByte[0] >= 'A') && (daByte[0] <= 'Z')) || (daByte[0] == '\r'))
                {

                    if (packetComplete)  // If starting new packet clear the text box
                    {
                        txtValue.Text = "";
                        packetComplete = false;
                    }

                    if (daByte[0] == '\r') // If end of letters reached
                    {
                        packetComplete = true;
                        // Check if text says Pressed
                        if (txtValue.Text == "Pressed")
                        {
                            lblButtonStatus.BackColor = Color.Green;
                        }
                        // Check if text says Not Pressed
                        else if (txtValue.Text == "Not Pressed")
                        {
                            lblButtonStatus.BackColor = Color.Red;
                        }
                    }
                    
                    else // still inside packet
                    {
                        txtValue.AppendText(((char)daByte[0]).ToString());
                    }
                }

            }

            //myES.SendData(daByte);
        }

        void ES_ComLoggingUI_Callback(object sender, ES_UART.RecievedByteEventArgs e)
        {
            this.BeginInvoke(new ES_UART.ByteRecievedEventHandler(ES_LogComData), sender, e);
        }

        void ES_LogComData(object sender, ES_UART.RecievedByteEventArgs e)
        {
            if(e.receivedByte != 0)
            {
                txtRxData.AppendText(((char)e.receivedByte).ToString());
            }
            tslInPtr.Text = e.nextBufferInputPoint.ToString();
            tslOutPtr.Text = e.nextByteToProcess.ToString();
        }

        private void btnClearTx_Click(object sender, EventArgs e)
        {
            txtTxData.Text = "";
        }

        private void btnQuit_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void btnGPI_Click(object sender, EventArgs e)
        {
            txtRxData.Text = "------ ALL ------\r\n";

            foreach (var item in myES.FindAllPorts())
            {
                txtRxData.Text += (item.ToString() + "\r\n");
            }

            txtRxData.Text += "------ Port by DESC ------\r\n";
            string desc = myES.FindPortByDescription("STMicroelectronics STLink Virtual COM Port") + "\r\n";
            if (desc != null)
                txtRxData.Text += desc;

            txtRxData.Text += "------ Port by MFR ------\r\n";
            string mfr = myES.FindPortByManufacturer("STMicroelectronics" + "\r\n");
            if (desc != null)
                txtRxData.Text += desc;

            txtRxData.Text += "------ DESC ------\r\n";
            List<string> results = myES.ListPortsByDescription();
            foreach (string item in results)
            {
                txtRxData.Text += (item + "\r\n");
            }

            txtRxData.Text += "------ MFR ------\r\n";
            results = myES.ListPortsByManufacturer();
            foreach (string item in results)
            {
                txtRxData.Text += (item + "\r\n");
            }

            txtRxData.Text += "------ NAME ------\r\n";
            results = myES.ListPortsByName();
            foreach (string item in results)
            {
                txtRxData.Text += (item + "\r\n");
            }
        }

        private void btnCommand1_Click(object sender, EventArgs e)
        {
            myES.SendData("$r\n"); // turn on LED
        }

        private void btnSend_Click(object sender, EventArgs e)
        {
            myES.SendData(txtTxData.Text);
        }

        private void btnClearRx_Click(object sender, EventArgs e)
        {
            txtTxData.Text = "";
        }

        private void btnCommand2_Click(object sender, EventArgs e)
        {
            myES.SendData("$s\n"); // turn off LED
        }

        private void btnCommand3_Click(object sender, EventArgs e)
        {
            myES.SendData("$v\n"); // get button status
        }
    }
}