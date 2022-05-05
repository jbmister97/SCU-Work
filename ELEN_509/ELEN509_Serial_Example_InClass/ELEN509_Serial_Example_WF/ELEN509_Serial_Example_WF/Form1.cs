using System.IO.Ports;

namespace ELEN509_Serial_Example_WF
{
    public partial class Form1 : Form
    {
        ES_UART myES = new ES_UART();

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

        private void button1_Click(object sender, EventArgs e)
        {
            if ((cboCommPort.SelectedIndex == 0) || (cboBaudRate.SelectedIndex == 0))
            {
                MessageBox.Show("Please ensure you've slelected a BAUD rate and a valid comm port");
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

                    btnOpenClose.Text = "Close Port";
                    tslPortStatus.Text = "Open";
                    tslPortStatus.BackColor = Color.Green;

                    myES.OpenPort();
                    //tmrCommBuffer.Enabled = true;
                }
                else
                {
                    myES.ClosePort();
                    btnOpenClose.Text = "Open Port";
                    tslPortStatus.Text = "Closed";
                    tslPortStatus.BackColor = Color.Red;
                    //tmrCommBuffer.Enabled = false;
                }
            }

            myES.TurnOnLogging(txtRxData, tslInPtr, tslOutPtr);
        }

        void ProcessReceivedByte_Callback(object sender, byte rcvdByte)
        {
            this.BeginInvoke(new ES_UART.RxBufferRemoveByteEventHandler(ES_RxBufferProcessor), sender, rcvdByte);
        }

        void ES_RxBufferProcessor(object sender, byte e)
        {
            byte[] daByte = new byte[1];
            daByte[0] = e;
            //myES.SendData(daByte);
        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
            
        }

        private void tslInPtr_Click(object sender, EventArgs e)
        {

        }

        private void statusStrip1_ItemClicked(object sender, ToolStripItemClickedEventArgs e)
        {

        }

        private void btnClear_Click(object sender, EventArgs e)
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
            myES.SendData("command 1");
        }

        private void txtRxData_TextChanged(object sender, EventArgs e)
        {

        }
    }
}