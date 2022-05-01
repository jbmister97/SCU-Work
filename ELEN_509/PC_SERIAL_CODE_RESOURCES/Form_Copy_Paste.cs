using System.IO.Ports;


namespace SerialExample_CLS
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

            cboComPort.Items.Add("Select COM Port...");
            cboComPort.Items.AddRange(myES.ListComPorts());
            cboComPort.SelectedIndex = 0;

            cboBaudRate.SelectedIndex = 0;
        }

        private void btnQuit_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void btnPortOpenClose_Click(object sender, EventArgs e)
        {
            if ((cboComPort.SelectedIndex == 0) || (cboBaudRate.SelectedIndex == 0))
            {
                MessageBox.Show("Please ensure you've slelected a BAUD rate and a valid comm port");
            }
            else
            {
                if (myES.PortIsOpen == false)
                {
                    myES.PortName = cboComPort.Text;
                    myES.BaudRate = Convert.ToInt32(cboBaudRate.Text);
                    myES.Parity = Parity.None;
                    myES.DataBits = 8;
                    myES.ReceivedBytesThreshold = 1;

                    btnPortOpenClose.Text = "Close Port";
                    tslPortStatus.Text = "Open";
                    tslPortStatus.BackColor = Color.Green;

                    myES.OpenPort();
                    //tmrCommBuffer.Enabled = true;
                }
                else
                {
                    myES.ClosePort();
                    btnPortOpenClose.Text = "Open Port";
                    tslPortStatus.Text = "Closed";
                    tslPortStatus.BackColor = Color.Red;
                    //tmrCommBuffer.Enabled = false;
                }
            }

            myES.TurnOnLogging(txtRxData, tslInPtr, tslOutPtr);

        }

        private void btnSend_Click(object sender, EventArgs e)
        {
            myES.SendData(txtTxData.Text);
        }

        private void btnClearRx_Click(object sender, EventArgs e)
        {
            txtRxData.Text = "";
        }

        private void btnClearTX_Click(object sender, EventArgs e)
        {
            txtTxData.Text = "";
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
    }
}