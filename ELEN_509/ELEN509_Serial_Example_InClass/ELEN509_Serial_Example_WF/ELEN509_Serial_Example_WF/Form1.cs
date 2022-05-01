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
    }
}