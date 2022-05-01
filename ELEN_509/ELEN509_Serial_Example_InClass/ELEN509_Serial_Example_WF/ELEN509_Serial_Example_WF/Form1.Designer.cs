namespace ELEN509_Serial_Example_WF
{
    partial class Form1
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.btnOpenClose = new System.Windows.Forms.Button();
            this.btnGPI = new System.Windows.Forms.Button();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.cboBaudRate = new System.Windows.Forms.ComboBox();
            this.cboCommPort = new System.Windows.Forms.ComboBox();
            this.grpReceiveData = new System.Windows.Forms.GroupBox();
            this.btnClearRx = new System.Windows.Forms.Button();
            this.txtRxData = new System.Windows.Forms.TextBox();
            this.grpTransmitData = new System.Windows.Forms.GroupBox();
            this.btnSend = new System.Windows.Forms.Button();
            this.btnClearTx = new System.Windows.Forms.Button();
            this.txtTxData = new System.Windows.Forms.TextBox();
            this.btnQuit = new System.Windows.Forms.Button();
            this.statusStrip1 = new System.Windows.Forms.StatusStrip();
            this.tslPortStatus = new System.Windows.Forms.ToolStripStatusLabel();
            this.tslInPtr = new System.Windows.Forms.ToolStripStatusLabel();
            this.tslOutPtr = new System.Windows.Forms.ToolStripStatusLabel();
            this.toolStripStatusLabel4 = new System.Windows.Forms.ToolStripStatusLabel();
            this.groupBox1.SuspendLayout();
            this.grpReceiveData.SuspendLayout();
            this.grpTransmitData.SuspendLayout();
            this.statusStrip1.SuspendLayout();
            this.SuspendLayout();
            // 
            // btnOpenClose
            // 
            this.btnOpenClose.Location = new System.Drawing.Point(301, 33);
            this.btnOpenClose.Name = "btnOpenClose";
            this.btnOpenClose.Size = new System.Drawing.Size(80, 23);
            this.btnOpenClose.TabIndex = 0;
            this.btnOpenClose.Text = "Open Port";
            this.btnOpenClose.UseVisualStyleBackColor = true;
            this.btnOpenClose.Click += new System.EventHandler(this.button1_Click);
            // 
            // btnGPI
            // 
            this.btnGPI.Location = new System.Drawing.Point(301, 61);
            this.btnGPI.Name = "btnGPI";
            this.btnGPI.Size = new System.Drawing.Size(80, 23);
            this.btnGPI.TabIndex = 1;
            this.btnGPI.Text = "GPI";
            this.btnGPI.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.cboBaudRate);
            this.groupBox1.Controls.Add(this.cboCommPort);
            this.groupBox1.Controls.Add(this.btnOpenClose);
            this.groupBox1.Controls.Add(this.btnGPI);
            this.groupBox1.Location = new System.Drawing.Point(12, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(401, 115);
            this.groupBox1.TabIndex = 2;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "groupBox1";
            // 
            // cboBaudRate
            // 
            this.cboBaudRate.FormattingEnabled = true;
            this.cboBaudRate.Items.AddRange(new object[] {
            "Choose Baud Rate...",
            "9600",
            "19200",
            "115200"});
            this.cboBaudRate.Location = new System.Drawing.Point(20, 62);
            this.cboBaudRate.Name = "cboBaudRate";
            this.cboBaudRate.Size = new System.Drawing.Size(236, 23);
            this.cboBaudRate.TabIndex = 5;
            // 
            // cboCommPort
            // 
            this.cboCommPort.FormattingEnabled = true;
            this.cboCommPort.Location = new System.Drawing.Point(20, 33);
            this.cboCommPort.Name = "cboCommPort";
            this.cboCommPort.Size = new System.Drawing.Size(236, 23);
            this.cboCommPort.TabIndex = 2;
            this.cboCommPort.SelectedIndexChanged += new System.EventHandler(this.comboBox1_SelectedIndexChanged);
            // 
            // grpReceiveData
            // 
            this.grpReceiveData.Controls.Add(this.btnClearRx);
            this.grpReceiveData.Controls.Add(this.txtRxData);
            this.grpReceiveData.Location = new System.Drawing.Point(12, 273);
            this.grpReceiveData.Name = "grpReceiveData";
            this.grpReceiveData.Size = new System.Drawing.Size(401, 192);
            this.grpReceiveData.TabIndex = 3;
            this.grpReceiveData.TabStop = false;
            this.grpReceiveData.Text = "Receive Data";
            // 
            // btnClearRx
            // 
            this.btnClearRx.Location = new System.Drawing.Point(306, 163);
            this.btnClearRx.Name = "btnClearRx";
            this.btnClearRx.Size = new System.Drawing.Size(75, 23);
            this.btnClearRx.TabIndex = 7;
            this.btnClearRx.Text = "Clear";
            this.btnClearRx.UseVisualStyleBackColor = true;
            // 
            // txtRxData
            // 
            this.txtRxData.Location = new System.Drawing.Point(20, 22);
            this.txtRxData.Multiline = true;
            this.txtRxData.Name = "txtRxData";
            this.txtRxData.Size = new System.Drawing.Size(361, 131);
            this.txtRxData.TabIndex = 7;
            // 
            // grpTransmitData
            // 
            this.grpTransmitData.Controls.Add(this.btnSend);
            this.grpTransmitData.Controls.Add(this.btnClearTx);
            this.grpTransmitData.Controls.Add(this.txtTxData);
            this.grpTransmitData.Location = new System.Drawing.Point(12, 133);
            this.grpTransmitData.Name = "grpTransmitData";
            this.grpTransmitData.Size = new System.Drawing.Size(401, 125);
            this.grpTransmitData.TabIndex = 4;
            this.grpTransmitData.TabStop = false;
            this.grpTransmitData.Text = "Transmit Data";
            // 
            // btnSend
            // 
            this.btnSend.Location = new System.Drawing.Point(20, 80);
            this.btnSend.Name = "btnSend";
            this.btnSend.Size = new System.Drawing.Size(75, 23);
            this.btnSend.TabIndex = 2;
            this.btnSend.Text = "Send";
            this.btnSend.UseVisualStyleBackColor = true;
            // 
            // btnClearTx
            // 
            this.btnClearTx.Location = new System.Drawing.Point(306, 80);
            this.btnClearTx.Name = "btnClearTx";
            this.btnClearTx.Size = new System.Drawing.Size(75, 23);
            this.btnClearTx.TabIndex = 1;
            this.btnClearTx.Text = "Clear";
            this.btnClearTx.UseVisualStyleBackColor = true;
            this.btnClearTx.Click += new System.EventHandler(this.btnClear_Click);
            // 
            // txtTxData
            // 
            this.txtTxData.Location = new System.Drawing.Point(20, 31);
            this.txtTxData.Name = "txtTxData";
            this.txtTxData.Size = new System.Drawing.Size(361, 23);
            this.txtTxData.TabIndex = 0;
            // 
            // btnQuit
            // 
            this.btnQuit.Location = new System.Drawing.Point(318, 488);
            this.btnQuit.Name = "btnQuit";
            this.btnQuit.Size = new System.Drawing.Size(75, 23);
            this.btnQuit.TabIndex = 5;
            this.btnQuit.Text = "Quit";
            this.btnQuit.UseVisualStyleBackColor = true;
            this.btnQuit.Click += new System.EventHandler(this.btnQuit_Click);
            // 
            // statusStrip1
            // 
            this.statusStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.tslPortStatus,
            this.tslInPtr,
            this.tslOutPtr,
            this.toolStripStatusLabel4});
            this.statusStrip1.Location = new System.Drawing.Point(0, 537);
            this.statusStrip1.Name = "statusStrip1";
            this.statusStrip1.Size = new System.Drawing.Size(425, 24);
            this.statusStrip1.TabIndex = 6;
            this.statusStrip1.Text = "statusStrip1";
            this.statusStrip1.ItemClicked += new System.Windows.Forms.ToolStripItemClickedEventHandler(this.statusStrip1_ItemClicked);
            // 
            // tslPortStatus
            // 
            this.tslPortStatus.AutoSize = false;
            this.tslPortStatus.BackColor = System.Drawing.Color.Red;
            this.tslPortStatus.BorderSides = ((System.Windows.Forms.ToolStripStatusLabelBorderSides)((System.Windows.Forms.ToolStripStatusLabelBorderSides.Left | System.Windows.Forms.ToolStripStatusLabelBorderSides.Right)));
            this.tslPortStatus.Name = "tslPortStatus";
            this.tslPortStatus.Size = new System.Drawing.Size(100, 19);
            this.tslPortStatus.Text = "Closed";
            // 
            // tslInPtr
            // 
            this.tslInPtr.AutoSize = false;
            this.tslInPtr.BorderSides = ((System.Windows.Forms.ToolStripStatusLabelBorderSides)((System.Windows.Forms.ToolStripStatusLabelBorderSides.Left | System.Windows.Forms.ToolStripStatusLabelBorderSides.Right)));
            this.tslInPtr.Name = "tslInPtr";
            this.tslInPtr.Size = new System.Drawing.Size(150, 19);
            this.tslInPtr.Text = "toolStripStatusLabel2";
            this.tslInPtr.Click += new System.EventHandler(this.tslInPtr_Click);
            // 
            // tslOutPtr
            // 
            this.tslOutPtr.AutoSize = false;
            this.tslOutPtr.BorderSides = ((System.Windows.Forms.ToolStripStatusLabelBorderSides)((System.Windows.Forms.ToolStripStatusLabelBorderSides.Left | System.Windows.Forms.ToolStripStatusLabelBorderSides.Right)));
            this.tslOutPtr.Name = "tslOutPtr";
            this.tslOutPtr.Size = new System.Drawing.Size(150, 19);
            this.tslOutPtr.Text = "toolStripStatusLabel3";
            // 
            // toolStripStatusLabel4
            // 
            this.toolStripStatusLabel4.Name = "toolStripStatusLabel4";
            this.toolStripStatusLabel4.Size = new System.Drawing.Size(0, 19);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(425, 561);
            this.Controls.Add(this.statusStrip1);
            this.Controls.Add(this.btnQuit);
            this.Controls.Add(this.grpTransmitData);
            this.Controls.Add(this.grpReceiveData);
            this.Controls.Add(this.groupBox1);
            this.Name = "Form1";
            this.Text = "PC-ES Serial Comm";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox1.ResumeLayout(false);
            this.grpReceiveData.ResumeLayout(false);
            this.grpReceiveData.PerformLayout();
            this.grpTransmitData.ResumeLayout(false);
            this.grpTransmitData.PerformLayout();
            this.statusStrip1.ResumeLayout(false);
            this.statusStrip1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private Button btnOpenClose;
        private Button btnGPI;
        private GroupBox groupBox1;
        private GroupBox grpReceiveData;
        private GroupBox grpTransmitData;
        private ComboBox cboBaudRate;
        private ComboBox cboCommPort;
        private Button btnQuit;
        private StatusStrip statusStrip1;
        private ToolStripStatusLabel tslPortStatus;
        private ToolStripStatusLabel tslInPtr;
        private ToolStripStatusLabel tslOutPtr;
        private ToolStripStatusLabel toolStripStatusLabel4;
        private Button btnSend;
        private Button btnClearTx;
        private TextBox txtTxData;
        private TextBox txtRxData;
        private Button btnClearRx;
    }
}