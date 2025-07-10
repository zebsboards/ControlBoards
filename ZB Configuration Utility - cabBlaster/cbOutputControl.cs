using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;

namespace Configuration_Utility
{
    public delegate void AnswerBack(bool ResetResponse);
    public partial class cbOutputControl : Form
    {
        byte UsePwm;
        byte FlipperHoldVal;
        
        public AnswerBack transferDelegate;
        WarningPopup warningPopup = null;

        public cbOutputControl()
        {
            InitializeComponent();
            transferDelegate += new AnswerBack(ReceiveInput);
        }

        private void cbOutputControl_Load(object sender, EventArgs e)
        {
            DisconnectedState();
        }

        int PortFound;
        int NumBoards;
        int LoadOutputs;

        private void ComDisconnectButton_Click_1(object sender, EventArgs e)
        {
            if (ComPort.IsOpen)
            {
                ClearDataBox();
                DisconnectedState();
                ComPort.Close();
            }
            else
            {
                ClearDataBox();
                DisconnectedState();
            }
        }
        private void ConnectButton_Click(object sender, EventArgs e)
        {
            ClearDataBox();
            EmergencyReset = false;
            PortFound = 0;
            ConnectToController("U", "41C5");
        }

        bool EmergencyReset;
        private void ConnectToController(string CommandCode, string VerificationID)
        {
            DataIn = null;
            string[] ports = SerialPort.GetPortNames();
            ComPortBox.Items.AddRange(ports);
            ComPort.DataReceived += new SerialDataReceivedEventHandler(ComPort_DataReceived);
            foreach (string pf in SerialPort.GetPortNames())
            {
                try
                {
                    ComPortBox.Text = pf;
                    ComPort.PortName = pf;
                    ComPort.BaudRate = 2000000;
                    ComPort.DtrEnable = true;
                    ComPort.RtsEnable = true;
                    ComPort.ReadTimeout = 500;
                    ComPort.WriteTimeout = 200;
                    ComPort.Open();
                    if (ComPort.IsOpen)
                    {
                        ComPort.Write(CommandCode);
                        Task.Delay(20).Wait();
                        if (DataIn != null)
                        {
                            try
                            {
                                if (DataIn == VerificationID )
                                {
                                    DataIn = null;
                                    ComPort.Write("B");
                                    Task.Delay(20).Wait();
                                    if (DataIn != null)
                                    {
                                        NumBoards = Convert.ToInt32(DataIn);
                                        if (NumBoards < 1)
                                        {
                                            NumBoards = 1;
                                        }
                                        LoadOutputs = 32 + (16 * (NumBoards - 1));
                                        OutputSelectionBox.Maximum = LoadOutputs;
                                    }
                                    ConnectedState();
                                    PortFound = 1;
                                    break;
                                }
                            }
                            catch (Exception err)
                            {
                                MessageBox.Show(err.Message, "OOPS, didn't get all of the verification info", MessageBoxButtons.OK, MessageBoxIcon.Error);
                                Task.Delay(1000).Wait();
                                ComPort.Close();
                            }
                        }
                        else
                        {
                            ComPort.Close();
                        }
                    }
                }
                catch (Exception err)
                {
                    if (ComPort != null)
                    {
                        ComPort.Close();
                        //Task.Delay(1000).Wait();
                        //MessageBox.Show(err.Message, "Incorrect Port .. Click Ok to Try Another", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                }
            }
            if (PortFound == 1)
            {
                rtbDataIn.Text += "Found " + Convert.ToString(NumBoards) + " boards for a total of " + Convert.ToString(LoadOutputs) + " available outputs.\r";
                rtbDiagData.Text += rtbDataIn.Text;
                if (EmergencyReset != true)
                {
                    RetrieveSettings();
                }
            }
            else
            {
                rtbDataIn.Text = "Controller Not Found.  Reccomend REBOOTING Controller and RESTART the Utility";
                rtbDiagData.Text = rtbDataIn.Text;
            }
        }
        private void ConnectedState()
        {
            ConnectButton.BackColor = Color.Green;
            ConnectButton.Text = "CONNECTED";
            ConnectButton.Enabled = false;
            DiagConnectButton.BackColor = Color.Green;
            DiagConnectButton.Text = "CONNECTED";
            DiagConnectButton.Enabled = false;
            ComDisconnectButton.BackColor = Color.DarkGray;
            ComDisconnectButton.Text = "DISCONNECT";
            ComDisconnectButton.Enabled = true;
            DiagDisconnectButton.BackColor = Color.DarkGray;
            DiagDisconnectButton.Text = "DISCONNECT";
            DiagDisconnectButton.Enabled = true;
            SetButton.Enabled = true;
            SendButton.Enabled = true;
            SavedValueButton.Enabled = true;
            OutputSelectionBox.Enabled = true;
            EnableSolPWM.Enabled = true;
            NightModeSetChkBox.Enabled = true;
            TimeAdjuster.Enabled = true;
            PWMAdjust.Enabled = true;
            TestButton.Enabled = true;
            ScanButton.Enabled = true;
            ResetButton.Enabled = true;
            DOFButton.Enabled = true;
            MemoryScanButton.Enabled = true;
        }
        private void DisconnectedState()
        {
            ConnectButton.BackColor = Color.DarkGray;
            ConnectButton.Text = "CONNECT";
            ConnectButton.Enabled = true;
            DiagConnectButton.BackColor = Color.DarkGray;
            DiagConnectButton.Text = "CONNECT";
            DiagConnectButton.Enabled = true;
            ComDisconnectButton.BackColor = Color.DarkRed;
            ComDisconnectButton.Text = "DISCONNECTED";
            ComDisconnectButton.Enabled = false;
            DiagDisconnectButton.BackColor = Color.DarkRed;
            DiagDisconnectButton.Text = "DISCONNECTED";
            DiagDisconnectButton.Enabled = false;
            SetButton.Enabled = false;
            SendButton.Enabled = false;
            SavedValueButton.Enabled = false;
            OutputSelectionBox.Enabled = false;
            EnableSolPWM.Enabled = false;
            NightModeSetChkBox.Enabled = false;
            TimeAdjuster.Enabled = false;
            PWMAdjust.Enabled = false;
            TestButton.Enabled = false;
            ScanButton.Enabled = false;
            ResetButton.Enabled = false;
            DOFButton.Text = "TEST";
            OutputActive = false;
            SliderRed.Value = 128;
            ValBoxRed.Text = "128";
            HexBoxRed.Text = "80";
            SliderGreen.Value = 128;
            ValBoxGreen.Text = "128";
            HexBoxGreen.Text = "80";
            SliderBlue.Value = 128;
            ValBoxBlue.Text = "128";
            HexBoxBlue.Text = "80";
            DOFButton.Enabled = false;
            MemoryScanButton.Enabled = false;
            PortFound = 0;
        }

        string DataIn;
        private void ComPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            DataIn = ComPort.ReadLine();
            if (DataIn != null)
            {
                if (PortFound != 0)
                {
                    this.Invoke(new EventHandler(ComPort_ShowData));
                }
                else
                {
                    DataIn = DataIn.Trim();
                    if (DataIn == "A" || DataIn == "F")
                    {
                        this.Invoke(new EventHandler(ComPort_AckReceived));
                    }
                }
            }
        }
        private void ComPort_ShowData(object sender, EventArgs e)
        {
            if (DataIn.Trim() != "A")
            {
                string[] incomingData = DataIn.Split(',');

                if (incomingData[0] == "M")                                // incoming Message
                {
                    DataIn = incomingData[1];
                    rtbDataIn.Text += DataIn;
                    rtbDiagData.Text = rtbDataIn.Text;
                }
                else if (incomingData[0] == "C")                           // incoming Memory Scan Report
                {
                    DataIn = incomingData[1];
                    rtbDiagData.Text += DataIn;
                }
                else if (incomingData[0] == "P")                           // incoming PWM Usage
                {
                    UsePwm = byte.Parse(incomingData[1].Trim());
                    EnableSolPWM.Checked = UsePwm != 0;
                    ComPort.Write("A");
                }
                else if (incomingData[0] == "H")                           // incoming Flipper Hold PWM Value
                {
                    FlipperHoldVal = byte.Parse(incomingData[1].Trim());
                    PWMAdjust.Text = Convert.ToString(FlipperHoldVal);
                    ComPort.Write("A");
                }
                else if (incomingData[0] == "N")                           // incoming Night Mode Assignments
                {
                    for (int a = 0; a < 64; a++)
                    {
                        NMSettings[a] = byte.Parse(incomingData[a + 1].Trim());
                    }
                    ComPort.Write("A");
                }
                else if (incomingData[0] == "T")                           // incoming Timer Assignments
                {
                    for (int a = 0; a < 64; a++)
                    {
                        TimerSettings[a] = byte.Parse(incomingData[a + 1].Trim());
                    }
                    SavedValueButton.PerformClick();
                    PortFound = 1;
                    rtbDataIn.Text += "\r\rSettings Load Complete.\r";
                    rtbDiagData.Text = rtbDataIn.Text;
                }
                else
                {
                    
                }
            }
            else
            {
                
            }
        }
        private void ComPort_AckReceived(object sender, EventArgs e)
        {
            if (DataIn == "A")
            {
                SendButton.BackColor = Color.DarkGray;
                SendButton.Text = "SAVING CONFIGURATION";
                rtbDataIn.Text += "..............................Sending Complete";
                PortFound = 1;
            }
            else if (DataIn == "F")
            {
                rtbDiagData.Text = "Factory Settings Restored";
                PortFound = 1;
                if (EmergencyReset == true)
                {
                    EmergencyReset = false;
                }
            }
        }
        private void ClearDataBox()
        {
            this.rtbDataIn.Clear();
            this.rtbDiagData.Clear();
        }

        Byte[] NMSettings = new Byte[64];
        Byte[] TimerSettings = new Byte[64];
        private void RetrieveSettings()
        {
            rtbDataIn.Text += "Retrieving Settings From Board...........Please wait until loading completes\r\rAdjust Output Number with UP/DOWN arrows.  Click Show Saved Values to show current settings for selected output.";
            ComPort.Write("Q");
        }
        private void ClearButton_Click_1(object sender, EventArgs e)
        {
            ClearDataBox();
        }
        public void ReceiveInput(bool ResetResponse)
        {
            if (ResetResponse == true)
            {
                rtbDiagData.Text = "Restoring Factory Defaults.....Please Wait";
                ComPort.Write("F");
            }
            else
            {
                rtbDiagData.Text = "RESET REQUEST CANCELLED";
            }
        }
        private void SetNightMode()
        {
            if (NightModeSetChkBox.Checked == true)
            {
                NMSettings[OutputSelected] = 0x01;
                rtbDataIn.Text += "\rNightMode Monitoring Applied to Output Number " + OutputSelectionBox.Text + " using a value of " + NMSettings[OutputSelected] + " \r";
            }
            else
            {
                NMSettings[OutputSelected] = 0x00;
                rtbDataIn.Text += "\rNightMode Monitoring Removed From Output Number " + OutputSelectionBox.Text + " using a value of " + NMSettings[OutputSelected] + " \r";
            }
        }
        private void SetTimerValues()
        {
            if (TimeAdjuster.Text != "0")
            {
                int TimerTemp = Convert.ToInt32(TimeAdjuster.Text) / 60;
                TimerSettings[OutputSelected] = Convert.ToByte(TimerTemp);
                Double ValOut = Convert.ToDouble(TimeAdjuster.Text) / 1000;
                rtbDataIn.Text += "Timer value of " + TimeAdjuster.Text + " ms / " + Convert.ToString(ValOut) + " second(s) applied to Output Number " + OutputSelectionBox.Text + "\r";
            }
            else
            {
                TimerSettings[OutputSelected] = 0;
                rtbDataIn.Text += "Timer usage turned OFF on Output Number " + OutputSelectionBox.Text + "\r";
            }
        }

        int OutputSelected;
        private void SetButton_Click_1(object sender, EventArgs e)
        {
            ClearDataBox();
            SetButton.BackColor = Color.Green;
            if (EnableSolPWM.Checked == true)
            {
                UsePwm = 1;
                rtbDataIn.Text += "\rFlipper PWM control Enabled";
            }
            else
            {
                UsePwm = 0;
                rtbDataIn.Text += "\rFlipper PWM control Disabled";
            }
            rtbDataIn.Text += "\rFlipper PWM set to ";
            rtbDataIn.Text += PWMAdjust.Text;
            rtbDataIn.Text += " out of 255 possible counts\r";
            FlipperHoldVal = byte.Parse(PWMAdjust.Text);
            OutputSelected = Convert.ToInt32(OutputSelectionBox.Text);
            OutputSelected--;
            SetNightMode();
            SetTimerValues();
            SetButton.BackColor = Color.DarkGray;
        }
        private void SendButton_Click_1(object sender, EventArgs e)
        {
            PortFound = 0;
            SendButton.BackColor = Color.DarkRed;
            SendButton.Text = "SENDING";
            ClearDataBox();
            rtbDataIn.Text += "Sending Settings to Board";
            byte[] buf = new byte[131];
            buf[0] = 0x52;                                                          // 'R' denotes settings data coming in
            buf[1] = UsePwm;
            buf[2] = FlipperHoldVal;                                                //  FlipperHoldVal is the PWM signal used 
            for (int a = 0; a < 64; a++)
            {
                buf[a + 3] = NMSettings[a];
                buf[a + 67] = TimerSettings[a];
                Task.Delay(1).Wait();
            }
            ComPort.Write(buf, 0, buf.Length);
        }
        private void SavedValueButton_Click_1(object sender, EventArgs e)
        {
            int CurrentSelection = Convert.ToInt32(OutputSelectionBox.Text);
            CurrentSelection--;
            EnableSolPWM.Checked = UsePwm != 0;
            PWMAdjust.Text = Convert.ToString(FlipperHoldVal);
            NightModeSetChkBox.Checked = NMSettings[CurrentSelection] != 0;
            int TempTimer = TimerSettings[CurrentSelection] * 60;
            TimeAdjuster.Text = Convert.ToString(TempTimer);
        }
        private void EnableSolPWM_CheckedChanged_1(object sender, EventArgs e)
        {
            rtbDataIn.Text += "\rClick Set Button to lock in any changes.  Click Save Configuration to save configuration to board before exiting";
        }

        private void cbOutputControl_FormClosed(object sender, FormClosedEventArgs e)
        {
            PortFound = 0;
            ComPort.Close();
            Menu formMenu = new Menu();
            formMenu.Show();
        }

        // Diagnostics & Test Tab

        private void DiagConnectButton_Click(object sender, EventArgs e)
        {
            ClearDataBox();
            rtbDiagData.Text = "Connecting without Settings Retrieval (Troubleshooting Mode)\r";
            EmergencyReset = true;
            PortFound = 0;
            ConnectToController("U", "41C5");
        }
        private void DiagDisconnectButton_Click(object sender, EventArgs e)
        {
            if (ComPort.IsOpen)
            {
                ClearDataBox();
                DisconnectedState();
                ComPort.Close();
            }
            else
            {
                ClearDataBox();
                DisconnectedState();
            }
        }
        private void ResetButton_Click_1(object sender, EventArgs e)
        {
            if (ComPort.IsOpen)
            {
                PortFound = 0;
                warningPopup = new WarningPopup(transferDelegate);
                warningPopup.ShowDialog();
            }
            else
            {
                EmergencyReset = true;
                ConnectToController("U", "41C5");
                if (ComPort.IsOpen)
                {
                    PortFound = 0;
                    warningPopup = new WarningPopup(transferDelegate);
                    warningPopup.ShowDialog();
                }
            }

        }
        private void TestButton_Click_1(object sender, EventArgs e)
        {
            ClearDataBox();
            PortFound = 0;
            ComPort.Write("T");
            PortFound = 1;
        }
        private void ScanButton_Click_1(object sender, EventArgs e)
        {
            ClearDataBox();
            rtbDiagData.Text = "Requesting Scan\r";
            if (ComPort.IsOpen)
            {
                ComPort.Write("S");
                rtbDiagData.Text += "Scanning i2c bus for device(s)...\r\r";
            }
            else
            {
                rtbDiagData.Text += "!! Port NOT Open\r";
            }
        }
        private void MemoryScanButton_Click(object sender, EventArgs e)
        {
            ClearDataBox();
            ComPort.Write("X");
        }
        private void ClearDiagBoxButton_Click(object sender, EventArgs e)
        {
            ClearDataBox();
        }
        private void SliderRed_Scroll(object sender, EventArgs e)
        {
            ValBoxRed.Text = SliderRed.Value.ToString();
        }
        private void SliderGreen_Scroll(object sender, EventArgs e)
        {
            ValBoxGreen.Text = SliderGreen.Value.ToString();
        }
        private void SliderBlue_Scroll(object sender, EventArgs e)
        {
            ValBoxBlue.Text = SliderBlue.Value.ToString();
        }
        private void ValBoxRed_TextChanged(object sender, EventArgs e)
        {
            int Val = Convert.ToInt32(SliderRed.Value);
            if (Val < 16)
            {
                HexBoxRed.Text = "0" + Val.ToString("X");
            }
            else
            {
                HexBoxRed.Text = Val.ToString("X");
            }
            if (Val < 256)
            {
                Hexbox();
            }
        }
        private void ValBoxGreen_TextChanged(object sender, EventArgs e)
        {
            int Val = Convert.ToInt32(SliderGreen.Value);
            if (Val < 16)
            {
                HexBoxGreen.Text = "0" + Val.ToString("X");
            }
            else
            {
                HexBoxGreen.Text = Val.ToString("X");
            }
            if (Val < 256)
            {
                Hexbox();
            }
        }
        private void ValBoxBlue_TextChanged(object sender, EventArgs e)
        {
            int Val = Convert.ToInt32(SliderBlue.Value);
            if (Val < 16)
            {
                HexBoxBlue.Text = "0" + Val.ToString("X");
            }
            else
            {
                HexBoxBlue.Text = Val.ToString("X");
            }
            if (Val < 256)
            {
                Hexbox();
            }
            
        }
        private void HexBoxRed_TextChanged(object sender, EventArgs e)
        {
            if (HexBoxRed.TextLength < 3)
            {
                int Val = int.Parse(HexBoxRed.Text, System.Globalization.NumberStyles.HexNumber);
                if (Val < 256)
                {
                    SliderRed.Value = Val;
                    Hexbox();
                }
            }
            
            
        }
        private void HexBoxGreen_TextChanged(object sender, EventArgs e)
        {
            if (HexBoxGreen.TextLength < 3)
            {
                int Val = int.Parse(HexBoxGreen.Text, System.Globalization.NumberStyles.HexNumber);
                if (Val < 256)
                {
                    SliderGreen.Value = Val;
                    Hexbox();
                }
            }
        }
        private void HexBoxBlue_TextChanged(object sender, EventArgs e)
        {
            if (HexBoxBlue.TextLength < 3)
            {
                int Val = int.Parse(HexBoxBlue.Text, System.Globalization.NumberStyles.HexNumber);
                if (Val < 256)
                {
                    SliderBlue.Value = Val;
                    Hexbox();
                }
            }
        }
        private void Hexbox()
        {
            HexOutputBox.Text = "#" + HexBoxRed.Text + HexBoxGreen.Text + HexBoxBlue.Text + "FF";
            ValBoxRed.Text = SliderRed.Value.ToString();
            ValBoxGreen.Text = SliderGreen.Value.ToString();
            ValBoxBlue.Text = SliderBlue.Value.ToString();
        }
        private void ColorEntryBox_TextChanged(object sender, EventArgs e)
        {
            string[] EntryVals = new string[3];
            if ( ColorEntryBox.TextLength > 7)
            {
                EntryVals = ColorEntryBox.Text.Split(',');
                HexBoxRed.Text = EntryVals[0];
                HexBoxGreen.Text = EntryVals[1];
                HexBoxBlue.Text = EntryVals[2];
            }
        }

        int NumberOfBanks = 1;
        byte[] BankId = new byte[3];
        bool OutputActive;
        bool Bank1and2Match;
        private void DOFButton_Click(object sender, EventArgs e)
        {
            OutputActive = !OutputActive;
            if (OutputActive != true)
            {
                DOFButton.BackColor = Color.SkyBlue;
                DOFButton.Text = "TEST";
            }
            else
            {
                DOFButton.BackColor = Color.DarkRed;
                DOFButton.Text = "STOP";
            }
            ClearDataBox();
            byte ON = 255;
            byte OFF = 0;
            if (UseRGBChecked.Checked == false)
            {
                BankId[0] = (byte)(((Convert.ToByte(OutputSelectBox.Value) - 1) / 8) + 200);
            }
            else
            {
                BankId[0] = (byte)(((Convert.ToByte(OutputSelectBox.Value) - 1) / 8) + 200);
                BankId[1] = (byte)((Convert.ToByte(OutputSelectBox.Value) / 8) + 200);
                BankId[2] = (byte)(((Convert.ToByte(OutputSelectBox.Value) + 1) / 8) + 200);
                if (BankId[0] != BankId[1] || BankId[0] != BankId[2])
                {
                    NumberOfBanks = 2;
                    if (BankId[0] == BankId[1])
                    {
                        Bank1and2Match = true;
                    }
                    else
                    {
                        Bank1and2Match = false;
                    }
                    BankId[1] = BankId[2];
                }
                else
                {
                    NumberOfBanks = 1;
                }
            }
            for (int a = 0; a < NumberOfBanks; a++)
            {
                rtbDiagData.Text += "BankID  " + BankId[a].ToString() + "\r";
            }
            rtbDiagData.Text += "\rSending  ";
            byte[] buf = new byte[10];
            buf[0] = 0x4F;             // 'O' denotes output data coming in
            rtbDiagData.Text += buf[0].ToString() + ",";
            for (int a = 0; a < NumberOfBanks; a++)
            {
                buf[1] = BankId[a];           // BankId of the data to be processed
                if (a == 0)
                {
                    rtbDiagData.Text += buf[1].ToString() + ",";
                }
                else
                {
                    rtbDiagData.Text += "Sending  " + buf[0] + "," + buf[1].ToString() + ",";
                }

                for (int i = 2; i < 10; i++)
                {
                    int OutputPositionTmp = (Convert.ToInt32(OutputSelectBox.Value) - 1) - ((BankId[a] - 200) * 8);
                    if (OutputPositionTmp < 0)
                    {
                        OutputPositionTmp = 0;
                    }
                    if (i - 2 == OutputPositionTmp && UseRGBChecked.Checked == false)     //  if we found the port to test and it ISN'T RGB
                    {
                        if (OutputActive != false)
                        {
                            buf[i] = ON;
                        }
                        else
                        {
                            buf[i] = OFF;
                        }
                    }
                    else if (i - 2 == OutputPositionTmp && UseRGBChecked.Checked == true)     //  if we found the port to test and it IS RGB
                    {
                        if (OutputActive != false)
                        {
                            if (NumberOfBanks == 1)
                            {
                                buf[i] = Convert.ToByte(SliderRed.Value);
                                buf[i + 1] = Convert.ToByte(SliderGreen.Value);
                                buf[i + 2] = Convert.ToByte(SliderBlue.Value);
                                rtbDiagData.Text += buf[i].ToString() + ", " + buf[i + 1].ToString() + ", ";
                                i += 2;
                            }
                            else
                            {
                                if (Bank1and2Match == true && a == 0)
                                {
                                    buf[i] = Convert.ToByte(SliderRed.Value);
                                    rtbDiagData.Text += buf[i].ToString() + ", ";
                                    buf[i + 1] = Convert.ToByte(SliderGreen.Value);
                                    i++;
                                }
                                else if (Bank1and2Match == true && a == 1)
                                {
                                    buf[i] = Convert.ToByte(SliderBlue.Value);
                                }
                                else if (Bank1and2Match == false && a == 1)
                                {
                                    buf[i] = Convert.ToByte(SliderGreen.Value);
                                    rtbDiagData.Text += buf[i].ToString() + ", ";
                                    buf[i + 1] = Convert.ToByte(SliderBlue.Value);
                                    //i++;
                                }
                                else
                                {
                                    buf[i] = Convert.ToByte(SliderRed.Value);
                                }
                            }

                        }
                        else
                        {
                            if (Bank1and2Match == true && a == 0)
                            {
                                buf[i] = OFF;
                                rtbDiagData.Text += buf[i].ToString() + ", ";
                                buf[i + 1] = OFF;
                                i++;
                            }
                            else if (Bank1and2Match == true && a == 1)
                            {
                                buf[i] = OFF;
                            }
                            else if (Bank1and2Match == false && a == 1)
                            {
                                buf[i] = OFF;
                                rtbDiagData.Text += buf[i].ToString() + ", ";
                                buf[i + 1] = OFF;
                                //i++;
                            }
                            else
                            {
                                buf[i] = OFF;
                            }
                        }
                    }
                    else
                    {
                        buf[i] = OFF;
                    }
                    rtbDiagData.Text += buf[i].ToString() + ", ";
                }
                rtbDiagData.Text += "\r";
                ComPort.Write(buf, 0, buf.Length);
            }
        }

    }
}
