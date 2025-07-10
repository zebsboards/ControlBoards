using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Configuration_Utility
{
    public partial class cbPlungerControl : Form
    {
        public cbPlungerControl()
        {
            InitializeComponent();
        }
        byte[] byteSettings = new byte[25];
        byte[] PrevByteSettings = new byte[25];
        byte[] Keys = new byte[32];
        byte[] PrevKeys = new byte[32];
        byte[] Labels = new byte[32];
        Int16[] LargeVals = new short[2];
        Int16[] PrevLargeVals = new short[2];
        bool UpdateByteSettings = false;
        bool UpdateKeys = false;
        bool UpdatePlungerVals = false;
        Byte[] LabelCodes =  { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
                               0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                               0x18, 0x19, 0x1A, 0x1B };
        string[] LabelCodeNames = { "Not Assigned", "L Flipper", "R Flipper", "L Magnasave", "R Magnasave", "Start", "Exit", "Buy In", "Coin 1", "Coin 2", "Launch Ball",
                                    "Fire", "Coin Door", "Pause", "Volume Up", "Volume Down", "Menu Up", "Menu Down", "Menu Enter", "Menu Cancel", "L Nudge", "R Nudge",
                                    "F Nudge", "Tilt", "HSW1 U", "HSW1 R", "HSW1 D", "HSW1 L" };
        byte[] AsciiCodes =  { 0x00, 0xE1, 0xE0, 0xE2, 0xE3, 0xE5, 0xE4, 0xE6, 0xE7, 0x04, 0x05, 0x06,
                               0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
                               0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
                               0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2B,
                               0x2C, 0x2D, 0x2E, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51,
                               0x52, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45 };
        string[] AsciiNames = { "NOT_ASSIGNED", "SHIFT_LEFT", "CRTL_LEFT", "ALT_LEFT", "GUI_LEFT", "SHIFT_RIGHT", "CTRL_RIGHT", "ALT_RIGHT", "GUI_RIGHT", "a", "b",
                                "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z", "1", "2", "3",
                                "4", "5", "6", "7", "8", "9", "0", "ENTER", "ESCAPE", "TAB", "SPACE", "MINUS", "EQUAL", "INSERT", "HOME", "PAGE_UP", "DELETE", "END",
                                "PAGE_DOWN", "ARROW_RIGHT", "ARROW_LEFT", "ARROW_DOWN", "ARROW_UP", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12" };

        private void cbPlungerControl_Load(object sender, EventArgs e)
        {
            Btn4g.Checked = true;
            OrientBox.Text = "CFcableBackFlat";
            xDzTbar.Value = 0;
            yDzTbar.Value = 0;
            SendBtn.BackColor = Color.Silver;
            SendBtn.Text = "Save Settings\nto Plunger";
            DisconnectedState();
            UseHSbox.Enabled = false;
        }
        private void cbPlungerControl_FormClosed(object sender, FormClosedEventArgs e)
        {
            Menu formMenu = new Menu();
            formMenu.Show();
        }
        private void DisconnectedState()
        {
            ComConnectButton.BackColor = Color.DarkGray;
            ComConnectButton.Text = "CONNECT";
            ComConnectButton.Enabled = true;
            ButtonsDisabled();
            ClearDataBox();
            DataIn = "\r\r\rClick Connect Button to Connect to Controller ";
            RtbDataIn.Text = DataIn;
            RtbDataIn.SelectAll();
            RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
            RtbDataIn.DeselectAll();
        }
        private void ButtonsDisabled()
        {
            ComDisconnectButton.BackColor = Color.DarkRed;
            ComDisconnectButton.Text = "DISCONNECTED";
            ComDisconnectButton.Enabled = false;
            ReceiveBtn.Enabled = false;
            SendBtn.Enabled = false;
            ClearFlashBtn.Enabled = false;
            ResetBtn.Enabled = false;
            groupBox1.Enabled = false;
            groupBox5.Enabled = false;
            groupBox6.Enabled = false;
            groupBox7.Enabled = false;
            groupBox8.Enabled = false;
            groupBox9.Enabled = false;
            groupBox10.Enabled = false;
        }
        private void ButtonsEnabled()
        {
            ComDisconnectButton.BackColor = Color.DarkGray;
            ComDisconnectButton.Text = "DISCONNECT";
            ComDisconnectButton.Enabled = true;
            ReceiveBtn.Enabled = true;
            SendBtn.Enabled = true;
            ClearFlashBtn.Enabled = true;
            ResetBtn.Enabled = true;
            groupBox1.Enabled = true;
            groupBox5.Enabled = true;
            groupBox6.Enabled = true;
            groupBox7.Enabled = true;
            groupBox8.Enabled = true;
            groupBox9.Enabled = true;
            groupBox10.Enabled = true;
        }
        private void ConnectedState()
        {
            DataIn = null;
            if (PortFound != 1)
            {
                RtbDataIn.Text = "\r\rNo Plunger Found";
                RtbDataIn.SelectAll();
                RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
            }
            else
            {
                RtbDataIn.Text = "\r\r\rConnected to\rcabBlaster Plunger";
                RtbDataIn.SelectAll();
                RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
            }
            ComConnectButton.BackColor = Color.Green;
            ComConnectButton.Text = "CONNECTED";
            ComConnectButton.Enabled = false;
            ButtonsEnabled();
            ReceiveBtn.PerformClick();
        }

        /// <summary>
        /// Start Tab Controls
        /// </summary>

        int PortFound;
        private void ComConnectButton_Click(object sender, EventArgs e)
        {
            ClearDataBox();
            ConnectToController("v", "ZBV7");
            
        }
        private void ConnectToController(string CommandCode, string VerificationID)
        {
            string[] ports = SerialPort.GetPortNames();
            ComPortBox.Items.AddRange(ports);
            ComPort.DataReceived += new SerialDataReceivedEventHandler(ComPort_DataReceived);
            foreach (string sp in SerialPort.GetPortNames())
            {
                try
                {
                    ComPortBox.Text = sp;
                    ComPort.PortName = sp;
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
                                DataIn = DataIn.Trim();
                                string[] reply = DataIn.Split(',');
                                if ( reply [0] == VerificationID)
                                {
                                    PlungeName.Text = reply[0];
                                    PlungerRev.Text = reply[1];
                                    PortFound = 1;
                                    break;
                                }
                                else
                                {
                                    ComPort.Close();
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
                        MessageBox.Show(err.Message, "Incorrect Port .. Click Ok to Try Another", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                }
            }
            if (PortFound != 0)
            {
                ConnectedState();
            }
            else
            {
                RtbDataIn.Text = "\r\rController Not Found.\rReccomend REBOOTING Controller and RESTART the Utility";
                RtbDataIn.SelectAll();
                RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
            }
        }
        private void ComDisconnectButton_Click_1(object sender, EventArgs e)
        {
            // *********************************** Send clear all Leds command before closing
            if (ComPort.IsOpen)
            {
                DisconnectedState();
                ComPort.Close();
            }
            else
            {
                DisconnectedState();
            }
        }
        
        bool LoadSettingsFlag;
        private void ReceiveBtn_Click(object sender, EventArgs e)
        {
            ComPort.Write("ES");
            LoadSettingsFlag = true;
            ReceiveBtn.Text = "Reload Settings";
        }
        private void SendBtn_Click(object sender, EventArgs e)
        {
            if (UpdateByteSettings == true || UpdateKeys == true || UpdatePlungerVals == true)
            {
                ButtonsDisabled();
                SendSettings();
                UpdateByteSettings = false; UpdateKeys = false; UpdatePlungerVals = false;
            }
            else
            {
                RtbDataIn.Text = "No Changes to Send";
            }
            
        }
        private void SendSettings()
        {
            RtbDataIn.Text = "Sending Settings ... Please Wait \r\n ... Option Settings\r\n";
            rtbDataAxis.Text = "Sending Settings ... Please Wait \r\n ... Option Settings\r\n";
            ComPort.Write("ER" + byteSettings[0] + ",");
            for (int a = 1; a < 23; a++)
            {
                ComPort.Write(byteSettings[a] + ",");
            }
            ComPort.WriteLine(byteSettings[23] + "," + byteSettings[24]);
            for (int a = 0; a < 25; a++)
            {
                PrevByteSettings[a] = byteSettings[a];
            }
        }
        private void ClearFlashBtn_Click(object sender, EventArgs e)
        {
            ComPort.Write("EE");
            ButtonsDisabled();
            UpdateByteSettings = true; UpdateKeys = true; UpdatePlungerVals = true;
        }
        private void ResetBtn_Click(object sender, EventArgs e)
        {
            RtbDataIn.Text = "Resetting Flash Memory to Factory Settings\r\nPlease Stand By";
            ComPort.Write("EF");
            ButtonsDisabled();
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

                }
            }
            
        }
        private void ComPort_ShowData(object sender, EventArgs e)
        {
            if (DataIn != null)
            {
                DataIn = DataIn.Trim();
                string[] incomingData = DataIn.Split(',');
                if (incomingData[0] == "A")
                {
                    if (incomingData[1] == "R")
                    {
                        RtbDataIn.AppendText(" ... Key Assignments\r\n");
                        rtbDataAxis.AppendText(" ... Key Assignments\r\n");
                        ComPort.Write("EK" + Keys[0] + ",");
                        PrevKeys[0] = Keys[0];
                        for (int a = 1; a < 30; a++)
                        {
                            ComPort.Write(Keys[a] + ",");
                            PrevKeys[a] = Keys[a];
                        }
                        ComPort.Write(Keys[30] + "," + Keys[31]);
                        PrevKeys[30] = Keys[30]; PrevKeys[31] = Keys[31];
                    }
                    else if (incomingData[1] == "K")
                    {
                        RtbDataIn.AppendText(" ... Label Assignments\r\n");
                        rtbDataAxis.AppendText(" ... Label Assignments\r\n");
                        ComPort.Write("EL" + Labels[0] + ",");
                        for (int a = 1; a < 30; a++)
                        {
                            ComPort.Write(Labels[a] + ",");
                        }
                        ComPort.Write(Labels[30] + "," + Labels[31]);
                    }
                    else if (incomingData[1] == "L")
                    {
                        RtbDataIn.AppendText(" ... Plunger Counts\r\n");
                        rtbDataAxis.AppendText(" ... Plunger Counts\r\n");
                        ComPort.Write("EP" + LargeVals[0] + "," + LargeVals[1]);
                        PrevLargeVals[0] = LargeVals[0]; PrevLargeVals[1] = LargeVals[1];

                    }
                    else if (incomingData[1] == "P")
                    {
                        RtbDataIn.AppendText(" Transfer Successfully Saved\r\n");
                        rtbDataAxis.AppendText(" Transfer Successfully Saved\r\n");
                        UpdateByteSettings = false; UpdateKeys = false; UpdatePlungerVals = false;
                        SendBtn.BackColor = Color.Silver;
                        SendBtn.Text = "Save Settings\nto Plunger";
                        ButtonsEnabled();
                    }
                    else if (incomingData[1] == "B")                        // Factory reset routine requests reload of values from plunger
                    {
                        ButtonsEnabled();
                        ReceiveBtn.PerformClick();
                    }
                    else if (incomingData[1] == "D")
                    {
                        ButtonsEnabled();
                    }
                }
                else if (incomingData[0] == "B")
                {
                    AtRestReadingBox.Text = incomingData[1];                        // incoming At Rest Adjustment Reading - encoder strip alignment
                    DataIn = null;
                }
                else if (incomingData[0] == "C")                                // incoming 16 bit values (Push counts / Pull counts) 
                {
                    RtbDataIn.AppendText("   16bit Values ...\r\n");
                    for (int a = 1; a < 3; a++)
                    {
                        LargeVals[a - 1] = Int16.Parse(incomingData[a]);
                        PrevLargeVals[a - 1] = LargeVals[a - 1];
                    }
                    AssignValuesToBoxes();
                }
                else if (incomingData[0] == "K")                           // incoming keys array values from settings retrieval
                {
                    RtbDataIn.AppendText("   Keys ...\r\n");
                    for (int a = 1; a < 33; a++)
                    {
                        Keys[a - 1] = Convert.ToByte(incomingData[a]);
                        PrevKeys[a - 1] = Convert.ToByte(incomingData[a]);
                    }
                    DataIn = null;
                    ComPort.Write("A");
                }
                else if (incomingData[0] == "L")                           // incoming label array values from settings retrieval
                {
                    RtbDataIn.AppendText("   Labels ...\r\n");
                    for (int a = 1; a < 33; a++)
                    {
                        Labels[a - 1] = Convert.ToByte(incomingData[a]);
                    }
                    DataIn = null;
                    ComPort.Write("A");
                }
                else if (incomingData[0] == "M")                           // incoming Message
                {
                    RtbDataIn.Text = incomingData[1];
                    rtbDataAxis.Text = incomingData[1];
                    DataIn = null;
                    ComPort.Write("A");
                }
                else if (incomingData[0] == "P")                           // incoming Pull calibration Reading
                {
                    PullCountsBox.Text = incomingData[1];
                    LargeVals[0] = Convert.ToInt16(PullCountsBox.Text);
                    DataIn = null;
                    ComPort.Write("A");
                }
                else if (incomingData[0] == "Q")
                {
                    AtRestReadingBox.Text = incomingData[1];                        // incoming At Rest calibration Reading
                    DataIn = null;
                    ComPort.Write("A");
                }
                else if (incomingData[0] == "R")
                {
                    PushCountsBox.Text = incomingData[1];                           // incoming Push calibration Reading
                    LargeVals[1] = Convert.ToInt16(PushCountsBox.Text);
                    DataIn = null;
                    ComPort.Write("A");
                    SendBtn.BackColor = Color.Red;
                    SendBtn.Text = "Click to\r\nSave Changes";
                }
                else if (incomingData[0] == "S")                                   // incoming byteSettings values from settings retrieval
                {
                    RtbDataIn.Text = "RECEIVING SETTINGS, PLEASE WAIT... \r\n   Configs ...\r\n";
                    for (int a = 1; a < 20; a++)
                    {
                        byteSettings[a - 1] = Convert.ToByte(incomingData[a]);
                        PrevByteSettings[a - 1] = Convert.ToByte(incomingData[a]);
                    }
                    DataIn = null;
                    ComPort.Write("A");
                }
            }
            else 
            {
                
            }
        }
        private void AssignValuesToBoxes()
        {
            switch (byteSettings[0])
            {
                case 0:
                    GamepadBox.Checked = false;
                    break;
                case 1:
                    GamepadBox.Checked = true;
                    break;
            }
            switch (byteSettings[1])
            {
                case 0:
                    UseCDbox.Checked = false;
                    break;
                case 1:
                    UseCDbox.Checked = true;
                    break;
            }
            CDbuttonBox.Text = Convert.ToString(byteSettings[2]);
            LBbuttonBox.Text = Convert.ToString(byteSettings[3]);
            LBkeypress.Text = byteSettings[4].ToString("X2");
            switch (byteSettings[5])
            {
                case 0:
                    UseHSbox.Checked = false;
                    break;
                case 1:
                    UseHSbox.Checked = true;
                    break;
            }
            switch (byteSettings[6])
            {
                case 0:
                    UseNudgeBox.Checked = false;
                    break;
                case 1:
                    UseNudgeBox.Checked = true;
                    break;
            }
            switch (byteSettings[7])
            {
                case 2:
                    Btn2g.Checked = true;
                    break;
                case 4:
                    Btn4g.Checked = true;
                    break;
                case 8:
                    Btn8g.Checked = true;
                    break;
                case 16:
                    Btn16g.Checked = true;
                    break;
            }
            switch (byteSettings[8])
            {
                case 0:
                    OrientBox.SelectedItem = 0;
                    break;
                case 1:
                    OrientBox.SelectedItem = 1;
                    break;
                case 2:
                    OrientBox.SelectedItem = 2;
                    break;
                case 3:
                    OrientBox.SelectedItem = 3;
                    break;
                case 4:
                    OrientBox.SelectedItem = 4;
                    break;
                case 5:
                    OrientBox.SelectedItem = 5;
                    break;
                case 6:
                    OrientBox.SelectedItem = 6;
                    break;
                case 7:
                    OrientBox.SelectedItem = 7;
                    break;
            }
            Debounce.Value = byteSettings[9];
            XgainBar.Value = byteSettings[11]; xGainlabel.Text = Convert.ToString(XgainBar.Value);
            YgainBar.Value = byteSettings[12]; YgainVal.Text = Convert.ToString(YgainBar.Value);
            xDzTbar.Value = byteSettings[13]; int xdzone = xDzTbar.Value * 256; xDZval.Text = Convert.ToString(xdzone);
            yDzTbar.Value = byteSettings[14]; int ydzone = xDzTbar.Value * 256; yDZval.Text = Convert.ToString(ydzone);
            switch (byteSettings[15])
            {
                case 0:
                    UseTiltBox.Checked = false;
                    break;
                case 1:
                    UseTiltBox.Checked = true;
                    break;
            }
            TiltButtonBox.Text = Convert.ToString(byteSettings[16]);
            XtiltVal.Value = byteSettings[17];  XtVal.Text = Convert.ToString(((XtiltVal.Value -127) * 256));
            YtiltVal.Value = byteSettings[18]; YtVal.Text = Convert.ToString(((YtiltVal.Value - 127) * 256));

            PullCountsBox.Text = Convert.ToString(LargeVals[0]);
            PushCountsBox.Text = Convert.ToString(LargeVals[1]);

            ConvertKeyArray();
            ConvertLabelArray();

            RtbDataIn.AppendText("Settings Received and Screens Updated");
            if (LoadSettingsFlag == true)
            {
                UpdateByteSettings = false; UpdateKeys = false; UpdatePlungerVals = false;
                LoadSettingsFlag = false;
            }

            UpdateConnectionLables();
        }
        private void ConvertKeyArray() 
        {
            KeysBox.Text = "";
            for (int a = 0; a < 32; a++)
            {
                KeysBox.AppendText(Keys[a].ToString("X2"));
                if (a < 31)
                {
                    KeysBox.AppendText(",");
                }
            }
        }
        private void ConvertLabelArray()
        {
            LabelsBox.Text = "";
            for (int a = 0; a < 32; a++)
            {
                LabelsBox.AppendText(Labels[a].ToString("X2"));
                if (a < 31)
                {
                    LabelsBox.AppendText(",");
                }
            }
        }
        private void CompareByteArray()
        {
            int count = 0;
            for (int a = 0; a < 25; a++)
            {
                if (byteSettings[a] != PrevByteSettings[a])
                {
                    count++;
                }
                if (count > 0)
                {
                    UpdateByteSettings = true;
                    SendBtn.BackColor = Color.Red;
                    SendBtn.Text = "Click to\r\nSave Changes";
                }
                else
                {
                    UpdateByteSettings = false;
                    if (UpdateKeys == false & UpdatePlungerVals == false)
                    {
                        SendBtn.BackColor = Color.Silver;
                        SendBtn.Text = "Save Settings\nto Plunger";
                    }
                }
            }
        }
        private void CompareKeysArray()
        {
            int count = 0;
            for (int a = 0; a < 32; a++)
            {
                if (Keys[a] != PrevKeys[a])
                {
                    count++;
                }
                if (count > 0)
                {
                    UpdateKeys = true;
                    SendBtn.BackColor = Color.Red;
                    SendBtn.Text = "Click to\r\nSave Changes";
                }
                else
                {
                    UpdateKeys = false;
                    if (UpdateByteSettings == false & UpdatePlungerVals == false)
                    {
                        SendBtn.BackColor = Color.Silver;
                        SendBtn.Text = "Save Settings\nto Plunger";
                    }
                }
            }
        }
        private void UpdateConnectionLables()
        {
            Label[] KeyLabels = new Label[] { keyass1, keyass2, keyass3, keyass4, keyass5, keyass6, keyass7, keyass8, keyass9, keyass10, keyass11, keyass12, keyass13, keyass14,
                               keyass15, keyass16, keyass17, keyass18, keyass19, keyass20, keyass21, keyass22, keyass23, keyass24, keyass25, keyass26, keyass27, keyass28, keyass29, keyass30, keyass31, keyass32 };
            Label[] UseLabels = new Label[] { keyLbl1, keyLbl2, keyLbl3, keyLbl4, keyLbl5, keyLbl6, keyLbl7, keyLbl8, keyLbl9, keyLbl10, keyLbl11, keyLbl12, keyLbl13, keyLbl14,
                               keyLbl15, keyLbl16, keyLbl17, keyLbl18, keyLbl19, keyLbl20, keyLbl21, keyLbl22, keyLbl23, keyLbl24, keyLbl25, keyLbl26, keyLbl27, keyLbl28, keyLbl29, keyLbl30, keyLbl31, keyLbl32 };
            for (int a = 0; a < 32; a++)
            {
                for (int b = 0; b < 73; b++)
                {
                    if (Keys[a] == AsciiCodes[b])
                    {
                        KeyLabels[a].Text = AsciiNames[b];
                    }
                }
                for (int c = 0; c < 28; c++)
                {
                    if (Labels[a] == LabelCodes[c])
                    {
                        UseLabels[a].Text = LabelCodeNames[c];
                    }
                }
            }
        }
        private void ClearDataBox()
        {
            this.RtbDataIn.Clear();
        }

        /// <summary>
        /// Axis Configuration Tab
        /// </summary>
        private void CalButton_Click_1(object sender, EventArgs e)
        {
            string SendData = "P";
            if (ComPort.IsOpen)
            {
                try
                {
                    ComPort.Write(SendData);
                    UpdatePlungerVals = true;                                      // Plunger calibrated set update flag to force saving    
                }
                catch (Exception err)
                {
                }
            }
        }
        bool ON = false;
        private void EncoderAlignBtn_Click_1(object sender, EventArgs e)
        {
            if (ON != true)
            {
                EncoderAlignBtn.Text = "Click To END";
                rtbDataAxis.Text = "Rotate encoder gear\r\nto about 582 counts in the Resting box for cabinets made out of" +
                    "\r\n 3/4in material.\r\n\r\nResting box should read 1023 with the rack\r\ninstalled when properly calibrated.";
            }
            else
            {
                EncoderAlignBtn.Text = "Align Encoder";
            }
            ON = !ON;
            if (ComPort.IsOpen)
            {
                try
                {
                    ComPort.Write("B");
                }
                catch (Exception err)
                {
                }
            }
        }
        private void UseNudgeBox_CheckedChanged(object sender, EventArgs e)
        {
            if (UseNudgeBox.Checked == true)
            {
                byteSettings[6] = 1;
            }
            else
            {
                byteSettings[6] = 0;
            }
            CompareByteArray();
        }
        private void Btn2g_CheckedChanged(object sender, EventArgs e)
        {
            if (Btn2g.Checked == true)
            {
                byteSettings[7] = 2;
                Btn4g.Checked = false; Btn8g.Checked = false; Btn16g.Checked = false;
            }
            CompareByteArray();
        }
        private void Btn4g_CheckedChanged(object sender, EventArgs e)
        {
            if (Btn4g.Checked == true)
            {
                byteSettings[7] = 4;
                Btn2g.Checked = false; Btn8g.Checked = false; Btn16g.Checked = false;
            }
            CompareByteArray();
        }
        private void Btn8g_CheckedChanged(object sender, EventArgs e)
        {
            if (Btn8g.Checked == true)
            {
                byteSettings[7] = 8;
                Btn2g.Checked = false; Btn4g.Checked = false; Btn16g.Checked = false;
            }
            CompareByteArray();
        }
        private void Btn16g_CheckedChanged_1(object sender, EventArgs e)
        {
            if (Btn16g.Checked == true)
            {
                byteSettings[7] = 16;
                Btn2g.Checked = false; Btn4g.Checked = false; Btn8g.Checked = false;
            }
            CompareByteArray();
        }
        private void UseTiltBox_CheckedChanged(object sender, EventArgs e)
        {
            if (UseTiltBox.Checked == true)
            {
                byteSettings[15] = 1;
            }
            else
            {
                byteSettings[15] = 0;
            }
            CompareByteArray();
        }
        private void OrientBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (OrientBox.SelectedItem)
            {
                case 0:
                    byteSettings[8] = 0;
                    break;
                case 1:
                    byteSettings[8] = 1;
                    break;
                case 2:
                    byteSettings[8] = 2;
                    break;
                case 3:
                    byteSettings[8] = 3;
                    break;
                case 4:
                    byteSettings[8] = 4;
                    break;
                case 5:
                    byteSettings[8] = 5;
                    break;
                case 6:
                    byteSettings[8] = 6;
                    break;
                case 7:
                    byteSettings[8] = 7;
                    break;
            }
            CompareByteArray();
        }
        private void xDzTbar_Scroll(object sender, EventArgs e)
        {
            byteSettings[13] = Convert.ToByte(xDzTbar.Value);
            xDZval.Text = Convert.ToString(xDzTbar.Value * 256);
            CompareByteArray();
        }
        private void XgainBar_Scroll(object sender, EventArgs e)
        {
            byteSettings[11] = Convert.ToByte(XgainBar.Value);
            xGainlabel.Text = Convert.ToString(XgainBar.Value);
            CompareByteArray();
        }
        private void YDzTbar_Scroll(object sender, EventArgs e)
        {
            byteSettings[14] = Convert.ToByte(yDzTbar.Value);
            yDZval.Text = Convert.ToString(yDzTbar.Value * 256);
            CompareByteArray();
        }
        private void YgainBar_Scroll(object sender, EventArgs e)
        {
            byteSettings[12] = Convert.ToByte(YgainBar.Value);
            YgainVal.Text = Convert.ToString(YgainBar.Value);
            CompareByteArray();
        }
        private void XtiltVal_Scroll(object sender, EventArgs e)
        {
            byteSettings[17] = Convert.ToByte(XtiltVal.Value);
            int xtemp = (XtiltVal.Value - 127) * 256;
            XtVal.Text = Convert.ToString(xtemp);
            CompareByteArray();
        }
        private void YtiltVal_Scroll(object sender, EventArgs e)
        {
            byteSettings[18] = Convert.ToByte(YtiltVal.Value);
            int ytemp = (YtiltVal.Value - 127) * 256;
            YtVal.Text = Convert.ToString(ytemp);
            CompareByteArray();
        }
        private void Debounce_ValueChanged(object sender, EventArgs e)
        {
            byteSettings[9] = Convert.ToByte(Debounce.Value);
            CompareByteArray();
        }

        /// <summary>
        /// Button Configuration Tab
        /// </summary>

        private void GamepadBox_CheckedChanged(object sender, EventArgs e)
        {
            if (GamepadBox.Checked == true)
            {
                byteSettings[0] = 1;
            }
            else
            {
                byteSettings[0] = 0;
            }
            CompareByteArray();
        }
        private void UseCDbox_CheckedChanged(object sender, EventArgs e)
        {
            if (UseCDbox.Checked == true)
            {
                byteSettings[1] = 1;
            }
            else
            {
                byteSettings[1] = 0;
            }
            CompareByteArray();
        }
        private void UseHSbox_CheckedChanged(object sender, EventArgs e)
        {
            if (UseHSbox.Checked == true)
            {
                byteSettings[5] = 1;
            }
            else
            {
                byteSettings[5] = 0;
            }
            CompareByteArray();
        }

        int SelectedButton;
        private void ButtonBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            SelectedButton = ButtonBox.SelectedIndex;
        }

        int asciiIndex;
        private void KeypressBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            asciiIndex = KeypressBox.SelectedIndex;
            AsciiLbl.Text = "0x" + AsciiCodes[asciiIndex].ToString("X2");
        }

        int SelectedUsage;
        private void UsageBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            SelectedUsage = UsageBox.SelectedIndex;
        }
        private void ApplyBtn_Click(object sender, EventArgs e)
        {
            Keys[SelectedButton] = AsciiCodes[asciiIndex];
            Labels[SelectedButton] = LabelCodes[SelectedUsage];
            if (SelectedUsage == 12)                                    // if usage is coindoor
            {
                byteSettings[2] = Convert.ToByte(SelectedButton);
                CDbuttonBox.Text = Convert.ToString(SelectedButton + 1);
            }
            else if (SelectedUsage == 10)
            {
                byteSettings[3] = Convert.ToByte(SelectedButton);
                LBbuttonBox.Text = Convert.ToString(byteSettings[3] + 1);
                byteSettings[4] = Keys[SelectedButton];
                LBkeypress.Text = "0x" + byteSettings[4].ToString("X2");
            }
            else if (SelectedUsage == 23)
            {
                byteSettings[16] = Convert.ToByte(SelectedButton);
                TiltButtonBox.Text = Convert.ToString(byteSettings[16] + 1);
            }
            UpdateConnectionLables();
            CompareKeysArray();
            ConvertKeyArray();
            ConvertLabelArray();
            CompareByteArray();
        }
        private void GPButtonSelectBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            int ButtonSelected = GPButtonSelectBox.SelectedIndex;
            AsCodeLbl.Text = "0x" + Keys[ButtonSelected].ToString("X2");
            byte KeySelected = Keys[ButtonSelected];
            for (int a = 0; a < 73; a++)
            {
                if (KeySelected == AsciiCodes[a])
                {
                    KeypressLbl.Text = AsciiNames[a];
                }
            }
            byte labelUsed = Labels[ButtonSelected];
            for (int a = 0; a < 28; a++)
            {
                if (LabelCodes[a] == labelUsed)
                {
                    ButtonUseLbl.Text = LabelCodeNames[a];
                }
            }
         }

        /// <summary>
        /// Connections Tab
        /// </summary>
         

    }

}
