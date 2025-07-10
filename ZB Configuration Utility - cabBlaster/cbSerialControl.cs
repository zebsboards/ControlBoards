using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Configuration_Utility
{
    public partial class cbSerialControl : Form
    {
        public cbSerialControl()
        {
            InitializeComponent();
        }

        private void cbSerialControl_Load(object sender, EventArgs e)
        {
            DisconnectedState();
            TbLedNumCh1.Text = "0"; TbLedNumCh2.Text = "0"; TbLedNumCh3.Text = "0"; TbLedNumCh4.Text = "0"; TbLedNumCh5.Text = "0"; TbLedNumCh6.Text = "0"; TbLedNumCh7.Text = "0";
            TbLedNumCh8.Text = "0"; TbLedNumCh9.Text = "0"; TbLedNumCh10.Text = "0"; TbLedNumCh11.Text = "0"; TbLedNumCh12.Text = "0"; TbLedNumCh13.Text = "0"; TbLedNumCh14.Text = "0";
            LboxCh1.Text = TbLedNumCh1.Text; LboxCh2.Text = TbLedNumCh2.Text; LboxCh3.Text = TbLedNumCh3.Text; LboxCh4.Text = TbLedNumCh4.Text; LboxCh5.Text = TbLedNumCh5.Text; LboxCh6.Text = TbLedNumCh6.Text; LboxCh7.Text = TbLedNumCh7.Text;
            LboxCh8.Text = TbLedNumCh8.Text; LboxCh9.Text = TbLedNumCh9.Text; LboxCh10.Text = TbLedNumCh10.Text; LboxCh11.Text = TbLedNumCh11.Text; LboxCh12.Text = TbLedNumCh12.Text; LboxCh13.Text = TbLedNumCh13.Text; LboxCh14.Text = TbLedNumCh14.Text;
            Hbox1.Text = "1"; Hbox2.Text = "1"; Hbox3.Text = "1"; Hbox4.Text = "1"; Hbox5.Text = "1"; Hbox6.Text = "1"; Hbox7.Text = "1"; Hbox8.Text = "1"; Hbox9.Text = "1"; Hbox10.Text = "1"; Hbox11.Text = "1"; Hbox12.Text = "1"; Hbox13.Text = "1"; Hbox14.Text = "1";
        }
        private void DisconnectedState()
        {
            ErrorLabel.Hide();
            ChannelSelectionBox.Maximum = 8;
            ComConnectButton.BackColor = Color.DarkGray;
            ComConnectButton.Text = "CONNECT";
            ComConnectButton.Enabled = true;
            ComDisconnectButton.BackColor = Color.DarkRed;
            ComDisconnectButton.Text = "DISCONNECTED";
            ComDisconnectButton.Enabled = false;
            ClearDataBox();
            DataIn = "\r\r\rDisconnected from controller ";
            RtbDataIn.Text = DataIn;
            RtbDataIn.SelectAll();
            RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
            RtbDataIn.DeselectAll();
        }
        private void ConnectedState()
        {
            DataIn = null;
            if (PortFound != 1)
            {
                RtbDataIn.Text = "\r\rLegacy Device Detected.\rLimiting Available Channels to 8";
                RtbDataIn.SelectAll();
                RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
                ChannelSelectionBox.Maximum = 8;
                NameBox.Text = "ZBSerial";

            }
            else 
            {
                RtbDataIn.Text = "\r\r\rConnected to\rcabBlaster 14port Serial Controller";
                RtbDataIn.SelectAll();
                RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
                ChannelSelectionBox.Maximum = 14;
                NameBox.Text = "ZBSerial14Port";
            }
            ComConnectButton.BackColor = Color.Green;
            ComConnectButton.Text = "CONNECTED";
            ComConnectButton.Enabled = false;
            ComDisconnectButton.BackColor = Color.DarkGray;
            ComDisconnectButton.Text = "DISCONNECT";
            ComDisconnectButton.Enabled = true;
            SliderTimer.Value = 1000;
            SliderBrightness.Value = 128;
        }

        int PortFound;
        private void ComConnectButton_Click(object sender, EventArgs e)
        {
            ClearDataBox();
            ConnectToController("D", "ZBPort14", "ZBSerial", "S2");
            
        }
        private void ConnectToController(string CommandCode, string VerificationID, string VerificationID2, string VerificationID3)
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
                                if (DataIn.Trim() == VerificationID)
                                {
                                    PortFound = 1;
                                    break;
                                }
                                else if (DataIn.Trim() == VerificationID2 || DataIn.Trim() == VerificationID3)
                                {
                                    PortFound = 2;
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
        private void ComDisconnectButton_Click(object sender, EventArgs e)
        {
            // *********************************** Send clear all Leds command before closing
            if (ComPort.IsOpen)
            {
                ClearLEDs();
                Task.Delay(100).Wait();
                DisconnectedState();
                ComPort.Close();
            }
            else
            {
                DisconnectedState();
            }
        }
        public bool FileExists(string fileName)
        {
            var workingDirectory = Environment.CurrentDirectory;
            var file = $"{workingDirectory}\\{fileName}";
            return System.IO.File.Exists(file);
        }

        int count;
        private void LoadButton_Click(object sender, EventArgs e)
        {
            string[] Temp;
            char[] delimiter = { ',' };
            if (FileExists("ConfigUtility.csv") == true)
            {
                try
                {
                    System.IO.StreamReader file = new System.IO.StreamReader(@"ConfigUtility.csv");
                    string Received = file.ReadLine();
                    Temp = Received.Split(delimiter);
                    NameBox.Text = Temp[1]; PortBox.Text = Temp[2]; UseDtrBox.Text = Temp[3];
                    while ((Received = file.ReadLine()) != null)
                    {
                        Temp = Received.Split(delimiter);
                        LedNums[count] = Convert.ToUInt16(Temp[1]); Order[count] = Temp[2]; WidthBox[count] = Temp[3]; HeightBox[count] = Temp[4]; DirBox[count] = Temp[5];
                        count++;
                    }
                    file.Close();
                    count = 0;
                }
                catch (Exception f)
                {
                    throw new ApplicationException("Error: " + f);
                }
                if (LedNums[0] > 0) { TbLedNumCh1.Text = Convert.ToString(LedNums[0]); LboxCh1.Text = TbLedNumCh1.Text; OrderSel1.Text = Order[0]; ColBox1.Text = OrderSel1.Text; Wbox1.Text = WidthBox[0]; Hbox1.Text = HeightBox[0]; DirBox1.Text = DirBox[0]; NumberOfLeds.Text = LboxCh1.Text; TestColor.Text = OrderSel1.Text; }
                if (LedNums[1] > 0) { TbLedNumCh2.Text = Convert.ToString(LedNums[1]); LboxCh2.Text = TbLedNumCh2.Text; OrderSel2.Text = Order[1]; ColBox2.Text = OrderSel2.Text; Wbox2.Text = WidthBox[1]; Hbox2.Text = HeightBox[1]; DirBox2.Text = DirBox[1]; }
                if (LedNums[2] > 0) { TbLedNumCh3.Text = Convert.ToString(LedNums[2]); LboxCh3.Text = TbLedNumCh3.Text; OrderSel3.Text = Order[2]; ColBox3.Text = OrderSel3.Text; Wbox3.Text = WidthBox[2]; Hbox3.Text = HeightBox[2]; DirBox3.Text = DirBox[2]; }
                if (LedNums[3] > 0) { TbLedNumCh4.Text = Convert.ToString(LedNums[3]); LboxCh4.Text = TbLedNumCh4.Text; OrderSel4.Text = Order[3]; ColBox4.Text = OrderSel4.Text; Wbox4.Text = WidthBox[3]; Hbox4.Text = HeightBox[3]; DirBox4.Text = DirBox[3]; }
                if (LedNums[4] > 0) { TbLedNumCh5.Text = Convert.ToString(LedNums[4]); LboxCh5.Text = TbLedNumCh5.Text; OrderSel5.Text = Order[4]; ColBox5.Text = OrderSel5.Text; Wbox5.Text = WidthBox[4]; Hbox5.Text = HeightBox[4]; DirBox5.Text = DirBox[4]; }
                if (LedNums[5] > 0) { TbLedNumCh6.Text = Convert.ToString(LedNums[5]); LboxCh6.Text = TbLedNumCh6.Text; OrderSel6.Text = Order[5]; ColBox6.Text = OrderSel6.Text; Wbox6.Text = WidthBox[5]; Hbox6.Text = HeightBox[5]; DirBox6.Text = DirBox[5]; }
                if (LedNums[6] > 0) { TbLedNumCh7.Text = Convert.ToString(LedNums[6]); LboxCh7.Text = TbLedNumCh7.Text; OrderSel7.Text = Order[6]; ColBox7.Text = OrderSel7.Text; Wbox7.Text = WidthBox[6]; Hbox7.Text = HeightBox[6]; DirBox7.Text = DirBox[6]; }
                if (LedNums[7] > 0) { TbLedNumCh8.Text = Convert.ToString(LedNums[7]); LboxCh8.Text = TbLedNumCh8.Text; OrderSel8.Text = Order[7]; ColBox8.Text = OrderSel8.Text; Wbox8.Text = WidthBox[7]; Hbox8.Text = HeightBox[7]; DirBox8.Text = DirBox[7]; }
                if (LedNums[8] > 0) { TbLedNumCh9.Text = Convert.ToString(LedNums[8]); LboxCh9.Text = TbLedNumCh9.Text; OrderSel9.Text = Order[8]; ColBox9.Text = OrderSel9.Text; Wbox9.Text = WidthBox[8]; Hbox9.Text = HeightBox[8]; DirBox9.Text = DirBox[8]; }
                if (LedNums[9] > 0) { TbLedNumCh10.Text = Convert.ToString(LedNums[9]); LboxCh10.Text = TbLedNumCh10.Text; OrderSel10.Text = Order[9]; ColBox10.Text = OrderSel10.Text; Wbox10.Text = WidthBox[9]; Hbox10.Text = HeightBox[9]; DirBox10.Text = DirBox[9]; }
                if (LedNums[10] > 0) { TbLedNumCh11.Text = Convert.ToString(LedNums[10]); LboxCh11.Text = TbLedNumCh11.Text; OrderSel11.Text = Order[10]; ColBox11.Text = OrderSel11.Text; Wbox11.Text = WidthBox[10]; Hbox11.Text = HeightBox[10]; DirBox11.Text = DirBox[10]; }
                if (LedNums[11] > 0) { TbLedNumCh12.Text = Convert.ToString(LedNums[11]); LboxCh12.Text = TbLedNumCh12.Text; OrderSel12.Text = Order[11]; ColBox12.Text = OrderSel12.Text; Wbox12.Text = WidthBox[11]; Hbox12.Text = HeightBox[11]; DirBox12.Text = DirBox[11]; }
                if (LedNums[12] > 0) { TbLedNumCh13.Text = Convert.ToString(LedNums[12]); LboxCh13.Text = TbLedNumCh13.Text; OrderSel13.Text = Order[12]; ColBox13.Text = OrderSel13.Text; Wbox13.Text = WidthBox[12]; Hbox13.Text = HeightBox[12]; DirBox13.Text = DirBox[12]; }
                if (LedNums[13] > 0) { TbLedNumCh14.Text = Convert.ToString(LedNums[13]); LboxCh14.Text = TbLedNumCh14.Text; OrderSel14.Text = Order[13]; ColBox14.Text = OrderSel14.Text; Wbox14.Text = WidthBox[13]; Hbox14.Text = HeightBox[13]; DirBox14.Text = DirBox[13]; }
                RtbDataIn.Text = "\r\r\rPrevious Data loaded";
                RtbDataIn.SelectAll();
                RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
            }
            else
            {
                RtbDataIn.Text = "\r\r\rNo File To Load";
                RtbDataIn.SelectAll();
                RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
            }
        }
        private void SaveButton_Click(object sender, EventArgs e)
        {
            PortSettings(0, NameBox.Text, PortBox.Text, UseDtrBox.Text, "ConfigUtility.csv");
            for (int Entry = 0; Entry < 14; Entry++)
            {
                if (LedNums[Entry] > 0)
                {
                    NewRecord(Entry + 1, LedNums[Entry], Order[Entry], WidthBox[Entry], HeightBox[Entry], DirBox[Entry], "ConfigUtility.csv");
                }
            }
            RtbDataIn.Text = "\r\r\rData saved";
            RtbDataIn.SelectAll();
            RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
        }
        static void NewRecord(int ID, ushort NumberLEDs, string PixelOrder, string Width, string Height, string Arrangement, string Filepath)
        {
            try
            {
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(@Filepath, true))
                {
                    file.WriteLine(ID + "," + NumberLEDs + "," + PixelOrder + "," + Width + "," + Height + "," + Arrangement);
                }
            }
            catch (Exception e)
            {
                throw new ApplicationException("Error: " + e);
            }
        }
        static void PortSettings(int ID, string Controller, string COM, string DTR, string Filepath)
        {
            try
            {
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(@Filepath, false))
                {
                    file.WriteLine(ID + "," + Controller + "," + COM + "," + DTR);
                }
            }
            catch (Exception e)
            {
                throw new ApplicationException("Error: " + e);
            }
        }

        string DataIn;
        private void ComPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            DataIn = ComPort.ReadExisting();
            if (DataIn != "N" && DataIn != "A")
            {
                if (PortFound != 0)
                {
                    this.Invoke(new EventHandler(ComPort_ShowData));
                }
                else
                {

                }
            }
            else
            {
                DataIn = null;
            }
        }
        private void ComPort_ShowData(object sender, EventArgs e)
        {
            RtbDataIn.Text += DataIn;
            //Task.Delay(50).Wait();
        }
        private void ClearDataBox()
        {
            this.RtbDataIn.Clear();
            //Task.Delay(50).Wait();
        }
        private void NumberOfLeds_TextChanged(object sender, EventArgs e)
        {
            if (NumberOfLeds.Text.Length > 0)
            {
                try
                {
                    Convert.ToDecimal(NumberOfLeds.Text);
                    if (Convert.ToDecimal(NumberOfLeds.Text) > 1024)
                    {
                        NumberOfLeds.Text = "1024";
                    }
                }
                catch (Exception exception)
                {
                    NumberOfLeds.Text = NumberOfLeds.Text.Remove(NumberOfLeds.TextLength - 1, 1);
                    NumberOfLeds.SelectionStart = NumberOfLeds.Text.Length;
                    NumberOfLeds.SelectionLength = 0;
                }
            }
        }

        ushort[] LedNums = new ushort[14] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        string[] Order = new string[14];
        private void UpdateButton_Click(object sender, EventArgs e)
        {
            if (NumberOfLeds.Text == "" || TestColor.Text == "")
            {
                ErrorLabel.Show();
            }
            else
            {
                ErrorLabel.Hide();
                UInt16 Tmp = Convert.ToUInt16(NumberOfLeds.Text);
                if (ChannelSelectionBox.Text == "1") { TbLedNumCh1.Text = NumberOfLeds.Text; LboxCh1.Text = NumberOfLeds.Text; Wbox1.Text = NumberOfLeds.Text; LedNums[0] = Tmp; ColBox1.Text = TestColor.Text; OrderSel1.Text = TestColor.Text; Order[0] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "2") { TbLedNumCh2.Text = NumberOfLeds.Text; LboxCh2.Text = NumberOfLeds.Text; Wbox2.Text = NumberOfLeds.Text; LedNums[1] = Tmp; ColBox2.Text = TestColor.Text; OrderSel2.Text = TestColor.Text; Order[1] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "3") { TbLedNumCh3.Text = NumberOfLeds.Text; LboxCh3.Text = NumberOfLeds.Text; Wbox3.Text = NumberOfLeds.Text; LedNums[2] = Tmp; ColBox3.Text = TestColor.Text; OrderSel3.Text = TestColor.Text; Order[2] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "4") { TbLedNumCh4.Text = NumberOfLeds.Text; LboxCh4.Text = NumberOfLeds.Text; Wbox4.Text = NumberOfLeds.Text; LedNums[3] = Tmp; ColBox4.Text = TestColor.Text; OrderSel4.Text = TestColor.Text; Order[3] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "5") { TbLedNumCh5.Text = NumberOfLeds.Text; LboxCh5.Text = NumberOfLeds.Text; Wbox5.Text = NumberOfLeds.Text; LedNums[4] = Tmp; ColBox5.Text = TestColor.Text; OrderSel5.Text = TestColor.Text; Order[4] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "6") { TbLedNumCh6.Text = NumberOfLeds.Text; LboxCh6.Text = NumberOfLeds.Text; Wbox6.Text = NumberOfLeds.Text; LedNums[5] = Tmp; ColBox6.Text = TestColor.Text; OrderSel6.Text = TestColor.Text; Order[5] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "7") { TbLedNumCh7.Text = NumberOfLeds.Text; LboxCh7.Text = NumberOfLeds.Text; Wbox7.Text = NumberOfLeds.Text; LedNums[6] = Tmp; ColBox7.Text = TestColor.Text; OrderSel7.Text = TestColor.Text; Order[6] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "8") { TbLedNumCh8.Text = NumberOfLeds.Text; LboxCh8.Text = NumberOfLeds.Text; Wbox8.Text = NumberOfLeds.Text; LedNums[7] = Tmp; ColBox8.Text = TestColor.Text; OrderSel8.Text = TestColor.Text; Order[7] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "9") { TbLedNumCh9.Text = NumberOfLeds.Text; LboxCh9.Text = NumberOfLeds.Text; Wbox9.Text = NumberOfLeds.Text; LedNums[8] = Tmp; ColBox9.Text = TestColor.Text; OrderSel9.Text = TestColor.Text; Order[8] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "10") { TbLedNumCh10.Text = NumberOfLeds.Text; LboxCh10.Text = NumberOfLeds.Text; Wbox10.Text = NumberOfLeds.Text; LedNums[9] = Tmp; ColBox10.Text = TestColor.Text; OrderSel10.Text = TestColor.Text; Order[9] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "11") { TbLedNumCh11.Text = NumberOfLeds.Text; LboxCh11.Text = NumberOfLeds.Text; Wbox11.Text = NumberOfLeds.Text; LedNums[10] = Tmp; ColBox11.Text = TestColor.Text; OrderSel11.Text = TestColor.Text; Order[10] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "12") { TbLedNumCh12.Text = NumberOfLeds.Text; LboxCh12.Text = NumberOfLeds.Text; Wbox12.Text = NumberOfLeds.Text; LedNums[11] = Tmp; ColBox12.Text = TestColor.Text; OrderSel12.Text = TestColor.Text; Order[11] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "13") { TbLedNumCh13.Text = NumberOfLeds.Text; LboxCh13.Text = NumberOfLeds.Text; Wbox13.Text = NumberOfLeds.Text; LedNums[12] = Tmp; ColBox13.Text = TestColor.Text; OrderSel13.Text = TestColor.Text; Order[12] = TestColor.Text; }
                else if (ChannelSelectionBox.Text == "14") { TbLedNumCh14.Text = NumberOfLeds.Text; LboxCh14.Text = NumberOfLeds.Text; Wbox14.Text = NumberOfLeds.Text; LedNums[13] = Tmp; ColBox14.Text = TestColor.Text; OrderSel14.Text = TestColor.Text; Order[13] = TestColor.Text; }
                else { }
            }
        }
        private void TestButton_Click(object sender, EventArgs e)
        {
            if (TestColor.Text == "GRB")
            {
                ErrorLabel.Hide();
                DataIn = "\r\r\rTesting LED color order Green, Red, Blue.";
                StartTest();
            }
            else if (TestColor.Text == "RGB")
            {
                ErrorLabel.Hide();
                DataIn = "\r\r\rTesting LED color order Red, Green Blue.";
                StartTest();
            }
            else
            {
                ErrorLabel.Show();
            }

        }
        private void StartTest()
        {
            RtbDataIn.Text = DataIn;
            RtbDataIn.SelectAll();
            RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
            Task.Delay(100).Wait();
            ShowLeds(1, Convert.ToUInt16(NumberOfLeds.Text), Convert.ToUInt16(ChannelSelectionBox.Text), Convert.ToByte(SliderBrightness.Value), Convert.ToUInt16(SliderTimer.Value));
            Task.Delay(500).Wait();
            RtbDataIn.Text = "\r\rIf RED & GREEN are reversed\rchange the selection in the\rCOLOR ORDER box";
            RtbDataIn.SelectAll();
            RtbDataIn.SelectionAlignment = HorizontalAlignment.Center;
        }
        private void ShowLeds(UInt16 FirstLed, UInt16 NmbrLeds, UInt16 CurrentStrip, Byte Brightness, UInt16 DisplayTime)
        {
            //Colours: Firstcolor = "000000FF"; string Secondcolor = "0000FF00"; string Thirdcolor = "00FF0000";
            byte[] ColorToSend = new byte[9] { (byte)255, (byte)0, (byte)0 & 255, (byte)0, (byte)255, (byte)0 & 255, (byte)0, (byte)0, (byte)255 & 255 };
            ClearLEDs();
            for (int a = 0; a < 9; a++)
            {
                byte[] SendData = new byte[11] { (byte)'Y', (byte)(FirstLed >> 8), (byte)(FirstLed & 255), (byte)(NmbrLeds >> 8), (byte)(NmbrLeds & 255), (byte)(ColorToSend[a]), (byte)(ColorToSend[a + 1]), (byte)(ColorToSend[a + 2]), (byte)(Brightness), (byte)(CurrentStrip >> 8), (byte)(CurrentStrip & 255) };
                ComPort.Write(SendData, 0, 11);
                Task.Delay(DisplayTime).Wait();
                DataIn = null;
                SendData = new byte[1] { (byte)'C' };
                ComPort.Write(SendData, 0, 1);
                a += 2;
                Task.Delay(100).Wait();
                DataIn = null;
            }
            ClearLEDs();
            ClearDataBox();
        }
        private void ClearLEDs()
        {
            byte[] ClearData = new byte[1] { (byte)'X' };
            ComPort.Write(ClearData, 0, 1);
        }
        private void SliderBrightness_Scroll(object sender, EventArgs e)
        {
            BrightReading.Text = Convert.ToString(SliderBrightness.Value);
        }
        private void SliderTimer_Scroll(object sender, EventArgs e)
        {
            TimeReading.Text = Convert.ToString(SliderTimer.Value / 1000);
        }

        int HaltGen;
        int StripCount;
        string[] WidthBox = new string[14]; 
        string[] HeightBox = new string[14];
        string[] DirBox = new string[14];

        private void BtnCabGen_Click(object sender, EventArgs e)
        {
            if (ComPort.IsOpen)
            {
                PortBox.Text = ComPortBox.Text;
            }
            else
            {
                PortBox.Items.Clear();
                string[] ports = SerialPort.GetPortNames();
                PortBox.Items.AddRange(ports);
            }
            HaltGen = 0;
            StripCount = 0;
            WidthBox = new string[14] { Wbox1.Text, Wbox2.Text, Wbox3.Text, Wbox4.Text, Wbox5.Text, Wbox6.Text, Wbox7.Text, Wbox8.Text, Wbox9.Text, Wbox10.Text, Wbox11.Text, Wbox12.Text, Wbox13.Text, Wbox14.Text };
            HeightBox = new string[14] { Hbox1.Text, Hbox2.Text, Hbox3.Text, Hbox4.Text, Hbox5.Text, Hbox6.Text, Hbox7.Text, Hbox8.Text, Hbox9.Text, Hbox10.Text, Hbox11.Text, Hbox12.Text, Hbox13.Text, Hbox14.Text };
            DirBox = new string[14] { DirBox1.Text, DirBox2.Text, DirBox3.Text, DirBox4.Text, DirBox5.Text, DirBox6.Text, DirBox7.Text, DirBox8.Text, DirBox9.Text, DirBox10.Text, DirBox11.Text, DirBox12.Text, DirBox13.Text, DirBox14.Text };
            int[] Position = new int[14];
            for (int StripNumCheck = 0; StripNumCheck < 14; StripNumCheck++)
            {
                if (LedNums[StripNumCheck] > 0)
                {
                    Position[StripCount] = StripNumCheck;
                    StripCount++;
                }
            }
            for (int Verify = 0; Verify < StripCount; Verify ++)
            {
                if (WidthBox[Position[Verify]] == "" || HeightBox[Position[Verify]] == "" || DirBox[Position[Verify]] == "")
                {
                    RtbPreviewPane.Text = "\r\r!! Configuration ERROR !! Check Channel " + (Verify + 1) + " Entries";
                    RtbPreviewPane.SelectAll();
                    RtbPreviewPane.SelectionAlignment = HorizontalAlignment.Center;
                    RtbPreviewPane.DeselectAll();
                    HaltGen = 1;
                    break;
                }
                else if (NameBox.Text == "")
                {
                    RtbPreviewPane.Text = "\r\r!! Configuration ERROR !! Select Controller Name in Controller Name Box";
                    RtbPreviewPane.SelectAll();
                    RtbPreviewPane.SelectionAlignment = HorizontalAlignment.Center;
                    RtbPreviewPane.DeselectAll();
                    HaltGen = 1;
                    break;
                }
                else if (PortBox.Text == "")
                {
                    RtbPreviewPane.Text = "\r\r!! Configuration ERROR !! No ComPort Selected.\rSelect/Enter ComPort In Box ";
                    RtbPreviewPane.SelectAll();
                    RtbPreviewPane.SelectionAlignment = HorizontalAlignment.Center;
                    RtbPreviewPane.DeselectAll();
                    HaltGen = 1;
                    break;
                }
                else if (UseDtrBox.Text == "" )
                {
                    RtbPreviewPane.Text = "\r\r!! Configuration ERROR !! Select DTR Setting in DTR Box";
                    RtbPreviewPane.SelectAll();
                    RtbPreviewPane.SelectionAlignment = HorizontalAlignment.Center;
                    RtbPreviewPane.DeselectAll();
                    HaltGen = 1;
                    break;
                }
            }
            if (HaltGen != 1)
            {
                int tmp1 = LedNums[0];
                int tmp2 = tmp1 + 1;
                int tmp3 = LedNums[1] + tmp2;
                int tmp4 = LedNums[2] + tmp3;
                int tmp5 = LedNums[3] + tmp4;
                int tmp6 = LedNums[4] + tmp5;
                int tmp7 = LedNums[5] + tmp6;
                int tmp8 = LedNums[6] + tmp7;
                int tmp9 = LedNums[7] + tmp8;
                int tmp10 = LedNums[8] + tmp9;
                int tmp11 = LedNums[9] + tmp10;
                int tmp12 = LedNums[10] + tmp11;
                int tmp13 = LedNums[11] + tmp12;
                int tmp14 = LedNums[12] + tmp13;
                int[] tmp = new int[14] { 1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, tmp8, tmp9, tmp10, tmp11, tmp12, tmp13, tmp14 };

                RtbPreviewPane.Text = RtbHeader.Text + Environment.NewLine + "<" + NameBox.Text + ">" + Environment.NewLine;

                RtbPreviewPane.AppendText("<Name>LEDStripController</Name>" + Environment.NewLine + "<NumberOfStrips>" + Convert.ToString(StripCount) + "</NumberOfStrips>" + Environment.NewLine);

                for (int i = 0; i < StripCount; i++)
                {
                    RtbPreviewPane.AppendText("<NumberOfLedsStrip" + Convert.ToString(Position[i] + 1) + ">" + Convert.ToString(LedNums[Position[i]]) + "</NumberOfLedsStrip" + Convert.ToString(Position[i] + 1) + ">" + Environment.NewLine);
                }

                RtbPreviewPane.AppendText("<ComPortName>" + PortBox.Text + "</ComPortName>" + Environment.NewLine + "<ComPortBaudRate>2000000</ComPortBaudRate>" + Environment.NewLine + "<ComPortDtrEnable>" + UseDtrBox.Text + "</ComPortDtrEnable>" + Environment.NewLine + "</" + NameBox.Text + ">" + Environment.NewLine + "</OutputControllers>" + Environment.NewLine + "<Toys>" + Environment.NewLine);

                for (int i = 0; i < StripCount; i++)
                {
                    RtbPreviewPane.AppendText("<LedStrip>" + Environment.NewLine + "<Name>Channel" + Convert.ToString(Position[i] + 1) + "</Name>" + Environment.NewLine + "<Width>" + WidthBox[Position[i]] + "</Width>" + Environment.NewLine + "<Height>" + HeightBox[Position[i]] + "</Height>" + Environment.NewLine + "<LedStripArrangement>" + DirBox[Position[i]] + "</LedStripArrangement>" + Environment.NewLine + "<ColorOrder>" + Order[Position[i]] + "</ColorOrder>" + Environment.NewLine + "<FirstLedNumber>" + Convert.ToString(tmp[Position[i]]) + "</FirstLedNumber>" + Environment.NewLine + "<FadingCurveName>SwissLizardsLedCurve</FadingCurveName>" + Environment.NewLine + "<OutputControllerName>LEDStripController</OutputControllerName>" + Environment.NewLine + "</LedStrip>" + Environment.NewLine);
                }

                RtbPreviewPane.AppendText("<LedWizEquivalent>" + Environment.NewLine + "<Name>LedWizEquivalent30</Name>" + Environment.NewLine + "<Outputs>" + Environment.NewLine);

                int LwPort = 1;
                for (int i = 0; i < StripCount; i++)
                {
                    RtbPreviewPane.AppendText("<LedWizEquivalentOutput>" + Environment.NewLine + "<OutputName>Channel" + Convert.ToString(Position[i] + 1) + "</OutputName>" + Environment.NewLine + "<LedWizEquivalentOutputNumber>" + Convert.ToString(LwPort) + "</LedWizEquivalentOutputNumber>" + Environment.NewLine + "</LedWizEquivalentOutput>");
                    LwPort += 3;
                }

                RtbPreviewPane.AppendText(Environment.NewLine + "</Outputs>" + Environment.NewLine + "<LedWizNumber>30</LedWizNumber>" + Environment.NewLine + "</LedWizEquivalent>" + Environment.NewLine + "</Toys>" + Environment.NewLine + "<AutoConfigEnabled>true</AutoConfigEnabled>" + Environment.NewLine + "</Cabinet>");
            }
        }
        private void BtnSaveCab_Click(object sender, EventArgs e)
        {
            var folderBrowserDialog1 = new FolderBrowserDialog();
            DialogResult result = folderBrowserDialog1.ShowDialog();
            if (result == DialogResult.OK)
            {
                string folderName = folderBrowserDialog1.SelectedPath;
                string fileName = "Cabinet.xml";
                string fullPath = folderName + "\\" + fileName;
                System.IO.File.WriteAllLines(@fullPath, RtbPreviewPane.Lines);
                TbSavePath.Text = fullPath;
                Task.Delay(1000).Wait();
                TbComplete.Text = "File Save Complete";
            }
        }
        private void CmBoxLayout_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (CmBoxLayout.Text == "LeftRightTopDown")
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.LeftRightTopDown;
            }
            else if (CmBoxLayout.Text == "LeftRightBottomUp")
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.LeftRightBottomUp;
            }
            else if (CmBoxLayout.Text == "RightLeftTopDown")
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.RightLeftTopDown;
            }
            else if (CmBoxLayout.Text == "RightLeftBottomUp")
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.RightLeftBottomUp;
            }
            else if (CmBoxLayout.Text == "TopDownAlternateLeftRight")
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.TopDownAlternateLeftRight;
            }
            else if (CmBoxLayout.Text == "TopDownAlternateRightLeft")
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.TopDownAlternateRightLeft;
            }
            else if (CmBoxLayout.Text == "BottomUpAlternateLeftRight")
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.BottomUpAlternateLeftRight;
            }
            else if (CmBoxLayout.Text == "BottomUpAlternateRightLeft")
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.BottomUpAlternateRightLeft;
            }
            else
            {
                PicBoxLayout.Image = Configuration_Utility.Properties.Resources.LedStripArrangementEnum;
            }
        }
        private void cbSerialControl_FormClosed(object sender, FormClosedEventArgs e)
        {
            ComDisconnectButton.PerformClick();
            Menu formMenu = new Menu();
            formMenu.Show();
        }

        
    }
}