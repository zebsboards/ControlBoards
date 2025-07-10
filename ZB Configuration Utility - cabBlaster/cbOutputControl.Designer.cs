
namespace Configuration_Utility
{
    partial class cbOutputControl
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
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
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(cbOutputControl));
            this.ComPort = new System.IO.Ports.SerialPort(this.components);
            this.picBoxcbLogo = new System.Windows.Forms.PictureBox();
            this.label2 = new System.Windows.Forms.Label();
            this.tabControl = new System.Windows.Forms.TabControl();
            this.MainPage = new System.Windows.Forms.TabPage();
            this.ConnectButton = new System.Windows.Forms.Button();
            this.rtbInfo = new System.Windows.Forms.RichTextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.PWMAdjust = new System.Windows.Forms.NumericUpDown();
            this.SavedValueButton = new System.Windows.Forms.Button();
            this.SendButton = new System.Windows.Forms.Button();
            this.TimeAdjuster = new System.Windows.Forms.NumericUpDown();
            this.ClearButton = new System.Windows.Forms.Button();
            this.SetButton = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.NightModeSetChkBox = new System.Windows.Forms.CheckBox();
            this.rtbDataIn = new System.Windows.Forms.RichTextBox();
            this.ComDisconnectButton = new System.Windows.Forms.Button();
            this.OutputSelectionBox = new System.Windows.Forms.NumericUpDown();
            this.EnableSolPWM = new System.Windows.Forms.CheckBox();
            this.label1 = new System.Windows.Forms.Label();
            this.ComPortBox = new System.Windows.Forms.ComboBox();
            this.tabPage2 = new System.Windows.Forms.TabPage();
            this.label11 = new System.Windows.Forms.Label();
            this.ColorEntryBox = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.MemoryScanButton = new System.Windows.Forms.Button();
            this.OutputSelectBox = new System.Windows.Forms.NumericUpDown();
            this.UseRGBChecked = new System.Windows.Forms.CheckBox();
            this.DOFButton = new System.Windows.Forms.Button();
            this.HexOutputBox = new System.Windows.Forms.TextBox();
            this.HexBoxBlue = new System.Windows.Forms.TextBox();
            this.HexBoxGreen = new System.Windows.Forms.TextBox();
            this.HexBoxRed = new System.Windows.Forms.TextBox();
            this.ValBoxBlue = new System.Windows.Forms.TextBox();
            this.ValBoxGreen = new System.Windows.Forms.TextBox();
            this.ValBoxRed = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.SliderBlue = new System.Windows.Forms.TrackBar();
            this.SliderGreen = new System.Windows.Forms.TrackBar();
            this.SliderRed = new System.Windows.Forms.TrackBar();
            this.DiagDisconnectButton = new System.Windows.Forms.Button();
            this.DiagConnectButton = new System.Windows.Forms.Button();
            this.ClearDiagBoxButton = new System.Windows.Forms.Button();
            this.rtbDiagData = new System.Windows.Forms.RichTextBox();
            this.ResetButton = new System.Windows.Forms.Button();
            this.ScanButton = new System.Windows.Forms.Button();
            this.TestButton = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.picBoxcbLogo)).BeginInit();
            this.tabControl.SuspendLayout();
            this.MainPage.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.PWMAdjust)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.TimeAdjuster)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.OutputSelectionBox)).BeginInit();
            this.tabPage2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.OutputSelectBox)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.SliderBlue)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.SliderGreen)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.SliderRed)).BeginInit();
            this.SuspendLayout();
            // 
            // ComPort
            // 
            this.ComPort.BaudRate = 2000000;
            this.ComPort.DtrEnable = true;
            // 
            // picBoxcbLogo
            // 
            this.picBoxcbLogo.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("picBoxcbLogo.BackgroundImage")));
            this.picBoxcbLogo.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.picBoxcbLogo.Location = new System.Drawing.Point(5, 2);
            this.picBoxcbLogo.Name = "picBoxcbLogo";
            this.picBoxcbLogo.Size = new System.Drawing.Size(535, 50);
            this.picBoxcbLogo.TabIndex = 32;
            this.picBoxcbLogo.TabStop = false;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Impact", 27.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.Color.Black;
            this.label2.Location = new System.Drawing.Point(546, 4);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(251, 45);
            this.label2.TabIndex = 29;
            this.label2.Text = "OUTPUT Config";
            // 
            // tabControl
            // 
            this.tabControl.Controls.Add(this.MainPage);
            this.tabControl.Controls.Add(this.tabPage2);
            this.tabControl.Location = new System.Drawing.Point(1, 52);
            this.tabControl.Name = "tabControl";
            this.tabControl.SelectedIndex = 0;
            this.tabControl.Size = new System.Drawing.Size(796, 395);
            this.tabControl.TabIndex = 35;
            // 
            // MainPage
            // 
            this.MainPage.BackColor = System.Drawing.SystemColors.ControlDark;
            this.MainPage.Controls.Add(this.ConnectButton);
            this.MainPage.Controls.Add(this.rtbInfo);
            this.MainPage.Controls.Add(this.label4);
            this.MainPage.Controls.Add(this.PWMAdjust);
            this.MainPage.Controls.Add(this.SavedValueButton);
            this.MainPage.Controls.Add(this.SendButton);
            this.MainPage.Controls.Add(this.TimeAdjuster);
            this.MainPage.Controls.Add(this.ClearButton);
            this.MainPage.Controls.Add(this.SetButton);
            this.MainPage.Controls.Add(this.label3);
            this.MainPage.Controls.Add(this.NightModeSetChkBox);
            this.MainPage.Controls.Add(this.rtbDataIn);
            this.MainPage.Controls.Add(this.ComDisconnectButton);
            this.MainPage.Controls.Add(this.OutputSelectionBox);
            this.MainPage.Controls.Add(this.EnableSolPWM);
            this.MainPage.Controls.Add(this.label1);
            this.MainPage.Controls.Add(this.ComPortBox);
            this.MainPage.Location = new System.Drawing.Point(4, 22);
            this.MainPage.Name = "MainPage";
            this.MainPage.Padding = new System.Windows.Forms.Padding(3);
            this.MainPage.Size = new System.Drawing.Size(788, 369);
            this.MainPage.TabIndex = 0;
            this.MainPage.Text = "Main";
            // 
            // ConnectButton
            // 
            this.ConnectButton.BackColor = System.Drawing.SystemColors.GrayText;
            this.ConnectButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.ConnectButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.ConnectButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ConnectButton.Location = new System.Drawing.Point(14, 61);
            this.ConnectButton.Name = "ConnectButton";
            this.ConnectButton.Size = new System.Drawing.Size(125, 49);
            this.ConnectButton.TabIndex = 65;
            this.ConnectButton.Text = "CONNECT";
            this.ConnectButton.UseVisualStyleBackColor = false;
            this.ConnectButton.Click += new System.EventHandler(this.ConnectButton_Click);
            // 
            // rtbInfo
            // 
            this.rtbInfo.BackColor = System.Drawing.SystemColors.AppWorkspace;
            this.rtbInfo.Location = new System.Drawing.Point(281, 9);
            this.rtbInfo.Name = "rtbInfo";
            this.rtbInfo.ReadOnly = true;
            this.rtbInfo.Size = new System.Drawing.Size(261, 185);
            this.rtbInfo.TabIndex = 55;
            this.rtbInfo.Text = resources.GetString("rtbInfo.Text");
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.Location = new System.Drawing.Point(102, 165);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(172, 15);
            this.label4.TabIndex = 54;
            this.label4.Text = "Flipper Hold PWM Setting";
            // 
            // PWMAdjust
            // 
            this.PWMAdjust.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.PWMAdjust.Increment = new decimal(new int[] {
            5,
            0,
            0,
            0});
            this.PWMAdjust.Location = new System.Drawing.Point(14, 160);
            this.PWMAdjust.Maximum = new decimal(new int[] {
            255,
            0,
            0,
            0});
            this.PWMAdjust.Minimum = new decimal(new int[] {
            25,
            0,
            0,
            0});
            this.PWMAdjust.Name = "PWMAdjust";
            this.PWMAdjust.Size = new System.Drawing.Size(82, 26);
            this.PWMAdjust.TabIndex = 53;
            this.PWMAdjust.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.PWMAdjust.Value = new decimal(new int[] {
            25,
            0,
            0,
            0});
            // 
            // SavedValueButton
            // 
            this.SavedValueButton.BackColor = System.Drawing.SystemColors.ControlDarkDark;
            this.SavedValueButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.SavedValueButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.SavedValueButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.SavedValueButton.Location = new System.Drawing.Point(558, 186);
            this.SavedValueButton.Name = "SavedValueButton";
            this.SavedValueButton.Size = new System.Drawing.Size(219, 56);
            this.SavedValueButton.TabIndex = 52;
            this.SavedValueButton.Text = "SHOW SAVED VALUES";
            this.SavedValueButton.UseVisualStyleBackColor = false;
            this.SavedValueButton.Click += new System.EventHandler(this.SavedValueButton_Click_1);
            // 
            // SendButton
            // 
            this.SendButton.BackColor = System.Drawing.SystemColors.ButtonFace;
            this.SendButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.SendButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.SendButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.SendButton.Location = new System.Drawing.Point(558, 9);
            this.SendButton.Name = "SendButton";
            this.SendButton.Size = new System.Drawing.Size(219, 68);
            this.SendButton.TabIndex = 51;
            this.SendButton.Text = "SAVE CONFIGURATION";
            this.SendButton.UseVisualStyleBackColor = false;
            this.SendButton.Click += new System.EventHandler(this.SendButton_Click_1);
            // 
            // TimeAdjuster
            // 
            this.TimeAdjuster.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.TimeAdjuster.Increment = new decimal(new int[] {
            60,
            0,
            0,
            0});
            this.TimeAdjuster.Location = new System.Drawing.Point(307, 217);
            this.TimeAdjuster.Maximum = new decimal(new int[] {
            15000,
            0,
            0,
            0});
            this.TimeAdjuster.Name = "TimeAdjuster";
            this.TimeAdjuster.Size = new System.Drawing.Size(87, 22);
            this.TimeAdjuster.TabIndex = 50;
            this.TimeAdjuster.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // ClearButton
            // 
            this.ClearButton.BackColor = System.Drawing.SystemColors.AppWorkspace;
            this.ClearButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.ClearButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.ClearButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ClearButton.Location = new System.Drawing.Point(694, 256);
            this.ClearButton.Name = "ClearButton";
            this.ClearButton.Size = new System.Drawing.Size(83, 103);
            this.ClearButton.TabIndex = 49;
            this.ClearButton.Text = "CLEAR STATUS BOX";
            this.ClearButton.UseVisualStyleBackColor = false;
            this.ClearButton.Click += new System.EventHandler(this.ClearButton_Click_1);
            // 
            // SetButton
            // 
            this.SetButton.BackColor = System.Drawing.Color.Gray;
            this.SetButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.SetButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.SetButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.SetButton.Location = new System.Drawing.Point(558, 98);
            this.SetButton.Name = "SetButton";
            this.SetButton.Size = new System.Drawing.Size(219, 65);
            this.SetButton.TabIndex = 48;
            this.SetButton.Text = "SET VALUES";
            this.SetButton.UseVisualStyleBackColor = false;
            this.SetButton.Click += new System.EventHandler(this.SetButton_Click_1);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.Location = new System.Drawing.Point(400, 221);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(117, 15);
            this.label3.TabIndex = 47;
            this.label3.Text = "Timer Value (ms)";
            // 
            // NightModeSetChkBox
            // 
            this.NightModeSetChkBox.AutoSize = true;
            this.NightModeSetChkBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.NightModeSetChkBox.Location = new System.Drawing.Point(129, 218);
            this.NightModeSetChkBox.Name = "NightModeSetChkBox";
            this.NightModeSetChkBox.Size = new System.Drawing.Size(134, 20);
            this.NightModeSetChkBox.TabIndex = 46;
            this.NightModeSetChkBox.Text = "Use NightMode";
            this.NightModeSetChkBox.UseVisualStyleBackColor = true;
            // 
            // rtbDataIn
            // 
            this.rtbDataIn.BackColor = System.Drawing.Color.DarkGray;
            this.rtbDataIn.Location = new System.Drawing.Point(12, 255);
            this.rtbDataIn.Name = "rtbDataIn";
            this.rtbDataIn.ReadOnly = true;
            this.rtbDataIn.Size = new System.Drawing.Size(682, 104);
            this.rtbDataIn.TabIndex = 44;
            this.rtbDataIn.Text = "";
            // 
            // ComDisconnectButton
            // 
            this.ComDisconnectButton.BackColor = System.Drawing.SystemColors.GrayText;
            this.ComDisconnectButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.ComDisconnectButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.ComDisconnectButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ComDisconnectButton.Location = new System.Drawing.Point(145, 61);
            this.ComDisconnectButton.Name = "ComDisconnectButton";
            this.ComDisconnectButton.Size = new System.Drawing.Size(125, 49);
            this.ComDisconnectButton.TabIndex = 43;
            this.ComDisconnectButton.Text = "DISCONNECT";
            this.ComDisconnectButton.UseVisualStyleBackColor = false;
            this.ComDisconnectButton.Click += new System.EventHandler(this.ComDisconnectButton_Click_1);
            // 
            // OutputSelectionBox
            // 
            this.OutputSelectionBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 24F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.OutputSelectionBox.Location = new System.Drawing.Point(14, 200);
            this.OutputSelectionBox.Maximum = new decimal(new int[] {
            64,
            0,
            0,
            0});
            this.OutputSelectionBox.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.OutputSelectionBox.Name = "OutputSelectionBox";
            this.OutputSelectionBox.Size = new System.Drawing.Size(82, 44);
            this.OutputSelectionBox.TabIndex = 41;
            this.OutputSelectionBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.OutputSelectionBox.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            // 
            // EnableSolPWM
            // 
            this.EnableSolPWM.AutoSize = true;
            this.EnableSolPWM.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.EnableSolPWM.Location = new System.Drawing.Point(14, 128);
            this.EnableSolPWM.Name = "EnableSolPWM";
            this.EnableSolPWM.Size = new System.Drawing.Size(206, 20);
            this.EnableSolPWM.TabIndex = 37;
            this.EnableSolPWM.Text = "Enable Flipper Hold PWM";
            this.EnableSolPWM.UseVisualStyleBackColor = true;
            this.EnableSolPWM.CheckedChanged += new System.EventHandler(this.EnableSolPWM_CheckedChanged_1);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(9, 18);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(167, 20);
            this.label1.TabIndex = 36;
            this.label1.Text = "DOF Controller Port";
            // 
            // ComPortBox
            // 
            this.ComPortBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.ComPortBox.FormattingEnabled = true;
            this.ComPortBox.Location = new System.Drawing.Point(185, 18);
            this.ComPortBox.Name = "ComPortBox";
            this.ComPortBox.Size = new System.Drawing.Size(83, 21);
            this.ComPortBox.TabIndex = 35;
            // 
            // tabPage2
            // 
            this.tabPage2.BackColor = System.Drawing.Color.Gray;
            this.tabPage2.Controls.Add(this.label11);
            this.tabPage2.Controls.Add(this.ColorEntryBox);
            this.tabPage2.Controls.Add(this.label10);
            this.tabPage2.Controls.Add(this.label9);
            this.tabPage2.Controls.Add(this.label8);
            this.tabPage2.Controls.Add(this.MemoryScanButton);
            this.tabPage2.Controls.Add(this.OutputSelectBox);
            this.tabPage2.Controls.Add(this.UseRGBChecked);
            this.tabPage2.Controls.Add(this.DOFButton);
            this.tabPage2.Controls.Add(this.HexOutputBox);
            this.tabPage2.Controls.Add(this.HexBoxBlue);
            this.tabPage2.Controls.Add(this.HexBoxGreen);
            this.tabPage2.Controls.Add(this.HexBoxRed);
            this.tabPage2.Controls.Add(this.ValBoxBlue);
            this.tabPage2.Controls.Add(this.ValBoxGreen);
            this.tabPage2.Controls.Add(this.ValBoxRed);
            this.tabPage2.Controls.Add(this.label7);
            this.tabPage2.Controls.Add(this.label6);
            this.tabPage2.Controls.Add(this.label5);
            this.tabPage2.Controls.Add(this.SliderBlue);
            this.tabPage2.Controls.Add(this.SliderGreen);
            this.tabPage2.Controls.Add(this.SliderRed);
            this.tabPage2.Controls.Add(this.DiagDisconnectButton);
            this.tabPage2.Controls.Add(this.DiagConnectButton);
            this.tabPage2.Controls.Add(this.ClearDiagBoxButton);
            this.tabPage2.Controls.Add(this.rtbDiagData);
            this.tabPage2.Controls.Add(this.ResetButton);
            this.tabPage2.Controls.Add(this.ScanButton);
            this.tabPage2.Controls.Add(this.TestButton);
            this.tabPage2.Location = new System.Drawing.Point(4, 22);
            this.tabPage2.Name = "tabPage2";
            this.tabPage2.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage2.Size = new System.Drawing.Size(788, 369);
            this.tabPage2.TabIndex = 1;
            this.tabPage2.Text = "Diagnostics & Testing";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label11.Location = new System.Drawing.Point(451, 200);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(215, 15);
            this.label11.TabIndex = 87;
            this.label11.Text = "Enter Code to Test eg (FF,FF,FF)";
            // 
            // ColorEntryBox
            // 
            this.ColorEntryBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ColorEntryBox.Location = new System.Drawing.Point(670, 196);
            this.ColorEntryBox.Name = "ColorEntryBox";
            this.ColorEntryBox.Size = new System.Drawing.Size(102, 22);
            this.ColorEntryBox.TabIndex = 86;
            this.ColorEntryBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.ColorEntryBox.TextChanged += new System.EventHandler(this.ColorEntryBox_TextChanged);
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label10.Location = new System.Drawing.Point(451, 174);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(198, 15);
            this.label10.TabIndex = 85;
            this.label10.Text = "DirectOutput51.ini Color Code";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label9.Location = new System.Drawing.Point(728, 24);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(42, 18);
            this.label9.TabIndex = 84;
            this.label9.Text = "HEX";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.Location = new System.Drawing.Point(673, 24);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(43, 18);
            this.label8.TabIndex = 83;
            this.label8.Text = "DEC";
            // 
            // MemoryScanButton
            // 
            this.MemoryScanButton.BackColor = System.Drawing.SystemColors.Menu;
            this.MemoryScanButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.MemoryScanButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.MemoryScanButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.MemoryScanButton.Location = new System.Drawing.Point(9, 163);
            this.MemoryScanButton.Name = "MemoryScanButton";
            this.MemoryScanButton.Size = new System.Drawing.Size(125, 55);
            this.MemoryScanButton.TabIndex = 82;
            this.MemoryScanButton.Text = "SCAN\r\nBOARD\r\nMEMORY";
            this.MemoryScanButton.UseVisualStyleBackColor = false;
            this.MemoryScanButton.Click += new System.EventHandler(this.MemoryScanButton_Click);
            // 
            // OutputSelectBox
            // 
            this.OutputSelectBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 21.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.OutputSelectBox.Location = new System.Drawing.Point(298, 44);
            this.OutputSelectBox.Maximum = new decimal(new int[] {
            64,
            0,
            0,
            0});
            this.OutputSelectBox.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.OutputSelectBox.Name = "OutputSelectBox";
            this.OutputSelectBox.Size = new System.Drawing.Size(75, 40);
            this.OutputSelectBox.TabIndex = 81;
            this.OutputSelectBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.OutputSelectBox.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            // 
            // UseRGBChecked
            // 
            this.UseRGBChecked.AutoSize = true;
            this.UseRGBChecked.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.UseRGBChecked.Location = new System.Drawing.Point(298, 186);
            this.UseRGBChecked.Name = "UseRGBChecked";
            this.UseRGBChecked.Size = new System.Drawing.Size(107, 24);
            this.UseRGBChecked.TabIndex = 80;
            this.UseRGBChecked.Text = "RGB LED";
            this.UseRGBChecked.UseVisualStyleBackColor = true;
            // 
            // DOFButton
            // 
            this.DOFButton.BackColor = System.Drawing.SystemColors.Highlight;
            this.DOFButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.DOFButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.DOFButton.Location = new System.Drawing.Point(298, 97);
            this.DOFButton.Name = "DOFButton";
            this.DOFButton.Size = new System.Drawing.Size(75, 76);
            this.DOFButton.TabIndex = 79;
            this.DOFButton.Text = "SINGLE\r\nTEST";
            this.DOFButton.UseVisualStyleBackColor = false;
            this.DOFButton.Click += new System.EventHandler(this.DOFButton_Click);
            // 
            // HexOutputBox
            // 
            this.HexOutputBox.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.HexOutputBox.Location = new System.Drawing.Point(670, 170);
            this.HexOutputBox.Name = "HexOutputBox";
            this.HexOutputBox.ReadOnly = true;
            this.HexOutputBox.Size = new System.Drawing.Size(102, 22);
            this.HexOutputBox.TabIndex = 78;
            this.HexOutputBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // HexBoxBlue
            // 
            this.HexBoxBlue.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.HexBoxBlue.Location = new System.Drawing.Point(727, 132);
            this.HexBoxBlue.Name = "HexBoxBlue";
            this.HexBoxBlue.Size = new System.Drawing.Size(44, 22);
            this.HexBoxBlue.TabIndex = 77;
            this.HexBoxBlue.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.HexBoxBlue.TextChanged += new System.EventHandler(this.HexBoxBlue_TextChanged);
            // 
            // HexBoxGreen
            // 
            this.HexBoxGreen.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.HexBoxGreen.Location = new System.Drawing.Point(727, 87);
            this.HexBoxGreen.Name = "HexBoxGreen";
            this.HexBoxGreen.Size = new System.Drawing.Size(44, 22);
            this.HexBoxGreen.TabIndex = 76;
            this.HexBoxGreen.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.HexBoxGreen.TextChanged += new System.EventHandler(this.HexBoxGreen_TextChanged);
            // 
            // HexBoxRed
            // 
            this.HexBoxRed.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.HexBoxRed.Location = new System.Drawing.Point(727, 45);
            this.HexBoxRed.Name = "HexBoxRed";
            this.HexBoxRed.Size = new System.Drawing.Size(44, 22);
            this.HexBoxRed.TabIndex = 75;
            this.HexBoxRed.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.HexBoxRed.TextChanged += new System.EventHandler(this.HexBoxRed_TextChanged);
            // 
            // ValBoxBlue
            // 
            this.ValBoxBlue.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ValBoxBlue.Location = new System.Drawing.Point(672, 132);
            this.ValBoxBlue.Name = "ValBoxBlue";
            this.ValBoxBlue.Size = new System.Drawing.Size(45, 22);
            this.ValBoxBlue.TabIndex = 74;
            this.ValBoxBlue.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.ValBoxBlue.TextChanged += new System.EventHandler(this.ValBoxBlue_TextChanged);
            // 
            // ValBoxGreen
            // 
            this.ValBoxGreen.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ValBoxGreen.Location = new System.Drawing.Point(672, 87);
            this.ValBoxGreen.Name = "ValBoxGreen";
            this.ValBoxGreen.Size = new System.Drawing.Size(45, 22);
            this.ValBoxGreen.TabIndex = 73;
            this.ValBoxGreen.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.ValBoxGreen.TextChanged += new System.EventHandler(this.ValBoxGreen_TextChanged);
            // 
            // ValBoxRed
            // 
            this.ValBoxRed.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ValBoxRed.Location = new System.Drawing.Point(672, 45);
            this.ValBoxRed.Name = "ValBoxRed";
            this.ValBoxRed.Size = new System.Drawing.Size(45, 22);
            this.ValBoxRed.TabIndex = 72;
            this.ValBoxRed.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.ValBoxRed.TextChanged += new System.EventHandler(this.ValBoxRed_TextChanged);
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.Location = new System.Drawing.Point(389, 131);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(51, 18);
            this.label7.TabIndex = 71;
            this.label7.Text = "BLUE";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.Location = new System.Drawing.Point(389, 86);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(67, 18);
            this.label6.TabIndex = 70;
            this.label6.Text = "GREEN";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.Location = new System.Drawing.Point(389, 44);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(43, 18);
            this.label5.TabIndex = 69;
            this.label5.Text = "RED";
            // 
            // SliderBlue
            // 
            this.SliderBlue.Location = new System.Drawing.Point(454, 126);
            this.SliderBlue.Maximum = 255;
            this.SliderBlue.Name = "SliderBlue";
            this.SliderBlue.Size = new System.Drawing.Size(212, 45);
            this.SliderBlue.TabIndex = 68;
            this.SliderBlue.Scroll += new System.EventHandler(this.SliderBlue_Scroll);
            // 
            // SliderGreen
            // 
            this.SliderGreen.Location = new System.Drawing.Point(454, 81);
            this.SliderGreen.Maximum = 255;
            this.SliderGreen.Name = "SliderGreen";
            this.SliderGreen.Size = new System.Drawing.Size(212, 45);
            this.SliderGreen.TabIndex = 67;
            this.SliderGreen.Scroll += new System.EventHandler(this.SliderGreen_Scroll);
            // 
            // SliderRed
            // 
            this.SliderRed.Location = new System.Drawing.Point(454, 39);
            this.SliderRed.Maximum = 255;
            this.SliderRed.Name = "SliderRed";
            this.SliderRed.Size = new System.Drawing.Size(212, 45);
            this.SliderRed.TabIndex = 66;
            this.SliderRed.Scroll += new System.EventHandler(this.SliderRed_Scroll);
            // 
            // DiagDisconnectButton
            // 
            this.DiagDisconnectButton.BackColor = System.Drawing.SystemColors.GrayText;
            this.DiagDisconnectButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.DiagDisconnectButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.DiagDisconnectButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.DiagDisconnectButton.Location = new System.Drawing.Point(145, 27);
            this.DiagDisconnectButton.Name = "DiagDisconnectButton";
            this.DiagDisconnectButton.Size = new System.Drawing.Size(125, 52);
            this.DiagDisconnectButton.TabIndex = 65;
            this.DiagDisconnectButton.Text = "DISCONNECT";
            this.DiagDisconnectButton.UseVisualStyleBackColor = false;
            this.DiagDisconnectButton.Click += new System.EventHandler(this.DiagDisconnectButton_Click);
            // 
            // DiagConnectButton
            // 
            this.DiagConnectButton.BackColor = System.Drawing.SystemColors.GrayText;
            this.DiagConnectButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.DiagConnectButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.DiagConnectButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.DiagConnectButton.Location = new System.Drawing.Point(9, 27);
            this.DiagConnectButton.Name = "DiagConnectButton";
            this.DiagConnectButton.Size = new System.Drawing.Size(125, 52);
            this.DiagConnectButton.TabIndex = 64;
            this.DiagConnectButton.Text = "CONNECT";
            this.DiagConnectButton.UseVisualStyleBackColor = false;
            this.DiagConnectButton.Click += new System.EventHandler(this.DiagConnectButton_Click);
            // 
            // ClearDiagBoxButton
            // 
            this.ClearDiagBoxButton.BackColor = System.Drawing.SystemColors.AppWorkspace;
            this.ClearDiagBoxButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.ClearDiagBoxButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.ClearDiagBoxButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ClearDiagBoxButton.Location = new System.Drawing.Point(696, 243);
            this.ClearDiagBoxButton.Name = "ClearDiagBoxButton";
            this.ClearDiagBoxButton.Size = new System.Drawing.Size(83, 114);
            this.ClearDiagBoxButton.TabIndex = 63;
            this.ClearDiagBoxButton.Text = "CLEAR STATUS BOX";
            this.ClearDiagBoxButton.UseVisualStyleBackColor = false;
            this.ClearDiagBoxButton.Click += new System.EventHandler(this.ClearDiagBoxButton_Click);
            // 
            // rtbDiagData
            // 
            this.rtbDiagData.BackColor = System.Drawing.Color.DarkGray;
            this.rtbDiagData.Location = new System.Drawing.Point(8, 242);
            this.rtbDiagData.Name = "rtbDiagData";
            this.rtbDiagData.ReadOnly = true;
            this.rtbDiagData.Size = new System.Drawing.Size(689, 115);
            this.rtbDiagData.TabIndex = 62;
            this.rtbDiagData.Text = "";
            // 
            // ResetButton
            // 
            this.ResetButton.BackColor = System.Drawing.Color.LightCoral;
            this.ResetButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.ResetButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.ResetButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ResetButton.Location = new System.Drawing.Point(145, 94);
            this.ResetButton.Name = "ResetButton";
            this.ResetButton.Size = new System.Drawing.Size(124, 55);
            this.ResetButton.TabIndex = 61;
            this.ResetButton.Text = "FACTORY\r\nRESET";
            this.ResetButton.UseVisualStyleBackColor = false;
            this.ResetButton.Click += new System.EventHandler(this.ResetButton_Click_1);
            // 
            // ScanButton
            // 
            this.ScanButton.BackColor = System.Drawing.SystemColors.Menu;
            this.ScanButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.ScanButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.ScanButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ScanButton.Location = new System.Drawing.Point(9, 94);
            this.ScanButton.Name = "ScanButton";
            this.ScanButton.Size = new System.Drawing.Size(125, 55);
            this.ScanButton.TabIndex = 60;
            this.ScanButton.Text = "SCAN\r\nFOR BOARDS";
            this.ScanButton.UseVisualStyleBackColor = false;
            this.ScanButton.Click += new System.EventHandler(this.ScanButton_Click_1);
            // 
            // TestButton
            // 
            this.TestButton.BackColor = System.Drawing.SystemColors.GradientInactiveCaption;
            this.TestButton.FlatAppearance.BorderColor = System.Drawing.Color.FromArgb(((int)(((byte)(64)))), ((int)(((byte)(64)))), ((int)(((byte)(64)))));
            this.TestButton.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.TestButton.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.TestButton.Location = new System.Drawing.Point(145, 163);
            this.TestButton.Name = "TestButton";
            this.TestButton.Size = new System.Drawing.Size(124, 55);
            this.TestButton.TabIndex = 57;
            this.TestButton.Text = "AUTO TEST";
            this.TestButton.UseVisualStyleBackColor = false;
            this.TestButton.Click += new System.EventHandler(this.TestButton_Click_1);
            // 
            // cbOutputControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.BackColor = System.Drawing.SystemColors.ControlDark;
            this.ClientSize = new System.Drawing.Size(800, 450);
            this.Controls.Add(this.tabControl);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.picBoxcbLogo);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "cbOutputControl";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "cabBlaster OUTPUT Control";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.cbOutputControl_FormClosed);
            this.Load += new System.EventHandler(this.cbOutputControl_Load);
            ((System.ComponentModel.ISupportInitialize)(this.picBoxcbLogo)).EndInit();
            this.tabControl.ResumeLayout(false);
            this.MainPage.ResumeLayout(false);
            this.MainPage.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.PWMAdjust)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.TimeAdjuster)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.OutputSelectionBox)).EndInit();
            this.tabPage2.ResumeLayout(false);
            this.tabPage2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.OutputSelectBox)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.SliderBlue)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.SliderGreen)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.SliderRed)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.IO.Ports.SerialPort ComPort;
        private System.Windows.Forms.PictureBox picBoxcbLogo;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TabControl tabControl;
        private System.Windows.Forms.TabPage MainPage;
        private System.Windows.Forms.TabPage tabPage2;
        private System.Windows.Forms.RichTextBox rtbInfo;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.NumericUpDown PWMAdjust;
        private System.Windows.Forms.Button SavedValueButton;
        private System.Windows.Forms.Button SendButton;
        private System.Windows.Forms.NumericUpDown TimeAdjuster;
        private System.Windows.Forms.Button ClearButton;
        private System.Windows.Forms.Button SetButton;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.CheckBox NightModeSetChkBox;
        private System.Windows.Forms.RichTextBox rtbDataIn;
        private System.Windows.Forms.Button ComDisconnectButton;
        private System.Windows.Forms.NumericUpDown OutputSelectionBox;
        private System.Windows.Forms.CheckBox EnableSolPWM;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.ComboBox ComPortBox;
        private System.Windows.Forms.Button DiagDisconnectButton;
        private System.Windows.Forms.Button DiagConnectButton;
        private System.Windows.Forms.Button ClearDiagBoxButton;
        private System.Windows.Forms.RichTextBox rtbDiagData;
        private System.Windows.Forms.Button ResetButton;
        private System.Windows.Forms.Button ScanButton;
        private System.Windows.Forms.Button TestButton;
        private System.Windows.Forms.Button ConnectButton;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TrackBar SliderBlue;
        private System.Windows.Forms.TrackBar SliderGreen;
        private System.Windows.Forms.TrackBar SliderRed;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox HexOutputBox;
        private System.Windows.Forms.TextBox HexBoxBlue;
        private System.Windows.Forms.TextBox HexBoxGreen;
        private System.Windows.Forms.TextBox HexBoxRed;
        private System.Windows.Forms.TextBox ValBoxBlue;
        private System.Windows.Forms.TextBox ValBoxGreen;
        private System.Windows.Forms.TextBox ValBoxRed;
        private System.Windows.Forms.Button DOFButton;
        private System.Windows.Forms.CheckBox UseRGBChecked;
        private System.Windows.Forms.NumericUpDown OutputSelectBox;
        private System.Windows.Forms.Button MemoryScanButton;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox ColorEntryBox;
    }
}

