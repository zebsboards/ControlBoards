using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Configuration_Utility
{
    public partial class Menu : Form
    {
        public Menu()
        {
            InitializeComponent();
        }

        string Revision = "Revision Date  2025.06.13";
        string Build = "Build     1.0.1.0";
        private void Menu_Load(object sender, EventArgs e)
        {
            lblRevision.Text = Revision;
            LabelBuild.Text = Build;
        }
        private void cbOutputSelect_MouseHover(object sender, EventArgs e)
        {
            ProductPic.Image = Properties.Resources.cbControlResized;
            cbOutputDesc.Show();
            PlungerDesc.Hide();
            cbSerialDesc.Hide();
        }
        private void cbOutputSelect_Click(object sender, EventArgs e)
        {
            this.Hide();
            cbOutputControl cbOutputControl = new cbOutputControl();
            cbOutputControl.Show();
        }
        private void cbSerialSelect_MouseHover(object sender, EventArgs e)
        {
            ProductPic.Image = Properties.Resources.cbSerial14;
            cbSerialDesc.Show();
            cbOutputDesc.Hide();
            PlungerDesc.Hide();
        }
        private void cbSerialSelect_Click(object sender, EventArgs e)
        {
            this.Hide();
            cbSerialControl cbSerialControl = new cbSerialControl();
            cbSerialControl.Show();
        }
        private void cbPlungerSelect_MouseHover(object sender, EventArgs e)
        {
            cbOutputDesc.Hide();
            PlungerDesc.Show();
            cbSerialDesc.Hide();
            ProductPic.Image = Properties.Resources.v7plunger;
        }
        private void cbPlungerSelect_Click(object sender, EventArgs e)
        {
            this.Hide();
            cbPlungerControl cbPlungerControl = new cbPlungerControl();
            cbPlungerControl.Show();
        }
        private void DOWNLOAD_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Process.Start("https://www.zebsboards.ca/public_html/support_docs/DOF%20Installation_Upgrade.zip");
        }
        private void PaypalButton_Click(object sender, EventArgs e)
        {
            System.Diagnostics.Process.Start("https://www.paypal.com/donate/?hosted_button_id=CHAG4YZRQCSFJ");
        }

        private void Menu_FormClosed(object sender, FormClosedEventArgs e)
        {
            Application.Exit();
        }
    }
}