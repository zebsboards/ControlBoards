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
    public partial class WarningPopup : Form
    {

        AnswerBack transferDel;
        public WarningPopup(AnswerBack del)
        {
            InitializeComponent();
            transferDel = del;
        }   
        
        public void CancelButton_Click(object sender, EventArgs e)
        {
            bool ResetResponse = false;
            transferDel.Invoke(ResetResponse);
            Close();
        }
        
        public void ProceedButton_Click(object sender, EventArgs e)
        {
            bool ResetResponse = true;
            transferDel.Invoke(ResetResponse);
            Close();
        }
    }
}
