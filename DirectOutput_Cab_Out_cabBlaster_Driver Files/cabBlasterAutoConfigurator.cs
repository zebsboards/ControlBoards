using DirectOutput.Cab.Toys.LWEquivalent;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;

namespace DirectOutput.Cab.Out.cabBlaster
{
    public class cabBlasterConfigurator : IAutoConfigOutputController
    {
        #region IAutoConfigOutputController Member

        /// <summary>
        /// This method detects and configures Zebsboards cabBlaster output controllers automatically.
        /// </summary>
        /// <param name="Cabinet">The cabinet object to which the automatically detected IOutputController objects are added if necessary.</param>
        public void AutoConfig(Cabinet Cabinet)
        {
            /// Emulate the Pinscape Device entry in the configTool to avoid having to create something new when a usable entry exists
			const int UnitBias = 50;
            List<string> Preconfigured = new List<string>(Cabinet.OutputControllers.Where(OC => OC is cabBlaster).Select(PO => ((cabBlaster)PO).ComPort));
            String comPort = GetDevice();

            if (!Preconfigured.Contains(comPort) && comPort != "")
            {
                cabBlaster p = new cabBlaster(comPort);
                if (!Cabinet.OutputControllers.Contains(p.Name))
                {
                    Cabinet.OutputControllers.Add(p);
                    Log.Write("Detected and added Zebsboards cabBlaster Controller Nr. {0} with name {1}".Build(p.Number, p.Name));

                    if (!Cabinet.Toys.Any(T => T is LedWizEquivalent && ((LedWizEquivalent)T).LedWizNumber == p.Number + UnitBias))
                    {
                        LedWizEquivalent LWE = new LedWizEquivalent();
                        LWE.LedWizNumber = p.Number + UnitBias;
                        LWE.Name = "{0} LEDWiz Equivalent".Build(p.Name);

                        for (int i = 1; i <= p.NumberOfOutputs; i++)
                        {
                            LedWizEquivalentOutput LWEO = new LedWizEquivalentOutput() { OutputName = "{0}\\{0}.{1:00}".Build(p.Name, i), LedWizEquivalentOutputNumber = i };
                            LWE.Outputs.Add(LWEO);
                        }

                        if (!Cabinet.Toys.Contains(LWE.Name))
                        {
                            Cabinet.Toys.Add(LWE);
                            Log.Write("Added LedwizEquivalent Nr. {0} with name {1} for Zebsboards cabBlaster Controller Nr. {2}".Build(
                                LWE.LedWizNumber, LWE.Name, p.Number) + ", with {0}".Build(p.NumberOfOutputs) +" available outputs");
                        }
                    }
                }
            }

        }
        
		/// Serach for the cabBlaster output board based on the PID that is returned by serial transmission - not using a USB device search, just a serial query/response search
		public static String GetDevice()
        {
            foreach (string pf in System.IO.Ports.SerialPort.GetPortNames())
            {
                SerialPort Port = null;
                try
                {

                    Port = new SerialPort(pf, 2000000, Parity.None, 8, StopBits.One);
                    Port.NewLine = "\r\n";
                    Port.ReadTimeout = 100;
                    Port.WriteTimeout = 100;
                    Port.Open();
                    Port.DtrEnable = true;
                    Port.Write("U");
                    while (true)
                    {
                        string result = Port.ReadLine();
                        if (result == "41C5")
                        {
                            Port.Close();
                            return pf;
                        }
                    }
                }
                catch (Exception ex)
                {
                    if (Port != null)
                    {
                        Port.Close();
                    }
                }
            }
            return "";
        }

        #endregion
    }
}
