using System;
using System.Linq;
using System.Threading;
using System.IO.Ports;

namespace DirectOutput.Cab.Out.cabBlaster
{

    public class cabBlaster : OutputControllerFlexCompleteBase
    {
        #region Number


        private object NumberUpdateLocker = new object();
        private int _Number = -1;

        /// <summary>
        /// Gets or sets the unit number of the controller.<br />
        /// The unit number must be unique.<br />
        /// Setting changes the Name property, if it is blank or if the Name coresponds to "Zebsboards Controller {Number}".
        /// </summary>
        /// <value>
        /// The unique unit number of the controller (Range 1-8 translates to directoutputconfig# ie: directouputconfig51, directoutputconfig52, etc).
		/// In reality the control board comes preconfigured as unit number 1 (address 0x40 - 0), so this is redundant unless a change in the board design and firmware opens this up
        /// </value>
        /// <exception cref="System.Exception">
        /// Zebsboards Unit Numbers must be between 1-16 (restriction in DOF limiting the max number of devices assignable).
        /// </exception>
        public int Number
        {
            get { return _Number; }
            set
            {
                if (!value.IsBetween(1, 8))
                {
                    throw new Exception("Error! Unit Numbers must be between 1-8. The supplied number {0} is out of range.".Build(value));
                }
                lock (NumberUpdateLocker)
                {
                    // if the unit number changed, update it and attach to the new unit
                    if (_Number != value)
                    {
                        // if we used a default name for the old unit number, change to the default
                        // name for the new unit number
                        if (Name.IsNullOrWhiteSpace() || Name == "cabBlaster {0:00}".Build(_Number))
                        {
                            Name = "cabBlaster {0:00}".Build(value);
                        }

                        // remember the new unit number
                        _Number = value;

                        // attach to the new device record for this unit number, updating the output list to match - can theoretically be expanded to 255 but there is no need for that many outputs ever
                        this.NumberOfOutputs = 64;
                        this.OldOutputValues = Enumerable.Repeat((byte)255, this.NumberOfOutputs).ToArray();
                    }
                }
            }
        }

        #endregion


        #region MinCommandIntervalMs property core parts
        private int _MinCommandIntervalMs = 10;
        private bool MinCommandIntervalMsSet = false;

        /// <summary>
        /// Gets or sets the mininimal interval between command in miliseconds (Default: 1ms).
        /// </summary>
        /// <value>
        /// The mininimal interval between command in miliseconds.  The default is 1ms, which is also the minimum, since it's
        /// the fastest that USB allows at the hardware protocol level.  The GlobalConfig.xml utimately controls this if set.
        /// </value>
        public int MinCommandIntervalMs
        {
            get { return _MinCommandIntervalMs; }
            set
            {
                _MinCommandIntervalMs = value.Limit(0, 1000);
                MinCommandIntervalMsSet = true;
            }
        }

        #endregion

        #region ComPort property core parts
        private string _ComPort = "comm1";
        private bool ComPortSet = false;
        private SerialPort Port = null;
        private object PortLocker = new object();

        /// <summary>
        /// Gets or sets the mininimal interval between command in miliseconds (Default: 1ms).
        /// </summary>
        /// <value>
        /// The mininimal interval between command in miliseconds.  The default is 1ms, which is also the minimum, since it's
        /// the fastest that USB allows at the hardware protocol level.
        /// </value>
        public string ComPort
        {
            get { return _ComPort; }
            set
            {
                _ComPort = value;
                ComPortSet = true;
            }
        }

        #endregion

        #region IOutputcontroller implementation

        /// <summary>
        /// Initializes the cabBlaster object.<br />
        /// This method does also start the workerthread which does the actual update work when Update() is called.<br />
        /// This method should only be called once. Subsequent calls have no effect.
        /// </summary>
        /// <param name="Cabinet">The Cabinet object which is using the cabBlaster instance.</param>
        public override void Init(Cabinet Cabinet)
        {
            // get the minimum update interval from the global config
            if (!MinCommandIntervalMsSet
                && Cabinet.Owner.ConfigurationSettings.ContainsKey("cabBlasterDefaultMinCommandIntervalMs")
                && Cabinet.Owner.ConfigurationSettings["cabBlasterDefaultMinCommandIntervalMs"] is int)
                MinCommandIntervalMs = (int)Cabinet.Owner.ConfigurationSettings["cabBlasterDefaultMinCommandIntervalMs"];
            // get the comm port from the global config
            if (!ComPortSet
                && Cabinet.Owner.ConfigurationSettings.ContainsKey("cabBlasterComPort")
                && Cabinet.Owner.ConfigurationSettings["cabBlasterComPort"] is string)
                ComPort = (string)Cabinet.Owner.ConfigurationSettings["cabBlasterComPort"];

            // do the base class work
            base.Init(Cabinet);
        }

        /// <summary>
        /// Finishes the Zebsboards object.<br/>
        /// Finish does also terminate the workerthread for updates.
        /// </summary>
        public override void Finish()
        {
            base.Finish();
        }
        #endregion


        #region OutputControllerFlexCompleteBase implementation

        /// <summary>
        /// Verify settings.  Returns true if settings are valid, false otherwise.  In the current implementation,
        /// there's nothing to check; we simply return true unconditionally.
        /// </summary>
        protected override bool VerifySettings()
        {
            if (ComPort.IsNullOrWhiteSpace())
            {
                Log.Warning("ComPort is not set for {0} {1}.".Build(this.GetType().Name, Name));
                return false;
            }

            if (!SerialPort.GetPortNames().Any(x => x.Equals(ComPort, StringComparison.InvariantCultureIgnoreCase)))
            {
                Log.Warning("ComPort {2} is defined for {0} {1}, but does not exist.".Build(this.GetType().Name, Name, ComPort));
                return false;
            };

            return true;
        }

        /// <summary>
        /// Send updated outputs to the physical device.
        /// </summary>
        protected override void UpdateOutputs(byte[] NewOutputValues)
        {
            // The extended protocol lets us update outputs in blocks of 7.
            // Run through our output list and send an update for each bank
            // that's changed.  The extended protocol message starts with
            // a byte set to 200+B, where B is the bank number - B=0 for
            // outputs 1-7, B=1 for outputs 8-14, etc.
            //
            // Note that, unlike the LedWiz protocol, the extended protocol
            // uses ONLY the brightness value to control each output.  There's
            // no separate on/off state.  "Off" is simply a brightness of 0 and ON is a bightness of 255.
            byte pfx = 200;
            for (int i = 0; i < NumberOfOutputs; i += 8, ++pfx)
            {
                // look for a change among this bank's 8 outputs
                int lim = Math.Min(i + 8, NumberOfOutputs);
                for (int j = i; j < lim; ++j)
                {
                    // if this output has changed, flush the bank
                    if (NewOutputValues[j] != OldOutputValues[j])
                    {
                        // found a change - send the bank
                        UpdateDelay();
                        byte[] buf = new byte[10];
                        buf[0] = 0x4F;          // USB report ID - always O for "OUTPUT"
                        buf[1] = pfx;			// message prefix which denotes the Bank identifier for the outputss being sent
                        Array.Copy(NewOutputValues, i, buf, 2, lim - i);
                        Port.Write(buf, 0, buf.Length);

                        // the new values are now the current values on the device
                        Array.Copy(NewOutputValues, i, OldOutputValues, i, lim - i);

                        // we've sent this whole bank of 8 - move on to the next 
                        break;
                    }
                }
            }
        }
		
		byte[] OldOutputValues;		// Used to store the previous values that were sent for comparison when looking for a change in the current values

        private DateTime LastUpdate = DateTime.Now;
        private void UpdateDelay()
        {
            int Ms = (int)DateTime.Now.Subtract(LastUpdate).TotalMilliseconds;
            if (Ms < MinCommandIntervalMs)
                Thread.Sleep((MinCommandIntervalMs - Ms).Limit(0, MinCommandIntervalMs));
            LastUpdate = DateTime.Now;
        }

        /// <summary>
        /// Connect to the controller.
        /// </summary>
        protected override void ConnectToController()
        {
            try
            {
                lock (PortLocker)
                {
                    if (Port != null)
                    {
                        DisconnectFromController();
                    }

                    Port = new SerialPort(ComPort, 1000000, Parity.None, 8, StopBits.One);
                    Port.NewLine = "\r\n";
                    Port.ReadTimeout = 500;
                    Port.WriteTimeout = 500;
                    Port.Open();
                    Port.DtrEnable = true;
                }
            }
            catch (Exception E)
            {
                string Msg = "A exception occured while opening comport {2} for {0} {1}.".Build(this.GetType().Name, Name, ComPort);
                Log.Exception(Msg, E);
                throw new Exception(Msg, E);
            }
        }

        /// <summary>
        /// Disconnect from the controller.
        /// </summary>
        protected override void DisconnectFromController()
        {
            lock (PortLocker)
            {
                if (Port != null)
                {
                    Port.Close();
                    Port = null;
                }

            }
        }
        /// <summary>
        /// Initializes a new instance of the <see cref="Zebsboards"/> class with a given com port.
        /// </summary>
        /// <param name="ComPort">The ComPort of the controller.</param>
        public cabBlaster(string ComPort)
        {
            this.ComPort = ComPort;
            this.Number = 1;
        }
        #endregion


    }
}
