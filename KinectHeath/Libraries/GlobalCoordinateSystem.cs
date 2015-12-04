using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;
using Vision.Systems.KinectHealth.CustomEventArgs;

namespace Vision.Systems.KinectHealth.Libraries
{
    class GlobalCoordinateSystem
    {
        public Vector3D x { get; private set; }
        public Vector3D y { get; private set; }
        public Vector3D z { get; private set; }

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        public KinectSensor kinectSensor = null;

        /// <summary>
        /// EventHandler which fires once the calibration is complete
        /// </summary>
        public EventHandler<EventArgs> calibrationComplete;

        /// <summary>
        /// EventHandler which fires once the calibration is starting
        /// </summary>
        public EventHandler<EventArgs> calibrationStarting;

        private BodyFrameReader bodyFrameReader;

        public GlobalCoordinateSystem(KinectSensor sensor)
        {
            this.kinectSensor = sensor;
        }

        /// <summary>
        /// Generates a camera independent coordinate system 
        /// PRE: All other frame arriving methods subscribed to the body-frame reader have been disabled
        /// </summary>
        public void Calibrate(EventHandler<BodyEventArgs> UIHandler)
        {
            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
        }

        /// <summary>
        /// PRE: Other readers have been disposed accordingly
        /// </summary>
        public void StartCalibration()
        {
            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            if (this.calibrationStarting != null)
                calibrationStarting(this, new EventArgs());
        }

         /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        public void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            returnReaderOwnershipToModel();
        }

        private void returnReaderOwnershipToModel()
        {
            this.bodyFrameReader.FrameArrived -= this.Reader_FrameArrived;
            this.bodyFrameReader.Dispose();

            if (calibrationComplete != null)
                calibrationComplete(this, new EventArgs());
        }

    }
}
