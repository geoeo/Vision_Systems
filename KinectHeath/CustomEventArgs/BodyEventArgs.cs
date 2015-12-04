using System;
using System.Collections.Generic;
using Microsoft.Kinect;
using System.Windows;

namespace Vision.Systems.KinectHealth.CustomEventArgs
{
    class BodyEventArgs : EventArgs
    {
        public Body[] bodies { get; set; }

        // joint points in depth (display) space per body
        public IDictionary<ulong,Dictionary<JointType, Point>> jointPointsPerBody { get; set; }

        //Angles between two joints as described in Wiedemann 3.2
        public IDictionary<Tuple<JointType, JointType>, double> jointAngles { get; set; }

        public double[] upperBodyAngles { get; set; }
    }
}
