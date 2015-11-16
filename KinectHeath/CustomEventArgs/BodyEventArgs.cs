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
        public Dictionary<ulong,Dictionary<JointType, Point>> jointPointsPerBody { get; set; }
    }
}
