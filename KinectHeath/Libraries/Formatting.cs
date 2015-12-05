using System;
using System.Windows;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using Microsoft.Kinect;

namespace Vision.Systems.KinectHealth.Libraries
{
    public static class Formatting
    {
        public static readonly IDictionary<Tuple<JointType, JointType>, int> offsetMap = new Dictionary<Tuple<JointType, JointType>,int>()
        {
            {new Tuple<JointType,JointType>(JointType.Neck,JointType.SpineShoulder),40},
            {new Tuple<JointType,JointType>(JointType.SpineBase,JointType.HipLeft),40},
            {new Tuple<JointType,JointType>(JointType.SpineBase,JointType.HipRight),-40},
            {new Tuple<JointType,JointType>(JointType.HipLeft,JointType.KneeLeft),40},
            {new Tuple<JointType,JointType>(JointType.HipRight,JointType.KneeRight),-40}
        };

        public static readonly IDictionary<Tuple<JointType, JointType>, FlowDirection> flowDirectionMap = new Dictionary<Tuple<JointType, JointType>, FlowDirection>()
        {
            {new Tuple<JointType,JointType>(JointType.Neck,JointType.SpineShoulder),FlowDirection.LeftToRight},
            {new Tuple<JointType,JointType>(JointType.SpineBase,JointType.HipLeft),FlowDirection.LeftToRight},
            {new Tuple<JointType,JointType>(JointType.SpineBase,JointType.HipRight),FlowDirection.RightToLeft},
            {new Tuple<JointType,JointType>(JointType.HipLeft,JointType.KneeLeft),FlowDirection.LeftToRight},
            {new Tuple<JointType,JointType>(JointType.HipRight,JointType.KneeRight),FlowDirection.RightToLeft}
        };

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        public static readonly List<Pen> bodyColors = new List<Pen>()
        {
            new Pen(Brushes.Red, 6),
            new Pen(Brushes.Orange, 6),
            new Pen(Brushes.Green, 6),
            new Pen(Brushes.Blue, 6),
            new Pen(Brushes.Indigo, 6),
            new Pen(Brushes.Violet, 6)
        };
    }
}
