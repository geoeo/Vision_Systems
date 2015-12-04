using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Vision.Systems.KinectHealth.Libraries
{
    public static class Constants
    {

       public const int V_NECK = 0;
       public const int V_SHOULDER = 1;
       public const int V_UPPER_BODY = 2;
       public const int V_THIGH_RIGHT = 3;
       public const int V_THIGH_LEFT = 4;
       public const int V_SHANK_LEFT = 5;
       public const int V_SHANK_RIGHT = 6;

       public const int A_NECK = 0;
       public const int A_HIP_LEFT = 1;
       public const int A_HIP_RIGHT = 2;
       public const int A_KNEE_LEFT = 3;
       public const int A_KNEE_RIGHT = 4;

       // a bone defined as a line between two joints
       public static readonly IList<Tuple<JointType, JointType>> bones = new List<Tuple<JointType, JointType>>() {
           // Torso
           new Tuple<JointType, JointType>(JointType.Head, JointType.Neck),
           new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder),
           new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid),
           new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase),
           new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight),
           new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft),
           new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight),
           new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft),
           
           // Right Arm
           new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight),
           new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight),
           new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight),
           new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight),
           new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight),
           
           // Left Arm
           new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft),
           new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft),
           new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft),
           new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft),
           new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft),
           
           // Right Leg
           new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight),
           new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight),
           new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight),
           
           // Left Leg
           new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft),
           new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft),
           new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft),
       };
    }
}
