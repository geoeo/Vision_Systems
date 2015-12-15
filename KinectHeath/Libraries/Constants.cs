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

       /// <summary>
       /// Constant for clamping Z values of camera space points from being negative
       /// </summary>
       public const float InferredZPositionClamp = 0.1f;

       public const int V_NECK = 0;
       public const int V_SHOULDER = 1;
       public const int V_UPPER_BODY = 2;
       public const int V_THIGH_RIGHT = 3;
       public const int V_THIGH_LEFT = 4;
       public const int V_SHANK_LEFT = 5;
       public const int V_SHANK_RIGHT = 6;
       public const int V_SCREEN_EDGE = 7;

       public const int A_NECK = 0;
       public const int A_HIP_LEFT = 1;
       public const int A_HIP_RIGHT = 2;
       public const int A_KNEE_LEFT = 3;
       public const int A_KNEE_RIGHT = 4;

       public const int UB_FORWARD = 0;
       public const int UB_LEAN = 1;
       public const int UB_ROTATION = 2;
       public const int UB_LOS = 3;

       public const int SIMPLE_INDEX = 0;
       public const int COMPLEX_INDEX = 1;
       public const int WHOLE_INDEX = 2;

       // Measure for 90 second i.e. 2700 frames, as stated in Wiedemann 3.3.1
       public const int NUMBER_OF_MEASUREMENT_FRAME = 10 * 30;

       public const int SAMPLE_SET_SIZE = 6;

       public const int UB_FORWARD_SAMPLE_INDEX = 0;
       public const int UB_LEAN_SAMPLE_INDEX = 1;
       public const int UB_ROTATION_SAMPLE_INDEX = 2;
       public const int NECK_SAMPLE_INDEX = 3;
       public const int LOS_SAMPLE_INDEX = 4;
       public const int VIEWING_DIST_SAMPLE_INDEX = 5;

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
