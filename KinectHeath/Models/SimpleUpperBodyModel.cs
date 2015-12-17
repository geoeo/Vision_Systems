using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;
using Vision.Systems.KinectHealth.CustomEventArgs;
using Vision.Systems.KinectHealth.Libraries;

namespace Vision.Systems.KinectHealth.Models
{
    class SimpleUpperBodyModel : BodyModel
    {

        public SimpleUpperBodyModel(double r)
        {
            this.r = r;
        }

        protected override double weighting(double mean)
        {
            var e_UB_forward = 0d; // error is 0 as detailed in section 3.3.1

            return mean <= EmpiricalData.LL_UB_FORWARD - e_UB_forward ? mean / (EmpiricalData.LL_UB_FORWARD - e_UB_forward) : 1;
        }
    }
}
