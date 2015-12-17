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
    class ComplexUpperBodyModelCm : BodyModel
    {


        public ComplexUpperBodyModelCm(double r, double ll, double lu)
        {
            this.r = r;

            this.ll = ll;

            this.lu = lu;
        }

        protected override double weighting(double mean)
        {
            if (mean < this.ll)
            {
                return mean / this.ll;
            }

            else if (this.ll <= mean && mean <= this.lu)
            {
                return 1;
            }

            else if(this.lu < mean && mean <= 2*this.lu)
            {
                return (mean - 2 * this.lu) / -this.lu;
            }

            else
            {
                return 0;
            }
        }
    }
}
