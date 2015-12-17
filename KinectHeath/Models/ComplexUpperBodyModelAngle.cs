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
    class ComplexUpperBodyModelAngle : BodyModel
    {


        public ComplexUpperBodyModelAngle(double r, double ll, double lu, double el, double eu)
        {
            this.r = r;

            this.ll = ll;

            this.lu = lu;

            this.el = el;

            this.eu = eu;
        }

        protected override double weighting(double mean)
        {
            if (mean < this.ll - this.el)
            {
                return mean / (this.ll - this.el);
            }

            else if (this.ll - this.el <= mean && mean <= this.lu + this.eu)
            {
                return 1;
            }

            else
            {
                return (mean - Math.PI)/(this.lu + this.eu - Math.PI);
            }
        }
    }
}
