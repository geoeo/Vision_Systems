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
    class SimpleUpperBodyModel : AbstractBodyModel
    {

        public SimpleUpperBodyModel(double r, double sigma_ref)
        {
            this.sigma_ref = sigma_ref;
            this.sigma = -1;
        }

        public override bool hasEnoughSamples(Sampler[] samplers)
        {
            return samplers[Constants.UB_FORWARD_SAMPLE_INDEX].measurementFrame.Count == Constants.NUMBER_OF_MEASUREMENT_FRAME;
        }

        public override double ComputeIndex(Sampler[] samplers)
        {
            var ub_forward_samples = samplers[Constants.UB_FORWARD_SAMPLE_INDEX].measurementFrame;
            var mean = ub_forward_samples.Aggregate(0d, (seed, v) => seed + v) / Constants.NUMBER_OF_MEASUREMENT_FRAME;
            var sum_diff = ub_forward_samples.Select(x => x - mean).Aggregate(0d, (seed, v) => seed + v * v);
            this.sigma = Math.Sqrt(sum_diff / Constants.NUMBER_OF_MEASUREMENT_FRAME);

            var weight = weighting(mean);

            return 1 - (this.sigma * weight / this.sigma_ref);
        }

        protected override double weighting(double mean)
        {
            var e_UB_forward = 0d; // error is 0 as detailed in section 3.3.1
            var l_UB_forward = Math.PI/2; // angle is 90 degrees as detailed in section 3.3.1

            return mean <= l_UB_forward - e_UB_forward ? mean / (l_UB_forward - e_UB_forward) : 1;
        }
    }
}
