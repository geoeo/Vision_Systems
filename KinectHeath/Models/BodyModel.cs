using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Vision.Systems.KinectHealth.Libraries;

namespace Vision.Systems.KinectHealth.Models
{
    abstract class BodyModel
    {

        protected double sigma;

        protected double r;

        protected double ll;

        protected double lu;

        protected double el;

        protected double eu;

        public bool hasEnoughSamples(Sampler[] samplers)
        {
            return samplers[Constants.UB_FORWARD_SAMPLE_INDEX].measurementFrame.Count == Constants.NUMBER_OF_MEASUREMENT_FRAME;
        }

        public double ComputeIndex(Sampler[] samplers)
        {
            var sigma_ref = EmpiricalData.sigma(this.r);
            var ub_forward_samples = samplers[Constants.UB_FORWARD_SAMPLE_INDEX].measurementFrame;
            var mean = ub_forward_samples.Aggregate(0d, (seed, v) => seed + v) / Constants.NUMBER_OF_MEASUREMENT_FRAME;
            var sum_diff = ub_forward_samples.Select(x => x - mean).Aggregate(0d, (seed, v) => seed + v * v);

            this.sigma = Math.Sqrt(sum_diff / Constants.NUMBER_OF_MEASUREMENT_FRAME);

            var weight = weighting(mean);

            return 1 - (this.sigma * weight / sigma_ref);
        }
        protected abstract double weighting(double mean);
    }
}
