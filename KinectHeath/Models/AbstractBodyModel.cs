using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Vision.Systems.KinectHealth.Libraries;

namespace Vision.Systems.KinectHealth.Models
{
    abstract class AbstractBodyModel
    {

        protected double sigma;

        protected double sigma_ref;

        public abstract bool hasEnoughSamples(Sampler[] samplers);

        public abstract double ComputeIndex(Sampler[] samplers);

        protected abstract double weighting(double mean);
    }
}
