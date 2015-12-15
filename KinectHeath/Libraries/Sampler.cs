using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace Vision.Systems.KinectHealth.Libraries
{
    class Sampler
    {

        private int measuredFrames;

        // Max size = Constants.NUMBER_OF_MEASUREMENT_FRAME
        public Queue<double> measurementFrame = null;

        public Sampler()
        {
            measurementFrame = new Queue<double>();
        }

        /// <summary>
        /// Adds samples to the measurement frame. If frame is full, it will remove the oldest (first)
        /// </summary>
        /// <param name="UB_Forward">forward angle</param>
        /// <returns>Returns true if there are enough samples</returns>
        public void SupplySamples(double sample)
        {
            if (measurementFrame.Count == Constants.NUMBER_OF_MEASUREMENT_FRAME)
                measurementFrame.Dequeue();
            measurementFrame.Enqueue(sample);
        }
    }
}
