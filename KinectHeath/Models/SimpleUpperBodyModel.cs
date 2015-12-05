using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;
using Vision.Systems.KinectHealth.CustomEventArgs;
namespace Vision.Systems.KinectHealth.Libraries
{
    class SimpleUpperBodyModel
    {

        // Measure for 90 second i.e. 2700 frames, as stated in Wiedemann 3.3.1
        private const int NUMBER_OF_MEASUREMENT_FRAME = 10*30;

        private int measuredFrames;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        public KinectSensor kinectSensor = null;

        /// <summary>
        /// EventHandler which fires once the observation is complete
        /// </summary>
        public EventHandler<EventArgs> modelComplete;

        /// <summary>
        /// EventHandler which fires once the observation is starting
        /// </summary>
        public EventHandler<EventArgs> modelStarting;

        /// <summary>
        /// The event handler the View-Model is subscribed too
        /// </summary>
        private EventHandler<BodyEventArgs> uiHandler;

        /// <summary>
        /// Reference to the EventHandler the View-Model is subscribed too.
        /// </summary>
        private BodyFrameReader bodyFrameReader;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        public double sigma_ub = -1;

        /// <summary>
        /// Experimental value r ( in radians ) as described in 3.3.1 TODO (in paper r is given in degrees ?)
        /// </summary>
        public const double r = (8 * Math.PI) / 180;

        public const double sigma_ref = r / (1.96 * 2);

        // Max size = NUMBER_OF_MEASUREMENT_FRAME
        public Queue<double> measurementFrame = null;



        public SimpleUpperBodyModel(KinectSensor sensor)
        {
            this.kinectSensor = sensor;

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.measurementFrame = new Queue<double>();
        }

        /// <summary>
        /// Adds samples to the measurement frame. If frame is full, it will remove the oldest (first)
        /// </summary>
        /// <param name="UB_Forward">forward angle</param>
        /// <returns>Returns true if there are enough samples</returns>
        internal bool SupplySamples(double UB_Forward)
        {
            if (measurementFrame.Count == NUMBER_OF_MEASUREMENT_FRAME)
                measurementFrame.Dequeue();
            measurementFrame.Enqueue(UB_Forward);
 
            return measurementFrame.Count == NUMBER_OF_MEASUREMENT_FRAME;
        }

        internal double ComputeIndex()
        {
            var mean = measurementFrame.Aggregate(0d, (seed, v) => seed + v) / NUMBER_OF_MEASUREMENT_FRAME;
            var sum_diff = measurementFrame.Select(x => x - mean).Aggregate(0d, (seed, v) => seed + v * v);
            sigma_ub = Math.Sqrt(sum_diff/NUMBER_OF_MEASUREMENT_FRAME);

            //TODO Compute REST

            return -1;
        }
    }
}
