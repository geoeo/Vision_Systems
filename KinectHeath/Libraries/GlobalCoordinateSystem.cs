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
    class GlobalCoordinateSystem
    {
        // Measure hip joints for 1 second i.e. 30 frames, as stated in Wiedemann 3.2.1
        private const int NUMBER_OF_MEASUREMENT_FRAME = 30;

        private int measuredFrames;

        public Vector3D x { get; private set; }
        public Vector3D y { get; private set; }
        public Vector3D z { get; private set; }
        public Vector3D screen_edge { get; private set; }

        public double viewingDistance
        {
            get
            {
                return screen_edge.Length;
            }
        }

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        public KinectSensor kinectSensor = null;

        /// <summary>
        /// EventHandler which fires once the calibration is complete
        /// </summary>
        public EventHandler<EventArgs> calibrationComplete;

        /// <summary>
        /// EventHandler which fires once the calibration is starting
        /// </summary>
        public EventHandler<EventArgs> calibrationStarting;

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

        /// <summary>
        /// List containing all the measurement from a calibration phase
        /// </summary>
        private IList<Vector3D> hipMeasurements;
        /// <summary>
        /// List containing all the measurement from a calibration phase
        /// </summary>
        private IList<Vector3D> screenEdgeMeasurements;

        public GlobalCoordinateSystem(KinectSensor sensor)
        {
            this.kinectSensor = sensor;

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.hipMeasurements = new List<Vector3D>();
            this.screenEdgeMeasurements = new List<Vector3D>();
        }

        /// <summary>
        /// Generates a camera independent coordinate system 
        /// PRE: All other frame arriving methods subscribed to the body-frame reader have been disabled
        /// </summary>
        public void Calibrate(EventHandler<BodyEventArgs> uiHandler)
        {
            this.uiHandler = uiHandler;
            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
        }

        /// <summary>
        /// PRE: Other readers have been disposed accordingly
        /// </summary>
        public void StartCalibration()
        {
            this.measuredFrames = 0;
            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // Clear current coordinate system
            this.x = new Vector3D(0,0,0);
            this.y = new Vector3D(0,0,0);
            this.z = new Vector3D(0,0,0);
            this.screen_edge = new Vector3D(0, 0, 0);

            this.hipMeasurements.Clear();
            this.screenEdgeMeasurements.Clear();

            if (this.calibrationStarting != null)
                calibrationStarting(this, new EventArgs());
        }


        #region Handle Kinect Frame
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        public void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            IDictionary<ulong, Dictionary<JointType, Point>> jointPointsPerBody_local = new Dictionary<ulong, Dictionary<JointType, Point>>();

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;

                    if (measuredFrames == 0)
                    {
                        this.y = new Vector3D(bodyFrame.FloorClipPlane.X, bodyFrame.FloorClipPlane.Y, bodyFrame.FloorClipPlane.Z);
                        this.y.Normalize();
                    }
                }
            }

            if (dataReceived)
            {

                foreach (Body body in this.bodies)
                {

                    if (body.IsTracked)
                    {

                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                        // convert the joint points to depth (display) space
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                        takeMeasurement(joints, JointType.HipRight, JointType.HipLeft,this.hipMeasurements);
                        takeMeasurement(joints, JointType.Head, JointType.HandTipLeft,this.screenEdgeMeasurements);

                        foreach (JointType jointType in joints.Keys)
                        {
                            // sometimes the depth(Z) of an inferred joint may show as negative
                            // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                            CameraSpacePoint position = joints[jointType].Position;
                            if (position.Z < 0)
                            {
                                position.Z = Constants.InferredZPositionClamp;
                            }

                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                            jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                            jointPointsPerBody_local[body.TrackingId] = jointPoints;

                        }

                    }
                }
            }

            if (this.uiHandler != null)
                this.uiHandler(this, new BodyEventArgs { bodies = this.bodies, jointPointsPerBody = jointPointsPerBody_local, jointAngles = null });

            if (++measuredFrames == NUMBER_OF_MEASUREMENT_FRAME)
            {

                var v_ref_x = CalculateAverageVector(this.hipMeasurements);
                this.screen_edge = CalculateAverageVector(this.screenEdgeMeasurements);

                this.z = Vector3D.CrossProduct(this.y, v_ref_x);
                this.x = Vector3D.CrossProduct(this.y,this.z);

                this.screen_edge.Normalize();
                this.z.Normalize();
                this.x.Normalize();

                returnReaderOwnershipToModel(); 
            }
                

        }

        #endregion

        #region Measurements

        private Vector3D CalculateAverageVector(IList<Vector3D> measurementVector)
        {
            var x = measurementVector.Aggregate(0d, (seed, v) => seed + v.X) / NUMBER_OF_MEASUREMENT_FRAME;
            var y = measurementVector.Aggregate(0d, (seed, v) => seed + v.Y) / NUMBER_OF_MEASUREMENT_FRAME;
            var z = measurementVector.Aggregate(0d, (seed, v) => seed + v.Z) / NUMBER_OF_MEASUREMENT_FRAME;

            return new Vector3D(x, y, z);
        }

        private void takeMeasurement(IReadOnlyDictionary<JointType, Joint> joints, JointType j0, JointType j1, IList<Vector3D> measurementList)
        {
            CameraSpacePoint j0Camera = joints[j0].Position;
            Vector3D j03D = new Vector3D(j0Camera.X, j0Camera.Y, j0Camera.Z);

            CameraSpacePoint j1Camera = joints[j1].Position;
            Vector3D j13D = new Vector3D(j1Camera.X, j1Camera.Y, j1Camera.Z);

            measurementList.Add(j03D - j13D);
        }
        #endregion

        private void returnReaderOwnershipToModel()
        {
            this.bodyFrameReader.FrameArrived -= this.Reader_FrameArrived;
            this.bodyFrameReader.Dispose();

            if (calibrationComplete != null)
                calibrationComplete(this, new EventArgs());
        }

    }
}
