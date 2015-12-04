using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media.Media3D;
using Microsoft.Kinect;
using Vision.Systems.KinectHealth.CustomEventArgs;
using Vision.Systems.KinectHealth.Libraries;

namespace Vision.Systems.KinectHealth.Models
{

    class JointVisualizerModel
    {

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        public KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        public BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// The vectors described in section 3.2 of Widermann Master Thesis
        /// Vector3D is a direction vector (in meters) between to joint in 3D.
        /// </summary>
        public Vector3D[] relativeSegmentToSegmentVectors;

        /// <summary>
        /// Stores the angles between the relative segment vectors
        /// </summary>
        public double[] relativeSegmentAngles;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Camera Independent Coordinate System
        /// </summary>
        public readonly GlobalCoordinateSystem gcs = null;

        /// <summary>
        /// Timer object to handle invocation of calibration procedure.
        /// </summary>
        private System.Timers.Timer aTimer = null;

        /// <summary>
        /// EventHandler View-Models can subscribe too
        /// </summary>
        public EventHandler<BodyEventArgs> frameArrivedInModel;

        public JointVisualizerModel()
        {

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();    

            relativeSegmentToSegmentVectors = new Vector3D[7];
            relativeSegmentAngles = new double[5];

            this.gcs = new GlobalCoordinateSystem(this.kinectSensor);

            // open the sensor
            this.kinectSensor.Open();

            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            this.gcs.calibrationComplete += this.Restore_FrameArrived;

        }

        public void KickOff_Calibration()
        {
            this.bodyFrameReader.FrameArrived -= this.Reader_FrameArrived;
            this.bodyFrameReader.Dispose();
            this.gcs.StartCalibration();
        }

        public void Calibrate_GCS()
        {
            this.gcs.Calibrate(this.frameArrivedInModel);
        }


        private void Restore_FrameArrived(object sender, EventArgs e)
        {
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
        }

        public void Closing()
        {

            this.bodyFrameReader.FrameArrived -= this.Reader_FrameArrived;
            this.gcs.calibrationComplete -= this.Restore_FrameArrived;

            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived -= this.Reader_FrameArrived;

                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }


        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        public void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            IDictionary<ulong,Dictionary<JointType, Point>> jointPointsPerBody_local = new Dictionary<ulong,Dictionary<JointType,Point>>();
            IDictionary<Tuple<JointType,JointType>, double> jointAngleMap = null;

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
                }
            }

            if (dataReceived)
            {

                foreach (Body body in this.bodies)
                {


                    if (body.IsTracked)
                    {

                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                        ComputeRelativeSegmentVectors(joints);
                        jointAngleMap = ComputeRelativeSegmentAngles();


                        // convert the joint points to depth (display) space
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                        foreach (JointType jointType in joints.Keys)
                        {
                            // sometimes the depth(Z) of an inferred joint may show as negative
                            // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                            CameraSpacePoint position = joints[jointType].Position;
                            if (position.Z < 0)
                            {
                                position.Z = InferredZPositionClamp;
                            }

                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                            jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                            jointPointsPerBody_local[body.TrackingId] = jointPoints;

                        }

                    }
                }         
            }

            if (frameArrivedInModel != null)
                frameArrivedInModel(this, new BodyEventArgs { bodies = this.bodies, jointPointsPerBody = jointPointsPerBody_local, jointAngles = jointAngleMap });
        }

        private IDictionary<Tuple<JointType,JointType>,double> ComputeRelativeSegmentAngles()
        {
            relativeSegmentAngles[Constants.A_NECK] = VectorMath.AngleBetweenUsingDot(relativeSegmentToSegmentVectors[Constants.V_NECK], relativeSegmentToSegmentVectors[Constants.V_UPPER_BODY]);
            relativeSegmentAngles[Constants.A_HIP_LEFT] = VectorMath.AngleBetweenUsingDot(relativeSegmentToSegmentVectors[Constants.V_UPPER_BODY], relativeSegmentToSegmentVectors[Constants.V_THIGH_LEFT]);
            relativeSegmentAngles[Constants.A_HIP_RIGHT] = VectorMath.AngleBetweenUsingDot(relativeSegmentToSegmentVectors[Constants.V_UPPER_BODY], relativeSegmentToSegmentVectors[Constants.V_THIGH_RIGHT]);
            relativeSegmentAngles[Constants.A_KNEE_LEFT] = VectorMath.AngleBetweenUsingDot(relativeSegmentToSegmentVectors[Constants.V_THIGH_LEFT], relativeSegmentToSegmentVectors[Constants.V_SHANK_LEFT]);
            relativeSegmentAngles[Constants.A_KNEE_RIGHT] = VectorMath.AngleBetweenUsingDot(relativeSegmentToSegmentVectors[Constants.V_THIGH_RIGHT],relativeSegmentToSegmentVectors[Constants.V_SHANK_RIGHT]);

            return new Dictionary<Tuple<JointType,JointType>, double>(){
                {new Tuple<JointType,JointType>(JointType.Neck,JointType.SpineShoulder),relativeSegmentAngles[Constants.A_NECK]},
                {new Tuple<JointType,JointType>(JointType.SpineBase,JointType.HipLeft),relativeSegmentAngles[Constants.A_HIP_LEFT]},
                {new Tuple<JointType,JointType>(JointType.SpineBase,JointType.HipRight),relativeSegmentAngles[Constants.A_HIP_RIGHT]},
                {new Tuple<JointType,JointType>(JointType.HipLeft,JointType.KneeLeft),relativeSegmentAngles[Constants.A_KNEE_LEFT]},
                {new Tuple<JointType,JointType>(JointType.HipRight,JointType.KneeRight),relativeSegmentAngles[Constants.A_KNEE_RIGHT]}
            };
        }

        /// <summary>
        /// Computs the differences between joint vectors.
        /// For now the assumption is only one tracked body
        /// </summary>
        /// <param name="joints"></param>
        private void ComputeRelativeSegmentVectors(IReadOnlyDictionary<JointType, Joint> joints)
        {
            CameraSpacePoint headCamera = joints[JointType.Head].Position;
            CameraSpacePoint neckCamera = joints[JointType.Neck].Position;
            CameraSpacePoint shoulderLeftCamera = joints[JointType.ShoulderLeft].Position;
            CameraSpacePoint shoulderRightCamera = joints[JointType.ShoulderRight].Position;
            CameraSpacePoint spineShoulderCamera = joints[JointType.SpineShoulder].Position;
            CameraSpacePoint spineBaseCamera = joints[JointType.SpineBase].Position;
            CameraSpacePoint hipRightCamera = joints[JointType.HipRight].Position;
            CameraSpacePoint kneeRightCamera = joints[JointType.KneeRight].Position;
            CameraSpacePoint hipLeftCamera = joints[JointType.HipLeft].Position;
            CameraSpacePoint kneeLeftCamera = joints[JointType.KneeLeft].Position;
            CameraSpacePoint ankleRightCamera = joints[JointType.AnkleRight].Position;
            CameraSpacePoint ankleLeftCamera = joints[JointType.AnkleLeft].Position;

            Vector3D head3D = new Vector3D(headCamera.X,headCamera.Y,headCamera.Z);
            Vector3D neck3D = new Vector3D(neckCamera.X,neckCamera.Y,neckCamera.Z);
            Vector3D shoulderLeft3D = new Vector3D(shoulderLeftCamera.X,shoulderLeftCamera.Y,shoulderLeftCamera.Z);
            Vector3D shoulderRight3D = new Vector3D(shoulderRightCamera.X,shoulderRightCamera.Y,shoulderRightCamera.Z);
            Vector3D spineShoulder3D = new Vector3D(spineShoulderCamera.X,spineShoulderCamera.Y,spineShoulderCamera.Z);
            Vector3D spineBase3D = new Vector3D(spineBaseCamera.X,spineBaseCamera.Y,spineBaseCamera.Z);
            Vector3D hipRight3D = new Vector3D(hipRightCamera.X,hipRightCamera.Y,hipRightCamera.Z);
            Vector3D kneeRight3D = new Vector3D(kneeRightCamera.X,kneeRightCamera.Y,kneeRightCamera.Z);
            Vector3D hipLeft3D = new Vector3D(hipLeftCamera.X,hipLeftCamera.Y,hipLeftCamera.Z);
            Vector3D kneeLeft3D = new Vector3D(kneeLeftCamera.X,kneeLeftCamera.Y,kneeLeftCamera.Z);
            Vector3D ankleRight3D = new Vector3D(ankleRightCamera.X,ankleRightCamera.Y,ankleRightCamera.Z);
            Vector3D ankleLeft3D = new Vector3D(ankleLeftCamera.X,ankleLeftCamera.Y,ankleLeftCamera.Z);

            relativeSegmentToSegmentVectors[Constants.V_NECK] = head3D - neck3D;
            relativeSegmentToSegmentVectors[Constants.V_SHOULDER] = shoulderRight3D - shoulderLeft3D;
            relativeSegmentToSegmentVectors[Constants.V_UPPER_BODY] = spineShoulder3D - spineBase3D;
            relativeSegmentToSegmentVectors[Constants.V_THIGH_RIGHT] = hipRight3D - kneeRight3D;
            relativeSegmentToSegmentVectors[Constants.V_THIGH_LEFT] = hipLeft3D - kneeLeft3D;
            relativeSegmentToSegmentVectors[Constants.V_SHANK_LEFT] = kneeLeft3D - ankleLeft3D;
            relativeSegmentToSegmentVectors[Constants.V_SHANK_RIGHT] = kneeRight3D - ankleRight3D;
         
        }

    }
}
