using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Threading;
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
        /// Camera Independent Coordinate System
        /// </summary>
        public readonly GlobalCoordinateSystem gcs = null;

        public readonly Sampler[] samplers = null;

        public readonly BodyModel simpleUpperBodyModel = null;

        public readonly BodyModel[] complexUpperBodyModels = null;

        /// <summary>
        /// Timer object to handle invocation of calibration procedure.
        /// </summary>
        private System.Timers.Timer aTimer = null;

        /// <summary>
        /// EventHandler View-Models can subscribe too
        /// </summary>
        public EventHandler<BodyEventArgs> frameArrivedInModel;

        /// <summary>
        /// Very important! Forces timer threads to execute their tasks on main thread
        /// </summary>
        private static Dispatcher mainDispatcher;
        /// <summary>
        /// flag set to true if calibration procedure completed
        /// </summary>
        private bool calibrated;

        public JointVisualizerModel()
        {

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();    

            relativeSegmentToSegmentVectors = new Vector3D[8];
            relativeSegmentAngles = new double[5];

            this.samplers = new Sampler[Constants.SAMPLE_SET_SIZE];

            for (int i = 0; i < Constants.SAMPLE_SET_SIZE; i++)
            {
                this.samplers[i] = new Sampler();
            }

            this.gcs = new GlobalCoordinateSystem(this.kinectSensor);
            this.simpleUpperBodyModel = new SimpleUpperBodyModel(EmpiricalData.R_UB_FORWARD);

            this.complexUpperBodyModels = new ComplexUpperBodyModelAngle[Constants.NUMBER_OF_COMPLEX_MODELS];

            this.complexUpperBodyModels[Constants.COMPLEX_UB_FORWARD] 
                = new ComplexUpperBodyModelAngle(EmpiricalData.R_UB_FORWARD, EmpiricalData.LL_UB_FORWARD, EmpiricalData.LU_UB_FORWARD, EmpiricalData.EL_UB_FORWARD, EmpiricalData.EU_UB_FORWARD);
            this.complexUpperBodyModels[Constants.COMPLEX_UB_LEAN]
                = new ComplexUpperBodyModelAngle(EmpiricalData.R_UB_LEAN, EmpiricalData.LL_UB_LEAN, EmpiricalData.LU_UB_LEAN, EmpiricalData.EL_UB_LEAN, EmpiricalData.EU_UB_LEAN);
            this.complexUpperBodyModels[Constants.COMPLEX_NECK]
                = new ComplexUpperBodyModelAngle(EmpiricalData.R_NECK, EmpiricalData.LL_NECK, EmpiricalData.LU_NECK, EmpiricalData.EL_NECK, EmpiricalData.EU_NECK);
            this.complexUpperBodyModels[Constants.COMPLEX_LOS]
                = new ComplexUpperBodyModelAngle(EmpiricalData.R_LOS, EmpiricalData.LL_LOS, EmpiricalData.LU_LOS, EmpiricalData.EL_LOS, EmpiricalData.EU_LOS);
            this.complexUpperBodyModels[Constants.COMPLEX_VIEWING_DIST]
                = new ComplexUpperBodyModelCm(EmpiricalData.R_VIEWING_DISTANCE, EmpiricalData.LL_VIEWING_DISTANCE, EmpiricalData.LU_VIEWING_DISTANCE); 

            // open the sensor
            this.kinectSensor.Open();

            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            this.gcs.calibrationComplete += this.Restore_FrameArrived;

            mainDispatcher = Application.Current.Dispatcher;

            this.calibrated = false;

        }

        #region Calibration
        public void StartCalibration()
        {

            // invoke the calibration on the main thread
            mainDispatcher.Invoke(() =>
            {
                this.bodyFrameReader.FrameArrived -= this.Reader_FrameArrived;
                this.bodyFrameReader.Dispose();
                this.gcs.StartCalibration();
            });

        }

        public void Calibrate_GCS()
        {
            this.gcs.Calibrate(this.frameArrivedInModel);
        }


        private void Restore_FrameArrived(object sender, EventArgs e)
        {
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            this.calibrated = true;
        }

        #endregion

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
            double[] upperBodyAngles_local = new double[4];
            double[] modelIndex_local = new double[3] {-100,-100,-100};

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
                        if (calibrated)
                        {
                            computeUpperBodyAngles(joints,upperBodyAngles_local);

                            #region collectSamples
                            this.samplers[Constants.UB_FORWARD_SAMPLE_INDEX].SupplySamples(upperBodyAngles_local[Constants.UB_FORWARD]);
                            this.samplers[Constants.UB_LEAN_SAMPLE_INDEX].SupplySamples(upperBodyAngles_local[Constants.UB_LEAN]);
                            this.samplers[Constants.UB_ROTATION_SAMPLE_INDEX].SupplySamples(upperBodyAngles_local[Constants.UB_ROTATION]);
                            this.samplers[Constants.NECK_SAMPLE_INDEX].SupplySamples(relativeSegmentAngles[Constants.A_NECK]);
                            this.samplers[Constants.LOS_SAMPLE_INDEX].SupplySamples(upperBodyAngles_local[Constants.UB_LOS]);
                            this.samplers[Constants.VIEWING_DIST_SAMPLE_INDEX].SupplySamples(relativeSegmentToSegmentVectors[Constants.V_SCREEN_EDGE].Length);
                            #endregion

                            if (this.simpleUpperBodyModel.hasEnoughSamples(this.samplers)) {
                                modelIndex_local[Constants.SIMPLE_INDEX] = this.simpleUpperBodyModel.ComputeIndex(this.samplers);
                            }

                            //TODO IMPLEMENT EVALUATION TRIGGER OF COMPLEX UPPER BODY MODEL
                        }

                        // convert the joint points to depth (display) space
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

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

            if (frameArrivedInModel != null)
                frameArrivedInModel(this, new BodyEventArgs { 
                    bodies = this.bodies, 
                    jointPointsPerBody = jointPointsPerBody_local, 
                    jointAngles = jointAngleMap, 
                    upperBodyAngles = upperBodyAngles_local,
                    modelIndex = modelIndex_local
                });
        }

        #region Angle Computation

        private void computeUpperBodyAngles(IReadOnlyDictionary<JointType, Joint> joints, double[] upperBodyAngles)
        {
            var UB = relativeSegmentToSegmentVectors[Constants.V_UPPER_BODY];
            var shoulder = relativeSegmentToSegmentVectors[Constants.V_SHOULDER];
            var screen_edge = relativeSegmentToSegmentVectors[Constants.V_SCREEN_EDGE];

            var proj_UB_yz = VectorMath.projectVectorOntoPlane(UB,this.gcs.y,this.gcs.z);
            var proj_UB_xy = VectorMath.projectVectorOntoPlane(UB,this.gcs.x,this.gcs.y);

            var proj_UB_xz = VectorMath.projectVectorOntoPlane(UB,this.gcs.x, this.gcs.z);
            var proj_SH_xz = VectorMath.projectVectorOntoPlane(shoulder,this.gcs.x,this.gcs.z);

            upperBodyAngles[Constants.UB_FORWARD] = VectorMath.AngleBetweenUsingDot(proj_UB_yz,this.gcs.z);
            upperBodyAngles[Constants.UB_LEAN] = VectorMath.AngleBetweenUsingDot(proj_UB_xy, this.gcs.x);
            upperBodyAngles[Constants.UB_ROTATION] = VectorMath.AngleBetweenUsingDot(proj_UB_xz, proj_SH_xz);
            upperBodyAngles[Constants.UB_LOS] = VectorMath.AngleBetweenUsingDot(this.gcs.z, screen_edge);

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

            if(this.calibrated)
                relativeSegmentToSegmentVectors[Constants.V_SCREEN_EDGE] = head3D - this.gcs.screen_edge_avg;

        }

        #endregion

    }
}
