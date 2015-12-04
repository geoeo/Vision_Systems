using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Windows;
using System.Windows.Threading;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Globalization;
using Microsoft.Kinect;
using Vision.Systems.KinectHealth.Models;
using Vision.Systems.KinectHealth.CustomEventArgs;
using Vision.Systems.KinectHealth.Libraries;

namespace Vision.Systems.KinectHealth.ViewModels
{
    class JointVisualizerVM : INotifyPropertyChanged
    {
         /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// The class which manages the app logic
        /// </summary>
        private JointVisualizerModel model;

        /// <summary>
        /// Toggle for showing angles in UI
        /// </summary>
        public bool showAngles { get; set; }

        /// <summary>
        /// Timer to handle the UI response up to actual calibration event
        /// </summary>
        private static System.Timers.Timer aTimer = null;

        /// <summary>
        /// Fixed number of seconds between button press and calibration
        /// </summary>
        private readonly int NUMBER_OF_SECONDS_UNTIL_CALIBRATION = 5;

        /// <summary>
        /// Number of seconds left until calibration starts
        /// </summary>
        private int calibrationCountDown = -1;

        public BodyFrameReader bodyFrameReader { get { return model.bodyFrameReader; } }

        public KinectSensor kinectSensor { get { return model.kinectSensor; } }

        public JointVisualizerVM(JointVisualizerModel model)
        {
            this.model = model;

            // get the depth (display) extents
            FrameDescription frameDescription = this.model.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // set IsAvailableChanged event notifier
            this.model.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            this.model.frameArrivedInModel += Model_FrameArrived;

            this.model.gcs.calibrationComplete += OnCalibrationComplete;
            this.model.gcs.calibrationStarting += OnCalibrationStarting;

        }

        public void Closing()
        {
            this.model.frameArrivedInModel -= Model_FrameArrived;
            this.model.gcs.calibrationComplete -= OnCalibrationComplete;
            this.model.gcs.calibrationStarting -= OnCalibrationStarting;

            model.Closing();
        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            ;

            // on failure, set the status text
            this.StatusText = this.model.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        public void Calibrate()
        {
            calibrationCountDown = NUMBER_OF_SECONDS_UNTIL_CALIBRATION;
            aTimer = new System.Timers.Timer(1000);

            aTimer.Elapsed += OnTimedCountDown;
            aTimer.AutoReset = true;
            aTimer.Enabled = true;
            this.StatusText = calibrationCountDown.ToString();
        }

        private void OnCalibrationComplete(object sender, EventArgs e)
        {
            this.StatusText = QueryStatus(); 
        }

        private void OnCalibrationStarting(object sender, EventArgs e)
        {
            aTimer.Stop();
            aTimer.Dispose();
            this.model.Calibrate_GCS();
        }

        private void OnTimedCountDown(object sender, System.Timers.ElapsedEventArgs e)
        {
            var countDown = --calibrationCountDown;
            this.StatusText = countDown.ToString();
            if (calibrationCountDown == 0)
            {
                this.StatusText = "Calibrating...";                
                this.model.StartCalibration();          
            }
        }

        public string QueryStatus()
        {
            return this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                                        : Properties.Resources.NoSensorStatusText;
        }

        
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        public void Model_FrameArrived(object sender, BodyEventArgs e)
        {

            if (e.bodies == null) return;

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                int penIndex = 0;
                foreach (Body body in e.bodies)
                {
                    Pen drawPen = this.bodyColors[penIndex++];

                    if (body.IsTracked)
                    {
                        this.DrawClippedEdges(body, dc);

                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                        // Border case, can happen. 
                        if (e.jointPointsPerBody.Count == 0) return;
                        // convert the joint points to depth (display) space
                        Dictionary<JointType, Point> jointPoints = e.jointPointsPerBody[body.TrackingId];

                        this.DrawBody(joints, jointPoints, e.jointAngles, dc, drawPen);

                        this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                        this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);

                        if (showAngles && e.upperBodyAngles != null)
                        {
                            var UB_Forward = e.upperBodyAngles[Constants.UB_FORWARD];
                            var UB_Lean = e.upperBodyAngles[Constants.UB_LEAN];
                            var UB_Rotation = e.upperBodyAngles[Constants.UB_ROTATION];


                            dc.DrawText(new FormattedText(UB_Forward.ToString(), CultureInfo.CurrentUICulture, FlowDirection.LeftToRight, new Typeface("Verdana"), 12, Brushes.LemonChiffon), new Point(10,10));
                            dc.DrawText(new FormattedText(UB_Lean.ToString(), CultureInfo.CurrentUICulture, FlowDirection.LeftToRight, new Typeface("Verdana"), 12, Brushes.LemonChiffon), new Point(210, 10));
                            dc.DrawText(new FormattedText(UB_Rotation.ToString(), CultureInfo.CurrentUICulture, FlowDirection.LeftToRight, new Typeface("Verdana"), 12, Brushes.LemonChiffon), new Point(410, 10));
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
            }

            
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints,IDictionary<Tuple<JointType,JointType>,double> jointAngleMap, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in Constants.bones)
            {
                var j0 = bone.Item1;
                var j1 = bone.Item2;

                double angle = -1;
                if (jointAngleMap != null)
                {
                    var key = new Tuple<JointType,JointType>(j0,j1);
                    if (jointAngleMap.ContainsKey(key)) angle = jointAngleMap[key];
                }
                this.DrawBone(joints, jointPoints, j0, j1,angle, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }


        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1,double angleBetweenJoints, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            var p1 = jointPoints[jointType0];
            var p2 = jointPoints[jointType1];

            drawingContext.DrawLine(drawPen, p1, p2);

            // if angles should be shown and an angle exists for the current joint pair
            if (showAngles && angleBetweenJoints != -1)
            {
                var tuple = new Tuple<JointType, JointType>(jointType0, jointType1);
                var offset = Formatting.offsetMap.ContainsKey(tuple) ? Formatting.offsetMap[tuple] : 0;
                var flowDirection = Formatting.flowDirectionMap.ContainsKey(tuple) ? Formatting.flowDirectionMap[tuple] : FlowDirection.LeftToRight;
                var textPoint = new Point(((p1.X + p2.X) / 2) + offset, (p1.Y + p2.Y) / 2);
                var angleString = angleBetweenJoints.ToString();
                drawingContext.DrawText(new FormattedText(angleString, CultureInfo.CurrentUICulture, flowDirection, new Typeface("Verdana"), 12, Brushes.White), textPoint);
            }


        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }
    }
}
