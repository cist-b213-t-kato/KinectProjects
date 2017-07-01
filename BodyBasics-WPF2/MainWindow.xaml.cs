//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Windows.Shapes;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 10;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
//      private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 255, 255));

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
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        //private BodyFrameReader bodyFrameReader = null;

        private MultiSourceFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

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

        //
        //private HandState tmpHandState;

        private Dictionary<Body, HandState> prevHandStateDictionary;

        private enum ControlState {
            Start,
            None,
            Drag
        };

        private ControlState controlState = ControlState.None;
        
        private Dictionary<Body, Boolean> isRightHandClosedDictionary;
        
        private Pen pen;

        //private List<List<Point>> pointListList;

        private Dictionary<Body, Point> prevPointDictionary;

        private Dictionary<Body, List<Tuple<Point, Point>>> bodyDrawDictionary;
        
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            //this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.bodyFrameReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
            
            pen = new Pen(Brushes.White, 2);
            pen.StartLineCap = PenLineCap.Round;
            pen.EndLineCap = PenLineCap.Round;

            //pointListList = new List<List<Point>>();
            //pointListList.Add(new List<Point>());
            bodyDrawDictionary = new Dictionary<Body, List<Tuple<Point, Point>>>();

            isRightHandClosedDictionary = new Dictionary<Body, Boolean>();

            prevHandStateDictionary = new Dictionary<Body, HandState>();

            prevPointDictionary = new Dictionary<Body, Point>();

        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

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
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
            }

        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
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


        //TODO ほんとにこれでよい？
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame().BodyFrameReference.AcquireFrame())
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

            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame().ColorFrameReference.AcquireFrame())
            {
                if ( colorFrame != null )
                {
                    if ( this.bodyColors == null )
                    {
                    }
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                if (dataReceived)
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {

                            if (!bodyDrawDictionary.ContainsKey(body))
                            {
                                //List<List<Point>> newPointListList = new List<List<Point>>();
                                //newPointListList.Add(new List<Point>());
                                bodyDrawDictionary.Add(body, new List<Tuple<Point, Point>>());
                            }

                            //List<List<Point>> pointListList = bodyDrawDictionary[body];

                            List<Tuple<Point, Point>> pointTupleList = new List<Tuple<Point, Point>>();

                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

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
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);

                            // クリア（全消去）する
                            //if ( body.HandLeftState == HandState.Closed )
                            //{
                            //    pointListList = new List<List<Point>>();
                            //    pointListList.Add(new List<Point>());
                            //}

                            //List<Point> lastPointList = pointListList[pointListList.Count-1];

                            //// 長過ぎる線は先端から消していく
                            //if ( lastPointList.Count > 100 )
                            //{
                            //    lastPointList.RemoveAt(0);
                            //}

                            // 条件を満たしたとき pointListに追加する
                            if ( prevPointDictionary.ContainsKey(body) && body.HandRightState == HandState.Closed)
                            {
                                Point prevPoint = prevPointDictionary[body];
                                Point currentPoint = jointPoints[JointType.HandRight];

                                if ( Math.Abs( currentPoint.X - prevPoint.X ) > 5 
                                    || Math.Abs( currentPoint.Y - prevPoint.Y ) > 5 )
                                {
                                    if ( !bodyDrawDictionary.ContainsKey(body) )
                                    {
                                        bodyDrawDictionary[body] = new List<Tuple<Point, Point>>();
                                    }
                                    List<Tuple<Point, Point>> lineList = bodyDrawDictionary[body];
                                    Tuple<Point, Point> newLine = new Tuple<Point, Point>(prevPoint, currentPoint);
                                    lineList.Add(newLine);
                                    //lastPointList.Add(tmpPoint);

                                    prevPointDictionary[body] = currentPoint;
                                }

                            }

                            HandState prevHandState;

                            if ( prevHandStateDictionary.ContainsKey(body) )
                            {
                                prevHandState = prevHandStateDictionary[body];
                            } else {
                                prevHandState = HandState.Unknown;
                            }

                            if ( body.HandRightState != HandState.NotTracked
                                && body.HandRightState != HandState.Unknown
                                && prevHandState != body.HandRightState)
                            {

                                if ( body.HandRightState == HandState.Closed )
                                {
                                    //開いた手等から閉じた手に変わった瞬間
                                    isRightHandClosedDictionary[body] = false;
                                    String str = "";
                                    str += " " + DateTime.Now.ToString();
                                    str += " Start";
                                    Console.WriteLine(str);
                                    controlState = ControlState.Start;

                                    //Point startPoint = jointPoints[JointType.HandRight];
                                    //lastPointList.Add(startPoint);

                                    prevPointDictionary[body] = jointPoints[JointType.HandRight];

                                } else if (prevHandState == HandState.Closed )
                                {
                                    //閉じた手だったのがそれ以外に変わった瞬間
                                    isRightHandClosedDictionary[body] = true;
                                    String str = "";
                                    str += body.ToString();
                                    str += " " + DateTime.Now.ToString();
                                    str += " End";
                                    str += " tmpHandState:" + prevHandState;
                                    str += " body.HandRightState: " + body.HandRightState;
                                    Console.WriteLine(str);
                                    controlState = ControlState.None;

                                    //Point endPoint = jointPoints[JointType.HandRight];
                                    //lastPointList.Add(endPoint);

                                    //pointListList.Add(new List<Point>());
                                    prevPointDictionary.Remove(body);

                                }

                            }

                            if (body.HandRightState != HandState.Unknown
                                && body.HandRightState != HandState.NotTracked)
                            {
                                prevHandStateDictionary[body] = body.HandRightState;
                            }

                            if ( body.HandLeftState == HandState.Closed )
                            {
                                Point cursor = jointPoints[JointType.HandLeft];
                                double cX = cursor.X;
                                double cY = cursor.Y;
                                
                                foreach (Body tmpBody in bodyDrawDictionary.Keys)
                                {
                                    foreach (Tuple<Point, Point> tuple in bodyDrawDictionary[tmpBody])
                                    {
                                        /* @see http://marupeke296.com/COL_2D_No5_PolygonToCircle.html */

                                        Point prevPoint = tuple.Item1;
                                        Point nextPoint = tuple.Item2;

                                        double sX = nextPoint.X - prevPoint.X;
                                        double sY = nextPoint.Y - prevPoint.Y;
                                        double aX = cX - prevPoint.X;
                                        double aY = cY - prevPoint.Y;
                                        double bX = aX - sX;
                                        double bY = aY - sY;
                                        double lenS = Math.Sqrt(Math.Pow(sX, 2) + Math.Pow(sY, 2));
                                        double crossSAAbs = Math.Abs( sX * aY - aX * sY );
                                        double d = crossSAAbs / lenS;

                                        FormattedText formattedText = new FormattedText(
                                            ""+d, CultureInfo.GetCultureInfo("en-us"),
                                            FlowDirection.LeftToRight,
                                            new Typeface("Verdana"),
                                            18,
                                            Brushes.White);

                                        dc.DrawText(formattedText, new Point(10,10));

                                        double r = HandSize;

                                        if (d <= r)
                                        {
                                            double dotAS = aX * sX + aY * sY;
                                            double dotBS = bX * sX + bY * sY;

                                            if ( dotAS * dotBS <= 0 )
                                            {
                                                bodyDrawDictionary[tmpBody].Remove(tuple);
                                                break;
                                            } else {
                                                double aLen = Math.Sqrt(Math.Pow(aX, 2) + Math.Pow(aY, 2));
                                                if ( r > aLen )
                                                {
                                                    bodyDrawDictionary[tmpBody].Remove(tuple);
                                                    break;
                                                }

                                                double bLen = Math.Sqrt(Math.Pow(bX, 2) + Math.Pow(bY, 2));
                                                if (r > bLen)
                                                {
                                                    bodyDrawDictionary[tmpBody].Remove(tuple);
                                                    break;
                                                }

                                            }
                                        }


                                    }
                                }

                            }


                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }

                foreach (KeyValuePair<Body, List<Tuple<Point, Point>>> pair in bodyDrawDictionary)
                {
                    //Console.WriteLine("");
                    foreach ( Tuple<Point, Point> tuple in pair.Value )
                    {
                        Point lineStartPoint = tuple.Item1;
                        Point lineEndPoint = tuple.Item2;
                        //Console.WriteLine(lineStartPoint + " " + lineEndPoint + " ");
                        dc.DrawLine(pen, lineStartPoint, lineEndPoint);
                    }

                }

            }

        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
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
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
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

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
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

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

    }
}
