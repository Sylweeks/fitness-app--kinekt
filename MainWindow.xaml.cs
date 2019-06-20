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
    
    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
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
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

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
        private BodyFrameReader bodyFrameReader = null;

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

        bool under_root = false;
        bool shoulder_height = false;
        bool pac = false;
        bool under_return_root = false;
        bool jumping_jack_done = false;
        int counter_jumping_jack = 1;
        bool sit_up_done = false;
        bool hands_on_head = false;
        bool root_goes_down = false;
        bool root_goes_up = false;

        double p_head;
        double p_lhand;
        double p_rhand;
        double p_base_spine;
        double p_mid_base;
        double p_shoulder;
        double p_lelbow;
        double p_relbow;
        double p_lknee;
        double p_rknee;
        double x_rfoot;
        double start_lhand;
        double p_lfoot;
        double x_head;
        double x_lknee;
        double x_lfoot;
        double start_rhand;
        
        bool sit_up = false;
        bool midspine_under_elbow = false;
        bool hands_touch_head = false;
        bool root_going_down = false;
        bool root_knee_level = false;
        bool sit_down = false;
        bool going_up = false;
        double head_position = 0;
        double distance_root_knee = 0;
        int counter_sit_up = 0;
        int counter_stopka_raczka = 0;
        double root_level = 999;
        bool slope_done=false;
        int counter_slope_done=0;
        double distance_foot=0;
        bool position_slope=false;
        bool return_position_slope=false;
        bool half_slope=false;
        bool hands_on_ground=false;
        bool position_crun=false;
        bool return_position_crun = false;
        bool half_crun = false;
        bool crun_done = false;
        double distance_head=0;
        bool head_on_knee = false;
        int counter_crun_done=0;
        private int counter_skret_done;
        bool skret_done=false;
        bool position_skret=false;
        bool return_position_skret=false;
        bool half_skret=false;
        bool hand_on_ground=false;

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
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

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
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
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
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

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
//======================================================================================================================================
            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {

                                CameraSpacePoint position = joints[jointType].Position;

                                CameraSpacePoint head = joints[JointType.Head].Position;
                                CameraSpacePoint HandLeft = joints[JointType.HandLeft].Position;
                                CameraSpacePoint HandRight = joints[JointType.HandRight].Position;
                                CameraSpacePoint SpineBase = joints[JointType.SpineBase].Position;
                                CameraSpacePoint SpineMid = joints[JointType.SpineMid].Position;
                                CameraSpacePoint Shoulder = joints[JointType.SpineShoulder].Position;
                                CameraSpacePoint ElbowLeft = joints[JointType.ElbowLeft].Position;
                                CameraSpacePoint ElbowRight = joints[JointType.ElbowRight].Position;
                                CameraSpacePoint KneeLeft = joints[JointType.KneeLeft].Position;
                                CameraSpacePoint KneeRight = joints[JointType.KneeRight].Position;
                                CameraSpacePoint FootLeft = joints[JointType.FootLeft].Position;

                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);


                                p_head = Math.Round(head.Y, 2);
                                p_lhand = Math.Round(HandLeft.Y, 2);
                                p_rhand = Math.Round(HandLeft.Y, 2);
                                p_base_spine = Math.Round(SpineBase.Y, 2);
                                p_mid_base = Math.Round(SpineMid.Y, 2);
                                p_shoulder = Math.Round(Shoulder.Y);
                                p_lelbow = Math.Round(ElbowLeft.Y, 2);
                                p_relbow = Math.Round(ElbowRight.Y, 2);
                                p_lknee = Math.Round(KneeLeft.Y, 2);
                                p_rknee = Math.Round(KneeRight.Y, 2);
                                p_lfoot = Math.Round(FootLeft.Y, 2);

                                x_head = Math.Round(head.X, 2);
                                x_lknee = Math.Round(head.X, 2);
                                x_lfoot = Math.Round(KneeLeft.X, 2);
                                x_rfoot = Math.Round(FootLeft.X, 2);

                                start_lhand = p_lhand;
                                start_rhand = p_rhand;
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);

                            distance_root_knee = p_base_spine - p_lknee; //dystans miedzy kolanem a poczatkiem kregoslopa
                            double distance_root_hand = p_lhand - p_lknee;//dystand lewe kolano lewa reka
                            
                            while (jumping_jack_done && distance_root_hand < 0.2)
                            {
                                counter_jumping_jack++;
                                jumping_jack_done = false;
                                JumpinJacksOutput.Text = counter_jumping_jack.ToString();
                                break;
                            }
                            while (distance_root_hand < 0.2  && !under_root)
                            {
                                under_root = true;
                                under_return_root = false;
                            }
                            while (p_lhand > p_shoulder && p_rhand > p_shoulder && under_root && !shoulder_height)
                            {
                                shoulder_height = true;
                                System.Console.WriteLine("Rece ponad barkami" + distance_root_hand);
                            }
                            while (p_lhand > p_head && p_rhand > p_head && shoulder_height && !pac)
                            {
                                pac = true;
                                System.Console.WriteLine("Rece nad glowa");
                            }
                            while (p_head < p_rhand && p_head < p_lhand && pac && !under_return_root)
                            {
                                under_return_root = true;
                                System.Console.WriteLine("Rece pod rootem -> returns");
                            }

                            while (under_return_root && !jumping_jack_done)
                            {
                                under_root = false;
                                shoulder_height = false;
                                pac = false;
                                under_return_root = false;
                                jumping_jack_done = true;
                            }

                            //skretosklony================================================================================

                            distance_foot = x_rfoot - x_lfoot;
                            
                            
                            double rece_ramiona = p_lhand - p_shoulder;
                            while (skret_done &&  p_lhand < p_mid_base && p_lhand > p_lknee)
                            {
                                counter_skret_done++;
                                skret_done = false;
                                skretOut.Text = counter_skret_done.ToString();
                                break;
                                System.Console.WriteLine("skretosklon");
                            }
                            while (rece_ramiona < 0.2 && !position_skret)
                            {
                                position_skret = true;
                                return_position_skret = false;
                                System.Console.WriteLine("aaaaaaaa");
                            }
                            while ((p_lhand < p_lknee && !half_skret) || (p_rhand < p_lknee && !half_skret) && position_skret)
                            {
                                half_skret = true;
                                System.Console.WriteLine("bbbbbb2");
                            }

                            while ( (p_lhand >p_head || p_rhand >p_head) && !hand_on_ground)
                            {
                                hand_on_ground = true;
                                System.Console.WriteLine("ccccccccc");

                            }
                            rece_ramiona = p_lhand - p_shoulder;
                            while (p_head > p_mid_base && hand_on_ground && !return_position_skret && rece_ramiona < 0.2)
                            {
                                return_position_skret = true;
                                position_skret = false;
                                System.Console.WriteLine("ddddddd");

                            }

                            while (!position_skret && return_position_skret)
                            {
                                position_skret = false;
                                return_position_skret = false;

                                hand_on_ground = false;
                                half_skret = false;
                                System.Console.WriteLine("eeeee");
                                skret_done = true;
                            }

                          
                                //=======================================================================================================
                                //////////////////////////////////////
                                //150 na zapisywanie
                                // sklony ====================================================================

                                distance_foot = x_rfoot - x_lfoot;
                            
                            while (slope_done && p_head<p_lhand)
                            {
                                counter_slope_done += 1;
                                sklonyOut.Text = counter_slope_done.ToString();
                                slope_done = false;
                                System.Console.WriteLine("skretosklon");
                                
                                break;
                                
                            }
                            
                            while ( p_lhand-p_shoulder<0.2 && !position_slope)
                              {
                                  position_slope = true;
                                  return_position_slope = false;
                                  System.Console.WriteLine("111111111111111");
                              }
                            while (p_lhand < p_lknee && !half_slope)
                              {
                                  half_slope = true;
                                System.Console.WriteLine("222222222");
                             }

                            while (p_rhand - p_lfoot < 0.2 && p_lhand-p_lfoot <0.2 && !hands_on_ground && half_slope)
                              {
                                  hands_on_ground = true;
                                  System.Console.WriteLine("333333");

                              }
                            while (p_head > p_mid_base && hands_on_ground && !return_position_slope)
                              {
                                  return_position_slope = true;
                                position_slope = false;
                                System.Console.WriteLine("44444444");

                              }

                            while (!position_slope && return_position_slope)
                              {
                                  position_slope = false;
                                  return_position_slope = false;

                                  hands_on_ground = false;
                                  half_slope = false;
                                  System.Console.WriteLine("5555555");
                                  slope_done = true;
                              }
                           
                            ////////////////////
                        }
                    }
                   
                    double root_move = p_base_spine - p_lknee;// dystans miedzy poczatkiem kregoslupa a kolanem
               

                    while (p_lelbow > p_mid_base && sit_up_done)
                    {
                        counter_sit_up++;
                        SitUps.Text = counter_sit_up.ToString();
                        sit_up_done = false;
                    }

                    if (p_lhand == p_head && p_rhand == p_head && root_move > 0.2 && root_goes_down == false)
                    {
                        hands_on_head = true;
                    }

                    if(root_move < 0.1)
                    {
                        root_goes_down = true;//przykucykuc 
                    }

                    if (root_move > 0.2 && root_goes_down == true)
                    {
                        sit_up_done = true;
                        root_goes_down = false;
                    }
                    
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));


                    //===========================sklonyOut,brzuszkiOut====================================================
              



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
