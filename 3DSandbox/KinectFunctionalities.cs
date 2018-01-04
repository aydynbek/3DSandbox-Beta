
using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows;
using System.Windows.Media;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Controls;

namespace _3DSandbox
{
    class KinectFunctionalities
    {
        /* Color Related Variables */
        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSourceColorHUD;

        /// <summary>
        /// Drawing group HUD
        /// </summary>
        private DrawingGroup drawingGroupColorHUD;

        private bool colorFirst = false;

        /* Depth Related Variables */
        private int depthFrameCounter = 0;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Bitmap to display color depth
        /// </summary>
        private WriteableBitmap depthBitmapColor = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;

        /// <summary>
        /// Depth Image
        /// </summary>
        private DrawingImage imageSourceDepth;

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

        private DrawingGroup drawingGroupDepth;

        /* Body Related Variables */
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
        public DrawingImage imageSource;

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

        /* Extras */
        private readonly Brush headGestureBrush = new SolidColorBrush(Color.FromArgb(100, 0, 255, 0));
        private const double headGestureSize = 17;

        public Image depthViewImage, colorViewImage;

        public Queue<Body> body_queue = new Queue<Body>();
        public bool gesture_MC_loaded = false;


        ColorMasterConrol colorMasterControl;
        DepthMasterControl depthMasterControl;
        GesturesMasterControl gestureMasterControl;

        public KinectFunctionalities(ref DepthMasterControl depthMasterControl, ref ColorMasterConrol colorMasterControl,
            ref GesturesMasterControl gestureMasterControl)
        {
            this.depthMasterControl = depthMasterControl;
            this.colorMasterControl = colorMasterControl;
            this.gestureMasterControl = gestureMasterControl;
        }

        public void initializeKinect(ref Image depthViewImage, ref Image colorViewImage)
        {
            this.depthViewImage = depthViewImage;
            this.colorViewImage = colorViewImage;

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // Instantiate the Color Processing:
            colorMasterControl = new ColorMasterConrol(colorFrameDescription.Width, colorFrameDescription.Height);

            // Create the drawing group we'll use for drawing
            this.drawingGroupColorHUD = new DrawingGroup();

            // Create an image source that we can use as the HUD
            this.imageSourceColorHUD = new DrawingImage(this.drawingGroupColorHUD);

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();
            
            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // create the bitmap to display color depth
            this.depthBitmapColor = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgra32, null);

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
            

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            this.drawingGroupDepth = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Create an image source for depth
            this.imageSourceDepth = new DrawingImage(this.drawingGroupDepth);

           
        }

        public void enableColorFrame()
        {
            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;
            }
        }

        public void dissableColorFrame()
        {
            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }
        }

        public void enableDepthFrame()
        {
            if (this.depthFrameReader != null)
            {
                this.depthFrameReader.FrameArrived += this.Reader_FrameArrived_Depth;
            }
        }

        public void dissableDepthFrame()
        {
            if (this.depthFrameReader != null)
            {
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }
        }


        public void assignListeners()
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }

            if (this.depthFrameReader != null)
            {
                this.depthFrameReader.FrameArrived += this.Reader_FrameArrived_Depth;
            }

            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;
        }
        
        public void deassignListeners()
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
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {

        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    colorViewImage.Source = colorFrame.ToBitmap();
                    //color_master_control.initializeColorArray(colorFrame);
                    
                }
            }
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived_Depth(object sender, DepthFrameArrivedEventArgs e)
        {
            
            if (depthFrameCounter % 3 == 0)
            {
                //bool depthFrameProcessed = false;

                using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
                {
                    if (depthFrame != null)
                    {
                        // the fastest way to process the body index data is to directly access 
                        // the underlying buffer
                        using (KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                        {
                            // verify data and write the color data to the display bitmap
                            if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) ==
                                (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                                (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) &&
                                (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                            {
                                // Note: In order to see the full range of depth (including the less reliable far field depth)
                                // we are setting maxDepth to the extreme potential depth threshold
                                ushort maxDepth = ushort.MaxValue;

                                // If you wish to filter by reliable depth distance, uncomment the following line:
                                //// maxDepth = depthFrame.DepthMaxReliableDistance

                                //this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                                if (depthFrameCounter % 3 == 0)
                                {
                                    
                                    depthMasterControl.runDepthAnalysis(depthBuffer.UnderlyingBuffer,
                                        depthFrame.DepthMinReliableDistance, maxDepth, depthFrameCounter,
                                        depthViewImage);
                                    

                                    //depth_master_control.getPointCloudBoundaries();

                                }
                                //depthFrameProcessed = true;
                            }
                        }
                    }
                }
            }
            depthFrameCounter++;
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize,
            ushort minDepth, ushort maxDepth)
        {
            // FYI: depthFrameDataSize = 434176

            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;


            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {

            }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.depthBitmapColor.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                 depthMasterControl.depthPixelsColor,
                this.depthBitmap.PixelWidth,
                0);
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary> 
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {

        }

        private void DrawGestureHead(System.Windows.Point head, DrawingContext drawingContext)
        {
            drawingContext.DrawEllipse(this.headGestureBrush, null, head, headGestureSize, headGestureSize);
        }

        private Body choose_skeleton_gesture(Body[] multiple_bodies)
        {
            Body body_to_run = null;
            int tracking_joint_count = 0;
            bool is_runnable = false, enough_tracked_joints = false, clipped_3_sides = false,
                clipped_bottom = false, past_Z_minimum = false, within_center = false, all_limbs_tracked = false;

            if (body_queue.Count > 0)
            {
                body_to_run = body_queue.Dequeue();

                // Try to see which skeleton is good for gesture:
                while (is_runnable == false)
                {

                    // OPTIONAL: CHECK THAT ALL THE LIMBS THAT ARE USED IN THE GESTURE ANALYSIS ARE TRACKED AND NOT INFERRED:
                    if (body_to_run.Joints[JointType.ShoulderLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ElbowLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.HandLeft].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.HandRight].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.SpineShoulder].TrackingState == TrackingState.Tracked &&
                        body_to_run.Joints[JointType.Head].TrackingState == TrackingState.Tracked)
                    {
                        all_limbs_tracked = true;
                    }


                    if (body_to_run.Joints[JointType.ShoulderLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ShoulderRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ElbowLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HandLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HandRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineShoulder].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineBase].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HipLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.HipRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.SpineMid].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.AnkleLeft].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.AnkleRight].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;
                    if (body_to_run.Joints[JointType.Head].TrackingState == TrackingState.Tracked)
                        tracking_joint_count++;

                    // Total checked joints are 14; lets check for 12 joints:
                    if (tracking_joint_count >= 7)
                    {
                        enough_tracked_joints = true;

                    }

                    //main_text_block_4.Text = "Enough Tracked Joints: " + enough_tracked_joints.ToString() + "\n";
                    //main_text_block_4.Text += "Tracked Joints: " + tracking_joint_count.ToString() + "\n";
                    // Lets check if the left, right, and top edges are clipped:
                    if (body_to_run.ClippedEdges.HasFlag(FrameEdges.Left) ||
                        body_to_run.ClippedEdges.HasFlag(FrameEdges.Right) ||
                        body_to_run.ClippedEdges.HasFlag(FrameEdges.Top))
                    {
                        clipped_3_sides = true;
                    }

                    //main_text_block_4.Text += "Clipped 3 Sides: " + clipped_3_sides.ToString() + "\n";

                    // Lets check if the bottom edge is clipped:
                    if (body_to_run.ClippedEdges.HasFlag(FrameEdges.Bottom))
                    {
                        clipped_bottom = true;
                    }

                    //main_text_block_4.Text += "Clipped Bottom: " + clipped_bottom.ToString() + "\n";

                    // We also need to delimit the center:
                    // I made a change: we are now measuring the position of the spine shoulder...
                    double X_position = body_to_run.Joints[JointType.SpineShoulder].Position.X;
                    double Z_position = body_to_run.Joints[JointType.SpineShoulder].Position.Z;

                    // Set the closest distance towards camera:
                    if (Z_position > 1.9)    /* OLD: 1.5 meters */
                    {
                        past_Z_minimum = true;
                    }

                    //main_text_block_4.Text += "Z distance: " + Z_position.ToString() + "\n" + "X distance: " + X_position.ToString() + "\n";

                    // Check to see if the skeleton is within the center:
                    if (Math.Abs(X_position) < .4)    /* OLD: 0.2 meters */
                    {
                        within_center = true;
                    }

                    //main_text_block_4.Text += "Within Center: " + within_center.ToString() + "\n";
                    //main_text_block_4.Text += "Past Z Minimum: " + past_Z_minimum.ToString() + "\n";

                    // Looking into all of the boolean variables:
                    if (all_limbs_tracked == true && enough_tracked_joints == true && (clipped_3_sides == false || clipped_bottom == true) &&
                       past_Z_minimum == true && within_center == true)
                    {
                        gestureMasterControl.setBody(body_to_run);
                        gestureMasterControl.runGestureAnalysis();
                        is_runnable = true;
                        //main_text_block_4.Text += "Running" + "\n";
                    }
                    else
                    {
                        //main_text_block_4.Text += "Not Running" + "\n";
                        tracking_joint_count = 0;
                        enough_tracked_joints = false;
                        clipped_3_sides = false;
                        clipped_bottom = false;
                        past_Z_minimum = false;
                        within_center = false;
                        all_limbs_tracked = false;

                        // Making sure that we do have something to run with:
                        if (body_queue.Count > 0)
                        {
                            body_to_run = body_queue.Dequeue();
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                body_queue.Clear();

            }
            return body_to_run;
        }

        private void drawColorHUD()
        {
            using (DrawingContext drawingContext = this.drawingGroupColorHUD.Open())
            {
                CameraSpacePoint position = new CameraSpacePoint();
                position.X = -950;
                position.Y = 0;
                position.Z = 0;

                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                System.Windows.Point x1 = new System.Windows.Point(depthSpacePoint.X, depthSpacePoint.Y);

                position.X = 950;
                position.Y = 0;
                position.Z = 0;

                depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                System.Windows.Point x2 = new System.Windows.Point(depthSpacePoint.X, depthSpacePoint.Y);

                Color colore = new Color();
                colore.ScA = 150;
                colore.ScR = 150;
                colore.ScG = 150;
                colore.ScB = 150;

                SolidColorBrush brushe = new SolidColorBrush(colore);

                Pen drawPen = new Pen(brushe, 10);

                drawingContext.DrawLine(drawPen, x1, x2);
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, System.Windows.Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
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
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, System.Windows.Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
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
        private void DrawHand(HandState handState, System.Windows.Point handPosition, DrawingContext drawingContext)
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
