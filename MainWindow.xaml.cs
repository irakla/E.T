//------------------------------------------------------------------------------

// <copyright file="MainWindow.xaml.cs" company="Microsoft">

//     Copyright (c) Microsoft Corporation.  All rights reserved.

// </copyright>

//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;

    using System;
    using System.Globalization;
    using System.Windows.Media.Imaging;
    using System.Collections.Generic;

    using System.Threading;
    using System.Windows.Threading;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>

        private WriteableBitmap colorBitmap;

        /// <summary>
        /// Intermediate storage for the depth data received from the camera
        /// </summary>
        private DepthImagePixel[] depthPixels;

        /// <summary>
        /// Intermediate storage for the depth data converted to color
        /// </summary>
        private byte[] colorPixels;

        public static readonly DependencyProperty pointxyProperty = DependencyProperty.Register(
        "pointxy", typeof(Boolean), typeof(MainWindow), new PropertyMetadata(false));

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>

        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).

            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Turn on the depth stream to receive depth frames
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                // Allocate space to put the depth pixels we'll receive
                this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];

                // Allocate space to put the color pixels we'll create
                this.colorPixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                //this.Image.Source = this.colorBitmap;

                // Add an event handler to be called whenever there is new depth frame data
                this.sensor.DepthFrameReady += this.SensorDepthFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }

                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                //this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }

            Thread gestureAnalyzingThread = new Thread(this.AnalyseHandForGesture);
            gestureAnalyzingThread.Start();

            Thread pointAnalyzingThread = new Thread(this.AnalyzeHandPoint);
            pointAnalyzingThread.Start();
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady even
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        bool skeletonProcessing = false;
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            if (skeletonProcessing)
                return;
            skeletonProcessing = true;

            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            dc.DrawImage(colorBitmap, new Rect(new Size(Image.Width, Image.Height)));
                            this.DrawHand(skel, dc);
                        }
                        else if(skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            leftHandPointX = leftHandPointY = leftHandDistance = rightHandPointX = rightHandPointY = rightHandDistance = 0;
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                /*if (leftHandInfo != null)
                    TrackHandFigure(leftHandInfo);*/
            }

            skeletonProcessing = false;
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;
                if (joint.JointType == JointType.HandRight || joint.JointType == JointType.HandLeft)
                {
                    if (joint.JointType == JointType.HandRight)
                    {
                        rightHandPointX = (int)SkeletonPointToScreen(joint.Position).X;
                        rightHandPointY = (int)SkeletonPointToScreen(joint.Position).Y;

                        rightHandDistance = (int)Math.Sqrt((joint.Position.X * joint.Position.X * 10000) +
                            (joint.Position.Y * joint.Position.Y * 10000) +
                            (joint.Position.Z * joint.Position.Z * 10000));

                        rightHandIsTracked = true;
                    }
                    else
                    {
                        rightHandIsTracked = false;
                    }

                    if (joint.JointType == JointType.ElbowRight && rightHandIsTracked)
                    {
                        double gradientX = rightHandPointX - SkeletonPointToScreen(joint.Position).X;
                        double gradientY = rightHandPointY - SkeletonPointToScreen(joint.Position).Y;

                        if (gradientX == 0)
                            gradient = 0;
                        else
                            gradient = gradientY / gradientX;
                    }
                    else
                        gradient = 0;

                    switch (joint.JointType)
                    {
                        case JointType.HandRight:
                            rightHandIsTracked = joint.TrackingState == JointTrackingState.Tracked;
                            if (rightHandIsTracked)
                                drawBrush = this.trackedJointBrush;
                            else
                                return;

                            lock (handPointCollection)
                            {
                                handPointCollection.Add(new Point(joint.Position.X, joint.Position.Y));
                            }

                            this.colorBitmap.WritePixels(
                                new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                                this.colorPixels,
                                this.colorBitmap.PixelWidth * sizeof(int),
                                0);
                            break;
                    }

                    if (drawBrush != null)
                    {
                        //draw hand location
                        drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                    }

                    if (rightHandInfo != null)
                    {
                        bool[][] rightHandFigure = TrackHandFigure(rightHandInfo);
                        if (rightHandFigure != null)
                        {
                            lock (handFigureCollection)
                            {
                                handFigureCollection.Add(rightHandFigure);
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        private int leftHandPointX = 0, leftHandPointY = 0;
        private int rightHandPointX = 0, rightHandPointY = 0;
        private int leftHandDistance = 0, rightHandDistance = 0;

        short[] leftHandInfo = null, rightHandInfo = null;

        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);

                    // Get the min and max reliable depth for the current frame
                    int minDepth = depthFrame.MinDepth;
                    int maxDepth = depthFrame.MaxDepth;

                    // Convert the depth to RGB
                    int colorPixelIndex = 0;
                    //for (int i = 0; i < this.depthPixels.Length; ++i)

                    int leftHandRadius = HandRadiusFromDistance(leftHandDistance);
                    int rightHandRadius = HandRadiusFromDistance(rightHandDistance);

                    if (leftHandRadius != 0)
                        leftHandInfo = new short[4 * leftHandRadius * (leftHandRadius - 1) + 1];
                    if (rightHandRadius != 0)
                        rightHandInfo = new short[4 * rightHandRadius * (rightHandRadius - 1) + 1];

                    int nowLeftHandInfo = 0, nowRightHandInfo = 0;

                    for (int i = 0; i < this.colorBitmap.PixelHeight; i++)
                        for (int j = 0; j < this.colorBitmap.PixelWidth; j++)
                        {

                            // Get the depth for this pixel
                            short depth = depthPixels[i * colorBitmap.PixelWidth + j].Depth;

                            bool leftHandNeighborhood = i > leftHandPointY - leftHandRadius && i < leftHandPointY + leftHandRadius && j > leftHandPointX - leftHandRadius && j < leftHandPointX + leftHandRadius;
                            bool rightHandNeighborhood = i > rightHandPointY - rightHandRadius && i < rightHandPointY + rightHandRadius && j > rightHandPointX - rightHandRadius && j < rightHandPointX + rightHandRadius;

                            int leftHandFigureCounter = 0;
                            if (leftHandNeighborhood)
                            {
                                leftHandInfo[nowLeftHandInfo++] = depth;
                            }

                            else if (rightHandNeighborhood)
                            {
                                rightHandInfo[nowRightHandInfo++] = depth;
                            }

                            else
                                depth = 0;

                            // To convert to a byte, we're discarding the most-significant
                            // rather than least-significant bits.
                            // We're preserving detail, although the intensity will "wrap."
                            // Values outside the reliable depth range are mapped to 0 (black).

                            // Note: Using conditionals in this loop could degrade performance.
                            // Consider using a lookup table instead when writing production code.
                            // See the KinectDepthViewer class used by the KinectExplorer sample
                            // for a lookup table example.
                            byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                            // Write out blue byte
                            this.colorPixels[colorPixelIndex++] = intensity;

                            // Write out green byte
                            this.colorPixels[colorPixelIndex++] = intensity;

                            // Write out red byte                
                            this.colorPixels[colorPixelIndex++] = intensity;

                            // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                            // If we were outputting BGRA, we would write alpha here.
                            ++colorPixelIndex;

                        }
                }
            }
        }

        private int HandRadiusFromDistance(int distance)
        {
            if (distance == 0)                  //hand tracking is failed
                return 0;

            return (450 - distance) / 4;            //hand tracking is successed
        }

        long prevFileMSeconds = -1;
        private bool[][] TrackHandFigure(short[] aroundHandDistanceData)
        {
            int dataMatrixLines = (int)Math.Sqrt(aroundHandDistanceData.Length);

            int startingCheckRange = dataMatrixLines & 1;
            bool dataMatrixLinesIsEven = startingCheckRange == 0;
            int centerCoordinate = aroundHandDistanceData.Length / 2;

            if(dataMatrixLinesIsEven)
                centerCoordinate += dataMatrixLines / 2;
            short latestHandValue;

            short leftMaximumDistance, rightMaximumDistance, upwardMaximumDistance, downwardMaximumDistance;
            leftMaximumDistance = rightMaximumDistance = upwardMaximumDistance = downwardMaximumDistance = aroundHandDistanceData[centerCoordinate];
            short minimum = aroundHandDistanceData[centerCoordinate];

            const int distanceSameObject = 50;
            
            latestHandValue = aroundHandDistanceData[centerCoordinate];
            int trackedLeftEndCoordinate = dataMatrixLines / 2;
            for (int coordinate = centerCoordinate; trackedLeftEndCoordinate > 0; coordinate--)                  //left search
            {
                if (aroundHandDistanceData[coordinate] > latestHandValue - distanceSameObject && aroundHandDistanceData[coordinate] < latestHandValue + distanceSameObject)
                {
                    if (leftMaximumDistance < aroundHandDistanceData[coordinate])
                        leftMaximumDistance = aroundHandDistanceData[coordinate];
                    latestHandValue = aroundHandDistanceData[coordinate];
                    trackedLeftEndCoordinate--;
                }
                else
                    break;                                      //here is not hand
            }
            leftMaximumDistance += distanceSameObject;

            latestHandValue = aroundHandDistanceData[centerCoordinate];
            int trackedRightEndCoordinate = dataMatrixLines / 2;
            for (int coordinate = centerCoordinate; trackedRightEndCoordinate < dataMatrixLines - 1; coordinate++)                //right search
            {
                if (aroundHandDistanceData[coordinate] > latestHandValue - distanceSameObject && aroundHandDistanceData[coordinate] < latestHandValue + distanceSameObject)
                {
                    if (rightMaximumDistance < aroundHandDistanceData[coordinate])
                        rightMaximumDistance = aroundHandDistanceData[coordinate];
                    latestHandValue = aroundHandDistanceData[coordinate];
                    trackedRightEndCoordinate++;
                }
                else
                    break;                                      //here is not hand
            }
            rightMaximumDistance += distanceSameObject;

            latestHandValue = aroundHandDistanceData[centerCoordinate];
            int trackedUpwardEndCoordinate = dataMatrixLines / 2;
            for (int coordinate = centerCoordinate; coordinate / dataMatrixLines != 0; coordinate -= dataMatrixLines)                //upward search
            {
                if (aroundHandDistanceData[coordinate] > latestHandValue - distanceSameObject && aroundHandDistanceData[coordinate] < latestHandValue + distanceSameObject)
                {
                    if (upwardMaximumDistance < aroundHandDistanceData[coordinate])
                        upwardMaximumDistance = aroundHandDistanceData[coordinate];
                    latestHandValue = aroundHandDistanceData[coordinate];
                    trackedUpwardEndCoordinate--;
                }
                 
                else
                    break;                                      //here is not hand
            }
            upwardMaximumDistance += distanceSameObject;

            latestHandValue = aroundHandDistanceData[centerCoordinate];
            int trackedDownwardEndCoordinate = dataMatrixLines / 2;
            for (int coordinate = centerCoordinate; coordinate / dataMatrixLines < dataMatrixLines - 1; coordinate += dataMatrixLines)                //downward search
            {
                if (aroundHandDistanceData[coordinate] > latestHandValue - distanceSameObject && aroundHandDistanceData[coordinate] < latestHandValue + distanceSameObject)
                {
                    if (downwardMaximumDistance < aroundHandDistanceData[coordinate])
                        downwardMaximumDistance = aroundHandDistanceData[coordinate];
                    latestHandValue = aroundHandDistanceData[coordinate];
                    trackedDownwardEndCoordinate++;
                }
                else
                    break;                                      //here is not hand
            }
            downwardMaximumDistance += distanceSameObject;

            for (int nowTrackedHandCoordinateRow = trackedUpwardEndCoordinate; nowTrackedHandCoordinateRow <= trackedDownwardEndCoordinate; nowTrackedHandCoordinateRow++)  //minimum distance search
            {
                for (int nowTrackedHandDataColumn = trackedLeftEndCoordinate, nowTrackedHandCoordinate = trackedUpwardEndCoordinate * dataMatrixLines + trackedLeftEndCoordinate;
                    nowTrackedHandDataColumn < trackedRightEndCoordinate;
                    nowTrackedHandDataColumn++, nowTrackedHandCoordinate++)
                {
                    bool trackIsFailed = aroundHandDistanceData[nowTrackedHandCoordinate] == 0;
                    if(trackIsFailed)
                        continue;

                    bool isMoreSmall = aroundHandDistanceData[nowTrackedHandCoordinate] < minimum;
                    if (isMoreSmall)
                        minimum = aroundHandDistanceData[nowTrackedHandCoordinate];
                }
            }

            ////////////////Ready Hand Track
            ///////////////////////////////////
            ////////////////Track Hand Figure

            int anomalHandRange = dataMatrixLines - dataMatrixLines / 10;
            bool isUntracked = trackedRightEndCoordinate - trackedLeftEndCoordinate >= anomalHandRange
                || trackedDownwardEndCoordinate - trackedUpwardEndCoordinate >= anomalHandRange;

            bool[] trackedInfo = new bool[aroundHandDistanceData.Length];
            for (int i = 0; i < trackedInfo.Length; i++)
                trackedInfo[i] = false;

            for (int nowCheckRange = startingCheckRange, distanceFromEdge = dataMatrixLines / 2; distanceFromEdge + startingCheckRange > 0; nowCheckRange += 2, distanceFromEdge--)
            {
                int nowLSideCheckCoordinate = distanceFromEdge * dataMatrixLines + distanceFromEdge;
                int nowUSideCheckCoordinate = nowLSideCheckCoordinate;
                int nowRSideCheckCoordinate = (dataMatrixLines - distanceFromEdge - startingCheckRange + 1) * dataMatrixLines - distanceFromEdge - startingCheckRange;
                int nowDSideCheckCoordinate = nowRSideCheckCoordinate;

                for (int nowCheckPosition = 0; nowCheckPosition < nowCheckRange; nowCheckPosition++)                           //track to one column
                {
                    bool isLSideHandPixel = aroundHandDistanceData[nowLSideCheckCoordinate] <= leftMaximumDistance && aroundHandDistanceData[nowLSideCheckCoordinate] >= minimum;
                    if (isLSideHandPixel)
                    {
                        trackedInfo[nowLSideCheckCoordinate] = true;                            //track to left side

                        if (distanceFromEdge < trackedLeftEndCoordinate)
                            trackedLeftEndCoordinate = distanceFromEdge;

                        int nowRowOnDataMatrix = distanceFromEdge + nowCheckPosition;
                        if (nowRowOnDataMatrix < trackedUpwardEndCoordinate)
                            trackedUpwardEndCoordinate = nowRowOnDataMatrix;
                        if (nowRowOnDataMatrix > trackedDownwardEndCoordinate)
                            trackedDownwardEndCoordinate = nowRowOnDataMatrix;
                    }
                    nowLSideCheckCoordinate += dataMatrixLines;

                    bool isUSideHandPixel = aroundHandDistanceData[nowUSideCheckCoordinate] <= upwardMaximumDistance && aroundHandDistanceData[nowUSideCheckCoordinate] >= minimum;
                    if (isUSideHandPixel)
                    {
                        trackedInfo[nowUSideCheckCoordinate] = true;                            //track to upward side

                        if (distanceFromEdge < trackedUpwardEndCoordinate)
                            trackedUpwardEndCoordinate = distanceFromEdge;

                        int nowColumnOnDataMatrix = distanceFromEdge + nowCheckPosition;
                        if (nowColumnOnDataMatrix < trackedLeftEndCoordinate)
                            trackedLeftEndCoordinate = nowColumnOnDataMatrix;
                        if (nowColumnOnDataMatrix > trackedRightEndCoordinate)
                            trackedRightEndCoordinate = nowColumnOnDataMatrix;
                    }
                    nowUSideCheckCoordinate++;

                    bool isRSideHandPixel = aroundHandDistanceData[nowRSideCheckCoordinate] <= rightMaximumDistance && aroundHandDistanceData[nowRSideCheckCoordinate] >= minimum;
                    if (isRSideHandPixel)
                    {
                        trackedInfo[nowRSideCheckCoordinate] = true;                            //track to right side

                        int nowColumnOnDataMatrix = dataMatrixLines - distanceFromEdge - startingCheckRange;
                        if (nowColumnOnDataMatrix > trackedRightEndCoordinate)
                            trackedRightEndCoordinate = nowColumnOnDataMatrix;

                        int nowRowOnDataMatrix = nowColumnOnDataMatrix - nowCheckPosition;
                        if (nowRowOnDataMatrix < trackedUpwardEndCoordinate)
                            trackedUpwardEndCoordinate = nowRowOnDataMatrix;
                        if (nowRowOnDataMatrix > trackedDownwardEndCoordinate)
                            trackedDownwardEndCoordinate = nowRowOnDataMatrix;
                    }
                    nowRSideCheckCoordinate -= dataMatrixLines;

                    bool isDSideHandPixel = aroundHandDistanceData[nowDSideCheckCoordinate] <= downwardMaximumDistance && aroundHandDistanceData[nowDSideCheckCoordinate] >= minimum;
                    if (isDSideHandPixel)
                    {
                        trackedInfo[nowDSideCheckCoordinate] = true;                            //track to downward side

                        int nowRowOnDataMatrix = dataMatrixLines - distanceFromEdge - startingCheckRange;
                        if (nowRowOnDataMatrix > trackedDownwardEndCoordinate)
                            trackedDownwardEndCoordinate = nowRowOnDataMatrix;

                        int nowColumnOnDataMatrix = nowRowOnDataMatrix - nowCheckPosition;
                        if (nowColumnOnDataMatrix < trackedLeftEndCoordinate)
                            trackedLeftEndCoordinate = nowColumnOnDataMatrix;
                        if (nowColumnOnDataMatrix > trackedRightEndCoordinate)
                            trackedRightEndCoordinate = nowColumnOnDataMatrix;
                    }
                    nowDSideCheckCoordinate--;
                }
            }

            const int minimumPixelsInOneLine = 3;
            bool allUDataIsImportant = false, allDDataIsImportant = false;
            for (int nowRow = 0, trackedRawHalfHeight = (trackedDownwardEndCoordinate - trackedUpwardEndCoordinate) / 2, trackedRawHalfWidth = (trackedRightEndCoordinate - trackedLeftEndCoordinate) / 2
                , orgTrackedUECoordinate = trackedUpwardEndCoordinate, orgTrackedDECoordinate = trackedDownwardEndCoordinate; nowRow < trackedRawHalfHeight; nowRow++)                               //process garbage lines in upward, downward
            {
                int upwardMaxSequence = 0, upwardLeftNowSequence = 0, upwardRightNowSequence = 0;
                int downwardMaxSequence = 0, downwardLeftNowSequence = 0, downwardRightNowSequence = 0;

                for (int nowColumn = 0; nowColumn < trackedRawHalfWidth; nowColumn++)
                {
                    if (trackedInfo[(orgTrackedUECoordinate + nowRow) * dataMatrixLines + trackedLeftEndCoordinate + nowColumn])            //upward left check
                        upwardLeftNowSequence++;
                    else                                                                                                    //process upward left sequence
                    {
                        if (upwardLeftNowSequence > upwardMaxSequence)
                            upwardMaxSequence = upwardLeftNowSequence;
                        upwardLeftNowSequence = 0;
                    }

                    if (trackedInfo[(orgTrackedUECoordinate + nowRow) * dataMatrixLines + trackedRightEndCoordinate - nowColumn])           //upward right check
                        upwardRightNowSequence++;
                    else                                                                                                    //process upward right sequence
                    {
                        if (upwardRightNowSequence > upwardMaxSequence)
                            upwardMaxSequence = upwardRightNowSequence;
                        upwardRightNowSequence = 0;
                    }


                    if (trackedInfo[(orgTrackedDECoordinate - nowRow) * dataMatrixLines + trackedLeftEndCoordinate + nowColumn])          //downward left check
                        downwardLeftNowSequence++;
                    else                                                                                                    //process downward left sequence
                    {
                        if (downwardLeftNowSequence > downwardMaxSequence)
                            downwardMaxSequence = downwardLeftNowSequence;
                        downwardLeftNowSequence = 0;
                    }

                    if (trackedInfo[(orgTrackedDECoordinate - nowRow) * dataMatrixLines + trackedRightEndCoordinate - nowColumn])       //downward right check
                        downwardRightNowSequence++;
                    else                                                                                                    //process downward right sequence
                    {
                        if (downwardRightNowSequence > downwardMaxSequence)
                            downwardMaxSequence = downwardRightNowSequence;
                        downwardRightNowSequence = 0;
                    }
                }

                if (upwardMaxSequence < minimumPixelsInOneLine && !allUDataIsImportant)                         //delete one upward trackedData Line
                    trackedUpwardEndCoordinate++;
                else if (!allUDataIsImportant)
                    allUDataIsImportant = true;

                if (downwardMaxSequence < minimumPixelsInOneLine && !allDDataIsImportant)                       //delete one downward trackedData Line
                    trackedDownwardEndCoordinate--;
                else if (!allDDataIsImportant)
                    allDDataIsImportant = true;
            }

            bool allLDataIsImportant = false, allRDataIsImportant = false;
            for (int nowColumn = 0, trackedRawHalfWidth = (trackedRightEndCoordinate - trackedLeftEndCoordinate) / 2, trackedRawHalfHeight = (trackedDownwardEndCoordinate - trackedUpwardEndCoordinate) / 2
                , orgTrackedLECoordinate = trackedLeftEndCoordinate, orgTrackedRECoordinate = trackedRightEndCoordinate; nowColumn < trackedRawHalfWidth; nowColumn++)                                      //process garbage lines in left, right
            {
                int leftMaxSequence = 0, leftUpwardNowSequence = 0, leftDownwardNowSequence = 0;
                int rightMaxSequence = 0, rightUpwardNowSequence = 0, rightDownwardNowSequence = 0;

                for (int nowRow = 0; nowRow < trackedRawHalfHeight; nowRow++)
                {
                    if (trackedInfo[(trackedUpwardEndCoordinate + nowRow) * dataMatrixLines + orgTrackedLECoordinate + nowColumn])              //left upward check
                        leftUpwardNowSequence++;
                    else                                                                                                                //process left upward sequence
                    {
                        if (leftUpwardNowSequence > leftMaxSequence)
                            leftMaxSequence = leftUpwardNowSequence;
                        leftUpwardNowSequence = 0;
                    }

                    if (trackedInfo[(trackedDownwardEndCoordinate - nowRow) * dataMatrixLines + orgTrackedLECoordinate + nowColumn])            //left downward check
                        leftDownwardNowSequence++;
                    else                                                                                                                //process left downward sequence
                    {
                        if (leftDownwardNowSequence > leftMaxSequence)
                            leftMaxSequence = leftDownwardNowSequence;
                        leftDownwardNowSequence = 0;
                    }


                    if (trackedInfo[(trackedUpwardEndCoordinate + nowRow) * dataMatrixLines + orgTrackedRECoordinate - nowColumn])              //right upward check
                        rightUpwardNowSequence++;
                    else                                                                                                                //process right upward sequence
                    {
                        if (rightUpwardNowSequence > rightMaxSequence)
                            rightMaxSequence = rightUpwardNowSequence;
                        rightUpwardNowSequence = 0;
                    }

                    if (trackedInfo[(trackedDownwardEndCoordinate - nowRow) * dataMatrixLines + orgTrackedRECoordinate - nowColumn])            //right downward check
                        rightDownwardNowSequence++;
                    else                                                                                                                //process right downward sequence
                    {
                        if (rightDownwardNowSequence > rightMaxSequence)
                            rightMaxSequence = rightDownwardNowSequence;
                        rightDownwardNowSequence = 0;
                    }
                }

                if (leftMaxSequence < minimumPixelsInOneLine && !allLDataIsImportant)                                 //delete one left trackedData Line
                    trackedLeftEndCoordinate++;
                else if (!allLDataIsImportant)
                    allLDataIsImportant = true;

                if (rightMaxSequence < minimumPixelsInOneLine && !allRDataIsImportant)                              //delete one right trackedData Line
                    trackedRightEndCoordinate--;
                else if (!allRDataIsImportant)
                    allRDataIsImportant = true;
            }

            bool isIrregularCase = trackedRightEndCoordinate == trackedLeftEndCoordinate || trackedUpwardEndCoordinate == trackedDownwardEndCoordinate;

            if (isIrregularCase)
                return null;

            ////////////////Track Hand Figure
            ///////////////////////////////////
            ////////////////Refine Hand Figure Data

            bool[][] refinedHandFigure = new bool[trackedDownwardEndCoordinate - trackedUpwardEndCoordinate][];
            for (int nowRow = 0; nowRow < refinedHandFigure.Length; nowRow++)
            {
                refinedHandFigure[nowRow] = new bool[trackedRightEndCoordinate - trackedLeftEndCoordinate];

                for (int nowColumn = 0; nowColumn < refinedHandFigure[nowRow].Length; nowColumn++)
                    refinedHandFigure[nowRow][nowColumn] = trackedInfo[(trackedUpwardEndCoordinate + nowRow) * dataMatrixLines + trackedLeftEndCoordinate + nowColumn];
            }

            /*long nowtime = timer.ElapsedMilliseconds;
            if (nowtime > prevFileMSeconds + 100)
            {
                
                testfilecontext = new StreamWriter("test" + (nowtime).ToString() + ".txt");

                for (int i = 0; i < refinedHandFigure.Length; i++)
                {
                    for (int j = 0; j < refinedHandFigure[i].Length; j++)
                    {
                        if (refinedHandFigure[i][j])
                            testfilecontext.Write(" 1");
                        else
                            testfilecontext.Write(" 0");
                    }
                    testfilecontext.WriteLine("");
                }

                //testfilecontext.Close();
                prevFileMSeconds = nowtime;

                AnalyseGesture(refinedHandFigure);
                testfilecontext.Close();
            }*/

            return refinedHandFigure;
        }

        enum Gesture
        {
            Paper, Rock, What
        }

        List<bool[][]> handFigureCollection = new List<bool[][]>();
        System.Diagnostics.Stopwatch gestureTimer = new System.Diagnostics.Stopwatch();
        bool clickStatus = false;

        private void AnalyseHandForGesture()
        {
            gestureTimer.Start();
            
            long nowTime, prevTime = gestureTimer.ElapsedMilliseconds, timeMilliInterval = 250;
            Gesture prevGesture = Gesture.What;

            while (true)
            {
                nowTime = gestureTimer.ElapsedMilliseconds;

                if (nowTime <= prevTime + timeMilliInterval)                                    // analyse period
                    continue;

                prevTime = gestureTimer.ElapsedMilliseconds;

                lock(handFigureCollection){
                    if (handFigureCollection.Count == 0)
                    {
                        prevGesture = Gesture.What;
                        continue;
                    }

                    bool[][] bestData = handFigureCollection[0];
                    foreach (bool[][] handData in handFigureCollection)                         // search best data()
                    {
                        if (handData[0].Length < bestData[0].Length)
                            continue;

                        bestData = handData;
                   }

                Gesture handGesture = AnalyseGesture(bestData);

            ReCheckGesture:
                if (handGesture == Gesture.Paper)
                    clickStatus = false;
                else if (handGesture == Gesture.Rock)
                    clickStatus = true;
                /*else if (handGesture == Gesture.What && prevGesture != Gesture.What)
                {
                    handGesture = prevGesture;
                    goto ReCheckGesture;
                }*/

                if (handGesture == Gesture.Rock)
                {
                    //MouseControl.MouseController.ControlMouse(0, 0, true);
                    //MouseControl.MouseController.ControlMouse(0, 0, false);
                    clickStatus = true;
                    prevTime += 1000;
                }

                /*ReCheckGesture:
                    if (handGesture == Gesture.Rock)
                    {
                        MouseControl.MouseController.ControlMouse(0, 0, true);
                        //MouseControl.MouseController.ControlMouse(0, 0, false);
                    }
                    else if (handGesture == Gesture.What && prevGesture != Gesture.What)
                    {
                        handGesture = prevGesture;
                        goto ReCheckGesture;
                    }

                    prevGesture = handGesture;
                 * */

                    handFigureCollection.Clear();
                }
            }
        }

        double gradient = 0;

        private Gesture AnalyseGesture(bool[][] handFigureData)
        {
            int rowHandLengthMax = 0;
            int rowHandLengthAverage = 0;

            for (int nowColumn = 0; nowColumn < handFigureData[0].Length; nowColumn++)
            {
                double nowX = 0;
                int handPixelStart = -1;
                int handPixelRange = 0;
                for (int nowRow = 0; nowRow < handFigureData.Length; nowRow++)
                {
                    bool xMoved = false;
                    if(gradient != 0){
                        if ((int)(nowX + gradient) >= 1)
                            xMoved = true;
                        nowX += gradient;
                    }

                    if(xMoved)
                    {
                        if (nowColumn + (int)nowX >= handFigureData[0].Length)
                            break;
                        if (nowColumn + (int)nowX < 0)
                            continue;

                        nowColumn += (int)nowX;
                    }

                    if (handFigureData[nowRow][nowColumn] && handPixelStart == -1)
                        handPixelStart = nowRow;
                    else if (handFigureData[nowRow][nowColumn])
                        handPixelRange = nowRow - handPixelStart;

                    if(xMoved)
                        nowColumn -= (int)nowX;
                }

                rowHandLengthAverage += handPixelRange;

                if (handPixelRange > rowHandLengthMax)
                    rowHandLengthMax = handPixelRange;
            }

            rowHandLengthAverage /= handFigureData[0].Length;

            Gesture gesture = Gesture.What;
            if (rowHandLengthMax - rowHandLengthAverage > rowHandLengthMax * 7 / 24)
                gesture = Gesture.Paper;
            //else if (rowHandLengthMax - rowHandLengthAverage < rowHandLengthMax * 19 / 80)
            else
                gesture = Gesture.Rock;

            return gesture;
        }

        List<Point> handPointCollection = new List<Point>();
        System.Diagnostics.Stopwatch pointTimer = new System.Diagnostics.Stopwatch();
        bool rightHandIsTracked = false;

        private void AnalyzeHandPoint()
        {
            pointTimer.Start();

            long nowTime, prevTime = pointTimer.ElapsedMilliseconds, timeMilliInterval = 50;

            Point prevPoint = new Point(0, 0);

            while (true)
            {
                nowTime = pointTimer.ElapsedMilliseconds;

                if (nowTime <= prevTime + timeMilliInterval)
                    continue;

                prevTime = pointTimer.ElapsedMilliseconds;

                double averagedPositionX = 0, averagedPositionY = 0;
                lock (handPointCollection)
                {
                    if (handPointCollection.Count == 0 || !rightHandIsTracked)
                        continue;
                    
                    foreach (Point handPoint in handPointCollection)
                    {
                        averagedPositionX += handPoint.X;
                        averagedPositionY += handPoint.Y;
                    }

                    averagedPositionX /= handPointCollection.Count;
                    averagedPositionY /= handPointCollection.Count;
                    
                    handPointCollection.Clear();
                }

                int sensivity = (70 - rightHandDistance * 5 / 15);
                if (sensivity == 0)
                    sensivity = 1;

                int movingX = (int)((averagedPositionX - prevPoint.X) * 70000) / sensivity;
                int movingY = (int)((averagedPositionY - prevPoint.Y) * 70000) / sensivity;

                    //goto SkipMoving;

                if (clickStatus)
                {
                    MouseControl.MouseController.ControlMouse(movingX, movingY * (-1), true);
                    MouseControl.MouseController.ControlMouse(0, 0, false);
                    clickStatus = false;
                }
                else
                {
                    MouseControl.MouseController.ControlMouse(movingX, movingY * (-1), false);
                }

            SkipMoving:
                prevPoint = new Point(averagedPositionX, averagedPositionY);
                /*prevPoint.X = averagedPositionX;
                prevPoint.Y = averagedPositionY;*/
            }
        }
}

namespace MouseControl
{
    using System;
    using System.Collections.Generic;
    using System.Linq;
    using System.Runtime.InteropServices;
    using System.Text;

    [Flags]
    internal enum MouseEventDataXButtons : uint
    {
        Nothing = 0x00000000,
        XBUTTON1 = 0x00000001,
        XBUTTON2 = 0x00000002
    }

    [Flags]
    internal enum MOUSEEVENTF : uint
    {
        ABSOLUTE = 0x8000,
        MOVE = 0x0001,
        MOVE_NOCOALESCE = 0x2000,
        LEFTDOWN = 0x0002,
        LEFTUP = 0x0004,
        RIGHTDOWN = 0x0008,
        RIGHTUP = 0x0010,
        MIDDLEDOWN = 0x0020,
        MIDDLEUP = 0x0040,
        VIRTUALDESK = 0x4000,
        WHEEL = 0x0800,
        XDOWN = 0x0080,
        XUP = 0x0100
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct MOUSEINPUT
    {
        internal int dx;
        internal int dy;
        internal MouseEventDataXButtons mouseData;
        internal MOUSEEVENTF dwFlags;
        internal uint time;
        internal UIntPtr dwExtraInfo;
    }

    [StructLayout(LayoutKind.Explicit)]
    internal struct INPUT
    {
        [FieldOffset(0)]
        internal int type;
#if x86
        [FieldOffset(4)]
#else
        [FieldOffset(8)]
#endif
        internal MOUSEINPUT mi;

        public static int size
        {
            get { return Marshal.SizeOf(typeof(INPUT)); }
        }
    }

    public static class MouseController
    {
        [DllImport("user32.dll")]
        private static extern uint SendInput(int nInputs, INPUT[] inputs, int size);

        static bool leftClickDown;

        public static void ControlMouse(int dx, int dy, bool leftClick)
        {
            INPUT[] inputs = new INPUT[2];

            inputs[0] = new INPUT();
            inputs[0].type = 0;
            inputs[0].mi.dx = dx;
            inputs[0].mi.dy = dy;
            inputs[0].mi.dwFlags = MOUSEEVENTF.MOVE;

            if (!leftClickDown && leftClick){
                inputs[1] = new INPUT();
                inputs[1].type = 0;
                inputs[1].mi.dwFlags = MOUSEEVENTF.LEFTDOWN;
                leftClickDown = true;
            }
            else if (leftClickDown && !leftClick)
            {
                inputs[1] = new INPUT();
                inputs[1].type = 0;
                inputs[1].mi.dwFlags = MOUSEEVENTF.LEFTUP;
                leftClickDown = false;
            }

            SendInput(inputs.Length, inputs, INPUT.size);
        }
    }
}
    }