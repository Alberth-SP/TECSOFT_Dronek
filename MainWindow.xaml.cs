//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace DRONEK
{
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Diagnostics;

   
    public partial class MainWindow : Window
    {
       
        private const float RenderWidth = 640.0f;

      
        private const float RenderHeight = 480.0f;

        
        
        
        private const double JointThickness = 3;

        
        
        
        private const double BodyCenterThickness = 10;

        
    
        
        private const double ClipBoundsThickness = 10;

        
      
        
        private readonly Brush centerPointBrush = Brushes.Blue;

        
    
        
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        
   
                
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        
        
                
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        
  
        
        private KinectSensor sensor;

        
        
        private DrawingGroup drawingGroup;

        
    
        
        private DrawingImage imageSource;

        
     
        
        private DroneController droneController;


        
        private bool isDroneOn;

        
     
        
        public MainWindow()
        {
            InitializeComponent();
        }

 
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

        
     
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {

            this.drawingGroup = new DrawingGroup();

            
            this.imageSource = new DrawingImage(this.drawingGroup);

           
            Image.Source = this.imageSource;

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
               
                droneController = new DroneController();
                droneController.SubscribeToGestures();
                droneController.DroneCommandChanged += droneController_DroneCommandChanged;

                
                this.sensor.SkeletonStream.Enable();

               
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

      
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
                CommandTextBlock.Text = "Conectando Kinect antes de enviar comandos!!!!";

                StartButton.IsEnabled = false;
                EmergencyButton.IsEnabled = false;
                ResetEmergencyButton.IsEnabled = false;

                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }

            GestureDetection.RightHandUpDownChanged += OnRightHandUpDownChanged;
            GestureDetection.RightHandLeftRightChanged += OnRightHandLeftRightChanged;
            GestureDetection.RightHandBackForwardsChanged += OnRightHandBackFordwardChanged;

            GestureDetection.LeftHandUpDownChanged += OnLeftHandUpDownChanged;
            GestureDetection.LeftHandLeftRightChanged += OnLeftHandLeftRightChanged;
            GestureDetection.LeftHandBackForwardsChanged += OnLeftHandBackFordwardChanged;
        }

        void droneController_DroneCommandChanged(object sender, DroneController.DroneCommandChangedEventArgs args)
        {
            if (isDroneOn)
                CommandTextBlock.Text = args.CommandText;
        }

        void OnRightHandUpDownChanged(object sender, HandPositionChangedArgs args)
        {
            RightUpButton.IsEnabled = args.Position == HandPosition.Up;
            RightCenter1Button.IsEnabled = args.Position == HandPosition.Center;
            RightDownButton.IsEnabled = args.Position == HandPosition.Down;
        }

        void OnRightHandLeftRightChanged(object sender, HandPositionChangedArgs args)
        {
            RightLeftButton.IsEnabled = args.Position == HandPosition.Left;
            RightCenter2Button.IsEnabled = args.Position == HandPosition.Center;
            RightRightButton.IsEnabled = args.Position == HandPosition.Right;
        }

        void OnRightHandBackFordwardChanged(object sender, HandPositionChangedArgs args)
        {
            RightFordwardsButton.IsEnabled = args.Position == HandPosition.Forwards;
            RightCenter3Button.IsEnabled = args.Position == HandPosition.Center;
            RightBackwardsButton.IsEnabled = args.Position == HandPosition.Backwards;
        }

        void OnLeftHandUpDownChanged(object sender, HandPositionChangedArgs args)
        {
            LeftUpButton.IsEnabled = args.Position == HandPosition.Up;
            LeftCenter1Button.IsEnabled = args.Position == HandPosition.Center;
            LeftDownButton.IsEnabled = args.Position == HandPosition.Down;
        }

        void OnLeftHandLeftRightChanged(object sender, HandPositionChangedArgs args)
        {
            LeftLeftButton.IsEnabled = args.Position == HandPosition.Left;
            LeftCenter2Button.IsEnabled = args.Position == HandPosition.Center;
            LeftRightButton.IsEnabled = args.Position == HandPosition.Right;
        }

        void OnLeftHandBackFordwardChanged(object sender, HandPositionChangedArgs args)
        {
            LeftFordwardsButton.IsEnabled = args.Position == HandPosition.Forwards;
            LeftCenter3Button.IsEnabled = args.Position == HandPosition.Center;
            LeftBackwardsButton.IsEnabled = args.Position == HandPosition.Backwards;
        }

        
       
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

     
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            foreach (Skeleton skeleton in skeletons)
            {
                if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                {
                    
                    GestureDetection.FrameReady(skeleton);

                   
                    break;
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        
       
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            //  Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

    
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
           
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        
        
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        
        
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

        private void EmergencyButton_Click(object sender, RoutedEventArgs e)
        {
            droneController.Emergency();
            CommandTextBlock.Text = "Emergencia";
        }

        private void ResetEmergencyButton_Click(object sender, RoutedEventArgs e)
        {
            droneController.ResetEmergency();
            CommandTextBlock.Text = "Reset Emergencia";
        }

        private void StartButton_Click(object sender, RoutedEventArgs e)
        {
            droneController.Start();
            isDroneOn = true;
            StartButton.IsEnabled = false;
            StopButton.IsEnabled = true;
            HoverButton.IsEnabled = true;
            CommandTextBlock.Text = "Iniciar";
        }

        private void StopButton_Click(object sender, RoutedEventArgs e)
        {
            droneController.Stop();
            isDroneOn = false;
            StartButton.IsEnabled = true;
            StopButton.IsEnabled = false;
            HoverButton.IsEnabled = false;
            CommandTextBlock.Text = "Detener";
        }

        private void Hyperlink_RequestNavigate(object sender, System.Windows.Navigation.RequestNavigateEventArgs e)
        {
            Process.Start(new ProcessStartInfo(e.Uri.AbsoluteUri));
            e.Handled = true;
        }

        private void HoverButton_Click(object sender, RoutedEventArgs e)
        {
            droneController.Hover();
        }
    }
}