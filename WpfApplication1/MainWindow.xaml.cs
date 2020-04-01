namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System.IO;
    using System.IO.Ports;
    using System.Threading;
    using System;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Toolkit.Fusion;
    using System.Windows.Media.Media3D;

    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Data;
    using System.Drawing;
    using System.Linq;
    using System.Text;
    using System.Net.Sockets;

    using System.Net.NetworkInformation;

    public class Angles
    {

        public double AngleBetweenTwoVectors(Vector3D vectorA, Vector3D vectorB)
        {
            double dotProduct;
            vectorA.Normalize();
            vectorB.Normalize();
            dotProduct = Vector3D.DotProduct(vectorA, vectorB);

            return (double)Math.Acos(dotProduct) / Math.PI * 180;
        }

        //DepthImagePoint depthpoint EKLENDİ.
        public byte[] GetVector(Skeleton skeleton, DepthImagePoint depthpoint)
        {

            Vector3D ShoulderCenter = new Vector3D(skeleton.Joints[JointType.ShoulderCenter].Position.X, skeleton.Joints[JointType.ShoulderCenter].Position.Y, skeleton.Joints[JointType.ShoulderCenter].Position.Z);
            Vector3D RightShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderRight].Position.X, skeleton.Joints[JointType.ShoulderRight].Position.Y, skeleton.Joints[JointType.ShoulderRight].Position.Z);
            Vector3D LeftShoulder = new Vector3D(skeleton.Joints[JointType.ShoulderLeft].Position.X, skeleton.Joints[JointType.ShoulderLeft].Position.Y, skeleton.Joints[JointType.ShoulderLeft].Position.Z);
            Vector3D RightElbow = new Vector3D(skeleton.Joints[JointType.ElbowRight].Position.X, skeleton.Joints[JointType.ElbowRight].Position.Y, skeleton.Joints[JointType.ElbowRight].Position.Z);
            Vector3D LeftElbow = new Vector3D(skeleton.Joints[JointType.ElbowLeft].Position.X, skeleton.Joints[JointType.ElbowLeft].Position.Y, skeleton.Joints[JointType.ElbowLeft].Position.Z);
            Vector3D RightWrist = new Vector3D(skeleton.Joints[JointType.WristRight].Position.X, skeleton.Joints[JointType.WristRight].Position.Y, skeleton.Joints[JointType.WristRight].Position.Z);
            Vector3D LeftWrist = new Vector3D(skeleton.Joints[JointType.WristLeft].Position.X, skeleton.Joints[JointType.WristLeft].Position.Y, skeleton.Joints[JointType.WristLeft].Position.Z);
            Vector3D UpVector = new Vector3D(0.0, 1.0, 0.0);

            double AngleRightElbow = AngleBetweenTwoVectors(RightElbow - RightShoulder, RightElbow - RightWrist);
            double AngleRightShoulder = AngleBetweenTwoVectors(UpVector, RightShoulder - RightElbow);
            double AngleLeftElbow = AngleBetweenTwoVectors(LeftElbow - LeftShoulder, LeftElbow - LeftWrist);
            double AngleLeftShoulder = AngleBetweenTwoVectors(UpVector, LeftShoulder - LeftElbow);


            // byte[] Angles = { Convert.ToByte(AngleRightElbow), Convert.ToByte(AngleRightShoulder), Convert.ToByte(AngleLeftElbow), Convert.ToByte(AngleLeftShoulder) };
            byte[] Angles = { Convert.ToByte(AngleRightElbow), Convert.ToByte(AngleRightShoulder), Convert.ToByte(AngleLeftElbow), Convert.ToByte(depthpoint.X / 3) };
            return Angles;
        }
    }

    /// Interaction logic for MainWindow.xaml
    public partial class MainWindow : Window
    {
       
        ////////////////////////////////////////////////////////////////
        private const int portNum = 110;

        delegate void SetTextCallback(string text);

        TcpClient client;
        NetworkStream ns;
        
       // private const string hostName = "192.168.1.105";
        //private const string hostName = "192.168.43.251";
        private const string hostName = "192.168.1.21";


        ////////////////////////////////////////////////////////////////
        SerialPort ArduinoPort;
        /// Width of output drawing
        private const float RenderWidth = 640.0f;

        /// Height of our output drawing
        private const float RenderHeight = 480.0f;

        /// Thickness of drawn joint lines
        private const double JointThickness = 3;

        /// Thickness of body center ellipse
        private const double BodyCenterThickness = 10;

        /// Thickness of clip edge rectangles
        private const double ClipBoundsThickness = 10;

        /// Brush used to draw skeleton center point
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// Brush used for drawing joints that are currently tracked
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// Brush used for drawing joints that are currently inferred
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// Pen used for drawing bones that are currently tracked
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// Pen used for drawing bones that are currently inferred
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// Active Kinect sensor
        private KinectSensor sensor;

        /// Drawing group for skeleton rendering output
        private DrawingGroup drawingGroup;

        /// Drawing image that we will display
        private DrawingImage imageSource;

        /// Initializes a new instance of the MainWindow class.
        public MainWindow()
        {
            InitializeComponent();


            client = new TcpClient(hostName, portNum);
            ns = client.GetStream();
            String s = "Connected";
            byte[] byteTime = Encoding.ASCII.GetBytes(s);
            ns.Write(byteTime, 0, byteTime.Length);
            


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
            //SERİ İÇİN AÇ//////////////////////////////////////////////
            //string MyCOM = "COM5";
            //ArduinoPort = new SerialPort(MyCOM, 9600);
            //ArduinoPort.Open();

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // bizim image control'ümüzü kullanarak çizim yapar.
            Image.Source = this.imageSource;

            // Tüm sensörelere bakar ve ilk açık olandan başlar.
            // Uygulama başlarken kinect sensörün açık olmasına ihtiyaç duyar.
            // Uygulamayı sensörü çıkarıp takmaya karşı korur, 
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

                // Sensörü başlatma!
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
                // this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                //ArduinoPort.Close();
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

                            Joint joint0 = skel.Joints[JointType.HandRight];
                            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(joint0.Position, DepthImageFormat.Resolution640x480Fps30);
                            this.DrawBonesAndJoints(skel, dc);
                            Angles MyAngles = new Angles();
                            byte[] ReadyAngles = MyAngles.GetVector(skel, depthPoint);


                            //RightElbow.Text = ReadyAngles[0].ToString();
                            //RightShoulder.Text = ReadyAngles[1].ToString();
                            //LeftElbow.Text = ReadyAngles[2].ToString();
                            // LeftShoulder.Text = ReadyAngles[3].ToString();
                            byte[] SequenceStart = { 255 };


                            //SERİ İÇİN AÇ//////////////////////////////////////////////
                            //if (ArduinoPort.IsOpen)
                            //{
                            //    //  ArduinoPort.Write(SequenceStart, 0, 1);
                            //    //  ArduinoPort.Write(ReadyAngles, 0, 4);
                            //    ArduinoPort.Write(ReadyAngles, 0, 4);
                            //    Console.Write("Sağ Dirsek Eklemi: " + RightElbow.Text + "\n");
                            //    Console.Write("Sağ Omuz(Sağ-Sol) Eklemi : " + RightElbow.Text + "\n");
                            //    Console.Write("Sağ Omuz(Yukarı-Aşağı) Eklemi : " + RightElbow.Text + "\n");
                            //    Console.Write("Sol Dirsek Eklemi: " + RightElbow.Text + "\n");
                            //}

                           


                            //String s = ReadyAngles[0].ToString() + "," + ReadyAngles[1].ToString() + "*" + ReadyAngles[2].ToString() + "/";
                            //byte[] byteTime = Encoding.ASCII.GetBytes(s);
                           // ns.Write(byteTime, 0, byteTime.Length);

                          



                            ns.Write(ReadyAngles, 0, ReadyAngles.Length);
                            
                            



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
                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
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
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }



        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
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

        private void CheckBoxConnectArduino(object sender, RoutedEventArgs e)
        {

            if (this.checkArduinoConnected.IsChecked.GetValueOrDefault())
            {


            }
            //else
            //{
            //    ArduinoPort.Close();
            //}
        }

        //Veri Onay Butonu.
        private void Button_Click(object sender, RoutedEventArgs e)
        {


        }

        //Veri Red Butonu.
        private void Button_Click_1(object sender, RoutedEventArgs e)
        {

        }

        private void RightShoulder_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {

        }

    }
}