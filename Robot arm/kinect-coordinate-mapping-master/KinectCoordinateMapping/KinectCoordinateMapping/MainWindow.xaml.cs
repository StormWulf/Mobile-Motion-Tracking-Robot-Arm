/*******************************************************************************
* Mobile Motion Tracking Robot Arm - Spring 2016 Senior Design Project
* Microsoft Kinect Xbox 360, Motion Tracking and Platform Control
* Author: Jeff Ruocco (jruoc2@unh.newhaven.edu)
* Co-Author: Jeff Falberg (jfalb1@unh.newhaven.edu)
* GitHub: https://github.com/StormWulf/Mobile-Motion-Tracking-Robot-Arm
*******************************************************************************/

using Microsoft.Kinect;
using System;
using System.ComponentModel;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading.Tasks;
using Microsoft.Kinect.Toolkit.Interaction;
using System.Windows.Input;

namespace KinectCoordinateMapping
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        CameraMode _mode = CameraMode.Color;    // Color mode

        KinectSensor _sensor;                   // Sensor object
        Skeleton[] _bodies = new Skeleton[6];   // Array to hold bodies in scene
        JointCollection jointsCache = null;     // Array to cache joints from previous frame
        List<Task> tasks = new List<Task>();    // List of tasks to send serial communication to arduino
        float posX = 0;
        float posY = 0;
        float posZ = 0;
        float changeNeeded = 0.009F;
        int bodyid = -1;
        InteractionStream _interactionStream;   // Interaction Stream for gestures
        UserInfo[] _userInfos;                  // Information about the interactive users

        /******************************************************************************************
        * Initialization and setup
        * Establish connection to Arduino
        ******************************************************************************************/
        public MainWindow()
        {
            InitializeComponent();
            SetComPort();
            if (portFound){
                Debug.WriteLine("\nARDUINO FOUND ON COM PORT: " + currentPort.PortName + "\n");
                labelError.Content = "Arduino found on COM Port: " + currentPort.PortName;
            }
            else{
                Debug.WriteLine("\nCOULD NOT CONNECT TO ARDUINO.\n");
                labelError.Content = "Could not connect to Arduino.";
                MessageBox.Show("Could not connect to Arduino!\nPlease reset Arduino", "Fatal Error",
                    MessageBoxButton.OK);
                System.Environment.Exit(1);
            }
        }

        /******************************************************************************************
        * Enable SkeletonStream and turn on sensor
        ******************************************************************************************/
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.KinectSensors.Where(s => s.Status == KinectStatus.Connected).FirstOrDefault();

            if (_sensor != null)
            {
                // Smoothing
                TransformSmoothParameters smoothingParam = new TransformSmoothParameters();
                {
                    smoothingParam.Smoothing = 0.4f;            // Higher = more smoothed skeletal positions
                    smoothingParam.Correction = 0.6f;           // Higher = correct to raw data more quickly
                    smoothingParam.Prediction = 0.6f;           // Number of frames to predict into the future
                    smoothingParam.JitterRadius = 0.9f;        // Any jitter beyond this radius is clamped to radius
                    smoothingParam.MaxDeviationRadius = 0.04f;  // Maximum radius(m) filtered positions are allowed to deviate from raw data
                }

                _sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;  // Seated mode
                _sensor.ColorStream.Enable();                   // Enable color stream
                _sensor.DepthStream.Enable();                   // Enable depth stream
                _sensor.SkeletonStream.Enable(smoothingParam);  // Enable skeleton stream

                _sensor.AllFramesReady += Sensor_AllFramesReady;    // Assign event handler

                // Assign interaction stream and assign event handler
                _interactionStream = new InteractionStream(_sensor, new DummyInteractionClient());
                _interactionStream.InteractionFrameReady += InteractionStreamOnInteractionFrameReady;

                _sensor.Start();                                // Turn on sensor
                currentPort.Open();                             // Open arduino COM port
            }
            else
            {
                MessageBox.Show("No Kinect found.", "Fatal Error", MessageBoxButton.OK);
                System.Environment.Exit(1);
            }
        }

        /******************************************************************************************
        * Called every frame
        * Read new position of joints and send update over serial if needed
        * Draw position of joints on screen
        ******************************************************************************************/
        void Sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            // Color
            using (var frame = e.OpenColorImageFrame())
            {
                if (frame != null)
                {
                    if (_mode == CameraMode.Color)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Depth
            using (var frame = e.OpenDepthImageFrame())
            {
                if (frame != null)
                {
                    _interactionStream.ProcessDepth(frame.GetRawPixelData(), frame.Timestamp);
                    if (_mode == CameraMode.Depth)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Body
            using (var frame = e.OpenSkeletonFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();
                    frame.CopySkeletonDataTo(_bodies);
                    var accelerometerReading = _sensor.AccelerometerGetCurrentReading();
                    _interactionStream.ProcessSkeleton(_bodies, accelerometerReading, frame.Timestamp);
                    // Fill jointsCache with empty joints
                    if (jointsCache == null) jointsCache = _bodies[0].Joints;

                    // Track only one skeleton (first in scene)
                    int firstSkeleton = _bodies[0].TrackingId;

                    foreach (var body in _bodies)
                    {
                        if (body.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            if (bodyid <= -1)                   // Save first body id
                            {
                                bodyid = body.TrackingId;
                            }
                            else if (body.TrackingId != bodyid) continue;   // Only track first body

                            // COORDINATE MAPPING
                            foreach (Joint joint in body.Joints)
                            {  
                                // Only track right hand
                                if (joint.JointType != JointType.HandRight) continue;

                                // 3D coordinates in meters
                                SkeletonPoint skeletonPoint = joint.Position;
                                this.label1.Content = "Hand: " + skeletonPoint.X + ", " + 
                                    skeletonPoint.Y + ", " + skeletonPoint.Z;

                                // 2D coordinates in pixels
                                Point point = new Point();

                                if (_mode == CameraMode.Color)
                                {
                                    // Skeleton-to-Color mapping
                                    ColorImagePoint colorPoint = _sensor.CoordinateMapper.
                                        MapSkeletonPointToColorPoint(skeletonPoint, 
                                        ColorImageFormat.RgbResolution640x480Fps30);

                                    point.X = colorPoint.X;
                                    point.Y = colorPoint.Y;
                                }
                                else if (_mode == CameraMode.Depth) // Remember to change the Image and Canvas size to 320x240.
                                {
                                    // Skeleton-to-Depth mapping
                                    DepthImagePoint depthPoint = _sensor.CoordinateMapper.
                                        MapSkeletonPointToDepthPoint(skeletonPoint, 
                                        DepthImageFormat.Resolution320x240Fps30);

                                    point.X = depthPoint.X;
                                    point.Y = depthPoint.Y;
                                }

                                // DRAWING...
                                Ellipse ellipse = new Ellipse
                                {
                                    Fill = Brushes.Red,
                                    Width = 20,
                                    Height = 20
                                };

                                Line line = new Line
                                {
                                    Fill = Brushes.Red,
                                    Width = 10,
                                    Height = 10
                                };

                                Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
                                Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

                                canvas.Children.Add(ellipse);

                                // Send updated joint position to Arduino
                                if (joint.JointType == JointType.HandRight)
                                {
                                    float changeX = Math.Abs(posX - skeletonPoint.X);
                                    float changeY = Math.Abs(posY - skeletonPoint.Y);
                                    float changeZ = Math.Abs(posZ - skeletonPoint.Z);

                                    if (changeX > changeNeeded || changeY > changeNeeded 
                                        || changeZ > changeNeeded)
                                    {
                                        Task t1 = Task.Factory.StartNew(() => SendSerial(joint));
                                        posX = skeletonPoint.X;
                                        posY = skeletonPoint.Y;
                                        posZ = skeletonPoint.Z;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        /******************************************************************************************
        * Read data from com port
        ******************************************************************************************/
        private void ReadComPort()
        {
            try
            {
                string message = currentPort.ReadLine();
                Debug.WriteLine(message);
            }
            catch(Exception ex) { Debug.WriteLine(ex.Message); }
        }

        /******************************************************************************************
        * Search all com ports for Arduino
        * Arduino will send acknowledment signal
        ******************************************************************************************/
        SerialPort currentPort;
        bool portFound;

        private void SetComPort()
        {
            try
            {
                string[] ports = SerialPort.GetPortNames();
                foreach (string port in ports)
                {
                    currentPort = new SerialPort(port, 9600);
                    if (DetectArduino())
                    {
                        portFound = true;
                        break;
                    }
                    else
                    {
                        portFound = false;
                    }
                }
            }
            catch (Exception e)
            {
                Debug.WriteLine("ERROR\n" + e.Message + "\n");
                System.Environment.Exit(1);
            }
        }
        private bool DetectArduino()
        {
            try
            {
                //The below setting are for the Hello handshake
                byte[] buffer = new byte[5];
                buffer[0] = Convert.ToByte(16);
                buffer[1] = Convert.ToByte(128);
                buffer[2] = Convert.ToByte(0);
                buffer[3] = Convert.ToByte(0);
                buffer[4] = Convert.ToByte(4);
                int intReturnASCII = 0;
                char charReturnValue = (Char)intReturnASCII;
                currentPort.Open();
                currentPort.Write(buffer, 0, 5);
                Thread.Sleep(1000);
                int count = currentPort.BytesToRead;
                string returnMessage = "";
                while (count > 0)
                {
                    intReturnASCII = currentPort.ReadByte();
                    returnMessage = returnMessage + Convert.ToChar(intReturnASCII);
                    count--;
                }
                Debug.WriteLine(returnMessage);
                currentPort.Close();
                if (returnMessage.Contains("HELLO FROM ARDUINO"))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            catch (Exception e)
            {
                return false;
            }
        }

        /******************************************************************************************
        * Send joint data over serial to Arduino
        ******************************************************************************************/
        private void SendSerial(Joint joint)
        {
            string data = joint.JointType + "," + joint.Position.X + "," + joint.Position.Y +
                "," + joint.Position.Z;
            if (currentPort.IsOpen)
            {
                currentPort.WriteLine(data);
            }
            else Debug.WriteLine("Com port not open.");
        }

        /******************************************************************************************
        * Window Closing: turn off sensor and send reset signal to arduino
        ******************************************************************************************/
        private void Window_Closing(object sender, CancelEventArgs e)
        {
            Debug.WriteLine("Window_Closing: Turning off sensor and closing COM port...");
            if (_sensor != null)
            {
                _sensor.Stop();
            }
            currentPort.WriteLine("reset");
            currentPort.Close();
            Debug.WriteLine("Quitting...");
        }

        /******************************************************************************************
        * Interaction Event Handler.  Used to detect whether hand is opened or closed.
        ******************************************************************************************/
        private void InteractionStreamOnInteractionFrameReady(object sender, InteractionFrameReadyEventArgs e)
        {
            using (InteractionFrame frame = e.OpenInteractionFrame())
            {
                if (frame != null)
                {
                    if (_userInfos == null)
                    {
                        _userInfos = new UserInfo[InteractionFrame.UserInfoArrayLength];
                    }

                    frame.CopyInteractionDataTo(_userInfos);
                }
                else
                {
                    return;
                }
            }

            foreach (UserInfo userInfo in _userInfos)
            {
                foreach (InteractionHandPointer handPointer in userInfo.HandPointers)
                {
                    string action = null;

                    switch (handPointer.HandEventType)
                    {
                        case InteractionHandEventType.Grip:
                            action = "gripped";
                            break;

                        case InteractionHandEventType.GripRelease:
                            action = "released";
                            break;
                    }

                    if (action != null)
                    {
                        string handSide = "unknown";

                        switch (handPointer.HandType)
                        {
                            case InteractionHandType.Left:
                                handSide = "left";
                                break;

                            case InteractionHandType.Right:
                                handSide = "right";
                                break;
                        }

                        if (handSide == "left")
                        {
                            if (action == "released")
                            {
                                // left hand released code here
                                //Debug.WriteLine("HandOpened");
                                //currentPort.WriteLine("HandOpened");
                            }
                            else
                            {
                                // left hand gripped code here
                                //Debug.WriteLine("HandClosed");
                                //currentPort.WriteLine("HandClosed");
                            }
                        }
                        else
                        {
                            if (action == "released")
                            {
                                // right hand released code here
                                Debug.WriteLine("HandOpened");
                                currentPort.WriteLine("HandOpened");
                            }
                            else
                            {
                                // right hand gripped code here
                                Debug.WriteLine("HandClosed");
                                currentPort.WriteLine("HandClosed");
                            }
                        }
                    }
                }
            }
        }

        /******************************************************************************************
        * Reset arm button clicked: reset arm position
        ******************************************************************************************/
        private void button_Click(object sender, RoutedEventArgs e)
        {
            currentPort.WriteLine("PosReset");
        }

        /******************************************************************************************
        * Reset sensor button clicked: reset kinect sensor
        ******************************************************************************************/
        private void button1_Click(object sender, RoutedEventArgs e)
        {
            _sensor.Stop();
            currentPort.WriteLine("PosReset");
            bodyid = -1;
            _sensor.Start();
        }

        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.W)
            {
                Debug.WriteLine("Key W");
                currentPort.WriteLine("forward");
            }
            if (e.Key == Key.S)
            {
                Debug.WriteLine("Key S");
                currentPort.WriteLine("backward");
            }
            if (e.Key == Key.A)
            {
                Debug.WriteLine("Key A");
                currentPort.WriteLine("left");
            }
            if (e.Key == Key.D)
            {
                Debug.WriteLine("Key D");
                currentPort.WriteLine("right");
            }
            if (e.Key == Key.Space)
            {
                Debug.WriteLine("Key space");
                currentPort.WriteLine("stop");
            }
            if(e.Key == Key.Up)
            {
                Debug.WriteLine("Up");
                currentPort.WriteLine("Up");
            }
            if (e.Key == Key.Down)
            {
                Debug.WriteLine("Down");
                currentPort.WriteLine("Down");
            }
        }

        // Stop sensor
        private void button2_Click(object sender, RoutedEventArgs e)
        {
            _sensor.Stop();
            currentPort.WriteLine("PosReset");
            bodyid = -1;
        }

        // Start sensor
        private void button3_Click(object sender, RoutedEventArgs e)
        {
            _sensor.Start();
        }
    }

    // Dummy class for interactions
    public class DummyInteractionClient : IInteractionClient
    {
        public InteractionInfo GetInteractionInfoAtLocation(
            int skeletonTrackingId,
            InteractionHandType handType,
            double x,
            double y)
        {
            var result = new InteractionInfo();
            result.IsGripTarget = true;
            result.IsPressTarget = true;
            result.PressAttractionPointX = 0.5;
            result.PressAttractionPointY = 0.5;
            result.PressTargetControlId = 1;

            return result;
        }
    }

    enum CameraMode
    {
        Color,
        Depth
    }
}
