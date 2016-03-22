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
        float changeNeeded = 0.05F;

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
                MessageBox.Show("Could not connect to Arduino!", "Fatal Error", MessageBoxButton.OK);
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
                    smoothingParam.Smoothing = 0.2f;            // Higher = more smoothed skeletal positions
                    smoothingParam.Correction = 0.2f;           // Higher = correct to raw data more quickly
                    smoothingParam.Prediction = 0.0f;           // Number of frames to predict into the future
                    smoothingParam.JitterRadius = 0.05f;        // Any jitter beyond this radius is clamped to radius
                    smoothingParam.MaxDeviationRadius = 0.04f;  // Maximum radius(m) filtered positions are allowed to deviate from raw data
                }

                _sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;  // Seated mode
                _sensor.ColorStream.Enable();                   // Enable color stream
                _sensor.DepthStream.Enable();                   // Enable depth stream
                _sensor.SkeletonStream.Enable(smoothingParam);  // Enable skeleton stream

                _sensor.AllFramesReady += Sensor_AllFramesReady;    // Assign event handler

                _sensor.Start();                                // Turn on sensor
                currentPort.Open();                             // Open arduin COM port
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
                    // Fill jointsCache with empty joints
                    if (jointsCache == null) jointsCache = _bodies[0].Joints;

                    // Track only one skeleton (first in scene)
                    int firstSkeleton = _bodies[0].TrackingId;
                    //_sensor.SkeletonStream.ChooseSkeletons(firstSkeleton);

                    foreach (var body in _bodies)
                    {
                        if (body.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            // COORDINATE MAPPING
                            foreach (Joint joint in body.Joints)
                            {  
                                // Only track right hand
                                if (joint.JointType != JointType.HandRight) continue;

                                // 3D coordinates in meters
                                SkeletonPoint skeletonPoint = joint.Position;
                                if(joint.JointType == JointType.HandRight)
                                {
                                    this.label1.Content = "Hand: " + skeletonPoint.X + ", " + 
                                        skeletonPoint.Y + ", " + skeletonPoint.Z;
                                }

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
                Debug.WriteLine("\n" + message + "\n");
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
                Debug.WriteLine("Serial write: " + data);
                currentPort.WriteLine(data);
                ReadComPort();
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
    }

    enum CameraMode
    {
        Color,
        Depth
    }
}
