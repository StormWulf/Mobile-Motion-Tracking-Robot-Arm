using Microsoft.Kinect;
using System;
using System.ComponentModel;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Diagnostics;
using System.IO.Ports;
using System.IO;
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
        float posX = 0;
        float posY = 0;
        float posZ = 0;
        float changeNeeded = 0.05F;
        //BackgroundWorker bw = new BackgroundWorker();   // Background worker to send updates to arduino
        List<Task> tasks = new List<Task>();    // List of tasks to send serial communication to arduino

        //SerialPort ComPort = new SerialPort("COM3", 9600);

        public MainWindow()
        {
            InitializeComponent();
            SetComPort();
            Thread.Sleep(500);
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

            //bw.RunWorkerCompleted += new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
            //bw.WorkerReportsProgress = true;
        }

        /**********************************************************************
        * Enable SkeletonStream and turn on sensor
        **********************************************************************/
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.KinectSensors.Where(s => s.Status == KinectStatus.Connected).FirstOrDefault();

            if (_sensor != null)
            {
                _sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;  // Seated mode
                _sensor.ColorStream.Enable();       // Enable color stream
                _sensor.DepthStream.Enable();       // Enable depth stream
                _sensor.SkeletonStream.Enable();    // Enable skeleton stream

                _sensor.AllFramesReady += Sensor_AllFramesReady;

                _sensor.Start();
                currentPort.Open();
            }
        }

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

                    if (jointsCache == null) jointsCache = _bodies[0].Joints;

                    foreach (var body in _bodies)
                    {
                        if (body.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            // COORDINATE MAPPING
                            foreach (Joint joint in body.Joints)
                            {
                                if (joint.JointType != JointType.HandRight)
                                {
                                    continue;
                                }
                                /*if (joint.TrackingState == JointTrackingState.NotTracked)
                                {
                                    continue;
                                }*/
                                /*switch (joint.JointType)
                                {
                                    case JointType.Head:
                                        continue;
                                    case JointType.ElbowLeft:
                                        continue;
                                    case JointType.HandLeft:
                                        continue;
                                    case JointType.ShoulderLeft:
                                        continue;
                                    case JointType.WristLeft:
                                        continue;
                                    default:
                                        break;
                                }*/
                                // 3D coordinates in meters
                                SkeletonPoint skeletonPoint = joint.Position;
                                if(joint.JointType == JointType.HandRight)
                                {
                                    this.label1.Content = "Hand: " + skeletonPoint.X + ", " + skeletonPoint.Y + ", " + skeletonPoint.Z;
                                }

                                // 2D coordinates in pixels
                                Point point = new Point();

                                if (_mode == CameraMode.Color)
                                {
                                    // Skeleton-to-Color mapping
                                    ColorImagePoint colorPoint = _sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeletonPoint, ColorImageFormat.RgbResolution640x480Fps30);

                                    point.X = colorPoint.X;
                                    point.Y = colorPoint.Y;
                                }
                                else if (_mode == CameraMode.Depth) // Remember to change the Image and Canvas size to 320x240.
                                {
                                    // Skeleton-to-Depth mapping
                                    DepthImagePoint depthPoint = _sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeletonPoint, DepthImageFormat.Resolution320x240Fps30);

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

                                    if (changeX > changeNeeded)
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

        private void Window_Unloaded(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Window_Unloaded: Turning off sensor and closing COM port...");
            if (_sensor != null)
            {
                _sensor.Stop();
            }
            currentPort.WriteLine("Closing");
            currentPort.Close();
            Debug.WriteLine("Quitting...");
        }

        private void ReadComPort()
        {
            try
            {
                string message = currentPort.ReadLine();
                Debug.WriteLine("\n" + message + "\n");
            }
            catch { }
        }

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
                Thread.Sleep(2000);
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
                //currentPort.PortName = returnMessage;
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

        private void SendSerial(Joint joint)
        {
            string data = joint.JointType + "," + joint.Position.X + "," + joint.Position.Y + "," + joint.Position.Z;
            if (currentPort.IsOpen)
            {
                Debug.WriteLine("Serial write: " + data);
                currentPort.WriteLine(data);
                ReadComPort();
            }
            else Debug.WriteLine("Com port not open.");
        }

        private void Window_Closing(object sender, CancelEventArgs e)
        {
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            Debug.WriteLine("Window_Closed: Turning off sensor and closing COM port...");
            currentPort.WriteLine("Closing");
            currentPort.Close();
            if (_sensor != null)
            {
                _sensor.Stop();
            }
            Debug.WriteLine("Quitting...");
        }
    }

    enum CameraMode
    {
        Color,
        Depth
    }
}
