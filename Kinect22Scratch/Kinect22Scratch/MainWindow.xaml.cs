using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using ScratchNet;

namespace Kinect22Scratch
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor kinect;
        DepthFrameReader depthReader;
        BodyFrameReader bodyReader;

        Body[] bodies;

        Scratch scratch = new Scratch();

        Stopwatch sw = new Stopwatch();
        int frameCount;

        public MainWindow()
        {
            InitializeComponent();

            Loaded += MainWindow_Loaded;
            Closing += MainWindow_Closing;
        }

        void MainWindow_Loaded( object sender, RoutedEventArgs e )
        {
            try {
                StartKinect();

                sw.Start();
            }
            catch ( Exception ex ) {
                MessageBox.Show( ex.Message );
            }
        }

        void MainWindow_Closing( object sender, System.ComponentModel.CancelEventArgs e )
        {
            StopKinect();
        }

        private void StartKinect()
        {
            kinect= KinectSensor.Default;
            if ( (kinect == null) || (kinect.Status != KinectStatus.Connected) ) {
                throw new Exception( "Kinectが利用可能ではありません。 " + kinect.Status.ToString() );
            }

            kinect.Open();

            depthReader = kinect.DepthFrameSource.OpenReader();
            depthReader.FrameArrived += depthReader_FrameArrived;

            bodyReader = kinect.BodyFrameSource.OpenReader();
            bodyReader.FrameArrived += bosyReader_FrameArrived;

            bodies = new Body[kinect.BodyFrameSource.BodyCount];
        }

        private void StopKinect()
        {
            if ( bodyReader != null ) {
                bodyReader.Dispose();
                bodyReader = null;
            }

            if ( depthReader != null ) {
                depthReader.Dispose();
                depthReader = null;
            }

            if ( kinect != null ) {
                kinect.Close();
                kinect = null;
            }

            if ( bodies != null ) {
                foreach ( var body in bodies ) {
                    if ( body != null ) {
                        body.Dispose();
                    }
                }
            }
        }

        void depthReader_FrameArrived( object sender, DepthFrameArrivedEventArgs e )
        {
            using ( var depthFrame = e.FrameReference.AcquireFrame() ) {
                if ( depthFrame != null ) {
                    ImageDepth.Source = ConverDepthToImage( depthFrame );
                }
            }
        }

        private ImageSource ConverDepthToImage( DepthFrame depthFrame )
        {
            int BytePerPixel = 4;

            var desc = depthFrame.FrameDescription;
            var depth = new ushort[desc.Width * desc.Height];

            depthFrame.CopyFrameDataToArray( depth );

            var pixel = new byte[desc.Width * desc.Height * BytePerPixel];
            for ( int i = 0; i < depth.Length; i++ ) {
                int index = i * BytePerPixel;

                var gray = (depth[i] * 255) / 4500;

                pixel[index + 0] = (byte)gray;
                pixel[index + 1] = (byte)gray;
                pixel[index + 2] = (byte)gray;
            }

            return BitmapSource.Create( desc.Width, desc.Height, 96, 96,
                PixelFormats.Bgr32, null, pixel, desc.Width * BytePerPixel );
        }

        void bosyReader_FrameArrived( object sender, BodyFrameArrivedEventArgs e )
        {
            try {
                using ( var bodyFrame = e.FrameReference.AcquireFrame() ) {
                    if ( bodyFrame != null ) {
                        ShowBody( bodyFrame );
                    }
                }
            }
            catch ( Exception ex ) {
                Trace.WriteLine( ex.Message );
            }
        }

        private void ShowBody( BodyFrame bodyFrame )
        {
            frameCount++;
            if ( sw.ElapsedMilliseconds >= 1000 ) {
                scratch.AddSensorValue( "FrameCount", frameCount.ToString() );

                sw.Restart();
                frameCount = 0;
            }

            var mapper = kinect.CoordinateMapper;

            CanvasBody.Children.Clear();

            bodyFrame.GetAndRefreshBodyData( bodies );
            var trackedBody = bodies.FirstOrDefault( b => b.IsTracked );
            if ( trackedBody != null ) {
                var body = trackedBody;
                foreach ( var jointType in body.Joints.Keys ) {
                    var joint = body.Joints[jointType];
                    if ( joint.TrackingState != TrackingState.NotTracked ) {
                        ShowJoint( mapper, joint );

                        int scale = 200;
                        AddSensorValue( jointType, "X", (int)(joint.Position.X * scale) );
                        AddSensorValue( jointType, "Y", (int)(joint.Position.Y * scale) );
                    }
                }
            }
            else {
                foreach ( JointType jointType in Enum.GetValues(typeof(JointType) ) ) {
                    if ( jointType != JointType.Count ) {
                        AddSensorValue( jointType, "X", 0 );
                        AddSensorValue( jointType, "Y", 0 );
                    }
                }
            }

            scratch.UpdateSensor();
        }

        private void AddSensorValue( JointType jointType, string x, int value )
        {
            var key = string.Format( "{0}_{1}", jointType.ToString(), x );
            scratch.AddSensorValue( key, value.ToString() );
        }

        private void ShowJoint( CoordinateMapper mapper, Joint joint )
        {
            var point = mapper.MapCameraPointToDepthSpace( joint.Position );

            var ellipse = new Ellipse()
            {
                Fill = Brushes.Red,
                Width = 5,
                Height = 5,
            };

            Canvas.SetLeft( ellipse, point.X );
            Canvas.SetTop( ellipse, point.Y );
            CanvasBody.Children.Add( ellipse );
        }
    }
}
