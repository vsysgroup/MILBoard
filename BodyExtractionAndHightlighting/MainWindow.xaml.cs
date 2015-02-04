using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
using System.ComponentModel;
using System.Runtime.InteropServices;

namespace BodyExtractionAndHightlighting
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor sensor = null;

        /// <summary>
        /// instead of creating readers for each stream, combine them using multiSourceFrameReader
        /// Allows the app to get a matched set of frames from multiple sources on a single event
        /// Caveat: Delivers frames at the lowest FPS of the selected sources
        /// No access to audio (because of framerate reduction)
        /// </summary>
        MultiSourceFrameReader reader;

        // coordinate mapper
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        Body[] bodies;

        // body-index
        byte[] biDataSource;
        uint[] biImageBuffer; //tmp storage for frame data converted to color
        WriteableBitmap biBitmap;

        ColorSpacePoint[] depthIntoColorSpace = null;
        DepthSpacePoint[] colorIntoDepthSpace = null;

        private static readonly uint[] BodyColor = 
        {
            0xFF00FF04,
            0xFFFF0021,
            0xFFFF4082,
            0xFFF2FF21,
            0xFF40F0CF,
            0xFF8080FF
        };

        // color
        WriteableBitmap colorBitmap;

        // depth
        ushort[] depthDataSource;

        // combined color - bodyIndex
        ushort[] stencilBuffer;
        byte[] combiColorBuffer1080p;
        WriteableBitmap combiBitmap1080p;
        // ------
        // for low res: 512x424
        byte[] combiColorBuffer; 
        WriteableBitmap combiBitmap;

        //check performance in ticks
        const long TICKS_PER_SECOND = 10000000;
        long prevTick = 0;

        //gui logic
        bool isFullHD = false;
        enum BackgroundType { Black, White, Custom };
        BackgroundType bgType = BackgroundType.Custom;
        private bool extendArm = false;

        // right lower arm detection for scaling
        bool isArmDetected = false;
        IReadOnlyDictionary<JointType, Joint> joints;
        Dictionary<JointType, Point> armJointPoints = new Dictionary<JointType, Point>();

        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += MainWindow_Loaded;
            this.Closing += MainWindow_Closing;
        }

        void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            sensor = KinectSensor.GetDefault();
            bodies = new Body[6];

            // color
            FrameDescription fdColor = sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            colorBitmap = new WriteableBitmap(fdColor.Width, fdColor.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
            //imageColor.Source = colorBitmap; //color img only

            // body-index
            FrameDescription fdBi = sensor.BodyIndexFrameSource.FrameDescription;
            biDataSource = new byte[fdBi.LengthInPixels];
            biImageBuffer = new uint[fdBi.LengthInPixels];
            biBitmap = new WriteableBitmap(fdBi.Width, fdBi.Height, 96, 96, PixelFormats.Bgra32, null);
            //imageBi.Source = biBitmap; //body index img only

            // depth (same resolution as body index)
            FrameDescription fdDepth = sensor.DepthFrameSource.FrameDescription;
            depthDataSource = new ushort[fdDepth.LengthInPixels];

            // combination 1080p
            stencilBuffer = new ushort[fdBi.LengthInPixels];
            combiColorBuffer1080p = new byte[fdColor.LengthInPixels * 4];
            combiBitmap1080p = new WriteableBitmap(fdColor.Width, fdColor.Height, 96, 96, PixelFormats.Bgra32, null);

            //combination 512x424 (depth resolution)
            combiColorBuffer = new byte[fdDepth.LengthInPixels * 4];
            combiBitmap = new WriteableBitmap(fdDepth.Width, fdDepth.Height, 96, 96, PixelFormats.Bgra32, null);
            imageCombi.Source = combiBitmap; //img with 512x424-color of body index frame;

            // get the coordinate mapper
            this.coordinateMapper = this.sensor.CoordinateMapper;

            depthIntoColorSpace = new ColorSpacePoint[depthDataSource.Length];
            colorIntoDepthSpace = new DepthSpacePoint[fdColor.LengthInPixels];

            if (sensor != null)
            {
                reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);
                reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

                sensor.Open();
            }
        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs args)
        {
            MultiSourceFrame reference = args.FrameReference.AcquireFrame();

            if (reference == null)
            {
                return;
            }

            using (var cFrame = reference.ColorFrameReference.AcquireFrame())
            using (var dFrame = reference.DepthFrameReference.AcquireFrame())
            using (var biFrame = reference.BodyIndexFrameReference.AcquireFrame())
            using (var bodyFrame = reference.BodyFrameReference.AcquireFrame())
            {
                if ((cFrame == null) || (dFrame == null) || (biFrame == null) || (bodyFrame == null))
                {
                    return;
                }

                //-----------------------
                //check performance
                long ticksNow = DateTime.Now.Ticks;
                float fps = ((float)TICKS_PER_SECOND) / (ticksNow - prevTick); // 1 / ((ticksNow - prevTick) / TICKS_PER_SECOND);
                Console.Out.WriteLine("fps: " + fps);
                prevTick = ticksNow;
                //-----------------------
                //return;


                dFrame.CopyFrameDataToArray(depthDataSource);
                biFrame.CopyFrameDataToArray(biDataSource);


                // body frame
                if (this.bodies == null)
                {
                    this.bodies = new Body[bodyFrame.BodyCount];
                }
                // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                // As long as those body objects are not disposed and not set to null in the array,
                // those body objects will be re-used.
                bodyFrame.GetAndRefreshBodyData(this.bodies);

                //######################################### Get Right Arm ###############################################
                if (extendArm)
                {
                    foreach (Body body in this.bodies)
                    {
                        if (body.IsTracked)
                        {
                            joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Joint wristRight = joints[JointType.WristRight];
                            CameraSpacePoint positionWrist = wristRight.Position;
                            if (positionWrist.Z < 0)
                            {
                                positionWrist.Z = InferredZPositionClamp;
                            }
                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(positionWrist);
                            if (wristRight.TrackingState == TrackingState.Tracked)
                            {
                                this.armJointPoints[JointType.WristRight] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            Joint elbowRight = joints[JointType.ElbowRight];
                            CameraSpacePoint positionElbow = elbowRight.Position;
                            if (positionElbow.Z < 0)
                            {
                                positionElbow.Z = InferredZPositionClamp;
                            }
                            depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(positionElbow);
                            
                            if (elbowRight.TrackingState == TrackingState.Tracked)
                            {
                                this.armJointPoints[JointType.ElbowRight] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }
                            isArmDetected = true;

                        }
                    }
                }
                //##############################


                //-------------------------------------------------------------------
                // Color Frame
                FrameDescription fdColor = cFrame.FrameDescription;

                // cut out color frame according to stencil mask
                if (cFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                {
                    cFrame.CopyRawFrameDataToArray(combiColorBuffer1080p);
                }
                else
                {
                    cFrame.CopyConvertedFrameDataToArray(combiColorBuffer1080p, ColorImageFormat.Bgra);
                }

                if (!isFullHD)
                {
                    sensor.CoordinateMapper.MapDepthFrameToColorSpace(depthDataSource, depthIntoColorSpace);
                    Array.Clear(combiColorBuffer, 0, combiColorBuffer.Length);


                    unsafe
                    {
                        fixed (byte* ptrCombiColorBuffer = combiColorBuffer)
                        fixed (byte* ptrCombiColorBuffer1080p = combiColorBuffer1080p)
                        {
                            bool stretchEnabled = false;
                            int xElbow = 0;
                            int yElbow = 0;
                            int xWrist = 0;
                            int yWrist = 0;

                            /* // could be used for better start / end conditions                            
                            // normal vector of the right lower arm 
                            float xNormalVector = 0;
                            float yNormalVector = 0;
                            */
                            double normalizedAngle = 0.0;

                            if (isArmDetected && extendArm)
                            {
                                if ((joints[JointType.WristRight].TrackingState == TrackingState.Tracked) && (joints[JointType.ElbowRight].TrackingState == TrackingState.Tracked))
                                {
                                    stretchEnabled = true;
                                    Point pWrist = armJointPoints[JointType.WristRight];
                                    Point pElbow = armJointPoints[JointType.ElbowRight];
                                    int depthWidth = sensor.DepthFrameSource.FrameDescription.Width;
                                    int depthHeight = sensor.DepthFrameSource.FrameDescription.Height;
                                    //Console.Out.WriteLine("pElbow: x =" + pElbow.X + ", y =" + pElbow.Y);
                                    int indexWrist = ((int)pWrist.Y) * depthWidth + ((int)pWrist.X);
                                    int indexElbow = ((int)pElbow.Y) * depthWidth + ((int)pElbow.X);
                                    xElbow = (int)pElbow.X;
                                    yElbow = (int)pElbow.Y;
                                    xWrist = (int)pWrist.X;
                                    yWrist = (int)pWrist.Y;

                                    double deltaX = pWrist.X - pElbow.X;
                                    double deltaY = pWrist.Y - pElbow.Y;
                                    // gradient is positive for vectors going up
                                    if (deltaX == 0) { deltaX = 0.01f; } // hot-fix to avoid division by zero 
                                    double gradient = deltaY / deltaX; // slope
                                    double angle = Math.Atan(gradient) * 180.0f / Math.PI; // value between -90 and 90 degree

                                    // no differentiation btw pos or neg slope; normalized direction
                                    normalizedAngle = Math.Abs(angle / 180.0f);

                                    // computes the left hand side normal vector
                                    // --> the arm is on the right hand side of this normal vector
                                    //xNormalVector = (deltaY) * -1;
                                    //yNormalVector = deltaX;
                                }
                            }

                            //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
                            for (int i = 0; i < depthIntoColorSpace.Length; i++)
                            {
                                bool extendedHandDrawn = false;

                                //always stretch in x-dir
                                if (stretchEnabled)
                                {
                                    // Determine the current 2 dimensional coordinate in the image (x, y)
                                    int w = sensor.DepthFrameSource.FrameDescription.Width;
                                    int x = i % w;
                                    int y = i / w;

                                    // Check if the point is on the right hand side of the normal vector
                                    if ((x > xElbow)) //&& (x < xWrist))
                                    {
                                        extendedHandDrawn = true; // hack

                                        int offsetX = x - xElbow;
                                        int lookupX = (int) (xElbow + (offsetX / (2.0 - normalizedAngle)));

                                        int offsetY = y - yElbow;
                                        int lookupY = (int) (yElbow + (offsetY / (1.0 + normalizedAngle)));

                                        int colorPointX_stretch = (int)(depthIntoColorSpace[w * lookupY + lookupX].X + 0.5);
                                        int colorPointY_stretch = (int)(depthIntoColorSpace[w * lookupY + lookupX].Y + 0.5);
                                        uint* intPtr_stretch = (uint*)(ptrCombiColorBuffer + i * 4); //stays the same

                                        if ((biDataSource[w * lookupY + lookupX] != 0xff) &&
                                            (colorPointY_stretch < fdColor.Height) && (colorPointX_stretch < fdColor.Width) &&
                                            (colorPointY_stretch >= 0) && (colorPointX_stretch >= 0))
                                        {
                                            uint* intPtr1080p = (uint*)(ptrCombiColorBuffer1080p + (colorPointY_stretch * fdColor.Width + colorPointX_stretch) * 4); // corresponding pixel in the 1080p image
                                            *intPtr_stretch = *intPtr1080p; // assign color value (4 bytes)
                                            *(((byte*)intPtr_stretch) + 3) = (byte)userTransparency.Value; // overwrite the alpha value                                            
                                        }
                                    }
                                }
                                if (extendedHandDrawn)
                                {
                                    continue;
                                }

                                int colorPointX = (int)(depthIntoColorSpace[i].X + 0.5);
                                int colorPointY = (int)(depthIntoColorSpace[i].Y + 0.5);
                                uint* intPtr = (uint*)(ptrCombiColorBuffer + i * 4);

                                if ((biDataSource[i] != 0xff) &&
                                    (colorPointY < fdColor.Height) && (colorPointX < fdColor.Width) &&
                                    (colorPointY >= 0) && (colorPointX >= 0))
                                {
                                    uint* intPtr1080p = (uint*)(ptrCombiColorBuffer1080p + (colorPointY * fdColor.Width + colorPointX) * 4); // corresponding pixel in the 1080p image
                                    *intPtr = *intPtr1080p; // assign color value (4 bytes)
                                    *(((byte*)intPtr) + 3) = (byte)userTransparency.Value; // overwrite the alpha value
                                }
                            } // for loop                            
                        }
                    } // unsafe

                    combiBitmap.Lock();
                    Marshal.Copy(combiColorBuffer, 0, combiBitmap.BackBuffer, combiColorBuffer.Length);
                    combiBitmap.AddDirtyRect(new Int32Rect(0, 0, (int)combiBitmap.Width, (int)combiBitmap.Height));
                    combiBitmap.Unlock();
                } // no full hd
                else
                {
                    sensor.CoordinateMapper.MapColorFrameToDepthSpace(depthDataSource, colorIntoDepthSpace);

                    //after the loop, only color pixels with a body index value that can be mapped to a depth value will remain in the buffer
                    for (int i = 0; i < fdColor.LengthInPixels; i++)
                    {
                        //where color map has no corresponding value in the depth map due to resolution/sensor position, the pixels are set to black
                        if (Single.IsInfinity(colorIntoDepthSpace[i].Y) || Single.IsInfinity(colorIntoDepthSpace[i].X))
                        {
                            combiColorBuffer1080p[i * 4] = 255; //b
                            combiColorBuffer1080p[i * 4 + 1] = 255; //g
                            combiColorBuffer1080p[i * 4 + 2] = 255; //r
                            combiColorBuffer1080p[i * 4 + 3] = 0; //a
                        }
                        else
                        {
                            //if bodyIndex pixel has no body assigned, draw a black pixel to the corresponding color pixel
                            int idx = (int)(sensor.BodyIndexFrameSource.FrameDescription.Width * colorIntoDepthSpace[i].Y + colorIntoDepthSpace[i].X); //2D to 1D
                            if ((biDataSource[idx] > 5) || (biDataSource[idx] < 0))
                            {
                                combiColorBuffer1080p[i * 4] = 255;
                                combiColorBuffer1080p[i * 4 + 1] = 255;
                                combiColorBuffer1080p[i * 4 + 2] = 255;
                                combiColorBuffer1080p[i * 4 + 3] = 0;
                            }
                            else
                            {
                                combiColorBuffer1080p[i * 4 + 3] = (byte)userTransparency.Value; //alpha of person set by gui-slider
                            }
                        }
                    } // for loop

                    //combiColorBuffer1080p contains all required information
                    combiBitmap1080p.WritePixels(new Int32Rect(0, 0, this.combiBitmap1080p.PixelWidth, this.combiBitmap1080p.PixelHeight), combiColorBuffer1080p, combiBitmap1080p.PixelWidth * sizeof(int), 0);
                } // full HD
            } // using Frames
        }

        private void nonFullHD_Checked(object sender, RoutedEventArgs e)
        {
            isFullHD = false;
            if (imageCombi != null)
            {
                imageCombi.Source = combiBitmap; //img with 512x424-color of body index frame;
            }
        }

        private void fullHD_Checked(object sender, RoutedEventArgs e)
        {
            isFullHD = true;
            if (imageCombi != null)
            {
                imageCombi.Source = combiBitmap1080p; //img with 1080p-color of body index frame;
            }
        }

        private void BlackBG_Checked(object sender, RoutedEventArgs e)
        {
            bgType = BackgroundType.Black;
            this.Background = new SolidColorBrush(Colors.Black);
        }

        private void WhiteBG_Checked(object sender, RoutedEventArgs e)
        {
            bgType = BackgroundType.White;
            this.Background = new SolidColorBrush(Colors.White);
        }

        private void CustomBG_Checked(object sender, RoutedEventArgs e)
        {
            bgType = BackgroundType.Custom;
            this.Background = new SolidColorBrush(Colors.Transparent);
        }

        private void checkBoxExtendArm_Checked(object sender, RoutedEventArgs e)
        {
            extendArm = true;
        }

        private void checkBoxExtendArm_Unchecked(object sender, RoutedEventArgs e)
        {
            extendArm = false;
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {

            if (this.sensor != null)
            {
                this.sensor.Close();
                this.sensor = null;
            }
        }

    } //  public partial class MainWindow : Window
} //namespace BodyExtractionAndHightlighting
