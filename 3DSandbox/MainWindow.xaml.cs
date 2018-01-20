namespace _3DSandbox
{
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
    
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        KinectFunctionalities kinectFunctionalities;
        Kinect3DOperations kinect3DOperations;
        GesturesMasterControl gestureMasterControl;
        ColorMasterConrol colorMasterControl;
        DepthMasterControl depthMasterControl;
        RenderViewFunctionalities renderViewFunctionalities;

        Viewport3D mainViewport = new Viewport3D();
        TextBlock informationTextBlock = new TextBlock();
        TextBox cubeInformationTextBox = new TextBox();

        public static string dataDirectoryPath = "";

        public static string example3DModelVerticeesName = "";
        public static string example3DModelTrianglesName = "";

        public static string rawDepthDataFileName = "";
        public static string pointCloudFileName = "";

        public static string cubeStructureSize = "";

        public MainWindow()
        {
            this.DataContext = this;
            InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;
        
       
        // Create the scene.
        // MainViewport is the Viewport3D defined
        // in the XAML code that displays everything.
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            informationTextBlock = InformationTextBlock;
            cubeInformationTextBox = CubeInformationTextBox;
            mainViewport = MainViewport;

            depthMasterControl = new DepthMasterControl(512, 424, 512 / 1, 424 / 1, false, null);

            renderViewFunctionalities = new RenderViewFunctionalities();
            kinectFunctionalities = new KinectFunctionalities(ref depthMasterControl, ref colorMasterControl, ref gestureMasterControl);
            kinectFunctionalities.initializeKinect(ref depthViewImage, ref colorViewImage);


            kinect3DOperations = new Kinect3DOperations(ref depthMasterControl, ref renderViewFunctionalities,
                                                        ref InformationTextBlock, ref cubeInformationTextBox);

            renderViewFunctionalities.initialize3DRenderView(ref MainViewport, ref informationTextBlock, ref kinect3DOperations);

            //kinectFunctionalities.assignListeners();
        }
        
        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            kinectFunctionalities.deassignListeners();
        }
        
        public static ImageSource ToBitmapDepth(int height, int width, byte[] pixels)
        {
            PixelFormat format = PixelFormats.Bgr32;
            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }
        
        // Adjust the camera's position.
        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            renderViewFunctionalities.controlCameraDirectionPosition(e);
        }
        
        private void ProcessPointCloudButton_Click(object sender, RoutedEventArgs e)
        {
            kinect3DOperations.processPointCloudIntoCubes();
        }

        private void SaveRawDepthToFileButton_Click(object sender, RoutedEventArgs e)
        {
            kinect3DOperations.saveRawDepthToFile();
        }

        private void ExtractRawDepthFromFileButton_Click(object sender, RoutedEventArgs e)
        {
            rawDepthDataFileName = rawDepthDataFileTextBox.Text;

            kinect3DOperations.extractDepthDataFromFiles();
        }

        private void ExtractPointCloudFromFileButton_Click(object sender, RoutedEventArgs e)
        {
            pointCloudFileName = pointCloudFileTextBox.Text;
            cubeStructureSize = cubeStructureSizeTextBox.Text;

            kinect3DOperations.extractSavedPointCloud();
        }
        
        private void ExtractRawDepthFromFileButton2_Click(object sender, RoutedEventArgs e)
        {
            kinect3DOperations.getPointCloudOfDepthData();
        }

        private void ShowAccessabilityMapButton_Click(object sender, RoutedEventArgs e)
        {
            dataDirectoryPath = dataDirectoryPathTextBox.Text;
            cubeStructureSize = cubeStructureSizeTextBox.Text;
            
            kinect3DOperations.createSimplifiedMeshFromGridActual();
            //kinect3DOperations.createSimplifiedMeshFromGrid();
            kinect3DOperations.connectAdjacentPlanes2();
            kinect3DOperations.createTrianglesOfCubePlanes();
            kinect3DOperations.labelAccessibleCubes2();
            kinect3DOperations.renderTrianglePlanes();
        }
        
        private void RenderPointCloudButton_Click(object sender, RoutedEventArgs e)
        {
            kinect3DOperations.renderPointCloud();
        }

        private void ExtractMeshDataFromFilesButton_Click(object sender, RoutedEventArgs e)
        {
            example3DModelTrianglesName = example3DModelTrianglesTextBox.Text;
            example3DModelVerticeesName = example3DModelVerticeesTextBox.Text;

            kinect3DOperations.extractMeshDataFromFiles();
        }

        private void RenderExampleModelButton_Click(object sender, RoutedEventArgs e)
        {
            renderViewFunctionalities.renderDesignatedMesh(ref kinect3DOperations.triangles,
                ref kinect3DOperations.vertices);
        }
        
        private void TabControl_PreviewKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Left || e.Key == Key.Right)
                e.Handled = true;
        }
        
        private void GetPointCloudFromExample_Click(object sender, RoutedEventArgs e)
        {
           kinect3DOperations.getPointCloudOfExampleModel();
        }

        private void SavePointCloudButton_Click(object sender, RoutedEventArgs e)
        {
            dataDirectoryPath = dataDirectoryPathTextBox.Text;
            cubeStructureSize = cubeStructureSizeTextBox.Text;

            //kinect3DOperations.extractDepthDataFromFiles();
            //kinect3DOperations.getPointCloudOfDepthData();

            depthMasterControl.saveSinglePointCloudSignal();
        }

        private void ClearInformationTextButton_Click(object sender, RoutedEventArgs e)
        {
            dataDirectoryPath = dataDirectoryPathTextBox.Text;
            informationTextBlock.Text = "";
            //kinect3DOperations.extractSavedPointCloud();
            kinect3DOperations.runPlaneTest();
            //kinect3DOperations.runConvertPolygonIntoTrianglesTest();
            //kinect3DOperations.savePointCloudInFile();
        }

        private void CreatePointCloudMeshButton_Click(object sender, RoutedEventArgs e)
        {
            kinect3DOperations.createPointCloudMesh();
        }

        private void RenderPointCloudMeshButton_Click(object sender, RoutedEventArgs e)
        {
            kinect3DOperations.renderPointCloudMesh();
        }

        private void ProcessPointCloudMeshButton_Click(object sender, RoutedEventArgs e)
        {
            kinect3DOperations.processPointCloudMeshIntoCubes();
        }

        private void MainViewport_MouseDown(object sender, MouseButtonEventArgs e)
        {
            renderViewFunctionalities.viewport3DMouseDown(e);
        }

        private void GetColorFeedButton_Click(object sender, RoutedEventArgs e)
        {
            kinectFunctionalities.enableColorFrame();
        }

        private void RemoveColorFeedButton_Click(object sender, RoutedEventArgs e)
        {
            kinectFunctionalities.dissableColorFrame();
        }

        private void GetDepthFeedButton_Click(object sender, RoutedEventArgs e)
        {
            kinectFunctionalities.enableDepthFrame();
        }

        private void RemoveDepthFeedButton_Click(object sender, RoutedEventArgs e)
        {
            kinectFunctionalities.dissableDepthFrame();
        }
        
    }
}

