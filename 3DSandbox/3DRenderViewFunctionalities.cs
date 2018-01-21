using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace _3DSandbox
{
    class RenderViewFunctionalities
    {
        Kinect3DOperations kinect3DOperations;
        
        private bool cameraNeedsPositionUpdate = false;
        private bool cameraNeedsRotationUpdate = false;

        public TextBlock informationTextBlock;

        // The camera.
        public PerspectiveCamera TheCamera;

        // The camera's current location.
        private double CameraPhi = Math.PI / 4.0;       // 45 degrees
        private double CameraTheta = Math.PI / 2.0;     // 90 degrees
        private double CameraR = 75;

        private double CameraX = 0;
        private double CameraY = 0;
        private double CameraZ = 0;

        private double CameraDX = 0.5;
        private double CameraDY = 0.5;
        private double CameraDZ = 0.5;

        private double CameraLookX = 0;
        private double CameraLookY = 0;
        private double CameraLookZ = 0;

        private bool shiftKeyPressed = false;

        private Vector3D cameraVerticalAxis = new Vector3D(0, 1, 0);
        private Vector3D cameraHorizontalAxis = new Vector3D(1, 0, 0);

        // The change in CameraPhi when you press the up and down arrows.
        private const double CameraDPhi = 0.1;

        // The change in CameraTheta when you press the left and right arrows.
        private const double CameraDTheta = 0.1;

        // The change in CameraR when you press + or -.
        private const double CameraDR = 0.3;

        public Model3DGroup MainModel3Dgroup = new Model3DGroup();
        public Viewport3D mainViewPort;

        /// <summary> 
        /// Initialize all of the functionalities of the render view.
        /// </summary>
        /// <param name="viewport"></param>
        public void initialize3DRenderView(ref Viewport3D viewport, ref TextBlock informationTextBlock,
            ref Kinect3DOperations kinect3DOperations)
        {
            this.informationTextBlock = informationTextBlock;
            this.kinect3DOperations = kinect3DOperations;

            mainViewPort = viewport;
            // Give the camera its initial position.
            TheCamera = new PerspectiveCamera();
            TheCamera.FieldOfView = 90;
            viewport.Camera = TheCamera;
            PositionCamera(false, true);

            //extractData();

            // Define lights.
            DefineLights();

            // Create the model.
            // DefineModelScope(MainModel3Dgroup);

            // Add the group of models to a ModelVisual3D.
            ModelVisual3D model_visual = new ModelVisual3D();
            model_visual.Content = MainModel3Dgroup;

            // Add the main visual to the viewportt.
            viewport.Children.Add(model_visual);
        }

        public void viewport3DMouseDown(MouseButtonEventArgs e)
        {
            // Get the mouse's position relative to the viewport.
            Point mouse_pos = e.GetPosition(mainViewPort);

            // Perform the hit test.
            HitTestResult result =
                VisualTreeHelper.HitTest(mainViewPort, mouse_pos);
            
            // See if we hit a model.
            RayMeshGeometry3DHitTestResult mesh_result =
                result as RayMeshGeometry3DHitTestResult;
            
            if (mesh_result != null)
            {
                GeometryModel3D model =
                    (GeometryModel3D)mesh_result.ModelHit;

                /*
                if (SelectableModels.Contains(model))
                {
                    SelectedModel = model;
                    SelectedModel.Material = SelectedMaterial;
                }
                */

                kinect3DOperations.getRayCastedCube(mesh_result);
                informationTextBlock.Text += "HIT: " + mesh_result.PointHit.ToString() + "\n";
            }
        }

        /// <summary>
        /// Control the camera direction and position whenever the user presses a relevant key.
        /// </summary>
        /// <param name="key"></param>
        public void controlCameraDirectionPosition(KeyEventArgs key)
        {
            switch (key.Key)
            {
                case Key.F5:
                    CameraDY = CameraDY * 2;
                    CameraDX = CameraDX * 2;
                    CameraDZ = CameraDZ * 2;
                    break;
                case Key.F6:
                    CameraDY = CameraDY / 2;
                    CameraDX = CameraDX / 2;
                    CameraDZ = CameraDZ / 2;
                    break;
                case Key.R:
                    CameraY += CameraDY;
                    cameraNeedsPositionUpdate = true;
                    break;
                case Key.F:
                    CameraY -= CameraDY;
                    cameraNeedsPositionUpdate = true;
                    break;
                case Key.A:
                    CameraX -= CameraDX;
                    cameraNeedsPositionUpdate = true;
                    break;
                case Key.D:
                    CameraX += CameraDX;
                    cameraNeedsPositionUpdate = true;
                    break;
                case Key.W:
                    CameraZ -= CameraDZ;
                    cameraNeedsPositionUpdate = true;
                    break;
                case Key.S:
                    CameraZ += CameraDZ;
                    cameraNeedsPositionUpdate = true;
                    break;
                case Key.U:
                    CameraPhi += CameraDPhi;
                    cameraVerticalAxis = MathAncillary.rotateVectorAboutAxis(cameraVerticalAxis,
                        cameraHorizontalAxis, -1 * CameraDPhi, true);
                    if (CameraPhi > Math.PI / 2.0) CameraPhi = Math.PI / 2.0;
                    cameraNeedsRotationUpdate = true;
                    break;
                case Key.J:
                    CameraPhi -= CameraDPhi;
                    cameraVerticalAxis = MathAncillary.rotateVectorAboutAxis(cameraVerticalAxis,
                        cameraHorizontalAxis, CameraDPhi, true);
                    if (CameraPhi < -Math.PI / 2.0) CameraPhi = -Math.PI / 2.0;
                    cameraNeedsRotationUpdate = true;
                    break;
                case Key.H:
                    cameraHorizontalAxis = MathAncillary.rotateVectorAboutAxis(cameraHorizontalAxis,
                        cameraVerticalAxis, -1 * CameraDPhi, true);
                    CameraTheta += CameraDTheta;
                    cameraNeedsRotationUpdate = true;
                    break;
                case Key.K:
                    cameraHorizontalAxis = MathAncillary.rotateVectorAboutAxis(cameraHorizontalAxis,
                        cameraVerticalAxis, CameraDPhi, true);
                    CameraTheta -= CameraDTheta;
                    cameraNeedsRotationUpdate = true;
                    break;
                case Key.Add:
                case Key.OemPlus:
                    CameraR -= CameraDR * 5;
                    if (CameraR < CameraDR) CameraR = CameraDR;
                    cameraNeedsRotationUpdate = true;
                    break;
                case Key.Subtract:
                case Key.OemMinus:
                    CameraR += CameraDR * 5;
                    cameraNeedsRotationUpdate = true;
                    break;
            }

            if (cameraNeedsPositionUpdate == true || cameraNeedsRotationUpdate == true)
            {
                PositionCamera(cameraNeedsPositionUpdate, cameraNeedsRotationUpdate);
                cameraNeedsPositionUpdate = false;
                cameraNeedsRotationUpdate = false;
            }
        }
        
        

        /// <summary>
        /// Render point cloud vertices as cubes. This is done to give a perspective of what the raw
        /// point cloud looks like.
        /// </summary>
        /// <param name="verticesGrid"></param>
        public void renderPointCloudVerticesOnly(ref Dictionary<string, Dictionary<int, Point3D>> verticesGrid)
        {
            MeshGeometry3D cuboidMesh = new MeshGeometry3D();
            DiffuseMaterial surface_material = new DiffuseMaterial(Brushes.Violet);

            var keys = verticesGrid.Keys;
            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();

            foreach (string key in keys)
            {
                containedGridVertices = verticesGrid[key];
                var containedGridVerticesKeys = containedGridVertices.Keys;

                foreach (int vertex in containedGridVerticesKeys)
                {
                    Point3D point = containedGridVertices[vertex];
                    renderSingleVertex(cuboidMesh, point);
                }
            }

            GeometryModel3D surface_model_cuboid = new GeometryModel3D(cuboidMesh, surface_material);

            // Make the surface visible from both sides.
            surface_model_cuboid.BackMaterial = surface_material;

            // Add the model to the model groups.
            MainModel3Dgroup.Children.Add(surface_model_cuboid);
        }
        
        /// <summary>
        /// Position the camera according to the controls inputted by the user.
        /// </summary>
        public void PositionCamera(bool axisMovementRequested, bool rotationRequested)
        {
            double newX = 0, newY = 0, newZ = 0;
            double oldX = 0, oldY = 0, oldZ = 0;

            if (axisMovementRequested)
            {
                newY = CameraY;
                newX = CameraX;
                newZ = CameraZ;

                CameraR = Math.Sqrt((newX * newX) + (newY * newY) + (newZ * newZ));
                CameraTheta = Math.Asin(newY / CameraR);
                CameraPhi = Math.Atan(newX / newZ);

                if (CameraPhi > Math.PI / 2.0) CameraPhi = Math.PI / 2.0;
                if (CameraPhi < -Math.PI / 2.0) CameraPhi = -Math.PI / 2.0;

                CameraLookX = 0;
                CameraLookY = CameraLookY;
                CameraLookZ = CameraLookZ;
            }

            if(rotationRequested)
            {
               
                newY = CameraR * Math.Sin(CameraPhi);
                double hyp = CameraR * Math.Cos(CameraPhi);
                newX = hyp * Math.Cos(CameraTheta);
                newZ = hyp * Math.Sin(CameraTheta);
                
 /*
                newY = CameraR * Math.Sin(CameraTheta) * Math.Cos(CameraPhi);
                newX = CameraR * Math.Cos(CameraTheta) * Math.Sin(CameraPhi);
                newZ = CameraR * Math.Cos(CameraPhi);
*/
                CameraY = newY;
                CameraX = newX;
                CameraZ = newZ;

                oldX = newX;
                oldY = newY;
                oldZ = newZ;

                CameraLookX = -newX;
                CameraLookY = -newY;
                CameraLookZ = -newZ;
            }

            TheCamera.Position = new Point3D(newX, newY, newZ);

            // Look toward the origin.
            TheCamera.LookDirection = new Vector3D(CameraLookX, CameraLookY, CameraLookZ);

            // Set the Up direction.
            TheCamera.UpDirection = new Vector3D(0, 1, 0);
        }

        public void renderCubeGridLines(MeshGeometry3D mesh, double cubeSize)
        {
            int gridSize = 15;
            int gridHalfSize = gridSize / 2;
            int i = 0, j = 0;


            Point3D startingPoint = new Point3D(-1 * gridHalfSize * cubeSize, -1 * gridHalfSize * cubeSize,
                -1 * gridHalfSize * cubeSize);
            Point3D endingPoint = new Point3D(gridHalfSize * cubeSize, -1 * gridHalfSize * cubeSize,
                -1 * gridHalfSize * cubeSize);
            
            // Constant YZ plane: 11 * 11
            for (i = 0; i < gridSize; i++)
            {
                startingPoint.Y += cubeSize;
                endingPoint.Y += cubeSize;
                startingPoint.Z -= cubeSize * j;
                endingPoint.Z -= cubeSize * j;

                for (j = 0; j < gridSize; j++)
                {
                    startingPoint.Z += cubeSize;
                    endingPoint.Z += cubeSize;

                    renderSingleLine(mesh, 1, 0.001, startingPoint, endingPoint);
                }
            }

            // Constant XZ plane: 11 * 11

            startingPoint = new Point3D(-1 * gridHalfSize * cubeSize, -1 * gridHalfSize * cubeSize,
                -1 * gridHalfSize * cubeSize);
            endingPoint = new Point3D(-1 * gridHalfSize * cubeSize, gridHalfSize * cubeSize,
                -1 * gridHalfSize * cubeSize);
            j = 0;
            for (i = 0; i < gridSize; i++)
            {
                startingPoint.X += cubeSize;
                endingPoint.X += cubeSize;
                startingPoint.Z -= cubeSize * j;
                endingPoint.Z -= cubeSize * j;

                for (j = 0; j < gridSize; j++)
                {
                    startingPoint.Z += cubeSize;
                    endingPoint.Z += cubeSize;

                    renderSingleLine(mesh, 2, 0.001, startingPoint, endingPoint);
                }
            }


            // Constant XY plane: 11 * 11
            startingPoint = new Point3D(-1 * gridHalfSize * cubeSize, -1 * gridHalfSize * cubeSize,
                -1 * gridHalfSize * cubeSize);
            endingPoint = new Point3D(-1 * gridHalfSize * cubeSize, -1 * gridHalfSize * cubeSize,
                gridHalfSize * cubeSize);
            j = 0;
            for (i = 0; i < gridSize; i++)
            {
                startingPoint.X += cubeSize;
                endingPoint.X += cubeSize;
                startingPoint.Y -= cubeSize * j;
                endingPoint.Y -= cubeSize * j;

                for (j = 0; j < gridSize; j++)
                {
                    startingPoint.Y += cubeSize;
                    endingPoint.Y += cubeSize;

                    renderSingleLine(mesh, 3, 0.001, startingPoint, endingPoint);
                }
            }

        }

        public void renderSingleLine(MeshGeometry3D mesh, int axis, double thickness,
            Point3D startingPoint, Point3D endingPoint)
        {
            double X, Y, Z;
            Point3D A;
            Point3D B;
            Point3D C;
            Point3D D;
            Point3D E;
            Point3D F;
            Point3D G;
            Point3D H;

            if (axis == 1)
            {
                X = startingPoint.X;
                Y = startingPoint.Y;
                Z = startingPoint.Z;

                // X axis
                C = new Point3D(X, Y - thickness, Z - thickness);
                A = new Point3D(X, Y - thickness, Z + thickness);
                G = new Point3D(X, Y + thickness, Z - thickness);
                E = new Point3D(X, Y + thickness, Z + thickness);

                X = endingPoint.X;
                D = new Point3D(X, Y - thickness, Z - thickness);
                B = new Point3D(X, Y - thickness, Z + thickness);
                H = new Point3D(X, Y + thickness, Z - thickness);
                F = new Point3D(X, Y + thickness, Z + thickness);

                
            } else if(axis == 2)
            {
                X = startingPoint.X;
                Y = startingPoint.Y;
                Z = startingPoint.Z;

                // Y axis
                C = new Point3D(X - thickness, Y, Z - thickness);
                D = new Point3D(X + thickness, Y, Z - thickness);
                A = new Point3D(X - thickness, Y, Z + thickness);
                B = new Point3D(X + thickness, Y, Z + thickness);

                Y = endingPoint.Y;
                G = new Point3D(X - thickness, Y, Z - thickness);
                H = new Point3D(X + thickness, Y, Z - thickness);
                E = new Point3D(X - thickness, Y, Z + thickness);
                F = new Point3D(X + thickness, Y, Z + thickness);
            } else
            {
                X = startingPoint.X;
                Y = startingPoint.Y;
                Z = startingPoint.Z;

                // Z axis
                
                A = new Point3D(X - thickness, Y - thickness, Z);
                B = new Point3D(X + thickness, Y - thickness, Z);
                E = new Point3D(X - thickness, Y + thickness, Z);
                F = new Point3D(X + thickness, Y + thickness, Z);

                Z = endingPoint.Z;
                C = new Point3D(X - thickness, Y - thickness, Z);
                D = new Point3D(X + thickness, Y - thickness, Z);
                G = new Point3D(X - thickness, Y + thickness, Z);
                H = new Point3D(X + thickness, Y + thickness, Z);
               
            }

            AddTriangle(mesh, A, E, F);
            AddTriangle(mesh, A, F, B);

            AddTriangle(mesh, C, G, H);
            AddTriangle(mesh, C, H, D);

            AddTriangle(mesh, A, E, G);
            AddTriangle(mesh, A, G, C);

            AddTriangle(mesh, A, B, C);
            AddTriangle(mesh, B, C, D);

            AddTriangle(mesh, B, F, H);
            AddTriangle(mesh, B, H, D);

            AddTriangle(mesh, F, E, G);
            AddTriangle(mesh, F, G, H);
        }

        public void renderSingleTransparentCube(MeshGeometry3D mesh, Point3D vertex, double sideLength)
        {
            double X = vertex.X;
            double Y = vertex.Y;
            double Z = vertex.Z;
            double sideLengthHalf = sideLength / 2;

            /* g h
             *  e f
             *  
             * c d
             *  a b
            */

            Point3D A = new Point3D(X + -1 * sideLengthHalf, Y + -1 * sideLengthHalf, Z + sideLengthHalf);
            Point3D B = new Point3D(X + sideLengthHalf, Y + -1 * sideLengthHalf, Z + sideLengthHalf);
            Point3D C = new Point3D(X + -1 * sideLengthHalf, Y + -1 * sideLengthHalf, Z + -1 * sideLengthHalf);
            Point3D D = new Point3D(X + sideLengthHalf, Y + -1 * sideLengthHalf, Z + -1 * sideLengthHalf);
            Point3D E = new Point3D(X + -1 * sideLengthHalf, Y + sideLengthHalf, Z + sideLengthHalf);
            Point3D F = new Point3D(X + sideLengthHalf, Y + sideLengthHalf, Z + sideLengthHalf);
            Point3D G = new Point3D(X + -1 * sideLengthHalf, Y + sideLengthHalf, Z + -1 * sideLengthHalf);
            Point3D H = new Point3D(X + sideLengthHalf, Y + sideLengthHalf, Z + -1 * sideLengthHalf);

            AddTriangle(mesh, A, E, F);
            AddTriangle(mesh, A, F, B);

            AddTriangle(mesh, C, G, H);
            AddTriangle(mesh, C, H, D);

            AddTriangle(mesh, A, E, G);
            AddTriangle(mesh, A, G, C);

            AddTriangle(mesh, A, B, C);
            AddTriangle(mesh, B, C, D);

            AddTriangle(mesh, B, F, H);
            AddTriangle(mesh, B, H, D);

            AddTriangle(mesh, F, E, G);
            AddTriangle(mesh, F, G, H);
        }

        public void renderSingleVertex(MeshGeometry3D mesh, Point3D vertex)
        {

            double X = vertex.X;
            double Y = vertex.Y;
            double Z = vertex.Z;
            double sideLengthHalf = 0.008;
            
            Point3D A = new Point3D(X + -1 * sideLengthHalf, Y + -1 * sideLengthHalf, Z + sideLengthHalf);
            Point3D B = new Point3D(X + sideLengthHalf, Y + -1 * sideLengthHalf, Z + sideLengthHalf);
            Point3D C = new Point3D(X + -1 * sideLengthHalf, Y + -1 * sideLengthHalf, Z + -1 * sideLengthHalf);
            Point3D D = new Point3D(X + sideLengthHalf, Y + -1 * sideLengthHalf, Z + -1 * sideLengthHalf);
            Point3D E = new Point3D(X + -1 * sideLengthHalf, Y + sideLengthHalf, Z + sideLengthHalf);
            Point3D F = new Point3D(X + sideLengthHalf, Y + sideLengthHalf, Z + sideLengthHalf);
            Point3D G = new Point3D(X + -1 * sideLengthHalf, Y + sideLengthHalf, Z + -1 * sideLengthHalf);
            Point3D H = new Point3D(X + sideLengthHalf, Y + sideLengthHalf, Z + -1 * sideLengthHalf);

            AddTriangle(mesh, A, E, F);
            AddTriangle(mesh, A, F, B);

            AddTriangle(mesh, C, G, H);
            AddTriangle(mesh, C, H, D);

            AddTriangle(mesh, A, E, G);
            AddTriangle(mesh, A, G, C);

            AddTriangle(mesh, A, B, C);
            AddTriangle(mesh, B, C, D);

            AddTriangle(mesh, B, F, H);
            AddTriangle(mesh, B, H, D);

            AddTriangle(mesh, F, E, G);
            AddTriangle(mesh, F, G, H);
        }

        // Define the lights.
        public void DefineLights()
        {
            AmbientLight ambient_light = new AmbientLight(Colors.Gray);
            DirectionalLight directional_light =
                new DirectionalLight(Colors.Gray, new Vector3D(-1.0, -3.0, -2.0));
            MainModel3Dgroup.Children.Add(ambient_light);
            MainModel3Dgroup.Children.Add(directional_light);
        }

        /// <summary>
        /// Render a specific model given a list of vertices and triangles
        /// </summary>
        /// <param name="triangles"></param>
        /// <param name="vertices"></param>
        public void renderDesignatedMesh(ref List<string> triangles, ref List<string> vertices)
        {
            MeshGeometry3D scopeMesh = new MeshGeometry3D();
            string[] triangleLineSeperated = new string[3];
            string[] triangleLineVertexSeperated = new string[3];
            char splitter = ' ';

            string vertecesLine = "";
            string[] vertexLineSeperated = new string[3];

            Point3D A = new Point3D();
            Point3D B = new Point3D();
            Point3D C = new Point3D();

            foreach (string triangleLine in triangles)
            {
                
                triangleLineSeperated = triangleLine.Split(splitter);

                triangleLineVertexSeperated = triangleLineSeperated[0].Split('/');
                vertecesLine = vertices[int.Parse(triangleLineVertexSeperated[0]) - 1];
                vertexLineSeperated = vertecesLine.Split(splitter);
                A = new Point3D(Double.Parse(vertexLineSeperated[0]), Double.Parse(vertexLineSeperated[1]),
                    Double.Parse(vertexLineSeperated[2]));

                triangleLineVertexSeperated = triangleLineSeperated[1].Split('/');
                vertecesLine = vertices[int.Parse(triangleLineVertexSeperated[0]) - 1];
                vertexLineSeperated = vertecesLine.Split(splitter);
                B = new Point3D(Double.Parse(vertexLineSeperated[0]), Double.Parse(vertexLineSeperated[1]),
                    Double.Parse(vertexLineSeperated[2]));

                triangleLineVertexSeperated = triangleLineSeperated[2].Split('/');
                vertecesLine = vertices[int.Parse(triangleLineVertexSeperated[0]) - 1];
                vertexLineSeperated = vertecesLine.Split(splitter);
                C = new Point3D(Double.Parse(vertexLineSeperated[0]), Double.Parse(vertexLineSeperated[1]),
                    Double.Parse(vertexLineSeperated[2]));

                AddTriangle(scopeMesh, A, B, C);
            }

            DiffuseMaterial surface_material = new DiffuseMaterial(Brushes.Orange);
            GeometryModel3D surface_model_scope = new GeometryModel3D(scopeMesh, surface_material);
            surface_model_scope.BackMaterial = surface_material;
            MainModel3Dgroup.Children.Add(surface_model_scope);
        }

        /// <summary>
        /// Add a triangle to a given mesh.
        /// </summary>
        /// <param name="mesh"></param>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <param name="point3"></param>
        public void AddTriangle(MeshGeometry3D mesh, Point3D point1, Point3D point2, Point3D point3)
        {
            // Get the points' indices.
            int index1 = AddPoint(mesh.Positions, point1);
            int index2 = AddPoint(mesh.Positions, point2);
            int index3 = AddPoint(mesh.Positions, point3);

            // Create the triangle.
            mesh.TriangleIndices.Add(index1);
            mesh.TriangleIndices.Add(index2);
            mesh.TriangleIndices.Add(index3);
        }

        /// <summary>
        /// Create the point and return its new index.
        /// </summary>
        /// <param name="points"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public int AddPoint(Point3DCollection points, Point3D point)
        {
            // Create the point and return its index.
            points.Add(point);
            return points.Count - 1;
        }
        
    }
}
