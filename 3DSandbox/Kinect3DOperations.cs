using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace _3DSandbox
{
    class Kinect3DOperations
    {
        ConnectOfCube[,,] connectionToNeighboringCubes = new ConnectOfCube[3, 3, 3] {
        {{ConnectOfCube.EFG, ConnectOfCube.EF, ConnectOfCube.HEF},
            {ConnectOfCube.BF, ConnectOfCube.LEFT, ConnectOfCube.AE },
            {ConnectOfCube.ABC, ConnectOfCube.AB, ConnectOfCube.DAB}},
        {{ConnectOfCube.FG, ConnectOfCube.BOTTOM, ConnectOfCube.HE},
            {ConnectOfCube.BACK, ConnectOfCube.NONE, ConnectOfCube.FRONT},
            {ConnectOfCube.BC, ConnectOfCube.TOP, ConnectOfCube.DA}},
        {{ConnectOfCube.FGH, ConnectOfCube.GH, ConnectOfCube.GHE},
            {ConnectOfCube.CG, ConnectOfCube.RIGHT, ConnectOfCube.DH},
            {ConnectOfCube.BCD, ConnectOfCube.CD, ConnectOfCube.CDA}}
        };

        ConnectOfCube[,] edgeNeighbors = new ConnectOfCube[12, 3]
        {
            {ConnectOfCube.AB, ConnectOfCube.LEFT, ConnectOfCube.TOP },     // AB
            {ConnectOfCube.BC, ConnectOfCube.BACK, ConnectOfCube.TOP },     // BC
            {ConnectOfCube.CD, ConnectOfCube.RIGHT, ConnectOfCube.TOP },    // CD
            {ConnectOfCube.DA, ConnectOfCube.FRONT, ConnectOfCube.TOP },    // DA

            {ConnectOfCube.EF, ConnectOfCube.LEFT, ConnectOfCube.BOTTOM },     // EF
            {ConnectOfCube.FG, ConnectOfCube.BACK, ConnectOfCube.BOTTOM },     // FG
            {ConnectOfCube.GH, ConnectOfCube.RIGHT, ConnectOfCube.BOTTOM },    // GH
            {ConnectOfCube.HE, ConnectOfCube.FRONT, ConnectOfCube.BOTTOM },    // HE

            {ConnectOfCube.AE, ConnectOfCube.LEFT, ConnectOfCube.FRONT },    // AE
            {ConnectOfCube.BF, ConnectOfCube.LEFT, ConnectOfCube.BACK },     // BF
            {ConnectOfCube.CG, ConnectOfCube.RIGHT, ConnectOfCube.BACK },    // CG
            {ConnectOfCube.DH, ConnectOfCube.RIGHT, ConnectOfCube.FRONT}     // DH
        };

        EdgesOfCube[,] edgeNeighborsComplimentaryEdge = new EdgesOfCube[12, 3]
        {
            {EdgesOfCube.GH, EdgesOfCube.CD, EdgesOfCube.EF}, // AB
            {EdgesOfCube.HE, EdgesOfCube.DA, EdgesOfCube.FG}, // BC
            {EdgesOfCube.EF, EdgesOfCube.AB, EdgesOfCube.GH}, // CD
            {EdgesOfCube.FG, EdgesOfCube.BC, EdgesOfCube.HE}, // DAw

            {EdgesOfCube.CD, EdgesOfCube.GH, EdgesOfCube.AB}, // EF
            {EdgesOfCube.DA, EdgesOfCube.HE, EdgesOfCube.BC}, // FG
            {EdgesOfCube.AB, EdgesOfCube.EF, EdgesOfCube.CD}, // GH
            {EdgesOfCube.BC, EdgesOfCube.FG, EdgesOfCube.DA}, // HE

            {EdgesOfCube.CG, EdgesOfCube.DH, EdgesOfCube.BF}, // AE
            {EdgesOfCube.DH, EdgesOfCube.CG, EdgesOfCube.AE}, // BF
            {EdgesOfCube.AE, EdgesOfCube.BF, EdgesOfCube.DH}, // CG
            {EdgesOfCube.BF, EdgesOfCube.AE, EdgesOfCube.CG}  // DH
        };

        public EdgesOfCube[] edgesOfCubeListing = new EdgesOfCube[12]
        {
            EdgesOfCube.AB, EdgesOfCube.BC, EdgesOfCube.CD, EdgesOfCube.DA,
            EdgesOfCube.EF, EdgesOfCube.FG, EdgesOfCube.GH, EdgesOfCube.HE,
            EdgesOfCube.AE, EdgesOfCube.BF, EdgesOfCube.CG, EdgesOfCube.DH
        };

        public Kinect3DOperations(ref DepthMasterControl depthMasterControl,
                                  ref RenderViewFunctionalities renderViewFunctionalities,
                                  ref TextBox informationTextBlock, ref TextBox cubeInformationTextBox)
        {
            this.depthMasterControl = depthMasterControl;
            this.renderViewFunctionalities = renderViewFunctionalities;
            this.informationTextBlock = informationTextBlock;
            this.cubeInformationTextBox = cubeInformationTextBox;

            // For the transparent selected cubes:
            qOuterMaterial.Children.Add(qDiffTransYellow);
            qOuterMaterial.Children.Add(qSpecTransWhite);
        }

        /*
        EdgesOfCube[] edgeNeighborEdges = { EdgesOfCube.CD, EdgesOfCube.DA, EdgesOfCube.AB, EdgesOfCube.BC,
                                            EdgesOfCube.GH, EdgesOfCube.HE, EdgesOfCube.EF, EdgesOfCube.FG,
                                            EdgesOfCube.CG, EdgesOfCube.DH, EdgesOfCube.AE, EdgesOfCube.BF};
        */

        EdgesOfCube[] edgeNeighborEdges = { EdgesOfCube.GH, EdgesOfCube.HE, EdgesOfCube.EF, EdgesOfCube.FG,
                                            EdgesOfCube.CD, EdgesOfCube.DA, EdgesOfCube.AB, EdgesOfCube.BC,
                                            EdgesOfCube.CG, EdgesOfCube.DH, EdgesOfCube.AE, EdgesOfCube.BF};
        
        public DepthMasterControl depthMasterControl;
        public RenderViewFunctionalities renderViewFunctionalities;
        public TextBox informationTextBlock;
        public TextBox cubeInformationTextBox;

        public const int RealsenseWidth = 848;
        public const int RealsenseHeight = 480;

        //public double cubeSize = .3;
        public double cubeSize = .64;

        public double maximumAllowableVertexHeightDifference = 0.29;
        public double maximumAngleAllowed = 42.5;
        //public double maximumAngleAllowed = 12.5;
        public double badAngle = 80;

        public List<string> vertices = new List<string>();
        public List<string> triangles = new List<string>();
        public double[] depthDataFromFile = new double[512*424];

        /// <summary>
        /// List of raw point cloud vertices.
        /// </summary>
        List<Point3D> pointCloudVertices = new List<Point3D>();

        /// <summary>
        /// A Dictionary with all the point cloud vertices partitioned into equally sized cubes.
        /// These vertices will be later used to estimate a plane polygon for each cube.
        /// </summary>
        public Dictionary<string, List<Point3D>> cubePointCloudVertices = new Dictionary<string, List<Point3D>>();

        /// <summary>
        /// A Dictionary of all vertices created by simplification from point cloud data.
        /// </summary>
        public Dictionary<int, Vertex> allCubeCrossSectionVerticees = new Dictionary<int, Vertex>();

        /// <summary>
        /// A Dictionary of all triangles created from cube planes.
        /// </summary>
        public Dictionary<int, Triangle> allTriangles = new Dictionary<int, Triangle>();

        /// <summary>
        /// A Dictionary of all cubes.
        /// </summary>
        public Dictionary<string, Cube> allCubes = new Dictionary<string, Cube>();
        
        /// <summary>
        /// A Dictionary of all triangles associated with a specific cube.
        /// </summary>
        public Dictionary<string, Dictionary<int, Triangle>> cubeTriangles =
            new Dictionary<string, Dictionary<int, Triangle>>();

        /// <summary>
        /// A Dictionary of all vertices associated with a specific cube.
        /// </summary>
        public Dictionary<string, Dictionary<int, Vertex>> cubeVertices =
            new Dictionary<string, Dictionary<int, Vertex>>();

        /// <summary>
        /// A Dictionary with all the neightbors of a specific cube.
        /// </summary>
        public Dictionary<string, Dictionary<ConnectOfCube, string>> cubeNeighbors = 
            new Dictionary<string, Dictionary<ConnectOfCube, string>>();

        /// <summary>
        /// A Dictionary with all the edges of a specific cube that are occupied by vertices.
        /// </summary>
        public Dictionary<string, Dictionary<EdgesOfCube, Vertex>> edgesOfCubeVerticesPresent =
           new Dictionary<string, Dictionary<EdgesOfCube, Vertex>>();

        public Dictionary<string, Cube> selectedCubes = new Dictionary<string, Cube>();
        public Dictionary<string, MeshGeometry3D> selectedCubesMeshes =
            new Dictionary<string, MeshGeometry3D>();
        public Dictionary<string, GeometryModel3D> selectedCubesModels =
            new Dictionary<string, GeometryModel3D>();

        DiffuseMaterial qDiffTransYellow =
                    new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(75, 255, 255, 0)));
        SpecularMaterial qSpecTransWhite =
                    new SpecularMaterial(new SolidColorBrush(Color.FromArgb(75, 255, 255, 0)), 30.0);
        MaterialGroup qOuterMaterial = new MaterialGroup();

        public double acceptableDistance = (.75 * 30) * (.75 * 30);
        //public double acceptableDistance = 1.55;

        public Dictionary<int, Dictionary<int, int>> indexedPoints =
               new Dictionary<int, Dictionary<int, int>>();

        public Dictionary<int, Triangle> pointCloudTriangleList = new Dictionary<int, Triangle>();
        public Dictionary<int, Vector3D> pointCloudTriangleNormalVectors = new Dictionary<int, Vector3D>();
        public Dictionary<int, int[]> pointCloudTriangleListIndexed = new Dictionary<int, int[]>();

        public Dictionary<int, Point3D> pointCloudIndexed = new Dictionary<int, Point3D>();
        public Dictionary<string, List<Vector3D>> cubeNormalVectorActual = new Dictionary<string, List<Vector3D>>();



        public void getRayCastedCube(RayMeshGeometry3DHitTestResult meshResult)
        {
            MeshGeometry3D selectedCubesMesh = new MeshGeometry3D();
            MeshGeometry3D meshe = new MeshGeometry3D();

            // Get the position of the hit and the delimiter address of the potential cube:
            Point3D hitPosition = meshResult.PointHit;
            Point3D hitPositionGrid = new Point3D(hitPosition.X / cubeSize,
                hitPosition.Y / cubeSize, hitPosition.Z / cubeSize);

            string currentCubeId = getCubeAddressByPoint(hitPosition);
            double X_gridLimitFloor = 0.0;
            double X_gridLimitCeiling = 0.0;
            double Y_gridLimitFloor = 0.0;
            double Y_gridLimitCeiling = 0.0;
            double Z_gridLimitFloor = 0.0;
            double Z_gridLimitCeiling = 0.0;

            informationTextBlock.Text += "keke: " + currentCubeId + "\n";

            if (selectedCubesMeshes.ContainsValue(meshResult.MeshHit))
            {
                // Need to remove the highlight cube:
                foreach (string cubeIdToRemove in selectedCubesMeshes.Keys)
                {
                    if (selectedCubesMeshes[cubeIdToRemove].Equals(meshResult.MeshHit))
                    {
                        // Remove the cube:
                        renderViewFunctionalities.MainModel3Dgroup.Children.Remove(selectedCubesModels[cubeIdToRemove]);
                        selectedCubes.Remove(cubeIdToRemove);
                        selectedCubesMeshes.Remove(cubeIdToRemove);
                        selectedCubesModels.Remove(cubeIdToRemove);

                        break;
                    }
                }

                // Refresh the Cube information Text Box:
                cubeInformationTextBox.Clear();
                foreach(string cubeIdToPrint in selectedCubes.Keys)
                {
                    printCubeInformation(selectedCubes[cubeIdToPrint]);
                }
            } else
            {
                if (allCubes.ContainsKey(currentCubeId))
                {
                    Cube cubeToHandle = allCubes[currentCubeId];

                    if (cubeToHandle.hasPlane)
                    {
                        if (!selectedCubes.ContainsKey(currentCubeId))
                        {
                            selectedCubes.Add(currentCubeId, cubeToHandle);

                            foreach (Point3D pointCloud in cubePointCloudVertices[cubeToHandle.cubeId])
                            {
                               renderViewFunctionalities.renderSingleVertex(meshe, pointCloud);
                            }

                            DiffuseMaterial surface_material1 = new DiffuseMaterial(Brushes.Black);
                            GeometryModel3D surface_model1 = new GeometryModel3D(meshe, surface_material1);
                            surface_model1.BackMaterial = surface_material1;
                            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surface_model1);

                            printCubeInformation(cubeToHandle);

                            X_gridLimitFloor = cubeToHandle.xFloor * cubeSize;
                            X_gridLimitCeiling = cubeToHandle.xCeiling * cubeSize;
                            Y_gridLimitFloor = cubeToHandle.yFloor * cubeSize;
                            Y_gridLimitCeiling = cubeToHandle.yCeiling * cubeSize;
                            Z_gridLimitFloor = cubeToHandle.zFloor * cubeSize;
                            Z_gridLimitCeiling = cubeToHandle.zCeiling * cubeSize;

                            renderViewFunctionalities.renderSingleTransparentCube(selectedCubesMesh,
                                 new Point3D(X_gridLimitFloor + ((X_gridLimitCeiling - X_gridLimitFloor) / 2),
                                             Y_gridLimitFloor + ((Y_gridLimitCeiling - Y_gridLimitFloor) / 2),
                                             Z_gridLimitFloor + ((Z_gridLimitCeiling - Z_gridLimitFloor) / 2)),
                                cubeSize);
                            
                            GeometryModel3D surface_model_cuboid = new GeometryModel3D(selectedCubesMesh, qOuterMaterial);
                            // Make the surface visible from both sides.
                            surface_model_cuboid.BackMaterial = qOuterMaterial;

                            selectedCubesModels.Add(currentCubeId, surface_model_cuboid);
                            selectedCubesMeshes.Add(currentCubeId, selectedCubesMesh);

                            // Add the model to the model groups.
                            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surface_model_cuboid);
                        }
                    }
                }
            }
        }

        public string getCubeAddressByPoint(Point3D pointToHandle)
        {
            double X_divided = 0.0;
            double Y_divided = 0.0;
            double Z_divided = 0.0;
            double X = 0.0;
            double Y = 0.0;
            double Z = 0.0;
            double X_gridLimitFloor = 0.0;
            double X_gridLimitCeiling = 0.0;
            double Y_gridLimitFloor = 0.0;
            double Y_gridLimitCeiling = 0.0;
            double Z_gridLimitFloor = 0.0;
            double Z_gridLimitCeiling = 0.0;
            string[] gridLimitsStr = new string[6];
            string gridLimitsStrWholes = "";

            X = pointToHandle.X;
            Y = pointToHandle.Y;
            Z = pointToHandle.Z;

            X_divided = X / cubeSize;
            Y_divided = Y / cubeSize;
            Z_divided = Z / cubeSize;

            // Get the cube delimeters of this particular vertex:
            X_gridLimitFloor = Math.Floor(X_divided);
            X_gridLimitCeiling = Math.Ceiling(X_divided);
            Y_gridLimitFloor = Math.Floor(Y_divided);
            Y_gridLimitCeiling = Math.Ceiling(Y_divided);
            Z_gridLimitFloor = Math.Floor(Z_divided);
            Z_gridLimitCeiling = Math.Ceiling(Z_divided);

            gridLimitsStr = new string[6];
            gridLimitsStr[0] = X_gridLimitFloor.ToString();
            gridLimitsStr[2] = Y_gridLimitFloor.ToString();
            gridLimitsStr[4] = Z_gridLimitFloor.ToString();
            gridLimitsStr[1] = X_gridLimitCeiling.ToString();
            gridLimitsStr[3] = Y_gridLimitCeiling.ToString();
            gridLimitsStr[5] = Z_gridLimitCeiling.ToString();

            // Create the key for the cube:
            gridLimitsStrWholes = gridLimitsStr[0] + "/" + gridLimitsStr[1] + "," + gridLimitsStr[2]
                    + "/" + gridLimitsStr[3] + "," + gridLimitsStr[4] + "/" + gridLimitsStr[5];

            return gridLimitsStrWholes;
        }

        public void printCubeInformation(Cube cubeToHandle)
        {

            cubeInformationTextBox.Text += "************************" + "\n";
            cubeInformationTextBox.Text += "Cube Id: " + cubeToHandle.cubeId + "\n";
            cubeInformationTextBox.Text += "**********" + "\n";

            if(cubeToHandle.hasPlane)
            {
                cubeInformationTextBox.Text += "Cube X Floor: " + (cubeToHandle.xFloor * cubeSize).ToString() + "\n";
                cubeInformationTextBox.Text += "Cube X Ceiling: " + (cubeToHandle.xCeiling * cubeSize).ToString() + "\n";
                cubeInformationTextBox.Text += "Cube Y Floor: " + (cubeToHandle.yFloor * cubeSize).ToString() + "\n";
                cubeInformationTextBox.Text += "Cube Y Ceiling: " + (cubeToHandle.yCeiling * cubeSize).ToString() + "\n";
                cubeInformationTextBox.Text += "Cube Z Floor: " + (cubeToHandle.zFloor * cubeSize).ToString() + "\n";
                cubeInformationTextBox.Text += "Cube Z Ceiling: " + (cubeToHandle.zCeiling * cubeSize).ToString() + "\n";

                cubeInformationTextBox.Text += "**********" + "\n";
                cubeInformationTextBox.Text += "Cube Normal X: " + cubeToHandle.cubeNormalVector.X.ToString("n4") + "\n";
                cubeInformationTextBox.Text += "Cube Normal Y: " + cubeToHandle.cubeNormalVector.Y.ToString("n4") + "\n";
                cubeInformationTextBox.Text += "Cube Normal Z: " + cubeToHandle.cubeNormalVector.Z.ToString("n4") + "\n";

                cubeInformationTextBox.Text += "**********" + "\n";
                cubeInformationTextBox.Text += "Cube Plane Equation Constant: " + cubeToHandle.planeEquationConstant.ToString("n4") + "\n";
                
                cubeInformationTextBox.Text += "Cube Plane Normal X: " + cubeToHandle.planeEquationNormalVector.X.ToString("n4") + "\n";
                cubeInformationTextBox.Text += "Cube Plane Normal Y: " + cubeToHandle.planeEquationNormalVector.Y.ToString("n4") + "\n";
                cubeInformationTextBox.Text += "Cube Plane Normal Z: " + cubeToHandle.planeEquationNormalVector.Z.ToString("n4") + "\n";

                cubeInformationTextBox.Text += "**********" + "\n";
                cubeInformationTextBox.Text += "Cube Neighbors: \n";
                foreach (ConnectOfCube neighborConnection in cubeToHandle.neighborsConnectionType.Keys)
                {
                    cubeInformationTextBox.Text += "Neighbor Connection: " + neighborConnection.ToString() + "; NeighborId: "
                        + cubeToHandle.neighborsConnectionType[neighborConnection] + "\n";
                }

                cubeInformationTextBox.Text += "**********" + "\n";
                if (cubeToHandle.planeTrianglePoints != null)
                {
                    cubeInformationTextBox.Text += "Cube Plane Triangle Points: \n";
                    foreach (Point3D planePoint in cubeToHandle.planeTrianglePoints)
                    {
                        cubeInformationTextBox.Text += "(" + planePoint.X.ToString("n6") + ", "
                                + planePoint.Y.ToString("n6") + ", " + planePoint.Z.ToString("n6") + ")\n";
                    }
                }
                
                cubeInformationTextBox.Text += "**********" + "\n";
                cubeInformationTextBox.Text += "Cube Merges: ";

                int i = 0;
                foreach(bool edgeMerge in cubeToHandle.haveEdgesMerged)
                {
                    if(edgeMerge)
                    {
                        cubeInformationTextBox.Text += edgesOfCubeListing[i].ToString() + " ";
                    }
                    i++;
                }

                cubeInformationTextBox.Text += "\n";
                cubeInformationTextBox.Text += "**********" + "\n";
                cubeInformationTextBox.Text += "Cube Average Normal X: " + cubeToHandle.cubeNormalVector.X.ToString("n4") + "\n";
                cubeInformationTextBox.Text += "Cube Average Normal Y: " + cubeToHandle.cubeNormalVector.Y.ToString("n4") + "\n";
                cubeInformationTextBox.Text += "Cube Average Normal Z: " + cubeToHandle.cubeNormalVector.Z.ToString("n4") + "\n";

                cubeInformationTextBox.Text += "**********" + "\n";

                if(cubeToHandle.crossSectionVerticeesByEdge != null)
                {
                    cubeInformationTextBox.Text += "Cube Vertices: \n";
                    foreach (EdgesOfCube vertexEdge in cubeToHandle.crossSectionVerticeesByEdge.Keys)
                    {
                        cubeInformationTextBox.Text += "Vertex Edge: " + vertexEdge.ToString()
                             + "; " + cubeToHandle.crossSectionVerticeesByEdge[vertexEdge].ToString() + "\n";
                    }
                }

                

                cubeInformationTextBox.Text += "**********" + "\n";
                cubeInformationTextBox.Text += "Cube Triangles: \n";
                foreach (int triangleId in cubeToHandle.triangles.Keys)
                {
                    cubeInformationTextBox.Text += cubeToHandle.triangles[triangleId].ToString() + "\n";
                }

               

                cubeInformationTextBox.Text += "**********" + "\n";
                cubeInformationTextBox.Text += "Cube Point Cloud Vertices: \n";
                List<Point3D> pointCloudVertices = cubePointCloudVertices[cubeToHandle.cubeId];
                foreach (Point3D point in pointCloudVertices)
                {
                    cubeInformationTextBox.Text += "(" + point.X.ToString("n6") + ", "
                        + point.Y.ToString("n6") + ", " + point.Z.ToString("n6") + ")\n";
                }
            }
            else
            {
                cubeInformationTextBox.Text += "Cube does not have a plane." + "\n";
            }
        }


        public void findCubeCrossSectionsAndUpdateDataStructures(string cubeId, ref int newVertexId, double[] cubeLimit,
            double xCoeff, double yCoeff, double zCoeff, double constant)
        {
            double x, y, z;
            double[] cubeLimits = new double[6];
            double xLow = cubeLimit[0] * cubeSize;
            double xHigh = cubeLimit[1] * cubeSize;
            double yLow = cubeLimit[2] * cubeSize;
            double yHigh = cubeLimit[3] * cubeSize;
            double zLow = cubeLimit[4] * cubeSize;
            double zHigh = cubeLimit[5] * cubeSize;
            Point3D intersectedPoint;
            Vertex intersectedPointVertex;

            Cube cubeToHandle = allCubes[cubeId];

            Dictionary<int, Vertex> verticesOfCube = new Dictionary<int, Vertex>();
            Dictionary<EdgesOfCube, Vertex> edgesOfCube = new Dictionary<EdgesOfCube, Vertex>();
            
            // Side AB: z axis
            x = xLow; y = yHigh; z = (constant - (x * xCoeff) - (y * yCoeff)) / zCoeff;
            if (z.CompareTo(zLow) >= 0 && z.CompareTo(zHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.AB, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side BC: x axis
            z = zLow; y = yHigh; x = (constant - (z * zCoeff) - (y * yCoeff)) / xCoeff;
            if (x.CompareTo(xLow) >= 0 && x.CompareTo(xHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.BC, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side CD: z axis
            x = xHigh; y = yHigh; z = (constant - (x * xCoeff) - (y * yCoeff)) / zCoeff;
            if (z.CompareTo(zLow) >= 0 && z.CompareTo(zHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.CD, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side DA: x axis
            z = zHigh; y = yHigh; x = (constant - (z * zCoeff) - (y * yCoeff)) / xCoeff;
            if (x.CompareTo(xLow) >= 0 && x.CompareTo(xHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.DA, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }


            // Side EF: z axis
            x = xLow; y = yLow; z = (constant - (x * xCoeff) - (y * yCoeff)) / zCoeff;
            if (z.CompareTo(zLow) >= 0 && z.CompareTo(zHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.EF, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side FG: x axis
            z = zLow; y = yLow; x = (constant - (z * zCoeff) - (y * yCoeff)) / xCoeff;
            if (x.CompareTo(xLow) >= 0 && x.CompareTo(xHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.FG, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side GH: z axis
            x = xHigh; y = yLow; z = (constant - (x * xCoeff) - (y * yCoeff)) / zCoeff;
            if (z.CompareTo(zLow) >= 0 && z.CompareTo(zHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.GH, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side HE: x axis
            z = zHigh; y = yLow; x = (constant - (z * zCoeff) - (y * yCoeff)) / xCoeff;
            if (x.CompareTo(xLow) >= 0 && x.CompareTo(xHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.HE, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }


            // Side AE: y axis
            x = xLow; z = zHigh; y = (constant - (z * zCoeff) - (x * xCoeff)) / yCoeff;
            if (y.CompareTo(yLow) >= 0 && y.CompareTo(yHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.AE, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side BF: y axis
            x = xLow; z = zLow; y = (constant - (z * zCoeff) - (x * xCoeff)) / yCoeff;
            if (y.CompareTo(yLow) >= 0 && y.CompareTo(yHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.BF, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side CG: y axis
            x = xHigh; z = zLow; y = (constant - (z * zCoeff) - (x * xCoeff)) / yCoeff;
            if (y.CompareTo(yLow) >= 0 && y.CompareTo(yHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.CG, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            // Side DH: y axis
            x = xHigh; z = zHigh; y = (constant - (z * zCoeff) - (x * xCoeff)) / yCoeff;
            if (y.CompareTo(yLow) >= 0 && y.CompareTo(yHigh) <= 0)
            {
                intersectedPoint = new Point3D(x, y, z);
                intersectedPointVertex = new Vertex(newVertexId, intersectedPoint);

                allCubeCrossSectionVerticees.Add(newVertexId, intersectedPointVertex);
                edgesOfCube.Add(EdgesOfCube.DH, intersectedPointVertex);
                verticesOfCube.Add(newVertexId++, intersectedPointVertex);
            }
            
            // Update global data structures:
            edgesOfCubeVerticesPresent.Add(cubeId, edgesOfCube);
            cubeVertices.Add(cubeId, verticesOfCube);
            
            // Update the cube data structures:
            cubeToHandle.cubeCrossSectionVertices = verticesOfCube;
            cubeToHandle.crossSectionVerticeesByEdge = edgesOfCube;
            cubeToHandle.hasPlane = true;
        }

        /* Needs to be optimized */
        public string[,,] findAllNeighborKeys3DArray(Cube cubeToHandle, int neighborhoodSize)
        {
            double X_gridLimitFloor = 0;
            double X_gridLimitCeiling = 0;
            double Y_gridLimitFloor = 0;
            double Y_gridLimitCeiling = 0;
            double Z_gridLimitFloor = 0;
            double Z_gridLimitCeiling = 0;

            double X_gridLimitFloorCurr = 0;
            double X_gridLimitCeilingCurr = 0;
            double Y_gridLimitFloorCurr = 0;
            double Y_gridLimitCeilingCurr = 0;
            double Z_gridLimitFloorCurr = 0;
            double Z_gridLimitCeilingCurr = 0;

            string[,,] allNeighborKeys = new string[neighborhoodSize, neighborhoodSize, neighborhoodSize];
            
            int i, j, k;
            int midpoint = neighborhoodSize / 2;

            int offsetStarting = (neighborhoodSize / 2) * -1;
            int[] offsets = new int[neighborhoodSize];

            for (i = 0; i < neighborhoodSize; i++)
            {
                offsets[i] = offsetStarting;
                offsetStarting++;
            }

            X_gridLimitFloorCurr = cubeToHandle.xFloor;
            X_gridLimitCeilingCurr = cubeToHandle.xCeiling;
            
            Y_gridLimitFloorCurr = cubeToHandle.yFloor;
            Y_gridLimitCeilingCurr = cubeToHandle.yCeiling;

            Z_gridLimitFloorCurr = cubeToHandle.zFloor;
            Z_gridLimitCeilingCurr = cubeToHandle.zCeiling;

            for (i = 0; i < neighborhoodSize; i++)
            {
                X_gridLimitFloor = X_gridLimitFloorCurr + offsets[i];
                X_gridLimitCeiling = X_gridLimitCeilingCurr + offsets[i];

                for (j = 0; j < neighborhoodSize; j++)
                {
                    Y_gridLimitFloor = Y_gridLimitFloorCurr + offsets[j];
                    Y_gridLimitCeiling = Y_gridLimitCeilingCurr + offsets[j];

                    for (k = 0; k < neighborhoodSize; k++)
                    {
                        Z_gridLimitFloor = Z_gridLimitFloorCurr + offsets[k];
                        Z_gridLimitCeiling = Z_gridLimitCeilingCurr + offsets[k];

                        allNeighborKeys[i, j, k] = X_gridLimitFloor + "/" + X_gridLimitCeiling + "," +
                            Y_gridLimitFloor + "/" + Y_gridLimitCeiling + "," +
                            Z_gridLimitFloor + "/" + Z_gridLimitCeiling;
                    }
                }
            }

            return allNeighborKeys;
        }



        public string[] findAllNeighborKeys(string key, int neighborhoodSize)
        {
            double X_gridLimitFloor = 0;
            double X_gridLimitCeiling = 0;
            double Y_gridLimitFloor = 0;
            double Y_gridLimitCeiling = 0;
            double Z_gridLimitFloor = 0;
            double Z_gridLimitCeiling = 0;

            double X_gridLimitFloorCurr = 0;
            double X_gridLimitCeilingCurr = 0;
            double Y_gridLimitFloorCurr = 0;
            double Y_gridLimitCeilingCurr = 0;
            double Z_gridLimitFloorCurr = 0;
            double Z_gridLimitCeilingCurr = 0;

            string[] allNeighborKeys = new string[neighborhoodSize * neighborhoodSize * neighborhoodSize - 1];

            string[] pairSeperatedKey;
            string[] rangeSeperatedKey;

            int i, j, k, strIndex = 0;
            int midpoint = neighborhoodSize / 2;

            int offsetStarting = (neighborhoodSize / 2) * -1;
            int[] offsets = new int[neighborhoodSize];
            for (i = 0; i < neighborhoodSize; i++)
            {
                offsets[i] = offsetStarting;
                offsetStarting++;
            }

            pairSeperatedKey = key.Split(',');
            rangeSeperatedKey = pairSeperatedKey[0].Split('/');
            X_gridLimitFloorCurr = Double.Parse(rangeSeperatedKey[0]);
            X_gridLimitCeilingCurr = Double.Parse(rangeSeperatedKey[1]);

            rangeSeperatedKey = pairSeperatedKey[1].Split('/');
            Y_gridLimitFloorCurr = Double.Parse(rangeSeperatedKey[0]);
            Y_gridLimitCeilingCurr = Double.Parse(rangeSeperatedKey[1]);

            rangeSeperatedKey = pairSeperatedKey[2].Split('/');
            Z_gridLimitFloorCurr = Double.Parse(rangeSeperatedKey[0]);
            Z_gridLimitCeilingCurr = Double.Parse(rangeSeperatedKey[1]);

            for (i = 0; i < neighborhoodSize; i++)
            {
                for (j = 0; j < neighborhoodSize; j++)
                {
                    for (k = 0; k < neighborhoodSize; k++)
                    {
                        if (!(i == midpoint && j == midpoint && k == midpoint))
                        {
                            X_gridLimitFloor = X_gridLimitFloorCurr + offsets[i];
                            X_gridLimitCeiling = X_gridLimitCeilingCurr + offsets[i];

                            Y_gridLimitFloor = Y_gridLimitFloorCurr + offsets[j];
                            Y_gridLimitCeiling = Y_gridLimitCeilingCurr + offsets[j];

                            Z_gridLimitFloor = Z_gridLimitFloorCurr + offsets[k];
                            Z_gridLimitCeiling = Z_gridLimitCeilingCurr + offsets[k];

                            allNeighborKeys[strIndex] = X_gridLimitFloor + "/" + X_gridLimitCeiling + "," +
                                Y_gridLimitFloor + "/" + Y_gridLimitCeiling + "," +
                                Z_gridLimitFloor + "/" + Z_gridLimitCeiling;
                            strIndex++;
                        }
                    }
                }
            }

            return allNeighborKeys;
        }

        

        public void savePointCloud()
        {
            //depth_master_control.pointCloudArray
            /*
            string path = "D:/Solar Project/XBOX ONE/MESA 3D/3DSandbox/3DSandbox/Text Files/saved_point_cloud.txt";
            string strToWrite = "";
            int limit = depthMasterControl.pointCloudArrayDouble.Length;
            int k = 0;
            Byte[] info;
            double X = 0.0;
            double Y = 0.0;
            double Z = 0.0;
            double X_gridLimitFloor = 0.0;
            double X_gridLimitCeiling = 0.0;
            double Y_gridLimitFloor = 0.0;
            double Y_gridLimitCeiling = 0.0;
            double Z_gridLimitFloor = 0.0;
            double Z_gridLimitCeiling = 0.0;
            string[] gridLimitsStr = new string[6];
            string gridLimitsStrWholes = "";

            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();

            using (FileStream fs = File.Open(path, FileMode.Open, FileAccess.Write, FileShare.None))
            {
                for (int i = 0; i < limit; i = i + 3)
                {
                    if (depthMasterControl.pointCloudArray[i] != 0 && depthMasterControl.pointCloudArray[i + 1] != 0
                        && depthMasterControl.pointCloudArray[i + 2] != 0)
                    {

                        X = depthMasterControl.pointCloudArrayDouble[i + k++] / 8;
                        Y = depthMasterControl.pointCloudArrayDouble[i + k++] / 8;
                        Z = depthMasterControl.pointCloudArrayDouble[i + k++] / 8;
                        k = 0;

                        X_gridLimitFloor = Math.Floor(X);
                        Y_gridLimitFloor = Math.Floor(Y);
                        Z_gridLimitFloor = Math.Floor(Z);
                        X_gridLimitCeiling = Math.Ceiling(X);
                        Y_gridLimitCeiling = Math.Ceiling(Y);
                        Z_gridLimitCeiling = Math.Ceiling(Z);

                        gridLimitsStr = new string[6];
                        gridLimitsStr[0] = Math.Floor(X).ToString();
                        gridLimitsStr[2] = Math.Floor(Y).ToString();
                        gridLimitsStr[4] = Math.Floor(Z).ToString();
                        gridLimitsStr[1] = Math.Ceiling(X).ToString();
                        gridLimitsStr[3] = Math.Ceiling(Y).ToString();
                        gridLimitsStr[5] = Math.Ceiling(Z).ToString();

                        gridLimitsStrWholes = gridLimitsStr[0] + "/" + gridLimitsStr[1] + "," + gridLimitsStr[2]
                            + "/" + gridLimitsStr[3] + "," + gridLimitsStr[4] + "/" + gridLimitsStr[5];

                        if (verticesGrid.ContainsKey(gridLimitsStrWholes))
                        {
                            containedGridVertices = verticesGrid[gridLimitsStrWholes];
                            containedGridVertices.Add(i, new Point3D(X * 8, Y * 8, Z * 8));
                        }
                        else
                        {
                            containedGridVertices = new Dictionary<int, Point3D>();
                            containedGridVertices.Add(i, new Point3D(X * 8, Y * 8, Z * 8));
                            verticesGrid.Add(gridLimitsStrWholes, containedGridVertices);
                        }

                        strToWrite = (depthMasterControl.pointCloudArrayDouble[i + k++]) + " " +
                                     (depthMasterControl.pointCloudArrayDouble[i + k++]) + " " +
                                     (depthMasterControl.pointCloudArrayDouble[i + k++]) + "\n";
                        k = 0;
                        info = new UTF8Encoding(true).GetBytes(strToWrite);
                        fs.Write(info, 0, info.Length);
                    }
                }
            }

            int maxVerticesPerCube = 0;

            path = "D:/Solar Project/XBOX ONE/MESA 3D/3DSandbox/3DSandbox/Text Files/saved_point_cloud2.txt";
            using (FileStream fs = File.Open(path, FileMode.Open, FileAccess.Write, FileShare.None))
            {
                var keys = verticesGrid.Keys;

                foreach (string key in keys)
                {
                    containedGridVertices = verticesGrid[key];
                    var containedGridVerticesKeys = containedGridVertices.Keys;

                    int j = containedGridVerticesKeys.Count;

                    foreach (int vertex in containedGridVerticesKeys)
                    {
                        Point3D point = containedGridVertices[vertex];
                        strToWrite = "[" + key + "]";
                        strToWrite += "  :  v = " + vertex + " pos:";
                        strToWrite += point.ToString() + "\n";
                        info = new UTF8Encoding(true).GetBytes(strToWrite);
                        fs.Write(info, 0, info.Length);
                    }

                    if (j > maxVerticesPerCube)
                    {
                        maxVerticesPerCube = j;
                    }
                }

                info = new UTF8Encoding(true).GetBytes("Max: " + maxVerticesPerCube + "\n");
                fs.Write(info, 0, info.Length);

                int count = 0;
                int i = 0;
                keys = verticesGrid.Keys;

                for (i = 0; i < maxVerticesPerCube; i++)
                {
                    count = 0;
                    foreach (string key in keys)
                    {
                        containedGridVertices = verticesGrid[key];
                        var containedGridVerticesKeys = containedGridVertices.Keys;
                        if ((i + 1) == containedGridVerticesKeys.Count)
                        {
                            count++;
                        }
                    }

                    info = new UTF8Encoding(true).GetBytes("Number of Verticees: " + (i + 1) + " : " + count + "\n");
                    fs.Write(info, 0, info.Length);
                }

                int numOfLoneVerticees = verticesGrid.Keys.Count;
                int numOfLoneCubes = verticesGrid.Keys.Count;

                string[] neighborStr;

                
                foreach (string key in keys)
                {
                    containedGridVertices = verticesGrid[key];
                    var containedGridVerticesKeys = containedGridVertices.Keys;
                    if (containedGridVerticesKeys.Count == 1)
                    {
                        neighborStr = findAllNeighborKeys(key, 3);
                        foreach(string singleNeighbor in neighborStr)
                        {
                            if(verticesGrid.ContainsKey(singleNeighbor))
                            {
                                numOfLoneVerticees--;
                                break;
                            }
                        }
                    }
                    
                    if (containedGridVerticesKeys.Count > 0)
                    {
                        neighborStr = findAllNeighborKeys(key, 7);
                        foreach (string singleNeighbor in neighborStr)
                        {
                            if (verticesGrid.ContainsKey(singleNeighbor))
                            {
                                numOfLoneCubes--;
                                break;
                            }
                        }
                    }
                }
                

                info = new UTF8Encoding(true).GetBytes("Number of Lone Vertices" + " : " + numOfLoneVerticees + "\n");
                fs.Write(info, 0, info.Length);
                info = new UTF8Encoding(true).GetBytes("Number of Lone Cubes" + " : " + numOfLoneCubes + "\n");
                fs.Write(info, 0, info.Length);

            }
            */
        }


        /// <summary>
        /// This function renders a point cloud as small, black cloud of 3D cubes. WPF apparantly does not have support for 
        /// rendering points.
        /// </summary>
        public void renderPointCloud()
        {
            MeshGeometry3D mesh1 = new MeshGeometry3D();
            DiffuseMaterial surface_material1 = new DiffuseMaterial(Brushes.Black);

            Cube cubeToHandle;
            Triangle triangleToHandle;
            List<Point3D> pointCloudOfCube;

            foreach (string cubeId in allCubes.Keys)
            {
                cubeToHandle = allCubes[cubeId];
                pointCloudOfCube = cubePointCloudVertices[cubeId];

                foreach (Point3D vertexPoint in pointCloudOfCube)
                {
                    renderViewFunctionalities.renderSingleVertex(mesh1, vertexPoint);
                }
            }

            GeometryModel3D surface_model1 = new GeometryModel3D(mesh1, surface_material1);

            // Make the surface visible from both sides.
            surface_model1.BackMaterial = surface_material1;

            // Add the model to the model groups.
            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surface_model1);
        }
        
        /// <summary>
        /// This method renders the cube planes, marking them as accessible or not.
        /// </summary>
        public void renderTrianglePlanes()
        {
            MeshGeometry3D mesh1 = new MeshGeometry3D();
            MeshGeometry3D mesh2 = new MeshGeometry3D();
            MeshGeometry3D mesh3 = new MeshGeometry3D();
            MeshGeometry3D mesh4 = new MeshGeometry3D();
            MeshGeometry3D meshRegionOutline = new MeshGeometry3D();

            DiffuseMaterial surface_material1 = new DiffuseMaterial(Brushes.DarkOrange);
            DiffuseMaterial surface_material2 = new DiffuseMaterial(Brushes.Olive);
            DiffuseMaterial surface_material3 = new DiffuseMaterial(Brushes.Blue);
            DiffuseMaterial surface_materialRegionOutline = new DiffuseMaterial(Brushes.DarkViolet);

            DiffuseMaterial qDiffTransYellow2 =
                   new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(75, 100, 255, 100)));
            SpecularMaterial qSpecTransWhite2 =
                        new SpecularMaterial(new SolidColorBrush(Color.FromArgb(75, 100, 100, 0)), 30.0);
            SpecularMaterial qSpecTransViolet =
                        new SpecularMaterial(new SolidColorBrush(Color.FromArgb(75, 121, 38, 165)), 30.0);
            MaterialGroup qOuterMaterial2 = new MaterialGroup();

            Cube cubeToHandle;
            Triangle triangleToHandle;

            informationTextBlock.Text += "Cube Count: " + allCubes.Count.ToString() + "\n";
            int accessibleCount = 0;
            int unaccessibleCount = 0;

            foreach (string cubeId in allCubes.Keys)
            {
                cubeToHandle = allCubes[cubeId];

                if(cubeToHandle.hasPlane)
                {
                    foreach (int triangleId in cubeToHandle.triangles.Keys)
                    {
                        triangleToHandle = cubeToHandle.triangles[triangleId];
                        
                        if (cubeToHandle.useType == CubeUseType.REGION_OUTLINE)
                        {
                            accessibleCount++;
                            renderViewFunctionalities.AddTriangle(meshRegionOutline, triangleToHandle.vertex1.vertexPosition,
                                triangleToHandle.vertex2.vertexPosition,
                                triangleToHandle.vertex3.vertexPosition);
                        } else if(cubeToHandle.useType == CubeUseType.REGION_NONE)
                        {
                            unaccessibleCount++;
                            renderViewFunctionalities.AddTriangle(mesh1, triangleToHandle.vertex1.vertexPosition,
                            triangleToHandle.vertex2.vertexPosition,
                            triangleToHandle.vertex3.vertexPosition);
                        } else 
                        {
                            accessibleCount++;
                            renderViewFunctionalities.AddTriangle(mesh2, triangleToHandle.vertex1.vertexPosition,
                                triangleToHandle.vertex2.vertexPosition,
                                triangleToHandle.vertex3.vertexPosition);
                        }
                    }
                } else
                {
                    // Render a transparent cube if a cube data structure does not exist at this position:
                    /*
                    renderViewFunctionalities.renderSingleTransparentCube(mesh4,
                                  new Point3D(cubeToHandle.xFloor * cubeSize + ((cubeToHandle.xCeiling * cubeSize - cubeToHandle.xFloor * cubeSize) / 2),
                                              cubeToHandle.yFloor * cubeSize + ((cubeToHandle.yCeiling * cubeSize - cubeToHandle.yFloor * cubeSize) / 2),
                                              cubeToHandle.zFloor * cubeSize + ((cubeToHandle.zCeiling * cubeSize - cubeToHandle.zFloor * cubeSize) / 2)),
                                 cubeSize);
                    */
                }
            }

            informationTextBlock.Text += "Accessible Area Count: " + accessibleCount + "\n";
            informationTextBlock.Text += "Unaccessible Area Count: " + unaccessibleCount + "\n";

            qOuterMaterial2.Children.Add(qDiffTransYellow2);
            qOuterMaterial2.Children.Add(qSpecTransWhite2);

            GeometryModel3D surface_model_cuboid = new GeometryModel3D(mesh4, qOuterMaterial2);
            // Make the surface visible from both sides.
            surface_model_cuboid.BackMaterial = qOuterMaterial2;

            // Add the model to the model groups.
            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surface_model_cuboid);

            GeometryModel3D surface_model1 = new GeometryModel3D(mesh1, surface_material1);
            GeometryModel3D surface_model2 = new GeometryModel3D(mesh2, surface_material2);
            GeometryModel3D surface_model3 = new GeometryModel3D(mesh3, surface_material3);
            GeometryModel3D surfaceRegionOutline = new GeometryModel3D(meshRegionOutline, surface_materialRegionOutline);


            // Make the surface visible from both sides.
            surface_model1.BackMaterial = surface_material1;
            surface_model2.BackMaterial = surface_material2;
            surface_model3.BackMaterial = surface_material3;
            surfaceRegionOutline.BackMaterial = surface_materialRegionOutline;

            // Add the model to the model groups.
            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surface_model1);
            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surface_model2);
            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surface_model3);
            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surfaceRegionOutline);
        }
        

        /// <summary>
        /// This function takes the verticees of a 3D model and divides them into cubes.
        /// </summary>
        public void getPointCloudOfExampleModel()
        {
            double X_divided = 0.0;
            double Y_divided = 0.0;
            double Z_divided = 0.0;
            double X = 0.0;
            double Y = 0.0;
            double Z = 0.0;
            double X_gridLimitFloor = 0.0;
            double X_gridLimitCeiling = 0.0;
            double Y_gridLimitFloor = 0.0;
            double Y_gridLimitCeiling = 0.0;
            double Z_gridLimitFloor = 0.0;
            double Z_gridLimitCeiling = 0.0;
            string[] gridLimitsStr = new string[6];
            string gridLimitsStrWholes = "";
            string[] vertexStrArr;
            
            Cube cubeToHandle;
            List<Point3D> singleCubeVertices;

            foreach (string vertexStr in vertices)
            {
                vertexStrArr = vertexStr.Split(' ');

                X = Double.Parse(vertexStrArr[0]);
                Y = Double.Parse(vertexStrArr[1]);
                Z = Double.Parse(vertexStrArr[2]);

                X_divided = X / cubeSize;
                Y_divided = Y / cubeSize;
                Z_divided = Z / cubeSize;

                // Get the cube delimeters of this particular vertex:
                X_gridLimitFloor = Math.Floor(X_divided);
                X_gridLimitCeiling = Math.Ceiling(X_divided);
                Y_gridLimitFloor = Math.Floor(Y_divided);
                Y_gridLimitCeiling = Math.Ceiling(Y_divided);
                Z_gridLimitFloor = Math.Floor(Z_divided);
                Z_gridLimitCeiling = Math.Ceiling(Z_divided);

                gridLimitsStr = new string[6];
                gridLimitsStr[0] = X_gridLimitFloor.ToString();
                gridLimitsStr[2] = Y_gridLimitFloor.ToString();
                gridLimitsStr[4] = Z_gridLimitFloor.ToString();
                gridLimitsStr[1] = X_gridLimitCeiling.ToString();
                gridLimitsStr[3] = Y_gridLimitCeiling.ToString();
                gridLimitsStr[5] = Z_gridLimitCeiling.ToString();

                // Create the key for the cube:
                gridLimitsStrWholes = gridLimitsStr[0] + "/" + gridLimitsStr[1] + "," + gridLimitsStr[2]
                        + "/" + gridLimitsStr[3] + "," + gridLimitsStr[4] + "/" + gridLimitsStr[5];

                if (cubePointCloudVertices.ContainsKey(gridLimitsStrWholes))
                {
                    singleCubeVertices = cubePointCloudVertices[gridLimitsStrWholes];
                    singleCubeVertices.Add(new Point3D(X,Y,Z));
                }
                else
                {
                    // Create new cube:
                    cubeToHandle = new Cube(gridLimitsStrWholes, X_gridLimitFloor, X_gridLimitCeiling,
                        Y_gridLimitFloor, Y_gridLimitCeiling, Z_gridLimitFloor, Z_gridLimitCeiling);

                    allCubes[gridLimitsStrWholes] = cubeToHandle;

                    singleCubeVertices = new List<Point3D>();
                    singleCubeVertices.Add(new Point3D(X, Y, Z));

                    // Associate a new list with a cube:
                    cubePointCloudVertices[gridLimitsStrWholes] = singleCubeVertices;
                }
            }
        }

        /// <summary>
        /// This function takes the point cloud mesh triangles, finds the midpoint of the triangle, and assigns that
        /// triangle to the cube if it's midpoint is inside it. The normals of the triangles are used
        /// for mathematical analysis of the cubes. Triangle is only added to one cube.
        /// </summary>
        public void generateCubesWithPointCloudTrianglesMerging()
        {
            double X_dividedMerge = 0.0;
            double Y_dividedMerge = 0.0;
            double Z_dividedMerge = 0.0;

            double X_gridLimitFloor = 0.0;
            double X_gridLimitCeiling = 0.0;
            double Y_gridLimitFloor = 0.0;
            double Y_gridLimitCeiling = 0.0;
            double Z_gridLimitFloor = 0.0;
            double Z_gridLimitCeiling = 0.0;

            Point3D pointMerge;

            string[] gridLimitsStr = new string[6];

            string gridLimitsStrWholes = "";

            Cube cubeToHandle;
            Triangle triangleAtIndex;
            Vector3D normalVectorOfTriangle;
            List<Point3D> singleCubeVertices;
            int h = 0;
            /*
            if (pointCloudVertices.Count == 0)
            {
                pointCloudVertices = depthMasterControl.savedPointCloudList;
            }
            */

            foreach (int triangleId in pointCloudTriangleListIndexed.Keys)
            {
                normalVectorOfTriangle = pointCloudTriangleNormalVectors[triangleId];

                triangleAtIndex = pointCloudTriangleList[triangleId];

                if (h < 150)
                {
                    //informationTextBlock.Text += "DDDD: " + normalVectorOfTriangle.ToString() + "\n";
                    h++;
                }

                pointMerge = new Point3D((triangleAtIndex.vertex1.vertexPosition.X + triangleAtIndex.vertex2.vertexPosition.X +
                    triangleAtIndex.vertex3.vertexPosition.X) / 3,
                    (triangleAtIndex.vertex1.vertexPosition.Y + triangleAtIndex.vertex2.vertexPosition.Y +
                    triangleAtIndex.vertex3.vertexPosition.Y) / 3,
                    (triangleAtIndex.vertex1.vertexPosition.Z + triangleAtIndex.vertex2.vertexPosition.Z + 
                    triangleAtIndex.vertex3.vertexPosition.Z) / 3);
                
                X_dividedMerge = pointMerge.X / cubeSize;
                Y_dividedMerge = pointMerge.Y / cubeSize;
                Z_dividedMerge = pointMerge.Z / cubeSize;

                // Get the cube delimeters of this particular vertex:
                X_gridLimitFloor = Math.Floor(X_dividedMerge);
                X_gridLimitCeiling = Math.Ceiling(X_dividedMerge);
                Y_gridLimitFloor = Math.Floor(Y_dividedMerge);
                Y_gridLimitCeiling = Math.Ceiling(Y_dividedMerge);
                Z_gridLimitFloor = Math.Floor(Z_dividedMerge);
                Z_gridLimitCeiling = Math.Ceiling(Z_dividedMerge);

                gridLimitsStr = new string[6];
                gridLimitsStr[0] = X_gridLimitFloor.ToString();
                gridLimitsStr[2] = Y_gridLimitFloor.ToString();
                gridLimitsStr[4] = Z_gridLimitFloor.ToString();
                gridLimitsStr[1] = X_gridLimitCeiling.ToString();
                gridLimitsStr[3] = Y_gridLimitCeiling.ToString();
                gridLimitsStr[5] = Z_gridLimitCeiling.ToString();

                // Create the key for the cube:
                gridLimitsStrWholes = gridLimitsStr[0] + "/" + gridLimitsStr[1] + "," + gridLimitsStr[2]
                        + "/" + gridLimitsStr[3] + "," + gridLimitsStr[4] + "/" + gridLimitsStr[5];

                if (cubePointCloudVertices.ContainsKey(gridLimitsStrWholes))
                {
                    // Adding to the existing list:
                    singleCubeVertices = cubePointCloudVertices[gridLimitsStrWholes];
                    singleCubeVertices.Add(pointMerge);

                    cubeToHandle = allCubes[gridLimitsStrWholes];

                    if (!cubeToHandle.triangleNormalVectors.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangleNormalVectors.Add(triangleId, normalVectorOfTriangle);
                    }

                    if (!cubeToHandle.triangleMergePoints.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangleMergePoints.Add(triangleId, pointMerge);
                    }

                    if (!cubeToHandle.triangles.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangles.Add(triangleId, triangleAtIndex);
                    }
                }
                else
                {
                    // Create new cube:
                    cubeToHandle = new Cube(gridLimitsStrWholes, X_gridLimitFloor, X_gridLimitCeiling,
                        Y_gridLimitFloor, Y_gridLimitCeiling, Z_gridLimitFloor, Z_gridLimitCeiling);

                    // Add necessary triangle information:
                    cubeToHandle.triangleNormalVectors = new Dictionary<int, Vector3D>();
                    cubeToHandle.triangleNormalVectors.Add(triangleId, normalVectorOfTriangle);
                    cubeToHandle.triangleMergePoints = new Dictionary<int, Point3D>();
                    cubeToHandle.triangleMergePoints.Add(triangleId, pointMerge);
                    cubeToHandle.triangles = new Dictionary<int, Triangle>();
                    cubeToHandle.triangles.Add(triangleId, triangleAtIndex);

                    // Assigning to the global list of cubes:
                    allCubes[gridLimitsStrWholes] = cubeToHandle;

                    singleCubeVertices = new List<Point3D>();
                    singleCubeVertices.Add(pointMerge);

                    // Associate a new list with a cube:
                    cubePointCloudVertices[gridLimitsStrWholes] = singleCubeVertices;
                }
            }
        }
        
        /// <summary>
        /// This function takes the point cloud mesh and uses the individual triangles to 
        /// create the individual cube structures. The normals of the triangles are used
        /// for mathematical analysis of the cubes. Triangle is added to multiple cubes.
        /// </summary>
        public void generateCubesWithPointCloudTrianglesIndividual()
        {
            double X_divided0 = 0.0;
            double Y_divided0 = 0.0;
            double Z_divided0 = 0.0;
            double X_divided1 = 0.0;
            double Y_divided1 = 0.0;
            double Z_divided1 = 0.0;
            double X_divided2 = 0.0;
            double Y_divided2 = 0.0;
            double Z_divided2 = 0.0;
            double X_gridLimitFloor0 = 0.0;
            double X_gridLimitCeiling0 = 0.0;
            double Y_gridLimitFloor0 = 0.0;
            double Y_gridLimitCeiling0 = 0.0;
            double Z_gridLimitFloor0 = 0.0;
            double Z_gridLimitCeiling0 = 0.0;
            double X_gridLimitFloor1 = 0.0;
            double X_gridLimitCeiling1 = 0.0;
            double Y_gridLimitFloor1 = 0.0;
            double Y_gridLimitCeiling1 = 0.0;
            double Z_gridLimitFloor1 = 0.0;
            double Z_gridLimitCeiling1 = 0.0;
            double X_gridLimitFloor2 = 0.0;
            double X_gridLimitCeiling2 = 0.0;
            double Y_gridLimitFloor2 = 0.0;
            double Y_gridLimitCeiling2 = 0.0;
            double Z_gridLimitFloor2 = 0.0;
            double Z_gridLimitCeiling2 = 0.0;

            Point3D point0, point1, point2;

            Triangle triangleAtIndex;

            string[] gridLimitsStr0 = new string[6];
            string[] gridLimitsStr1 = new string[6];
            string[] gridLimitsStr2 = new string[6];

            string gridLimitsStrWholes0 = "";
            string gridLimitsStrWholes1 = "";
            string gridLimitsStrWholes2 = "";
            
            Cube cubeToHandle;
            Vector3D normalVectorOfTriangle;
            List<Point3D> singleCubeVertices;

            if (pointCloudVertices.Count == 0)
            {
                pointCloudVertices = depthMasterControl.savedPointCloudList;
            }

            foreach (int triangleId in pointCloudTriangleListIndexed.Keys)
            {
                normalVectorOfTriangle = pointCloudTriangleNormalVectors[triangleId];
                triangleAtIndex = pointCloudTriangleList[triangleId];

                point0 = triangleAtIndex.vertex1.vertexPosition;
                point1 = triangleAtIndex.vertex2.vertexPosition;
                point2 = triangleAtIndex.vertex3.vertexPosition;

                X_divided0 = point0.X / cubeSize;
                Y_divided0 = point0.Y / cubeSize;
                Z_divided0 = point0.Z / cubeSize;
                X_divided1 = point1.X / cubeSize;
                Y_divided1 = point1.Y / cubeSize;
                Z_divided1 = point1.Z / cubeSize;
                X_divided2 = point2.X / cubeSize;
                Y_divided2 = point2.Y / cubeSize;
                Z_divided2 = point2.Z / cubeSize;
                
                // Get the cube delimeters of this particular vertex:
                X_gridLimitFloor0 = Math.Floor(X_divided0);
                X_gridLimitCeiling0 = Math.Ceiling(X_divided0);
                Y_gridLimitFloor0 = Math.Floor(Y_divided0);
                Y_gridLimitCeiling0 = Math.Ceiling(Y_divided0);
                Z_gridLimitFloor0 = Math.Floor(Z_divided0);
                Z_gridLimitCeiling0 = Math.Ceiling(Z_divided0);

                X_gridLimitFloor1 = Math.Floor(X_divided1);
                X_gridLimitCeiling1 = Math.Ceiling(X_divided1);
                Y_gridLimitFloor1 = Math.Floor(Y_divided1);
                Y_gridLimitCeiling1 = Math.Ceiling(Y_divided1);
                Z_gridLimitFloor1 = Math.Floor(Z_divided1);
                Z_gridLimitCeiling1 = Math.Ceiling(Z_divided1);

                X_gridLimitFloor2 = Math.Floor(X_divided2);
                X_gridLimitCeiling2 = Math.Ceiling(X_divided2);
                Y_gridLimitFloor2 = Math.Floor(Y_divided2);
                Y_gridLimitCeiling2 = Math.Ceiling(Y_divided2);
                Z_gridLimitFloor2 = Math.Floor(Z_divided2);
                Z_gridLimitCeiling2 = Math.Ceiling(Z_divided2);
                
                gridLimitsStr0 = new string[6];
                gridLimitsStr0[0] = X_gridLimitFloor0.ToString();
                gridLimitsStr0[2] = Y_gridLimitFloor0.ToString();
                gridLimitsStr0[4] = Z_gridLimitFloor0.ToString();
                gridLimitsStr0[1] = X_gridLimitCeiling0.ToString();
                gridLimitsStr0[3] = Y_gridLimitCeiling0.ToString();
                gridLimitsStr0[5] = Z_gridLimitCeiling0.ToString();

                gridLimitsStr1 = new string[6];
                gridLimitsStr1[0] = X_gridLimitFloor1.ToString();
                gridLimitsStr1[2] = Y_gridLimitFloor1.ToString();
                gridLimitsStr1[4] = Z_gridLimitFloor1.ToString();
                gridLimitsStr1[1] = X_gridLimitCeiling1.ToString();
                gridLimitsStr1[3] = Y_gridLimitCeiling1.ToString();
                gridLimitsStr1[5] = Z_gridLimitCeiling1.ToString();

                gridLimitsStr2 = new string[6];
                gridLimitsStr2[0] = X_gridLimitFloor2.ToString();
                gridLimitsStr2[2] = Y_gridLimitFloor2.ToString();
                gridLimitsStr2[4] = Z_gridLimitFloor2.ToString();
                gridLimitsStr2[1] = X_gridLimitCeiling2.ToString();
                gridLimitsStr2[3] = Y_gridLimitCeiling2.ToString();
                gridLimitsStr2[5] = Z_gridLimitCeiling2.ToString();

                // Create the key for the cube:
                gridLimitsStrWholes0 = gridLimitsStr0[0] + "/" + gridLimitsStr0[1] + "," + gridLimitsStr0[2]
                        + "/" + gridLimitsStr0[3] + "," + gridLimitsStr0[4] + "/" + gridLimitsStr0[5];

                gridLimitsStrWholes1 = gridLimitsStr1[0] + "/" + gridLimitsStr1[1] + "," + gridLimitsStr1[2]
                        + "/" + gridLimitsStr1[3] + "," + gridLimitsStr1[4] + "/" + gridLimitsStr1[5];

                gridLimitsStrWholes2 = gridLimitsStr2[0] + "/" + gridLimitsStr2[1] + "," + gridLimitsStr2[2]
                        + "/" + gridLimitsStr2[3] + "," + gridLimitsStr2[4] + "/" + gridLimitsStr2[5];
                
                if (cubePointCloudVertices.ContainsKey(gridLimitsStrWholes0))
                {
                    singleCubeVertices = cubePointCloudVertices[gridLimitsStrWholes0];
                    singleCubeVertices.Add(point0);

                    cubeToHandle = allCubes[gridLimitsStrWholes0];

                    if (!cubeToHandle.triangleNormalVectors.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangleNormalVectors.Add(triangleId, normalVectorOfTriangle);
                    }
                    
                    if (!cubeToHandle.triangleMergePoints.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangleMergePoints.Add(triangleId, point0);
                    }
                }
                else
                {
                    // Create new cube:
                    cubeToHandle = new Cube(gridLimitsStrWholes0, X_gridLimitFloor0, X_gridLimitCeiling0,
                        Y_gridLimitFloor0, Y_gridLimitCeiling0, Z_gridLimitFloor0, Z_gridLimitCeiling0);
                    
                    cubeToHandle.triangleNormalVectors = new Dictionary<int, Vector3D>();
                    cubeToHandle.triangleNormalVectors.Add(triangleId, normalVectorOfTriangle);
                    cubeToHandle.triangleMergePoints = new Dictionary<int, Point3D>();
                    cubeToHandle.triangleMergePoints.Add(triangleId, point0);

                    allCubes[gridLimitsStrWholes0] = cubeToHandle;

                    singleCubeVertices = new List<Point3D>();
                    singleCubeVertices.Add(point0);

                    // Associate a new list with a cube:
                    cubePointCloudVertices[gridLimitsStrWholes0] = singleCubeVertices;
                }


                if (cubePointCloudVertices.ContainsKey(gridLimitsStrWholes1))
                {
                    singleCubeVertices = cubePointCloudVertices[gridLimitsStrWholes1];
                    singleCubeVertices.Add(point1);

                    cubeToHandle = allCubes[gridLimitsStrWholes1];
                    if (!cubeToHandle.triangleNormalVectors.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangleNormalVectors.Add(triangleId, normalVectorOfTriangle);
                    }
                    if (!cubeToHandle.triangleMergePoints.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangleMergePoints.Add(triangleId, point1);
                    }
                }
                else
                {
                    // Create new cube:
                    cubeToHandle = new Cube(gridLimitsStrWholes1, X_gridLimitFloor1, X_gridLimitCeiling1,
                        Y_gridLimitFloor1, Y_gridLimitCeiling1, Z_gridLimitFloor1, Z_gridLimitCeiling1);

                    cubeToHandle.triangleNormalVectors = new Dictionary<int, Vector3D>();
                    cubeToHandle.triangleNormalVectors.Add(triangleId, normalVectorOfTriangle);
                    cubeToHandle.triangleMergePoints = new Dictionary<int, Point3D>();
                    cubeToHandle.triangleMergePoints.Add(triangleId, point1);

                    allCubes[gridLimitsStrWholes1] = cubeToHandle;

                    singleCubeVertices = new List<Point3D>();
                    singleCubeVertices.Add(point1);

                    // Associate a new list with a cube:
                    cubePointCloudVertices[gridLimitsStrWholes1] = singleCubeVertices;
                }

                if (cubePointCloudVertices.ContainsKey(gridLimitsStrWholes2))
                {
                    singleCubeVertices = cubePointCloudVertices[gridLimitsStrWholes2];
                    singleCubeVertices.Add(point2);

                    cubeToHandle = allCubes[gridLimitsStrWholes2];
                    if (!cubeToHandle.triangleNormalVectors.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangleNormalVectors.Add(triangleId, normalVectorOfTriangle);
                    }
                    if (!cubeToHandle.triangleMergePoints.ContainsKey(triangleId))
                    {
                        cubeToHandle.triangleMergePoints.Add(triangleId, point2);
                    }
                }
                else
                {
                    // Create new cube:
                    cubeToHandle = new Cube(gridLimitsStrWholes2, X_gridLimitFloor2, X_gridLimitCeiling2,
                        Y_gridLimitFloor2, Y_gridLimitCeiling2, Z_gridLimitFloor2, Z_gridLimitCeiling2);

                    cubeToHandle.triangleNormalVectors = new Dictionary<int, Vector3D>();
                    cubeToHandle.triangleNormalVectors.Add(triangleId, normalVectorOfTriangle);
                    cubeToHandle.triangleMergePoints = new Dictionary<int, Point3D>();
                    cubeToHandle.triangleMergePoints.Add(triangleId, point2);

                    allCubes[gridLimitsStrWholes2] = cubeToHandle;

                    singleCubeVertices = new List<Point3D>();
                    singleCubeVertices.Add(point2);

                    // Associate a new list with a cube:
                    cubePointCloudVertices[gridLimitsStrWholes2] = singleCubeVertices;
                }
            }
        }

        public void renderPointCloudMesh()
        {
            MeshGeometry3D mesh1 = new MeshGeometry3D();
            DiffuseMaterial surface_material1 = new DiffuseMaterial(Brushes.DarkOrange);
            Triangle triangleToRender;

            foreach (int triangleId in pointCloudTriangleList.Keys)
            {
                triangleToRender = pointCloudTriangleList[triangleId];
                renderViewFunctionalities.AddTriangle(mesh1,
                            triangleToRender.vertex1.vertexPosition,
                            triangleToRender.vertex2.vertexPosition,
                            triangleToRender.vertex3.vertexPosition);
            }

            GeometryModel3D surface_model1 = new GeometryModel3D(mesh1, surface_material1);

            // Make the surface visible from both sides.
            surface_model1.BackMaterial = surface_material1;

            // Add the model to the model groups.
            renderViewFunctionalities.MainModel3Dgroup.Children.Add(surface_model1);
        }


        /// <summary>
        /// This function divides out the raw point clouds into 3D cubes of the same size.
        /// </summary>
        public void processPointCloudIntoCubes()
        {
            double X_divided = 0.0;
            double Y_divided = 0.0;
            double Z_divided = 0.0;
            double X_gridLimitFloor = 0.0;
            double X_gridLimitCeiling = 0.0;
            double Y_gridLimitFloor = 0.0;
            double Y_gridLimitCeiling = 0.0;
            double Z_gridLimitFloor = 0.0;
            double Z_gridLimitCeiling = 0.0;
            string[] gridLimitsStr = new string[6];
            string gridLimitsStrWholes = "";

            Cube cubeToHandle;
            List<Point3D> singleCubeVertices;

            if(pointCloudVertices.Count == 0)
            {
                // Get the point cloud from the depth frame:
                pointCloudVertices = depthMasterControl.savedPointCloudList;
            }

            foreach (Point3D pointCloudVertex in pointCloudVertices)
            {

                X_divided = pointCloudVertex.X / cubeSize;
                Y_divided = pointCloudVertex.Y / cubeSize;
                Z_divided = pointCloudVertex.Z / cubeSize;

                // Get the cube delimeters of this particular vertex:
                X_gridLimitFloor = Math.Floor(X_divided);
                X_gridLimitCeiling = Math.Ceiling(X_divided);
                Y_gridLimitFloor = Math.Floor(Y_divided);
                Y_gridLimitCeiling = Math.Ceiling(Y_divided);
                Z_gridLimitFloor = Math.Floor(Z_divided);
                Z_gridLimitCeiling = Math.Ceiling(Z_divided);

                gridLimitsStr = new string[6];
                gridLimitsStr[0] = X_gridLimitFloor.ToString();
                gridLimitsStr[2] = Y_gridLimitFloor.ToString();
                gridLimitsStr[4] = Z_gridLimitFloor.ToString();
                gridLimitsStr[1] = X_gridLimitCeiling.ToString();
                gridLimitsStr[3] = Y_gridLimitCeiling.ToString();
                gridLimitsStr[5] = Z_gridLimitCeiling.ToString();

                // Create the key for the cube:
                gridLimitsStrWholes = gridLimitsStr[0] + "/" + gridLimitsStr[1] + "," + gridLimitsStr[2]
                        + "/" + gridLimitsStr[3] + "," + gridLimitsStr[4] + "/" + gridLimitsStr[5];

                if (cubePointCloudVertices.ContainsKey(gridLimitsStrWholes))
                {
                    // We already have a cube for that key, just need to insert the 
                    // current vertex into this existing cube:
                    singleCubeVertices = cubePointCloudVertices[gridLimitsStrWholes];
                    singleCubeVertices.Add(pointCloudVertex);
                }
                else
                {
                    // Create new cube:
                    cubeToHandle = new Cube(gridLimitsStrWholes, X_gridLimitFloor, X_gridLimitCeiling,
                        Y_gridLimitFloor, Y_gridLimitCeiling, Z_gridLimitFloor, Z_gridLimitCeiling);

                    allCubes[gridLimitsStrWholes] = cubeToHandle;

                    singleCubeVertices = new List<Point3D>();
                    singleCubeVertices.Add(pointCloudVertex);

                    // Associate a new list with a cube:
                    cubePointCloudVertices[gridLimitsStrWholes] = singleCubeVertices;
                }
            }
        }

        /// <summary>
        /// This function renders the point cloud as a connected 3D mesh. The triangulation process
        /// is based on the fact that the depth data comes in a 2D array which therefore allows us to
        /// connect verticees together as triangles as long as their distances are not far in between.
        /// For Realsense camera testing.
        /// </summary>
        public void createPointCloudMeshRealsense()
        {
            Dictionary<int, int> currentColumnPoints, nextColumnPoints;
            int rowIndexCount = 0;
            int nextRowIndex = 0;
            int triangleIndex = 0;
            Point3D[] trianglePointsArray;
            Point3D basePoint, downPoint, rightPoint, diagonalPoint;
            int basePointId, downPointId, rightPointId, diagonalPointId;
            double distanceDown = 0, distanceRight = 0, distanceDiagonal = 0;
            double x, y, z;
            Vector3D normalVectorOfTriangle;
            int h = 0;

            createPointCloudActualMeshRealsense();
            
            foreach (int rowIndex in indexedPoints.Keys)
            {
                if (rowIndexCount < (RealsenseHeight - 1))
                {
                    nextRowIndex = rowIndex + 1;
                    currentColumnPoints = indexedPoints[rowIndex];
                    nextColumnPoints = indexedPoints[nextRowIndex];

                    foreach (int columnIndex in currentColumnPoints.Keys)
                    {
                        if (columnIndex < (RealsenseWidth - 1))
                        {
                            basePoint = pointCloudIndexed[(indexedPoints[rowIndex])[columnIndex]];
                            if(basePoint.Z < (-25*(0.2)))
                            {
                                downPoint = pointCloudIndexed[(indexedPoints[nextRowIndex])[columnIndex]];
                                rightPoint = pointCloudIndexed[(indexedPoints[rowIndex])[columnIndex + 1]];
                                diagonalPoint = pointCloudIndexed[(indexedPoints[nextRowIndex])[columnIndex + 1]];

                                basePointId = (indexedPoints[rowIndex])[columnIndex];
                                downPointId = (indexedPoints[nextRowIndex])[columnIndex];
                                rightPointId = (indexedPoints[rowIndex])[columnIndex + 1];
                                diagonalPointId = (indexedPoints[nextRowIndex])[columnIndex + 1];

                                x = basePoint.X - downPoint.X;
                                y = basePoint.Y - downPoint.Y;
                                z = basePoint.Z - downPoint.Z;
                                distanceDown = x * x + y * y + z * z;

                                x = basePoint.X - diagonalPoint.X;
                                y = basePoint.Y - diagonalPoint.Y;
                                z = basePoint.Z - diagonalPoint.Z;
                                distanceDiagonal = x * x + y * y + z * z;

                                x = basePoint.X - rightPoint.X;
                                y = basePoint.Y - rightPoint.Y;
                                z = basePoint.Z - rightPoint.Z;
                                distanceRight = x * x + y * y + z * z;

                                if (distanceDown < acceptableDistance && distanceDiagonal < acceptableDistance)
                                {
                                    trianglePointsArray = new Point3D[3];
                                    trianglePointsArray[0] = basePoint;
                                    trianglePointsArray[1] = downPoint;
                                    trianglePointsArray[2] = diagonalPoint;
                                    
                                    normalVectorOfTriangle = MathAncillary.getNormalVectorOfTriangle(trianglePointsArray[0],
                                        trianglePointsArray[1], trianglePointsArray[2]);

                                    pointCloudTriangleNormalVectors.Add(triangleIndex, normalVectorOfTriangle);
                                    pointCloudTriangleListIndexed.Add(triangleIndex, new int[3] { basePointId, downPointId, diagonalPointId });
                                    pointCloudTriangleList.Add(triangleIndex, new Triangle(triangleIndex, trianglePointsArray));
                                    triangleIndex++;
                                }

                                if (distanceRight < acceptableDistance && distanceDiagonal < acceptableDistance)
                                {
                                    trianglePointsArray = new Point3D[3];
                                    trianglePointsArray[0] = basePoint;
                                    trianglePointsArray[1] = diagonalPoint;
                                    trianglePointsArray[2] = rightPoint;

                                    normalVectorOfTriangle = MathAncillary.getNormalVectorOfTriangle(trianglePointsArray[0],
                                       trianglePointsArray[1], trianglePointsArray[2]);
                                    

                                    pointCloudTriangleNormalVectors.Add(triangleIndex, normalVectorOfTriangle);
                                    pointCloudTriangleListIndexed.Add(triangleIndex, new int[3] { basePointId, rightPointId, diagonalPointId });
                                    pointCloudTriangleList.Add(triangleIndex, new Triangle(triangleIndex, trianglePointsArray));
                                    triangleIndex++;
                                }
                            }
                            
                        }
                    }
                }
                
                rowIndexCount++;
            }
        }


        /// <summary>
        /// This function renders the point cloud as a connected 3D mesh. The triangulation process
        /// is based on the fact that the depth data comes in a 2D array which therefore allows us to
        /// connect verticees together as triangles as long as their distances are not far in between.
        /// </summary>
        public void createPointCloudMesh()
        {
            Dictionary<int, int> currentColumnPoints, nextColumnPoints;
            int rowIndexCount = 0;
            int nextRowIndex = 0;
            int triangleIndex = 0;
            Point3D[] trianglePointsArray;
            Point3D basePoint, downPoint, rightPoint, diagonalPoint;
            int basePointId, downPointId, rightPointId, diagonalPointId;
            double distanceDown = 0, distanceRight = 0, distanceDiagonal = 0;
            double x, y, z;
            Vector3D normalVectorOfTriangle;

            indexedPoints = depthMasterControl.indexedPoints;
            pointCloudIndexed = depthMasterControl.pointCloudIndexed;

            foreach (int rowIndex in indexedPoints.Keys)
            {
                if (rowIndexCount < 423)
                {
                    nextRowIndex = rowIndex + 1;
                    currentColumnPoints = indexedPoints[rowIndex];
                    nextColumnPoints = indexedPoints[nextRowIndex];

                    foreach (int columnIndex in currentColumnPoints.Keys)
                    {
                        if (columnIndex < 511)
                        {
                            basePoint = pointCloudIndexed[(indexedPoints[rowIndex])[columnIndex]];
                            downPoint = pointCloudIndexed[(indexedPoints[nextRowIndex])[columnIndex]];
                            rightPoint = pointCloudIndexed[(indexedPoints[rowIndex])[columnIndex + 1]];
                            diagonalPoint = pointCloudIndexed[(indexedPoints[nextRowIndex])[columnIndex + 1]];

                            basePointId = (indexedPoints[rowIndex])[columnIndex];
                            downPointId = (indexedPoints[nextRowIndex])[columnIndex];
                            rightPointId = (indexedPoints[rowIndex])[columnIndex + 1];
                            diagonalPointId = (indexedPoints[nextRowIndex])[columnIndex + 1];

                            x = basePoint.X - downPoint.X;
                            y = basePoint.Y - downPoint.Y;
                            z = basePoint.Z - downPoint.Z;
                            distanceDown = Math.Sqrt(x * x + y * y + z * z);

                            x = basePoint.X - diagonalPoint.X;
                            y = basePoint.Y - diagonalPoint.Y;
                            z = basePoint.Z - diagonalPoint.Z;
                            distanceDiagonal = Math.Sqrt(x * x + y * y + z * z);

                            x = basePoint.X - rightPoint.X;
                            y = basePoint.Y - rightPoint.Y;
                            z = basePoint.Z - rightPoint.Z;
                            distanceRight = Math.Sqrt(x * x + y * y + z * z);

                            if (distanceDown < acceptableDistance && distanceDiagonal < acceptableDistance)
                            {
                                trianglePointsArray = new Point3D[3];
                                trianglePointsArray[0] = basePoint;
                                trianglePointsArray[1] = downPoint;
                                trianglePointsArray[2] = diagonalPoint;

                                normalVectorOfTriangle = MathAncillary.getNormalVectorOfTriangle(trianglePointsArray[0],
                                    trianglePointsArray[1], trianglePointsArray[2]);
                                pointCloudTriangleNormalVectors.Add(triangleIndex, normalVectorOfTriangle);
                                pointCloudTriangleListIndexed.Add(triangleIndex, new int[3] { basePointId, downPointId, diagonalPointId });
                                pointCloudTriangleList.Add(triangleIndex, new Triangle(triangleIndex, trianglePointsArray));
                                triangleIndex++;
                            }

                            if (distanceRight < acceptableDistance && distanceDiagonal < acceptableDistance)
                            {
                                trianglePointsArray = new Point3D[3];
                                trianglePointsArray[0] = basePoint;
                                trianglePointsArray[1] = diagonalPoint;
                                trianglePointsArray[2] = rightPoint;

                                normalVectorOfTriangle = MathAncillary.getNormalVectorOfTriangle(trianglePointsArray[0],
                                   trianglePointsArray[1], trianglePointsArray[2]);
                                pointCloudTriangleNormalVectors.Add(triangleIndex, normalVectorOfTriangle);
                                pointCloudTriangleListIndexed.Add(triangleIndex, new int[3] { basePointId, rightPointId, diagonalPointId });
                                pointCloudTriangleList.Add(triangleIndex, new Triangle(triangleIndex, trianglePointsArray));
                                triangleIndex++;
                            }
                        }
                    }
                }

                rowIndexCount++;
            }
        }

        public void getPointCloudOfDepthData()
        {
            double X_divided = 0.0;
            double Y_divided = 0.0;
            double Z_divided = 0.0;
            double X_gridLimitFloor = 0.0;
            double X_gridLimitCeiling = 0.0;
            double Y_gridLimitFloor = 0.0;
            double Y_gridLimitCeiling = 0.0;
            double Z_gridLimitFloor = 0.0;
            double Z_gridLimitCeiling = 0.0;
            string[] gridLimitsStr = new string[6];
            string gridLimitsStrWholes = "";
            int i = 0;
            Cube cubeToHandle;
            List<Point3D> singleCubeVertices;

            // added 1/4/18
            // extractDepthDataFromFiles();

            List<Point3D> pointCloudVertices = depthMasterControl.createPointCloud3(depthDataFromFile);
            
            foreach (Point3D pointCloudVertex in pointCloudVertices)
            {
                i++;
                if(i < 500)
                {
                    informationTextBlock.Text += pointCloudVertex.ToString() + "\n";
                }

                X_divided = pointCloudVertex.X / cubeSize;
                Y_divided = pointCloudVertex.Y / cubeSize;
                Z_divided = pointCloudVertex.Z / cubeSize;

                // Get the cube delimeters of this particular vertex:
                X_gridLimitFloor = Math.Floor(X_divided);
                X_gridLimitCeiling = Math.Ceiling(X_divided);
                Y_gridLimitFloor = Math.Floor(Y_divided);
                Y_gridLimitCeiling = Math.Ceiling(Y_divided);
                Z_gridLimitFloor = Math.Floor(Z_divided);
                Z_gridLimitCeiling = Math.Ceiling(Z_divided);

                gridLimitsStr = new string[6];
                gridLimitsStr[0] = X_gridLimitFloor.ToString();
                gridLimitsStr[2] = Y_gridLimitFloor.ToString();
                gridLimitsStr[4] = Z_gridLimitFloor.ToString();
                gridLimitsStr[1] = X_gridLimitCeiling.ToString();
                gridLimitsStr[3] = Y_gridLimitCeiling.ToString();
                gridLimitsStr[5] = Z_gridLimitCeiling.ToString();

                // Create the key for the cube:
                gridLimitsStrWholes = gridLimitsStr[0] + "/" + gridLimitsStr[1] + "," + gridLimitsStr[2]
                        + "/" + gridLimitsStr[3] + "," + gridLimitsStr[4] + "/" + gridLimitsStr[5];

                if (cubePointCloudVertices.ContainsKey(gridLimitsStrWholes))
                {
                    singleCubeVertices = cubePointCloudVertices[gridLimitsStrWholes];
                    singleCubeVertices.Add(pointCloudVertex);
                }
                else
                {
                    // Create new cube:
                    cubeToHandle = new Cube(gridLimitsStrWholes, X_gridLimitFloor, X_gridLimitCeiling,
                        Y_gridLimitFloor, Y_gridLimitCeiling, Z_gridLimitFloor, Z_gridLimitCeiling);

                    allCubes[gridLimitsStrWholes] = cubeToHandle;

                    singleCubeVertices = new List<Point3D>();
                    singleCubeVertices.Add(pointCloudVertex);

                    // Associate a new list with a cube:
                    cubePointCloudVertices[gridLimitsStrWholes] = singleCubeVertices;
                }
            }
        }

        /// <summary>
        /// This method connects planes of each grid together by merging them if they are on the same
        /// edge of the cube.
        /// </summary>
        public void connectAdjacentPlanes2()
        {
            var allCubeIds = allCubes.Keys;
            Dictionary<EdgesOfCube, Vertex> edgesOfCurrentCube;
            double currentVertexParameter;
            double neighborVertexParameter;
            double mergeValue;
            Dictionary<string, Cube> currentCubeNeighbors;
            Dictionary<ConnectOfCube, string> currentCubeConnectionsTypes;
            Cube currentCubeToHandle, neighborCubeToHandle;
            Vertex currentVertex;
            
            string neighborId;
            int oldVertexId;
            Dictionary<string, int> neighborMatchingCube = new Dictionary<string, int>();
            Dictionary<string, EdgesOfCube> neighborsWithMatchingEdges = new Dictionary<string, EdgesOfCube>();
            List<Vertex> listOfVerticesToMerge = new List<Vertex>();
            int countOfVerticesToMerge = 0;

            foreach (string cubeId in allCubeIds)
            {
                // Get the current cube:
                currentCubeToHandle = allCubes[cubeId];

                // Check if cube has a plane:
                if (currentCubeToHandle.hasPlane)
                {
                    // Get the neighbors of cube and their connection type:
                    currentCubeNeighbors = currentCubeToHandle.neighbors;
                    currentCubeConnectionsTypes = currentCubeToHandle.neighborsConnectionType;

                    // Get the occupied edges of current cube:
                    edgesOfCurrentCube = currentCubeToHandle.crossSectionVerticeesByEdge;

                    // For each edge with a present vertex:
                    foreach(EdgesOfCube currentEdge in edgesOfCurrentCube.Keys)
                    {
                        if (currentCubeToHandle.haveEdgesMerged[(int)currentEdge] == false)
                        {
                            neighborMatchingCube.Clear();
                            neighborsWithMatchingEdges.Clear();
                            listOfVerticesToMerge.Clear();
                            countOfVerticesToMerge = 0;
                            neighborVertexParameter = 0;
                            currentVertex = edgesOfCurrentCube[currentEdge];

                            // Find neighbors that include the current edge:
                            foreach (ConnectOfCube neighborConnection in currentCubeConnectionsTypes.Keys)
                            {
                                // Get the neighbor cube:
                                neighborId = currentCubeConnectionsTypes[neighborConnection];
                                neighborCubeToHandle = currentCubeToHandle.neighbors[neighborId];

                                // Check if neighbor cube has a plane:
                                if (neighborCubeToHandle.hasPlane)
                                {
                                    if (neighborConnection == edgeNeighbors[(int)currentEdge, 0])
                                    {
                                        neighborsWithMatchingEdges.Add(neighborId,
                                            edgeNeighborsComplimentaryEdge[(int)currentEdge, 0]);
                                    }

                                    if (neighborConnection == edgeNeighbors[(int)currentEdge, 1])
                                    {
                                        neighborsWithMatchingEdges.Add(neighborId,
                                            edgeNeighborsComplimentaryEdge[(int)currentEdge, 1]);
                                    }

                                    if (neighborConnection == edgeNeighbors[(int)currentEdge, 2])
                                    {
                                        neighborsWithMatchingEdges.Add(neighborId,
                                            edgeNeighborsComplimentaryEdge[(int)currentEdge, 2]);
                                    }
                                }
                            }

                            // We now have all the neighbors connected to the edge and their complimentary
                            // edges as well.

                            // Check which edge type the current edge is, so that we may know if
                            // we need to make an X, Y, or Z component change:
                            
                            if (currentEdge == EdgesOfCube.DA || currentEdge == EdgesOfCube.BC
                                || currentEdge == EdgesOfCube.FG || currentEdge == EdgesOfCube.HE)
                            {
                                // X Axis:

                                // Lets retrieve the edges of the neighbors that are connected to the current edge:
                                foreach (string matchedNeighbor in neighborsWithMatchingEdges.Keys)
                                {
                                    neighborCubeToHandle = currentCubeToHandle.neighbors[matchedNeighbor];

                                    // Check to see if the potential neighbor actually has a vertex at the
                                    // complimentary edge:
                                    if (neighborCubeToHandle.crossSectionVerticeesByEdge.ContainsKey(neighborsWithMatchingEdges[matchedNeighbor]))
                                    {
                                        neighborCubeToHandle.haveEdgesMerged[(int)neighborsWithMatchingEdges[matchedNeighbor]]
                                            = true;
                                        neighborVertexParameter += neighborCubeToHandle.
                                            crossSectionVerticeesByEdge[neighborsWithMatchingEdges[matchedNeighbor]].vertexPosition.X;
                                        countOfVerticesToMerge++;
                                    }
                                }

                                currentVertexParameter = currentVertex.vertexPosition.X;
                                countOfVerticesToMerge++;
                                mergeValue = (neighborVertexParameter + currentVertexParameter) / countOfVerticesToMerge;
                                currentVertex.vertexPosition.X = mergeValue;
                                
                                foreach (string matchedNeighbor in neighborsWithMatchingEdges.Keys)
                                {
                                    neighborCubeToHandle = currentCubeToHandle.neighbors[matchedNeighbor];

                                    if (neighborCubeToHandle.crossSectionVerticeesByEdge.ContainsKey(neighborsWithMatchingEdges[matchedNeighbor]))
                                    {
                                        oldVertexId = neighborCubeToHandle.crossSectionVerticeesByEdge[neighborsWithMatchingEdges
                                            [matchedNeighbor]].vertexId;

                                        // Replace the neighbors vertex with the current one:
                                        neighborCubeToHandle.crossSectionVerticeesByEdge[neighborsWithMatchingEdges[matchedNeighbor]]
                                            = currentVertex;

                                        // Replace the vertex at the edge of the neighbor with the current vertex:
                                        neighborCubeToHandle.cubeCrossSectionVertices.Remove(oldVertexId);
                                        neighborCubeToHandle.cubeCrossSectionVertices.Add(currentVertex.vertexId, currentVertex);
                                    }
                                }
                                
                            }
                            

                            if (currentEdge == EdgesOfCube.AE || currentEdge == EdgesOfCube.BF
                                || currentEdge == EdgesOfCube.CG || currentEdge == EdgesOfCube.DH)
                            {
                                // Y Axis:

                                // Lets retrieve the edges of the neighbors that are connected to the current edge:
                                foreach (string matchedNeighbor in neighborsWithMatchingEdges.Keys)
                                {
                                    neighborCubeToHandle = currentCubeToHandle.neighbors[matchedNeighbor];

                                    // Check to see if the potential neighbor actually has a vertex at the
                                    // complimentary edge:
                                    if (neighborCubeToHandle.crossSectionVerticeesByEdge.ContainsKey(neighborsWithMatchingEdges[matchedNeighbor]))
                                    {
                                        neighborCubeToHandle.haveEdgesMerged[(int)neighborsWithMatchingEdges[matchedNeighbor]]
                                            = true;
                                        neighborVertexParameter += neighborCubeToHandle.
                                            crossSectionVerticeesByEdge[neighborsWithMatchingEdges[matchedNeighbor]].vertexPosition.Y;
                                        countOfVerticesToMerge++;
                                    }
                                }

                                currentVertexParameter = currentVertex.vertexPosition.Y;
                                countOfVerticesToMerge++;
                                mergeValue = (neighborVertexParameter + currentVertexParameter) / countOfVerticesToMerge;
                                currentVertex.vertexPosition.Y = mergeValue;
                                
                                foreach (string matchedNeighbor in neighborsWithMatchingEdges.Keys)
                                {
                                    neighborCubeToHandle = currentCubeToHandle.neighbors[matchedNeighbor];

                                    if (neighborCubeToHandle.crossSectionVerticeesByEdge.ContainsKey(neighborsWithMatchingEdges[matchedNeighbor]))
                                    {
                                        oldVertexId = neighborCubeToHandle.crossSectionVerticeesByEdge[neighborsWithMatchingEdges
                                            [matchedNeighbor]].vertexId;

                                        // Replace the neighbors vertex with the current one:
                                        neighborCubeToHandle.crossSectionVerticeesByEdge[neighborsWithMatchingEdges[matchedNeighbor]]
                                            = currentVertex;

                                        // Replace the vertex at the edge of the neighbor with the current vertex:
                                        neighborCubeToHandle.cubeCrossSectionVertices.Remove(oldVertexId);
                                        neighborCubeToHandle.cubeCrossSectionVertices.Add(currentVertex.vertexId, currentVertex);
                                    }
                                }
                                
                            }

                            
                            if (currentEdge == EdgesOfCube.AB || currentEdge == EdgesOfCube.CD
                                || currentEdge == EdgesOfCube.EF || currentEdge == EdgesOfCube.GH)
                            {
                                // Z Axis:

                                // Lets retrieve the edges of the neighbors that are connected to the current edge:
                                foreach (string matchedNeighbor in neighborsWithMatchingEdges.Keys)
                                {
                                    neighborCubeToHandle = currentCubeToHandle.neighbors[matchedNeighbor];

                                    // Check to see if the potential neighbor actually has a vertex at the
                                    // complimentary edge:
                                    if (neighborCubeToHandle.crossSectionVerticeesByEdge.ContainsKey(neighborsWithMatchingEdges[matchedNeighbor]))
                                    {
                                        neighborCubeToHandle.haveEdgesMerged[(int)neighborsWithMatchingEdges[matchedNeighbor]]
                                            = true;
                                        neighborVertexParameter += neighborCubeToHandle.
                                            crossSectionVerticeesByEdge[neighborsWithMatchingEdges[matchedNeighbor]].vertexPosition.Z;
                                        countOfVerticesToMerge++;
                                    }
                                }

                                currentVertexParameter = currentVertex.vertexPosition.Z;
                                countOfVerticesToMerge++;
                                mergeValue = (neighborVertexParameter + currentVertexParameter) / countOfVerticesToMerge;
                                currentVertex.vertexPosition.Z = mergeValue;
                                
                                foreach (string matchedNeighbor in neighborsWithMatchingEdges.Keys)
                                {
                                    neighborCubeToHandle = currentCubeToHandle.neighbors[matchedNeighbor];

                                    if (neighborCubeToHandle.crossSectionVerticeesByEdge.ContainsKey(neighborsWithMatchingEdges[matchedNeighbor]))
                                    {
                                        oldVertexId = neighborCubeToHandle.crossSectionVerticeesByEdge[neighborsWithMatchingEdges
                                            [matchedNeighbor]].vertexId;

                                        // Replace the neighbors vertex with the current one:
                                        neighborCubeToHandle.crossSectionVerticeesByEdge[neighborsWithMatchingEdges[matchedNeighbor]]
                                            = currentVertex;

                                        // Replace the vertex at the edge of the neighbor with the current vertex:
                                        neighborCubeToHandle.cubeCrossSectionVertices.Remove(oldVertexId);
                                        neighborCubeToHandle.cubeCrossSectionVertices.Add(currentVertex.vertexId, currentVertex);
                                    }
                                }
                                
                            }
                            

                            currentCubeToHandle.haveEdgesMerged[(int)currentEdge] = true;
                        }
                    }
                }
            }
        }


        /// <summary>
        /// This method connects planes of each grid together by merging them if they are on the same
        /// edge of the cube.
        /// </summary>
        public void connectAdjacentPlanes()
        {
            /*
            var allCubeIds = allCubes.Keys;
            ConnectOfCube connectionOfCube;
            Dictionary<EdgesOfCube, Vertex> edgesOfCurrentCube;
            Dictionary<EdgesOfCube, Vertex> edgesOfNeighborCube;
            double currentVertexParameter;
            double neighborVertexParameter;
            double mergeValue;
            EdgesOfCube currentEdge;
            EdgesOfCube edgeOfNeighbor;
            Dictionary<string, ConnectOfCube> neighborsOfCubeConnectionType;
            Cube cubeToHandle, neighborCubeToHandle;
            Vertex currentVertex, neighborVertex;
            Dictionary<int, Vertex> neighborsVertices;

            foreach (string cubeId in allCubeIds)
            {
                cubeToHandle = allCubes[cubeId];

                // Check if cube has a plane:
                if(cubeToHandle.hasPlane)
                {
                    // Get the neighbors of cube and their connection type:
                    neighborsOfCubeConnectionType = cubeToHandle.neighborsConnectionType;
                    edgesOfCurrentCube = cubeToHandle.edges;

                    // Check each neighbor:
                    foreach (string neighborCubeId in neighborsOfCubeConnectionType.Keys)
                    {
                        // Get neighboring cube and if it has a plane:
                        neighborCubeToHandle = allCubes[neighborCubeId];
                        if (neighborCubeToHandle.hasPlane)
                        {
                            // Get connection type with the neighbor:
                            connectionOfCube = neighborsOfCubeConnectionType[neighborCubeId];

                            // Get the edges of the neighboring cube:
                            edgesOfNeighborCube = neighborCubeToHandle.edges;

                            // Connection at an edge:
                            if (connectionOfCube >= ConnectOfCube.AB && connectionOfCube <= ConnectOfCube.DH)
                            {
                                currentEdge = (EdgesOfCube)connectionOfCube;
                                if (edgesOfCurrentCube.ContainsKey(currentEdge))
                                {
                                    // Get the edge of neighbor; this should be complementary to the edge of current cube:
                                    edgeOfNeighbor = edgeNeighborEdges[(int)currentEdge];

                                    // Check to see if the neighbor cube has a vertex at its complementary edge:
                                    if (edgesOfNeighborCube.ContainsKey(edgeOfNeighbor))
                                    {
                                        neighborsVertices = neighborCubeToHandle.vertices;

                                        // Get the 2 vertices:
                                        currentVertex = edgesOfCurrentCube[currentEdge];
                                        neighborVertex = edgesOfNeighborCube[edgeOfNeighbor];
                                        
                                        // Merge the vertices:
                                        if (currentEdge == EdgesOfCube.AB || currentEdge == EdgesOfCube.CD
                                            || currentEdge == EdgesOfCube.EF || currentEdge == EdgesOfCube.GH)
                                        {
                                            // Z Axis:
                                            currentVertexParameter = currentVertex.vertexPosition.Z;
                                            neighborVertexParameter = neighborVertex.vertexPosition.Z;
                                            mergeValue = currentVertexParameter +
                                                (neighborVertexParameter - currentVertexParameter) / 2;
                                            currentVertex.vertexPosition.Z = mergeValue;
                                        }

                                        if (currentEdge == EdgesOfCube.AE || currentEdge == EdgesOfCube.BF
                                            || currentEdge == EdgesOfCube.CG || currentEdge == EdgesOfCube.DH)
                                        {
                                            // Y Axis:
                                            currentVertexParameter = currentVertex.vertexPosition.Y;
                                            neighborVertexParameter = neighborVertex.vertexPosition.Y;
                                            mergeValue = currentVertexParameter +
                                                (neighborVertexParameter - currentVertexParameter) / 2;
                                            currentVertex.vertexPosition.Y = mergeValue;
                                        }

                                        if (currentEdge == EdgesOfCube.DA || currentEdge == EdgesOfCube.BC
                                            || currentEdge == EdgesOfCube.FG || currentEdge == EdgesOfCube.HE)
                                        {
                                            // X Axis:
                                            currentVertexParameter = currentVertex.vertexPosition.X;
                                            neighborVertexParameter = neighborVertex.vertexPosition.X;
                                            mergeValue = currentVertexParameter +
                                                (neighborVertexParameter - currentVertexParameter) / 2;
                                            currentVertex.vertexPosition.X = mergeValue;
                                        }
                                        
                                        // Remove the neighbors vertex from the global list of all vertices:
                                        allVertices.Remove(neighborVertex.vertexId);

                                        // Replace the neighbors vertex with the current one:
                                        neighborsVertices[neighborVertex.vertexId] = currentVertex;
                                        //neighborsVertices.Add(currentVertex.vertexId, currentVertex);

                                        // Replace the vertex at the edge of the neighbor with the current vertex:
                                        edgesOfNeighborCube[edgeOfNeighbor] = currentVertex;
                                    }
                                }
                            }

                            // Connection at corner:
                            if (connectionOfCube >= ConnectOfCube.ABC && connectionOfCube <= ConnectOfCube.HEF)
                            {

                            }

                            // Connection at face:
                            if (connectionOfCube >= ConnectOfCube.LEFT && connectionOfCube <= ConnectOfCube.BOTTOM)
                            {

                            }
                        
                        }


                    }
                }
            }
            */
        }

        public void calculatePlaneOfCubeNormalsMerging(Cube cubeToHandle, ref int facedVerticesIndex,
           List<Point3D> pointCloudVerticesOfCube)
        {
            Point3D[] trianglePoints = new Point3D[3];

            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();
            var allCubeIds = cubePointCloudVertices.Keys;
            int verticesCount = 0;
            int closestIndex = 0;
            Vector3D vector1, vector2, vector3;
            Point3D point1 = new Point3D();
            Point3D medianPoint = new Point3D(0, 0, 0);

            double[] cubeLimits = new double[6];
            List<Vector3D> listOfNormalVectors = new List<Vector3D>();

            Vector3D normalVector = new Vector3D();
            double contstantOfPlaneEq = 0;
            double normalVectorXAverage = 0;
            double normalVectorYAverage = 0;
            double normalVectorZAverage = 0;

            cubeLimits[0] = cubeToHandle.xFloor;
            cubeLimits[1] = cubeToHandle.xCeiling;
            cubeLimits[2] = cubeToHandle.yFloor;
            cubeLimits[3] = cubeToHandle.yCeiling;
            cubeLimits[4] = cubeToHandle.zFloor;
            cubeLimits[5] = cubeToHandle.zCeiling;
            
            if (pointCloudVerticesOfCube.Count >= 3)
            {
                point1 = pointCloudVerticesOfCube.First();

                // Go through each point and create a number of normal vectors:
                foreach (Point3D point2 in pointCloudVerticesOfCube)
                {
                    if(point1 != point2)
                    {
                        foreach (Point3D point3 in pointCloudVerticesOfCube)
                        {
                            if(point3 != point1 && point3 != point2)
                            {
                                listOfNormalVectors.Add(MathAncillary.getNormalVectorOfTriangle(point1, point2, point3));
                            }
                        }
                    }
                }

                // Get the average normal vector:
                foreach (Vector3D eachNormalVector in listOfNormalVectors)
                {
                    normalVectorXAverage += eachNormalVector.X;
                    normalVectorYAverage += eachNormalVector.Y;
                    normalVectorZAverage += eachNormalVector.Z;
                }

                normalVector = new Vector3D(normalVectorXAverage / listOfNormalVectors.Count,
                    normalVectorYAverage / listOfNormalVectors.Count,
                    normalVectorZAverage / listOfNormalVectors.Count);
                
                // Lets get the constant part of the equation of plane:
                contstantOfPlaneEq = point1.X * normalVector.X + point1.Y * normalVector.Y
                    + point1.Z * normalVector.Z;

                int countZeros = 0;
                double zeroDouble = 0.0;
                if (zeroDouble.CompareTo(normalVector.X) == 0)
                {
                    countZeros++;
                }
                if (zeroDouble.CompareTo(normalVector.Y) == 0)
                {
                    countZeros++;
                }
                if (zeroDouble.CompareTo(normalVector.Z) == 0)
                {
                    countZeros++;
                }

                if (countZeros >= 2)
                {
                    // Add small amount to the normal vector components:
                    normalVector.X += 0.0001;
                    normalVector.Y += 0.0001;
                    normalVector.Z += 0.0001;

                    contstantOfPlaneEq = point1.X * normalVector.X + point1.Y * normalVector.Y
                    + point1.Z * normalVector.Z;
                }

                cubeToHandle.planeEquationConstant = contstantOfPlaneEq;
                cubeToHandle.planeEquationNormalVector.Normalize();
                cubeToHandle.planeEquationNormalVector = normalVector;

                findCubeCrossSectionsAndUpdateDataStructures(cubeToHandle.cubeId, ref facedVerticesIndex, cubeLimits,
                       normalVector.X, normalVector.Y, normalVector.Z, contstantOfPlaneEq);
            }

            cubeToHandle.planeTrianglePoints = trianglePoints;

        }


        public void calculatePlaneOfCubeNormalsMergingFirst(Cube cubeToHandle, ref int facedVerticesIndex,
           List<Point3D> pointCloudVerticesOfCube)
        {
            Point3D[] trianglePoints = new Point3D[3];

            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();
            var allCubeIds = cubePointCloudVertices.Keys;
            int verticesCount = 0;
            int closestIndex = 0;
            Vector3D vector1, vector2, vector3;
            Point3D point1 = new Point3D(), point2 = new Point3D(), point3 = new Point3D();
            Point3D medianPoint = new Point3D(0,0,0);

            double[] cubeLimits = new double[6];
            List<Vector3D> listOfNormalVectors = new List<Vector3D>();

            Vector3D normalVector = new Vector3D();
            double contstantOfPlaneEq = 0;
            double normalVectorXAverage = 0;
            double normalVectorYAverage = 0;
            double normalVectorZAverage = 0;

            cubeLimits[0] = cubeToHandle.xFloor;
            cubeLimits[1] = cubeToHandle.xCeiling;
            cubeLimits[2] = cubeToHandle.yFloor;
            cubeLimits[3] = cubeToHandle.yCeiling;
            cubeLimits[4] = cubeToHandle.zFloor;
            cubeLimits[5] = cubeToHandle.zCeiling;

            if(pointCloudVerticesOfCube.Count >= 3)
            {
                listOfNormalVectors.Clear();

                // Go through each point and create a number of normal vectors:
                foreach (Point3D vertexPoint in pointCloudVerticesOfCube)
                {
                    verticesCount++;
                    medianPoint.X += vertexPoint.X;
                    medianPoint.Y += vertexPoint.Y;
                    medianPoint.Z += vertexPoint.Z;

                    if (verticesCount == 1)
                    {
                        point1 = vertexPoint;
                    }

                    if (verticesCount == 2)
                    {
                        point2 = point1;
                        point1 = vertexPoint;
                    }

                    if (verticesCount > 2)
                    {
                        point3 = point2;
                        point2 = point1;
                        point1 = vertexPoint;

                        listOfNormalVectors.Add(MathAncillary.getNormalVectorOfTriangle(point1, point2, point3));
                    }
                }

                // Get the average normal vector:
                foreach(Vector3D eachNormalVector in listOfNormalVectors)
                {
                    normalVectorXAverage += eachNormalVector.X;
                    normalVectorYAverage += eachNormalVector.Y;
                    normalVectorZAverage += eachNormalVector.Z;
                }

                normalVector = new Vector3D(normalVectorXAverage / listOfNormalVectors.Count,
                    normalVectorYAverage / listOfNormalVectors.Count,
                    normalVectorZAverage / listOfNormalVectors.Count);

                medianPoint.X += medianPoint.X / listOfNormalVectors.Count;
                medianPoint.Y += medianPoint.Y / listOfNormalVectors.Count;
                medianPoint.Z += medianPoint.Z / listOfNormalVectors.Count;

                if(facedVerticesIndex < 50)
                {
                    informationTextBlock.Text += "shehshe " + normalVector.X.ToString("n6") + "," +
                        normalVector.Y.ToString("n6") + "," + normalVector.Z.ToString("n6") + "\n";
                }

                // Lets get the constant part of the equation of plane:
                contstantOfPlaneEq = point1.X * normalVector.X + point1.Y * normalVector.Y
                    + point1.Z * normalVector.Z;

                int countZeros = 0;
                double zeroDouble = 0.0;
                if (zeroDouble.CompareTo(normalVector.X) == 0)
                {
                    countZeros++;
                }
                if (zeroDouble.CompareTo(normalVector.Y) == 0)
                {
                    countZeros++;
                }
                if (zeroDouble.CompareTo(normalVector.Z) == 0)
                {
                    countZeros++;
                }

                if (countZeros >= 2)
                {
                    // Add small amount to the normal vector components:
                    normalVector.X += 0.0001;
                    normalVector.Y += 0.0001;
                    normalVector.Z += 0.0001;

                    contstantOfPlaneEq = point1.X * normalVector.X + point1.Y * normalVector.Y
                    + point1.Z * normalVector.Z;
                }

                cubeToHandle.planeEquationConstant = contstantOfPlaneEq;
                
                cubeToHandle.planeEquationNormalVector = normalVector;
                cubeToHandle.planeEquationNormalVector.Normalize();

                findCubeCrossSectionsAndUpdateDataStructures(cubeToHandle.cubeId, ref facedVerticesIndex, cubeLimits,
                       normalVector.X, normalVector.Y, normalVector.Z, contstantOfPlaneEq);
            }
            
            cubeToHandle.planeTrianglePoints = trianglePoints;
            
        }

        /// <summary>
        /// This function will take the available triangles inside the cube and determine the plane of best 
        /// fit.
        /// </summary>
        /// <param name="cubeToHandle"></param>
        /// <param name="facedVerticesIndex"></param>
        /// <param name="pointCloudVerticesOfCube"></param>
        public void calculatePlaneOfCube(Cube cubeToHandle, ref int facedVerticesIndex,
           List<Point3D> pointCloudVerticesOfCube)
        {
            Point3D[] trianglePoints = new Point3D[3];

            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();
            var allCubeIds = cubePointCloudVertices.Keys;
            double[] cubeLimits = new double[6];

            Vector3D normalVector = new Vector3D(0,0,0);
            double contstantOfPlaneEq = 0;

            Point3D allTrianglesMergePoint = new Point3D(0,0,0);
            cubeToHandle.planeTrianglePoints = new Point3D[1];

            cubeLimits[0] = cubeToHandle.xFloor;
            cubeLimits[1] = cubeToHandle.xCeiling;
            cubeLimits[2] = cubeToHandle.yFloor;
            cubeLimits[3] = cubeToHandle.yCeiling;
            cubeLimits[4] = cubeToHandle.zFloor;
            cubeLimits[5] = cubeToHandle.zCeiling;
            
            foreach(int triangleId in cubeToHandle.triangleNormalVectors.Keys)
            {
                normalVector.X += cubeToHandle.triangleNormalVectors[triangleId].X;
                normalVector.Y += cubeToHandle.triangleNormalVectors[triangleId].Y;
                normalVector.Z += cubeToHandle.triangleNormalVectors[triangleId].Z;

                allTrianglesMergePoint.X += cubeToHandle.triangleMergePoints[triangleId].X;
                allTrianglesMergePoint.Y += cubeToHandle.triangleMergePoints[triangleId].Y;
                allTrianglesMergePoint.Z += cubeToHandle.triangleMergePoints[triangleId].Z;
            }
            
            allTrianglesMergePoint.X /= cubeToHandle.triangleMergePoints.Count;
            allTrianglesMergePoint.Y /= cubeToHandle.triangleMergePoints.Count;
            allTrianglesMergePoint.Z /= cubeToHandle.triangleMergePoints.Count;

            normalVector = Vector3D.Divide(normalVector,
                cubeToHandle.triangleNormalVectors.Keys.Count);

            // Lets get the constant part of the equation of plane:
            contstantOfPlaneEq = allTrianglesMergePoint.X * normalVector.X + allTrianglesMergePoint.Y * normalVector.Y
                + allTrianglesMergePoint.Z * normalVector.Z;

            cubeToHandle.planeEquationConstant = contstantOfPlaneEq;

            cubeToHandle.planeEquationNormalVector = normalVector;
            cubeToHandle.planeEquationNormalVector.Normalize();

            // Sometimes the equation of the plane comes with 2 coefficients that are 0;
            // These are the planes that have their normal vectors point in the direction
            // of an axis. This case creates problems when we try to calculate cross sections.
            // We solve this problem by adding a small value to the normal vector components.

            int countZeros = 0;
            double zeroDouble = 0.0;
            if (zeroDouble.CompareTo(normalVector.X) == 0)
            {
                countZeros++;
            }
            if (zeroDouble.CompareTo(normalVector.Y) == 0)
            {
                countZeros++;
            }
            if (zeroDouble.CompareTo(normalVector.Z) == 0)
            {
                countZeros++;
            }

            if (countZeros >= 2)
            {
                // Add small amount to the normal vector components:
                normalVector.X += 0.0001;
                normalVector.Y += 0.0001;
                normalVector.Z += 0.0001;

                contstantOfPlaneEq = allTrianglesMergePoint.X * normalVector.X + allTrianglesMergePoint.Y * normalVector.Y
                + allTrianglesMergePoint.Z * normalVector.Z;
            }

            // Now that we have the cube plane's equation, lets see which of the cubes edges this plane will 
            // intersect:
            //findCubeCrossSectionsAndUpdateDataStructures(cubeToHandle.cubeId, ref facedVerticesIndex, cubeLimits,
            //        normalVector.X, normalVector.Y, normalVector.Z, contstantOfPlaneEq);
        }

        /// <summary>
        /// This method generates a plane for the cube (in the form of a triangle) by using all
        /// point cloud points. First 3 points are chosen and the subsequent points then shift the 
        /// closest of the 3 triangle points until we have a plane of best fit.
        /// </summary>
        /// <param name="cubeToHandle"></param>
        /// <param name="facedVerticesIndex"></param>
        /// <param name="pointCloudVerticesOfCube"></param>
        public void calculatePlaneOfCubeTriangleIterativeGeneration(Cube cubeToHandle, ref int facedVerticesIndex,
           List<Point3D> pointCloudVerticesOfCube)
        {
            Point3D[] trianglePoints = new Point3D[3];

            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();
            var allCubeIds = cubePointCloudVertices.Keys;
            int verticesCount = 0;
            int closestIndex = 0;
            Vector3D vector1, vector2;
            double[] cubeLimits = new double[6];

            Vector3D normalVector = new Vector3D();
            double contstantOfPlaneEq = 0;

            cubeLimits[0] = cubeToHandle.xFloor;
            cubeLimits[1] = cubeToHandle.xCeiling;
            cubeLimits[2] = cubeToHandle.yFloor;
            cubeLimits[3] = cubeToHandle.yCeiling;
            cubeLimits[4] = cubeToHandle.zFloor;
            cubeLimits[5] = cubeToHandle.zCeiling;

            // Go through each point and merge all to just have 3 vertices for plane creation:
            foreach (Point3D vertexPoint in pointCloudVerticesOfCube)
            {
                if (verticesCount > 2)
                {
                    // Find closest point:
                    closestIndex = findClosestTriangleVertex(trianglePoints, vertexPoint);

                    // Merge current point with the closest point:
                    trianglePoints[closestIndex] = mergeTwoPoints(trianglePoints[closestIndex], vertexPoint);
                }
                else
                {
                    trianglePoints[verticesCount] = vertexPoint;
                    verticesCount++;
                }
            }

            cubeToHandle.planeTrianglePoints = trianglePoints;

            // If we have more than 3 vertices, we can create a plane:
            if (verticesCount > 2)
            {
                // Find normal vector of plane:
                vector1 = new Vector3D(trianglePoints[1].X - trianglePoints[0].X,
                    trianglePoints[1].Y - trianglePoints[0].Y,
                    trianglePoints[1].Z - trianglePoints[0].Z);

                vector2 = new Vector3D(trianglePoints[2].X - trianglePoints[0].X,
                    trianglePoints[2].Y - trianglePoints[0].Y,
                    trianglePoints[2].Z - trianglePoints[0].Z);

                normalVector = Vector3D.CrossProduct(vector1, vector2);

                // Lets get the constant part of the equation of plane:
                contstantOfPlaneEq = trianglePoints[1].X * normalVector.X + trianglePoints[1].Y * normalVector.Y
                    + trianglePoints[1].Z * normalVector.Z;

                cubeToHandle.planeEquationConstant = contstantOfPlaneEq;
                cubeToHandle.planeEquationNormalVector = normalVector;
                cubeToHandle.planeEquationNormalVector.Normalize();
                
                if (facedVerticesIndex < 50)
                {
                    informationTextBlock.Text += "shehshe " + normalVector.X.ToString("n6") + "," +
                        normalVector.Y.ToString("n6") + "," + normalVector.Z.ToString("n6") + "\n";
                }

                // Sometimes the equation of the plane comes with 2 coefficients that are 0;
                // These are the planes that have their normal vectors point in the direction
                // of an axis. This case creates problems when we try to calculate cross sections.
                // We solve this problem by adding a small value to the normal vector components.

                int countZeros = 0;
                double zeroDouble = 0.0;
                if (zeroDouble.CompareTo(normalVector.X) == 0)
                {
                    countZeros++;
                }
                if (zeroDouble.CompareTo(normalVector.Y) == 0)
                {
                    countZeros++;
                }
                if (zeroDouble.CompareTo(normalVector.Z) == 0)
                {
                    countZeros++;
                }

                if (countZeros >= 2)
                {
                    // Add small amount to the normal vector components:
                    normalVector.X += 0.0001;
                    normalVector.Y += 0.0001;
                    normalVector.Z += 0.0001;

                    contstantOfPlaneEq = trianglePoints[1].X * normalVector.X + trianglePoints[1].Y * normalVector.Y
                    + trianglePoints[1].Z * normalVector.Z;
                }


                findCubeCrossSectionsAndUpdateDataStructures(cubeToHandle.cubeId, ref facedVerticesIndex, cubeLimits,
                       normalVector.X, normalVector.Y, normalVector.Z, contstantOfPlaneEq);
            }
        }

        public void calculatePlaneOfCubePointMerging(Cube cubeToHandle, ref int facedVerticesIndex,
            List<Point3D> pointCloudVerticesOfCube)
        {
            Point3D[] trianglePoints = new Point3D[3];

            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();
            var allCubeIds = cubePointCloudVertices.Keys;
            int verticesCount = 0;
            int closestIndex = 0;
            Vector3D vector1, vector2;
            double[] cubeLimits = new double[6];

            Vector3D normalVector = new Vector3D();
            double constantOfPlaneEq = 0;

            cubeLimits[0] = cubeToHandle.xFloor;
            cubeLimits[1] = cubeToHandle.xCeiling;
            cubeLimits[2] = cubeToHandle.yFloor;
            cubeLimits[3] = cubeToHandle.yCeiling;
            cubeLimits[4] = cubeToHandle.zFloor;
            cubeLimits[5] = cubeToHandle.zCeiling;

            Point3D At = pointCloudVerticesOfCube[0];
            Point3D Bt = pointCloudVerticesOfCube[1];
            Point3D Ct = pointCloudVerticesOfCube[2];

            // Create the initial triangle:
            Point3D A = new Point3D(pointCloudVerticesOfCube[0].X, pointCloudVerticesOfCube[0].Y, pointCloudVerticesOfCube[0].Z);
            Point3D B = new Point3D(pointCloudVerticesOfCube[1].X, pointCloudVerticesOfCube[1].Y, pointCloudVerticesOfCube[1].Z);
            Point3D C = new Point3D(pointCloudVerticesOfCube[2].X, pointCloudVerticesOfCube[2].Y, pointCloudVerticesOfCube[2].Z);

            // Save the points so that we can add them back later:
            Point3D As = new Point3D(pointCloudVerticesOfCube[0].X, pointCloudVerticesOfCube[0].Y, pointCloudVerticesOfCube[0].Z);
            Point3D Bs = new Point3D(pointCloudVerticesOfCube[1].X, pointCloudVerticesOfCube[1].Y, pointCloudVerticesOfCube[1].Z);
            Point3D Cs = new Point3D(pointCloudVerticesOfCube[2].X, pointCloudVerticesOfCube[2].Y, pointCloudVerticesOfCube[2].Z);

            // Remove the initial triangle points so that they dont appear in iterative calculation:
            pointCloudVerticesOfCube.Remove(At);
            pointCloudVerticesOfCube.Remove(Bt);
            pointCloudVerticesOfCube.Remove(Ct);

            double dAx = 0;
            double dAy = 0;
            double dAz = 0;

            double dBx = 0;
            double dBy = 0;
            double dBz = 0;

            double dCx = 0;
            double dCy = 0;
            double dCz = 0;

            double distA = 0;
            double distB = 0;
            double distC = 0;

            int closestPoint = 0;

            int accumulatorA = 0;
            int accumulatorB = 0;
            int accumulatorC = 0;

            if (cubeToHandle.cubeId.Equals("-13/-12,-5/-4,-69/-68"))
            {
                /*
                pointCloudVerticesOfCube = new List<Point3D>();
                pointCloudVerticesOfCube.Add(new Point3D(-12.614800, -4.068210, -68.850000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.790400, -4.072640, -68.925000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.905300, -4.864430, -68.675000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.157400, -4.983660, -68.100000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.344500, -4.994630, -68.250000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.776500, -4.229930, -68.850000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.952300, -4.234540, -68.925000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.045900, -4.361360, -68.375000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.228800, -4.369330, -68.500000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.067900, -4.208430, -68.500000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.246700, -4.214570, -68.600000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.421400, -4.219180, -68.675000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.601000, -4.225320, -68.775000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.914700, -4.706540, -68.725000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.175300, -4.830780, -68.200000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.358100, -4.839630, -68.325000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.541500, -4.848490, -68.450000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.385200, -4.528570, -68.475000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.564400, -4.535190, -68.575000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.744000, -4.541800, -68.675000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.924100, -4.548420, -68.775000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.006300, -4.667160, -68.150000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.188700, -4.675720, -68.275000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.371700, -4.684280, -68.400000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.550600, -4.691130, -68.500000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.734700, -4.699690, -68.625000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.720800, -4.855570, -68.550000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.403300, -4.374110, -68.575000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.582700, -4.380490, -68.675000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.762600, -4.386870, -68.775000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.938200, -4.391650, -68.850000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.028300, -4.515350, -68.275000));
                pointCloudVerticesOfCube.Add(new Point3D(-12.206500, -4.521960, -68.375000));
                */

                cubeInformationTextBox.Text = "";
                cubeInformationTextBox.Text += "P1: " + A.X.ToString("n6") + "," +
                    A.Y.ToString("n6") + "," + A.Z.ToString("n6") + "\n";
                cubeInformationTextBox.Text += "P2: " + B.X.ToString("n6") + "," +
                    B.Y.ToString("n6") + "," + B.Z.ToString("n6") + "\n";
                cubeInformationTextBox.Text += "P3: " + C.X.ToString("n6") + "," +
                    C.Y.ToString("n6") + "," + C.Z.ToString("n6") + "\n";
                
            }

            // Go through each point and merge all to just have 3 vertices for plane creation:
            foreach (Point3D vertexPoint in pointCloudVerticesOfCube)
            {
                if (cubeToHandle.cubeId.Equals("-13/-12,-5/-4,-69/-68"))
                {
                    cubeInformationTextBox.Text += "P: " + vertexPoint.X.ToString("n6") + "," +
                        vertexPoint.Y.ToString("n6") + "," + vertexPoint.Z.ToString("n6") + "\n";
                }

                // Calculate distances between triangle points:
                dAx = vertexPoint.X - A.X;
                dAy = vertexPoint.Y - A.Y;
                dAz = vertexPoint.Z - A.Z;

                dBx = vertexPoint.X - B.X;
                dBy = vertexPoint.Y - B.Y;
                dBz = vertexPoint.Z - B.Z;

                dCx = vertexPoint.X - C.X;
                dCy = vertexPoint.Y - C.Y;
                dCz = vertexPoint.Z - C.Z;

                distA = dAx * dAx + dAy * dAy + dAz * dAz;
                distB = dBx * dBx + dBy * dBy + dBz * dBz;
                distC = dCx * dCx + dCy * dCy + dCz * dCz;

                // Find the closest triangle point:
                if (distA < distB && distA < distC)
                {
                    
                    closestPoint = 1;
                    accumulatorA++;

                    dAx /= 2;
                    dAy /= 2;
                    dAz /= 2;

                    dAx /= accumulatorA;
                    dAy /= accumulatorA;
                    dAz /= accumulatorA;

                    A.X += dAx;
                    A.Y += dAy;
                    A.Z += dAz;

                    if (cubeToHandle.cubeId.Equals("-13/-12,-5/-4,-69/-68"))
                    {
                        cubeInformationTextBox.Text += "A: " + A.X.ToString("n6") + "," +
                            A.Y.ToString("n6") + "," + A.Z.ToString("n6") + "\n";
                    }
                }
                else
                {
                    if (distB < distC)
                    {
                        closestPoint = 2;
                        accumulatorB++;

                        dBx /= 2;
                        dBy /= 2;
                        dBz /= 2;

                        dBx /= accumulatorB;
                        dBy /= accumulatorB;
                        dBz /= accumulatorB;

                        B.X += dBx;
                        B.Y += dBy;
                        B.Z += dBz;

                        if (cubeToHandle.cubeId.Equals("-13/-12,-5/-4,-69/-68"))
                        {
                            cubeInformationTextBox.Text += "B: " + B.X.ToString("n6") + "," +
                                B.Y.ToString("n6") + "," + B.Z.ToString("n6") + "\n";
                        }
                    }
                    else
                    {
                        closestPoint = 3;
                        accumulatorC++;

                        dCx /= 2;
                        dCy /= 2;
                        dCz /= 2;

                        dCx /= accumulatorC;
                        dCy /= accumulatorC;
                        dCz /= accumulatorC;

                        C.X += dCx;
                        C.Y += dCy;
                        C.Z += dCz;

                        if (cubeToHandle.cubeId.Equals("-13/-12,-5/-4,-69/-68"))
                        {
                            cubeInformationTextBox.Text += "C: " + C.X.ToString("n6") + "," +
                                C.Y.ToString("n6") + "," + C.Z.ToString("n6") + "\n";
                        }
                    }
                }
            }

            // Add points back:
            pointCloudVerticesOfCube.Add(As);
            pointCloudVerticesOfCube.Add(Bs);
            pointCloudVerticesOfCube.Add(Cs);
            
            trianglePoints[0] = A;
            trianglePoints[1] = B;
            trianglePoints[2] = C;

            cubeToHandle.planeTrianglePoints = trianglePoints;
            
            // Find normal vector of plane:
            vector1 = new Vector3D(trianglePoints[1].X - trianglePoints[0].X,
                trianglePoints[1].Y - trianglePoints[0].Y,
                trianglePoints[1].Z - trianglePoints[0].Z);

            vector2 = new Vector3D(trianglePoints[2].X - trianglePoints[0].X,
                trianglePoints[2].Y - trianglePoints[0].Y,
                trianglePoints[2].Z - trianglePoints[0].Z);

            normalVector = Vector3D.CrossProduct(vector1, vector2);

            // Lets get the constant part of the equation of plane:
            constantOfPlaneEq = trianglePoints[1].X * normalVector.X + trianglePoints[1].Y * normalVector.Y
                + trianglePoints[1].Z * normalVector.Z;

            cubeToHandle.planeEquationConstant = constantOfPlaneEq;
            cubeToHandle.planeEquationNormalVector = normalVector;
            cubeToHandle.planeEquationNormalVector.Normalize();

            if (cubeToHandle.cubeId.Equals("-13/-12,-5/-4,-69/-68"))
            {
                informationTextBlock.Text += "Cube Normallleee: " + cubeToHandle.planeEquationNormalVector.X.ToString("n6") + "," +
                    cubeToHandle.planeEquationNormalVector.Y.ToString("n6") + "," + cubeToHandle.planeEquationNormalVector.Z.ToString("n6") + "\n";
                informationTextBlock.Text += "A Normallleee: " + A.X.ToString("n6") + "," +
                    A.Y.ToString("n6") + "," + A.Z.ToString("n6") + "\n";
                informationTextBlock.Text += "B Normallleee: " + B.X.ToString("n6") + "," +
                    B.Y.ToString("n6") + "," + B.Z.ToString("n6") + "\n";
                informationTextBlock.Text += "C Normallleee: " + C.X.ToString("n6") + "," +
                    C.Y.ToString("n6") + "," + C.Z.ToString("n6") + "\n";
                informationTextBlock.Text += "***********\n";
                

                foreach (Point3D vertexPoint in pointCloudVerticesOfCube)
                {
                    informationTextBlock.Text += vertexPoint.X.ToString("n6") + "," +
                    vertexPoint.Y.ToString("n6") + "," + vertexPoint.Z.ToString("n6") + "\n";
                }

                informationTextBlock.Text += "***********\n";
            }

            // Sometimes the equation of the plane comes with 2 coefficients that are 0;
            // These are the planes that have their normal vectors point in the direction
            // of an axis. This case creates problems when we try to calculate cross sections.
            // We solve this problem by adding a small value to the normal vector components.

            int countZeros = 0;
            double zeroDouble = 0.0;
            if (zeroDouble.CompareTo(normalVector.X) == 0)
            {
                countZeros++;
            }
            if (zeroDouble.CompareTo(normalVector.Y) == 0)
            {
                countZeros++;
            }
            if (zeroDouble.CompareTo(normalVector.Z) == 0)
            {
                countZeros++;
            }

            if (countZeros >= 2)
            {
                // Add small amount to the normal vector components:
                normalVector.X += 0.0001;
                normalVector.Y += 0.0001;
                normalVector.Z += 0.0001;

                constantOfPlaneEq = trianglePoints[1].X * normalVector.X + trianglePoints[1].Y * normalVector.Y
                + trianglePoints[1].Z * normalVector.Z;
            }
                
            findCubeCrossSectionsAndUpdateDataStructures(cubeToHandle.cubeId, ref facedVerticesIndex, cubeLimits,
                    normalVector.X, normalVector.Y, normalVector.Z, constantOfPlaneEq);
        }

        /// <summary>
        /// Create a navigation mesh by combining trianles that occupy a single cube. Plane of cube is 
        /// created and neighbors of the cube are assigned.
        /// </summary>
        public void createNavigationMeshMergingTriangles()
        {
            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();
            var allCubeIds = cubePointCloudVertices.Keys;
            Point3D[] trianglePoints = new Point3D[3];
            Cube cubeToHandle;
            List<Point3D> pointCloudVerticesOfCube;
            string[,,] neighbors;
            int i = 0, j = 0, k = 0;
            Dictionary<string, Cube> listOfNeighbors;
            Dictionary<ConnectOfCube, string> listOfNeighborsConnection;
            Cube neighborCubeToHandle;
            string neighborCubeId;

            int neighborhoodSize = 3;
            Vector3D normalVector = new Vector3D(0,0,0);
            Point3D allTrianglesMergePoint = new Point3D(0, 0, 0);

            // Set the cube size according to the info on the textBox:
            cubeSize = Double.Parse(MainWindow.cubeStructureSize);
            int cubeTriangleCount = 0;
            int h = 0;

            foreach (string cubeId in allCubeIds)
            {
                // Get the list of all raw point cloud vertices of the cube:
                pointCloudVerticesOfCube = cubePointCloudVertices[cubeId];
                cubeToHandle = allCubes[cubeId];
                trianglePoints = new Point3D[3];

                cubeTriangleCount = cubeToHandle.triangles.Count;

                foreach (int triangleId in cubeToHandle.triangleNormalVectors.Keys)
                {
                    normalVector.X += cubeToHandle.triangleNormalVectors[triangleId].X;
                    normalVector.Y += cubeToHandle.triangleNormalVectors[triangleId].Y;
                    normalVector.Z += cubeToHandle.triangleNormalVectors[triangleId].Z;
                }

                normalVector = Vector3D.Divide(normalVector, cubeTriangleCount);
                cubeToHandle.cubeNormalVector = normalVector;

                cubeToHandle.hasPlane = true;

                if (h < 150)
                {
                    //informationTextBlock.Text += "KKSD1: " + normalVector.ToString() + "\n";
                    //informationTextBlock.Text += "KKSD2: " + cubeToHandle.cubeNormalVector.ToString() + "\n";
                    h++;
                }

                //calculatePlaneOfCube(cubeToHandle, ref facedVerticesIndex, pointCloudVerticesOfCube);
            }

            // Find the neighbors of the cube:
            foreach (string cubeId in allCubeIds)
            {
                cubeToHandle = allCubes[cubeId];

                if (cubeToHandle.hasPlane)
                {
                    neighbors = findAllNeighborKeys3DArray(cubeToHandle, neighborhoodSize);
                    listOfNeighborsConnection = new Dictionary<ConnectOfCube, string>();
                    listOfNeighbors = new Dictionary<string, Cube>();
                    
                    // Search through all possible neighbors:
                    for (i = 0; i < neighborhoodSize; i++)
                    {
                        for (j = 0; j < neighborhoodSize; j++)
                        {
                            for (k = 0; k < neighborhoodSize; k++)
                            {
                                neighborCubeId = neighbors[i, j, k];

                                if (allCubes.ContainsKey(neighborCubeId))
                                {
                                    // Neighbor found:
                                    neighborCubeToHandle = allCubes[neighborCubeId];
                                    if (neighborCubeToHandle.hasPlane)
                                    {
                                        listOfNeighbors.Add(neighborCubeId, neighborCubeToHandle);
                                        listOfNeighborsConnection.Add(connectionToNeighboringCubes[i, j, k],
                                            neighborCubeId);
                                    }
                                }
                            }
                        }
                    }
                    
                    cubeToHandle.neighbors = listOfNeighbors;
                    cubeToHandle.neighborsConnectionType = listOfNeighborsConnection;
                    cubeNeighbors.Add(cubeId, listOfNeighborsConnection);
                }
            }
        }
        
        /// <summary>
        /// Create a mesh that is simplified from the vertex grid that we have populated
        /// before. Planes are created at each grid, which are estimated based on the different
        /// vertices present. These planes are then connected to create a mesh.
        /// </summary>
        public void createNavigationMesh()
        {
            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();
            var allCubeIds = cubePointCloudVertices.Keys;
            Point3D[] trianglePoints = new Point3D[3];
            int facedVerticesIndex = 0;
            double[] cubeLimits = new double[6];
            Cube cubeToHandle;
            List<Point3D> pointCloudVerticesOfCube;
            string[,,] neighbors;
            int i = 0, j = 0, k = 0;
            Dictionary<string, Cube> listOfNeighbors;
            Dictionary<ConnectOfCube, string> listOfNeighborsConnection;
            Cube neighborCubeToHandle;
            string neighborCubeId;

            // Set the cube size according to the info on the textBox:
            cubeSize = Double.Parse(MainWindow.cubeStructureSize);
            
            foreach (string cubeId in allCubeIds)
            {
                // Get the list of all raw point cloud vertices of the cube:
                pointCloudVerticesOfCube = cubePointCloudVertices[cubeId];

                // If we have at least 3 points (minimum required to generate a plane),
                // determine the plane of the cube:
               
                if (pointCloudVerticesOfCube.Count >= 3)
                {
                    cubeToHandle = allCubes[cubeId];

                    // Get the cube delimiters:
                    cubeLimits[0] = cubeToHandle.xFloor;
                    cubeLimits[1] = cubeToHandle.xCeiling;
                    cubeLimits[2] = cubeToHandle.yFloor;
                    cubeLimits[3] = cubeToHandle.yCeiling;
                    cubeLimits[4] = cubeToHandle.zFloor;
                    cubeLimits[5] = cubeToHandle.zCeiling;
                    
                    trianglePoints = new Point3D[3];

                    calculatePlaneOfCube(cubeToHandle, ref facedVerticesIndex, pointCloudVerticesOfCube);

                    //calculatePlaneOfCubePointMerging(cubeToHandle, ref facedVerticesIndex, pointCloudVerticesOfCube);

                    //calculatePlaneOfCubeNormalsMerging(cubeToHandle, ref facedVerticesIndex, pointCloudVerticesOfCube);
                }
                
 /*
                cubeToHandle = allCubes[cubeId];

                // Get the cube delimiters:
                cubeLimits[0] = cubeToHandle.xFloor;
                cubeLimits[1] = cubeToHandle.xCeiling;
                cubeLimits[2] = cubeToHandle.yFloor;
                cubeLimits[3] = cubeToHandle.yCeiling;
                cubeLimits[4] = cubeToHandle.zFloor;
                cubeLimits[5] = cubeToHandle.zCeiling;

                trianglePoints = new Point3D[3];

                calculatePlaneOfCube(cubeToHandle, ref facedVerticesIndex, pointCloudVerticesOfCube);
*/

            }
            
            // Find the neighbors of the cube:
            foreach (string cubeId in allCubeIds)
            {
                cubeToHandle = allCubes[cubeId];

                if (cubeToHandle.hasPlane)
                {
                    neighbors = findAllNeighborKeys3DArray(cubeToHandle, 3);
                    listOfNeighborsConnection = new Dictionary<ConnectOfCube, string>();
                    listOfNeighbors = new Dictionary<string, Cube>();

                    // Search through all possible neighbors:
                    for (i = 0; i < 3; i++)
                    {
                        for (j = 0; j < 3; j++)
                        {
                            for (k = 0; k < 3; k++)
                            {
                                neighborCubeId = neighbors[i, j, k];

                                if (allCubes.ContainsKey(neighborCubeId))
                                {
                                    // Neighbor found:
                                    neighborCubeToHandle = allCubes[neighborCubeId];
                                    if (neighborCubeToHandle.hasPlane)
                                    {
                                        listOfNeighbors.Add(neighborCubeId, neighborCubeToHandle);
                                        listOfNeighborsConnection.Add(connectionToNeighboringCubes[i, j, k],
                                            neighborCubeId);
                                    }
                                }
                            }
                        }
                    }

                    cubeToHandle.neighbors = listOfNeighbors;
                    cubeToHandle.neighborsConnectionType = listOfNeighborsConnection;
                    cubeNeighbors.Add(cubeId, listOfNeighborsConnection);
                }
            }
        }



        /// <summary>
        /// Create a mesh that is simplified from the vectex grid that we have populated
        /// before. Planes are created at each grid, which are estimated based on the different
        /// vertices present. These planes are then connected to create a mesh.
        /// </summary>
        public void createSimplifiedMeshFromGrid()
        {
            Dictionary<int, Point3D> containedGridVertices = new Dictionary<int, Point3D>();
            var allCubeIds = cubePointCloudVertices.Keys;
            int verticesCount = 0;
            int closestIndex = 0;
            Point3D[] trianglePoints = new Point3D[3];
            int facedVerticesIndex = 0;
            Vector3D vector1, vector2, normalVector;
            double contstantOfPlaneEq;
            double[] cubeLimits = new double[6];
            Cube cubeToHandle;
            List<Point3D> pointCloudVerticesOfCube;
            string[,,] neighbors;
            int i = 0, j = 0, k = 0;
            Dictionary<string, Cube> listOfNeighbors;
            Dictionary<ConnectOfCube, string> listOfNeighborsConnection;
            Cube neighborCubeToHandle;
            string neighborCubeId;

            foreach (string cubeId in allCubeIds)
            {
                cubeToHandle = allCubes[cubeId];

                // Get the cube delimiters:
                cubeLimits[0] = cubeToHandle.xFloor;
                cubeLimits[1] = cubeToHandle.xCeiling;
                cubeLimits[2] = cubeToHandle.yFloor;
                cubeLimits[3] = cubeToHandle.yCeiling;
                cubeLimits[4] = cubeToHandle.zFloor;
                cubeLimits[5] = cubeToHandle.zCeiling;

                // Get the list of all raw point cloud vertices of the cube:
                pointCloudVerticesOfCube = cubePointCloudVertices[cubeId];
                
                trianglePoints = new Point3D[3];
                verticesCount = 0;
                closestIndex = 0;

                calculatePlaneOfCubePointMerging(cubeToHandle, ref facedVerticesIndex, pointCloudVerticesOfCube);
                //calculatePlaneOfCubeNormalsMerging(cubeToHandle, ref facedVerticesIndex, pointCloudVerticesOfCube);
            }

            int keke = 0;
            foreach (string cubeId in allCubeIds)
            {
                cubeToHandle = allCubes[cubeId];
                if(cubeToHandle.hasPlane)
                {
                    keke++;
                }
            }

            informationTextBlock.Text += "KOKOKOKO " + keke.ToString()  + "\n";
            
            // Find the neighbors of the cube:
            foreach (string cubeId in allCubeIds)
            {
                cubeToHandle = allCubes[cubeId];

                if(cubeToHandle.hasPlane)
                {
                    neighbors = findAllNeighborKeys3DArray(cubeToHandle, 3);
                    listOfNeighborsConnection = new Dictionary<ConnectOfCube, string>();
                    listOfNeighbors = new Dictionary<string, Cube>();

                    // Search through all possible neighbors:
                    for (i = 0; i < 3; i++)
                    {
                        for (j = 0; j < 3; j++)
                        {
                            for (k = 0; k < 3; k++)
                            {
                                neighborCubeId = neighbors[i, j, k];

                                if (allCubes.ContainsKey(neighborCubeId))
                                {
                                    // Neighbor found:
                                    neighborCubeToHandle = allCubes[neighborCubeId];
                                    if (neighborCubeToHandle.hasPlane)
                                    {
                                        listOfNeighbors.Add(neighborCubeId, neighborCubeToHandle);
                                        listOfNeighborsConnection.Add(connectionToNeighboringCubes[i, j, k],
                                            neighborCubeId);
                                    }
                                }
                                /*
                                pairSeperatedKey = neighbors[i, j, k].Split(',');
                                rangeSeperatedKey = pairSeperatedKey[0].Split('/');
                                cubeLimits[0] = Double.Parse(rangeSeperatedKey[0]);
                                cubeLimits[1] = Double.Parse(rangeSeperatedKey[1]);

                                rangeSeperatedKey = pairSeperatedKey[1].Split('/');
                                cubeLimits[2] = Double.Parse(rangeSeperatedKey[0]);
                                cubeLimits[3] = Double.Parse(rangeSeperatedKey[1]);

                                rangeSeperatedKey = pairSeperatedKey[2].Split('/');
                                cubeLimits[4] = Double.Parse(rangeSeperatedKey[0]);
                                cubeLimits[5] = Double.Parse(rangeSeperatedKey[1]);
                                renderViewFunctionalities.renderSingleTransparentCube(pointCloudMesh2,
                                      new Point3D(cubeLimits[1] * cubeSize - (cubeSize / 2),
                                      cubeLimits[3] * cubeSize - (cubeSize / 2),
                                      cubeLimits[5] * cubeSize - (cubeSize / 2)), cubeSize);
                                      */
                            }
                        }
                    }

                    cubeToHandle.neighbors = listOfNeighbors;
                    cubeToHandle.neighborsConnectionType = listOfNeighborsConnection;
                    cubeNeighbors.Add(cubeId, listOfNeighborsConnection);
                }
            }
        }


        /// <summary>
        /// This method is purely for graphical purposes. It will take the intersection verticees 
        /// the cube's plane and its intersection and try to create triangles with all these 
        /// verticees.
        /// </summary>
        public void createTrianglesOfCubePlanes()
        {
            int indexOfTriangles = 0;
            Cube cubeToHandle; 

            foreach (string cubeId in allCubes.Keys)
            {
                cubeToHandle = allCubes[cubeId];
                
                convertPolygonIntoTriangles(new Dictionary<int, Vertex>(cubeToHandle.cubeCrossSectionVertices),
                    cubeToHandle.cubeCrossSectionVertices.Count, ref indexOfTriangles, cubeId);
            }
        }

        /// <summary>
        /// This method will look at the normal vector of the cube plane and determine whether the 
        /// area under that cube is accessible or not.
        /// </summary>
        public void labelAccessibleCubes()
        {
            Cube cubeToHandle, neighborCubeToHandle;
            Vector3D  verticalVector = new Vector3D(0,1,0);

            int accessableCubes = 0;
            foreach (string cubeId in allCubes.Keys)
            {
                cubeToHandle = allCubes[cubeId];
                
                if (cubeToHandle.hasPlane)
                {
                    // Lets compare the cube normal vector against the vertical vector.
                    if (Vector3D.AngleBetween(verticalVector, cubeToHandle.cubeNormalVector)
                            < maximumAngleAllowed || Vector3D.AngleBetween(verticalVector,
                            Vector3D.Multiply(cubeToHandle.cubeNormalVector, -1)) < maximumAngleAllowed)
                    {
                        cubeToHandle.accessabilityType = CubeAccesabilityType.CANDIDATE;
                        cubeToHandle.useType = CubeUseType.REGION_FILL;
                        accessableCubes++;
                    } else
                    {
                        cubeToHandle.accessabilityType = CubeAccesabilityType.UNACCESSABLE;
                        cubeToHandle.useType = CubeUseType.REGION_NONE;
                    }
                }
            }

            informationTextBlock.Text += "Accessable Cubes: " + accessableCubes + "\n";

            // Find region outline candidates:
            foreach (string cubeId in allCubes.Keys)
            {
                cubeToHandle = allCubes[cubeId];

                if(cubeToHandle.useType == CubeUseType.REGION_FILL)
                {
                    foreach (string neighborId in cubeToHandle.neighbors.Keys)
                    {
                        neighborCubeToHandle = cubeToHandle.neighbors[neighborId];
                        if (neighborCubeToHandle.accessabilityType == CubeAccesabilityType.UNACCESSABLE)
                        {
                            cubeToHandle.useType = CubeUseType.REGION_OUTLINE_CANDIDATE;
                            break;
                        }
                    }
                }
            }

            int filledCount = 0;

            // Find actual region outlines:
            foreach (string cubeId in allCubes.Keys)
            {
                filledCount = 0;
                cubeToHandle = allCubes[cubeId];

                if (cubeToHandle.useType == CubeUseType.REGION_OUTLINE_CANDIDATE)
                {
                    foreach (string neighborId in cubeToHandle.neighbors.Keys)
                    {
                        neighborCubeToHandle = cubeToHandle.neighbors[neighborId];
                        if (neighborCubeToHandle.useType == CubeUseType.REGION_FILL)
                        {
                            filledCount++;
                            cubeToHandle.useType = CubeUseType.REGION_OUTLINE;
                            break;
                        }
                    }
                }
            }
        }
        
        public void runConvertPolygonIntoTrianglesTest()
        {
            List<Point3D> pointCloudVerticesOfCube = new List<Point3D>();
            List<Vector3D> listOfNormalVectors = new List<Vector3D>();
            Vector3D normalVector;
            double normalVectorXAverage = 0;
            double normalVectorYAverage = 0;
            double normalVectorZAverage = 0;
            int verticesCount = 0;

            pointCloudVerticesOfCube.Add(new Point3D(1,0,0));
            pointCloudVerticesOfCube.Add(new Point3D(-1,0,0));
            pointCloudVerticesOfCube.Add(new Point3D(1,1,0));
            pointCloudVerticesOfCube.Add(new Point3D(-1,1,0));
            pointCloudVerticesOfCube.Add(new Point3D(1, -1, 0));
            pointCloudVerticesOfCube.Add(new Point3D(-1, -1, 0));

            pointCloudVerticesOfCube.Clear();

            pointCloudVerticesOfCube.Add(new Point3D(1.1, -.1, 0));
            pointCloudVerticesOfCube.Add(new Point3D(-1, 0, 0));
            pointCloudVerticesOfCube.Add(new Point3D(1.2, 0, -1));
            pointCloudVerticesOfCube.Add(new Point3D(-1, -.2, -1));
            pointCloudVerticesOfCube.Add(new Point3D(1.05, 0, 1));
            pointCloudVerticesOfCube.Add(new Point3D(-1, -.11, 1));
            pointCloudVerticesOfCube.Add(new Point3D(0.3, .2, 1));
            pointCloudVerticesOfCube.Add(new Point3D(0, .12, 0));
            pointCloudVerticesOfCube.Add(new Point3D(0.1, -.1, -1));

            pointCloudVerticesOfCube.Clear();

            pointCloudVerticesOfCube.Add(new Point3D(1, 0, 0.0001));
            pointCloudVerticesOfCube.Add(new Point3D(-1, 0, 0.0002));
            pointCloudVerticesOfCube.Add(new Point3D(1, 1, -0.0001));
            pointCloudVerticesOfCube.Add(new Point3D(-1, 1, -0.0002));
            pointCloudVerticesOfCube.Add(new Point3D(1, -1, -0.00015));
            pointCloudVerticesOfCube.Add(new Point3D(-1, -1, 0));

            
            pointCloudVerticesOfCube.Clear();

            
            pointCloudVerticesOfCube.Add(new Point3D(-1, 0, 0));
            pointCloudVerticesOfCube.Add(new Point3D(0, 0, 1));
            pointCloudVerticesOfCube.Add(new Point3D(1, 0, 1));
            pointCloudVerticesOfCube.Add(new Point3D(-1, 0, 1));
            pointCloudVerticesOfCube.Add(new Point3D(0, 0, 0));
            pointCloudVerticesOfCube.Add(new Point3D(1, 1, -1));
            pointCloudVerticesOfCube.Add(new Point3D(-1, 1, -1));
            pointCloudVerticesOfCube.Add(new Point3D(0, 1, -1));
            pointCloudVerticesOfCube.Add(new Point3D(1, 0, 0));

            /*
            foreach (Point3D point1 in pointCloudVerticesOfCube)
            {
                foreach (Point3D point2 in pointCloudVerticesOfCube)
                {
                    if (point1 != point2)
                    {
                        foreach (Point3D point3 in pointCloudVerticesOfCube)
                        {
                            if (point3 != point1 && point3 != point2)
                            {
                                listOfNormalVectors.Add(MathAncillary.getNormalVectorOfTriangle(point1, point2, point3));
                            }
                        }
                    }
                }
            }
            */

            Point3D point1 = pointCloudVerticesOfCube.Last(), point2 = new Point3D(), point3 = new Point3D();
            // Go through each point and create a number of normal vectors:
            
            foreach (Point3D vertexPoint in pointCloudVerticesOfCube)
            {
                verticesCount++;

                if (verticesCount == 1)
                {
                    point1 = vertexPoint;
                }

                if (verticesCount == 2)
                {
                    point2 = point1;
                    point1 = vertexPoint;
                }

                if (verticesCount > 2)
                {
                    point3 = point2;
                    point2 = point1;
                    point1 = vertexPoint;

                    listOfNormalVectors.Add(MathAncillary.getNormalVectorOfTriangle(point1, point2, point3));
                }
            }


            //listOfNormalVectors.Clear();
            //listOfNormalVectors.Add(new Vector3D(0,1,1));
            //listOfNormalVectors.Add(new Vector3D(0,-1,1));

            // Get the average normal vector:
            foreach (Vector3D eachNormalVector in listOfNormalVectors)
            {
                normalVectorXAverage += eachNormalVector.X;
                normalVectorYAverage += eachNormalVector.Y;
                normalVectorZAverage += eachNormalVector.Z;
            }

            normalVector = new Vector3D(normalVectorXAverage / listOfNormalVectors.Count,
                normalVectorYAverage / listOfNormalVectors.Count,
                normalVectorZAverage / listOfNormalVectors.Count);

            informationTextBlock.Text += normalVector.X.ToString("n6") + ", " + normalVector.Y.ToString("n6") +
                ", " + normalVector.Z.ToString("n6") + "\n";

        }
        
        public int findClosestVertex(Vertex point, ref Dictionary<int, Vertex> pointsToCompare)
        {
            int closestVertexId = 0;
            double distance = 0;
            double minDistance = 100000;
            double dx, dy, dz;
            Vertex vertexToCompare;

            var keys = pointsToCompare.Keys;

            // The Dictionary does not contain the point that we wish to find the closest 
            // point of:
            foreach (int vertex in keys)
            {
                vertexToCompare = pointsToCompare[vertex];
                distance = 0;
                dx = point.vertexPosition.X - vertexToCompare.vertexPosition.X;
                dy = point.vertexPosition.Y - vertexToCompare.vertexPosition.Y;
                dz = point.vertexPosition.Z - vertexToCompare.vertexPosition.Z;

                distance = Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestVertexId = vertex;
                }
            }

            return closestVertexId;
        }

        public int[] orderVerticesOfPolygon(Dictionary<int, Vertex> verticesOfPolygon, int numberOfVertices)
        {
            Dictionary<int, int[]> listOfTwoClosestVertices = new Dictionary<int, int[]>();
            Dictionary<int, Vertex> verticesOfPolygonToHandle = new Dictionary<int, Vertex>(verticesOfPolygon);
            int closestVertexId, secondClosestVertexId;
            Vertex currentVertex, closestVertex, secondClosestVertex;
            int count = 0;
            int startingVertexId = 0;
            List<int> orderedVertices = new List<int>();
            List<int> verticesBank = new List<int>();
            bool closestEquals = false, secondClosestEquals = false;

            foreach (int currentVertexId in verticesOfPolygon.Keys)
            {
                verticesBank.Add(currentVertexId);

                currentVertex = verticesOfPolygonToHandle[currentVertexId];
                verticesOfPolygonToHandle.Remove(currentVertexId);

                closestVertexId = findClosestVertex(currentVertex, ref verticesOfPolygonToHandle);
                closestVertex = verticesOfPolygonToHandle[closestVertexId];
                verticesOfPolygonToHandle.Remove(closestVertexId);

                secondClosestVertexId = findClosestVertex(currentVertex, ref verticesOfPolygonToHandle);
                secondClosestVertex = verticesOfPolygonToHandle[secondClosestVertexId];
                verticesOfPolygonToHandle.Remove(secondClosestVertexId);

                // Save the 2 closest vertices:
                listOfTwoClosestVertices.Add(currentVertexId, new int[2] { closestVertexId, secondClosestVertexId });

                // Add back the 3 vertices:
                verticesOfPolygonToHandle.Add(currentVertexId, currentVertex);
                verticesOfPolygonToHandle.Add(closestVertexId, closestVertex);
                verticesOfPolygonToHandle.Add(secondClosestVertexId, secondClosestVertex);
            }

            // Find a good starting vertex, the one that does not appear more 
            // than once as the closest vertex:
            foreach (int currentVertexId in verticesOfPolygon.Keys)
            {
                count = 0;
                closestEquals = false;
                secondClosestEquals = false;

                foreach (int compareVertexId in listOfTwoClosestVertices.Keys)
                {
                    if(currentVertexId == listOfTwoClosestVertices[compareVertexId][0])
                    {
                        closestEquals = true;
                        count++;
                    }

                    if (currentVertexId == listOfTwoClosestVertices[compareVertexId][1])
                    {
                        secondClosestEquals = true;
                        count++;
                    }
                }

                // Found a good starting vertex:
                if(count == 0)
                {
                    startingVertexId = currentVertexId;
                    break;
                }

                if (count == 2)
                {
                    if(closestEquals && secondClosestEquals)
                    {
                        startingVertexId = currentVertexId;
                    }
                }

                if (count == 1)
                {
                    startingVertexId = currentVertexId;
                }
            }


            count = numberOfVertices;
            int currVertexId = startingVertexId, potentialNextVertexId = 0;
            verticesBank.Remove(currVertexId);
            orderedVertices.Add(currVertexId);
            count--;
            while (count != 0)
            {
                potentialNextVertexId = listOfTwoClosestVertices[currVertexId][0];
                // Check if closest point available:
                if(verticesBank.Contains(potentialNextVertexId))
                {
                    verticesBank.Remove(potentialNextVertexId);
                    orderedVertices.Add(potentialNextVertexId);
                    count--;

                }
                else
                {
                    potentialNextVertexId = listOfTwoClosestVertices[currVertexId][1];
                    // Check if second closest point available:
                    if (verticesBank.Contains(potentialNextVertexId))
                    {
                        verticesBank.Remove(potentialNextVertexId);
                        orderedVertices.Add(potentialNextVertexId);
                        count--;

                    }
                    else
                    {
                        // Both vertices unavailable, search through unaccessed points for the vertex
                        // that has the current vertex as closest:
                        
                        foreach (int k in verticesBank)
                        {
                            if(currVertexId == listOfTwoClosestVertices[k][0])
                            {
                                potentialNextVertexId = k;
                                verticesBank.Remove(potentialNextVertexId);
                                orderedVertices.Add(potentialNextVertexId);
                                count--;

                                break;
                            } else if (currVertexId == listOfTwoClosestVertices[k][1])
                            {
                                potentialNextVertexId = k;
                                verticesBank.Remove(potentialNextVertexId);
                                orderedVertices.Add(potentialNextVertexId);
                                count--;

                                break;
                            }
                        }
                        
                    }
                }
                currVertexId = potentialNextVertexId;
            }
            
            return orderedVertices.ToArray();
        }
        
        /// <summary>
        /// 
        /// </summary>
        /// <param name="verticesOfPolygon"></param>
        /// <param name="numberOfVertices"></param>
        /// <param name="triangleIndex"></param>
        /// <param name="cubeId"></param>
        public void convertPolygonIntoTriangles(Dictionary<int, Vertex> verticesOfPolygon, int numberOfVertices,
           ref int triangleIndex, string cubeId)
        {
            Vertex[] triangleVertices;
            Vertex currentVertex, closestVertex, secondClosestVertex;
            int i = 0, vertexId, closestVertexId, secondClosestVertexId;
            Triangle triangleToHandle;
            Cube cubeToHandle = allCubes[cubeId];
            Dictionary<int, Triangle> cubeTriangles = cubeToHandle.triangles;

            if (numberOfVertices == 3 || numberOfVertices == 6)
            {
                triangleVertices = new Vertex[6];

                foreach (int vertexIds in verticesOfPolygon.Keys)
                {
                    triangleVertices[i] = verticesOfPolygon[vertexIds];
                    i++;
                }

                triangleToHandle = new Triangle(triangleIndex, triangleVertices[0],
                    triangleVertices[1], triangleVertices[2]);

                triangleToHandle.normalVector = MathAncillary.getNormalVectorOfTriangle(
                    triangleVertices[0], triangleVertices[1], triangleVertices[2]);

                cubeTriangles.Add(triangleIndex++, triangleToHandle);
            }
            else if (numberOfVertices == 4)
            {
                /* How about finding the farthest point?*/

                // Get any key:
                vertexId = verticesOfPolygon.Keys.First();
                currentVertex = verticesOfPolygon[vertexId];

                // Remove that key from the dictionary:
                verticesOfPolygon.Remove(vertexId);

                // Find closest vertex:
                closestVertexId = findClosestVertex(currentVertex, ref verticesOfPolygon);
                closestVertex = verticesOfPolygon[closestVertexId];

                // Remove the closest point:
                verticesOfPolygon.Remove(closestVertexId);

                // Find the closest point to the closest point:
                int fourthPointId = findClosestVertex(closestVertex, ref verticesOfPolygon);
                Vertex fourthPoint = verticesOfPolygon[fourthPointId];

                // Find second closest vertex to the current vertex:
                secondClosestVertexId = findClosestVertex(currentVertex, ref verticesOfPolygon);
                secondClosestVertex = verticesOfPolygon[secondClosestVertexId];

                if (fourthPoint == secondClosestVertex)
                {
                    // Make the first triangle:
                    triangleVertices = new Vertex[3];
                    triangleVertices[0] = currentVertex;
                    triangleVertices[1] = closestVertex;
                    triangleVertices[2] = secondClosestVertex;
                    triangleToHandle = new Triangle(triangleIndex, triangleVertices[0],
                        triangleVertices[1], triangleVertices[2]);
                    triangleToHandle.normalVector = MathAncillary.getNormalVectorOfTriangle(
                    triangleVertices[0], triangleVertices[1], triangleVertices[2]);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);

                    // Remove the second closest point:
                    verticesOfPolygon.Remove(secondClosestVertexId);

                    // Find the fourth point:
                    fourthPointId = findClosestVertex(closestVertex, ref verticesOfPolygon);
                    fourthPoint = verticesOfPolygon[fourthPointId];

                    // Make the second triangle:
                    triangleVertices = new Vertex[3];
                    triangleVertices[0] = currentVertex;
                    triangleVertices[1] = secondClosestVertex;
                    triangleVertices[2] = fourthPoint;
                    triangleToHandle = new Triangle(triangleIndex, triangleVertices[0],
                        triangleVertices[1], triangleVertices[2]);
                    triangleToHandle.normalVector = MathAncillary.getNormalVectorOfTriangle(
                    triangleVertices[0], triangleVertices[1], triangleVertices[2]);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);
                } else
                {
                    // Make the first triangle:
                    triangleVertices = new Vertex[3];
                    triangleVertices[0] = currentVertex;
                    triangleVertices[1] = closestVertex;
                    triangleVertices[2] = fourthPoint;
                    triangleToHandle = new Triangle(triangleIndex, triangleVertices[0],
                        triangleVertices[1], triangleVertices[2]);
                    triangleToHandle.normalVector = MathAncillary.getNormalVectorOfTriangle(
                    triangleVertices[0], triangleVertices[1], triangleVertices[2]);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);

                    // Make a triangle with the fourth and last point:
                    triangleVertices = new Vertex[3];
                    triangleVertices[0] = currentVertex;
                    triangleVertices[1] = secondClosestVertex;
                    triangleVertices[2] = fourthPoint;
                    triangleToHandle = new Triangle(triangleIndex, triangleVertices[0],
                        triangleVertices[1], triangleVertices[2]);
                    triangleToHandle.normalVector = MathAncillary.getNormalVectorOfTriangle(
                    triangleVertices[0], triangleVertices[1], triangleVertices[2]);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);
                }
            }

            else if(numberOfVertices == 5)
            {
                int[] orderedList = orderVerticesOfPolygon(verticesOfPolygon, numberOfVertices);
                
                if(orderedList.Length == 5)
                {
                    currentVertex = cubeToHandle.cubeCrossSectionVertices[orderedList[0]];
                    closestVertex = cubeToHandle.cubeCrossSectionVertices[orderedList[1]];
                    secondClosestVertex = cubeToHandle.cubeCrossSectionVertices[orderedList[2]];
                    triangleToHandle = new Triangle(triangleIndex, currentVertex,
                            closestVertex, secondClosestVertex);
                    triangleToHandle.normalVector = MathAncillary.getNormalVectorOfTriangle(
                    currentVertex, closestVertex, secondClosestVertex);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);

                    closestVertex = cubeToHandle.cubeCrossSectionVertices[orderedList[3]];
                    triangleToHandle = new Triangle(triangleIndex, currentVertex,
                            closestVertex, secondClosestVertex);
                    triangleToHandle.normalVector = MathAncillary.getNormalVectorOfTriangle(
                            currentVertex, closestVertex, secondClosestVertex);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);

                    secondClosestVertex = cubeToHandle.cubeCrossSectionVertices[orderedList[4]];
                    triangleToHandle = new Triangle(triangleIndex, currentVertex,
                            closestVertex, secondClosestVertex);
                    triangleToHandle.normalVector = MathAncillary.getNormalVectorOfTriangle(
                            currentVertex, closestVertex, secondClosestVertex);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);
                }
            }
            /*
            else if (numberOfVertices == 6)
            {
                int[] orderedList = orderVerticesOfPolygon(verticesOfPolygon, numberOfVertices);

                if (orderedList.Length == 6)
                {
                    currentVertex = cubeToHandle.vertices[orderedList[0]];
                    closestVertex = cubeToHandle.vertices[orderedList[1]];
                    secondClosestVertex = cubeToHandle.vertices[orderedList[2]];
                    triangleToHandle = new Triangle(triangleIndex, currentVertex,
                            closestVertex, secondClosestVertex);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);

                    closestVertex = cubeToHandle.vertices[orderedList[3]];
                    triangleToHandle = new Triangle(triangleIndex, currentVertex,
                            closestVertex, secondClosestVertex);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);

                    secondClosestVertex = cubeToHandle.vertices[orderedList[4]];
                    triangleToHandle = new Triangle(triangleIndex, currentVertex,
                            closestVertex, secondClosestVertex);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);

                    closestVertex = cubeToHandle.vertices[orderedList[5]];
                    triangleToHandle = new Triangle(triangleIndex, currentVertex,
                            closestVertex, secondClosestVertex);
                    cubeTriangles.Add(triangleIndex++, triangleToHandle);
                }
            }

            
            else if (numberOfVertices == 5)
            {
                // Get any key:
                vertexId = verticesOfPolygon.Keys.First();
                point = verticesOfPolygon[vertexId];

                // Remove that key from the dictionary:
                verticesOfPolygon.Remove(vertexId);

                // Find closest vertex:
                closestVertexId = findClosestVertex(point, ref verticesOfPolygon);
                closestVertex = verticesOfPolygon[closestVertexId];

                // Remove the closest point:
                verticesOfPolygon.Remove(closestVertexId);

                // Find second closest vertex:
                secondClosestVertexId = findClosestVertex(closestVertex, ref verticesOfPolygon);
                secondClosestPoint = verticesOfPolygon[secondClosestVertexId];

                // Remove the second closest point:
                verticesOfPolygon.Remove(secondClosestVertexId);

                // Make the first triangle:
                triangleVertices = new Vertex[3];
                triangleVertices[0] = vertexId;
                triangleVertices[1] = closestVertexId;
                triangleVertices[2] = secondClosestVertexId;
                cubeTriangles.Add(triangleIndex, triangleVertices);
                planeTrianglesIndicesList.Add(triangleIndex++, triangleVertices);

                // Save the second closest point for later triangle:
                int prevSecondClosestVertexId = secondClosestVertexId;
                Vertex prevSecondClosestPoint = new Vertex(secondClosestPoint);

                // Find the next set of closest points:
                // Find closest vertex:
                closestVertexId = findClosestVertex(point, ref verticesOfPolygon);
                closestVertex = verticesOfPolygon[closestVertexId];

                // Remove the closest point:
                verticesOfPolygon.Remove(closestVertexId);

                // Maybe we dont need to calculate for this point because it would be the last
                // point in the dictionary?
                // Find second closest vertex:
                secondClosestVertexId = findClosestVertex(closestVertex, ref verticesOfPolygon);
                secondClosestPoint = verticesOfPolygon[secondClosestVertexId];

                // Make the second triangle:
                triangleVertices = new int[3];
                triangleVertices[0] = vertexId;
                triangleVertices[1] = closestVertexId;
                triangleVertices[2] = secondClosestVertexId;
                cubeTriangles.Add(triangleIndex, triangleVertices);
                planeTrianglesIndicesList.Add(triangleIndex++, triangleVertices);

                // Make the third triangle:
                triangleVertices = new int[3];
                triangleVertices[0] = vertexId;
                triangleVertices[1] = prevSecondClosestVertexId;
                triangleVertices[2] = secondClosestVertexId;
                cubeTriangles.Add(triangleIndex, triangleVertices);
                planeTrianglesIndicesList.Add(triangleIndex++, triangleVertices);
            }
            else if (numberOfVertices == 6)
            {
                // Get any key:
                vertexId = verticesOfPolygon.Keys.First();
                point = verticesOfPolygon[vertexId];

                // Remove that key from the dictionary:
                verticesOfPolygon.Remove(vertexId);

                // Find closest vertex:
                closestVertexId = findClosestVertex(point, ref verticesOfPolygon);
                closestVertex = verticesOfPolygon[closestVertexId];

                // Remove the closest point:
                verticesOfPolygon.Remove(closestVertexId);

                // Find second closest vertex:
                secondClosestVertexId = findClosestVertex(closestVertex, ref verticesOfPolygon);
                secondClosestPoint = verticesOfPolygon[secondClosestVertexId];

                // Remove the second closest point:
                verticesOfPolygon.Remove(secondClosestVertexId);

                // Make the first triangle:
                triangleVertices = new int[3];
                triangleVertices[0] = vertexId;
                triangleVertices[1] = closestVertexId;
                triangleVertices[2] = secondClosestVertexId;
                cubeTriangles.Add(triangleIndex, triangleVertices);
                planeTrianglesIndicesList.Add(triangleIndex++, triangleVertices);


                // Save the second closest point for later triangle:
                int prevSecondClosestVertexId = secondClosestVertexId;
                Point3D prevSecondClosestPoint = new Point3D(secondClosestPoint.X, secondClosestPoint.Y,
                                                            secondClosestPoint.Z);

                // Find the next set of closest points:
                // Find closest vertex:
                closestVertexId = findClosestVertex(point, ref verticesOfPolygon);
                closestVertex = verticesOfPolygon[closestVertexId];

                // Remove the closest point:
                verticesOfPolygon.Remove(closestVertexId);

                // Find second closest vertex:
                secondClosestVertexId = findClosestVertex(closestVertex, ref verticesOfPolygon);
                secondClosestPoint = verticesOfPolygon[secondClosestVertexId];

                // Remove the second closest point:
                verticesOfPolygon.Remove(secondClosestVertexId);

                // Make the second triangle:
                triangleVertices = new int[3];
                triangleVertices[0] = vertexId;
                triangleVertices[1] = closestVertexId;
                triangleVertices[2] = secondClosestVertexId;
                cubeTriangles.Add(triangleIndex, triangleVertices);
                planeTrianglesIndicesList.Add(triangleIndex++, triangleVertices);

                // Get last point:
                int lastPointVertexId = verticesOfPolygon.Keys.First();
                Point3D lastPoint = verticesOfPolygon[verticesOfPolygon.Keys.First()];

                // Make the third triangle:
                triangleVertices = new int[3];
                triangleVertices[0] = vertexId;
                triangleVertices[1] = secondClosestVertexId;
                triangleVertices[2] = lastPointVertexId;
                cubeTriangles.Add(triangleIndex, triangleVertices);
                planeTrianglesIndicesList.Add(triangleIndex++, triangleVertices);

                // Make the fourth triangle:
                triangleVertices = new int[3];
                triangleVertices[0] = vertexId;
                triangleVertices[1] = prevSecondClosestVertexId;
                triangleVertices[2] = lastPointVertexId;
                cubeTriangles.Add(triangleIndex, triangleVertices);
                planeTrianglesIndicesList.Add(triangleIndex++, triangleVertices);
            }
            */
            cubeToHandle.triangles = cubeTriangles;
        }
        
        public void convertPolygonIntoTriangles2(Dictionary<int, Point3D> pointsOfPolygon, int numberOfVertices,
            ref int triangleIndex, string cubeIndex)
        {
            /*
            Point3D[] trianglePoints;
            Point3D point, closestPoint, secondClosestPoint;
            int i = 0, vertexId, closestVertexId, secondClosestVertexId;

            if (numberOfVertices == 3)
            {
                trianglePoints = new Point3D[3];

                foreach (Point3D p in pointsOfPolygon.Values)
                {
                    trianglePoints[i] = p;
                    i++;
                }

                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);
            } else if (numberOfVertices == 4)
            {

                // Get any key:
                vertexId = pointsOfPolygon.Keys.First();
                point = pointsOfPolygon[vertexId];

                // Remove that key from the dictionary:
                pointsOfPolygon.Remove(vertexId);

                // Find closest vertex:
                closestVertexId = findClosestPoint(point, ref pointsOfPolygon);
                closestPoint = pointsOfPolygon[closestVertexId];

                // Remove the closest point:
                pointsOfPolygon.Remove(closestVertexId);

                // Find the second closest point to the main point:
                int fourthPointId = findClosestPoint(point, ref pointsOfPolygon);
                Point3D fourthPoint = pointsOfPolygon[fourthPointId];

                // Remove the second closest point:
                pointsOfPolygon.Remove(fourthPointId);

                // Find second closest vertex:
                secondClosestVertexId = findClosestPoint(closestPoint, ref pointsOfPolygon);
                secondClosestPoint = pointsOfPolygon[secondClosestVertexId];

                // Remove the second closest point:
                pointsOfPolygon.Remove(secondClosestVertexId);

                // Make the first triangle:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = closestPoint;
                trianglePoints[2] = secondClosestPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);

                // Make a triangle with the fourth and last point:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = secondClosestPoint;
                trianglePoints[2] = fourthPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);
            } else if(numberOfVertices == 5)
            {
                // Get any key:
                vertexId = pointsOfPolygon.Keys.First();
                point = pointsOfPolygon[vertexId];

                // Remove that key from the dictionary:
                pointsOfPolygon.Remove(vertexId);

                // Find closest vertex:
                closestVertexId = findClosestPoint(point, ref pointsOfPolygon);
                closestPoint = pointsOfPolygon[closestVertexId];

                // Remove the closest point:
                pointsOfPolygon.Remove(closestVertexId);

                // Find second closest vertex:
                secondClosestVertexId = findClosestPoint(closestPoint, ref pointsOfPolygon);
                secondClosestPoint = pointsOfPolygon[secondClosestVertexId];

                // Remove the second closest point:
                pointsOfPolygon.Remove(secondClosestVertexId);

                // Make the first triangle:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = closestPoint;
                trianglePoints[2] = secondClosestPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);

                // Save the second closest point for later triangle:
                Point3D prevSecondClosestPoint = new Point3D(secondClosestPoint.X, secondClosestPoint.Y,
                                                            secondClosestPoint.Z);

                // Find the next set of closest points:
                // Find closest vertex:
                closestVertexId = findClosestPoint(point, ref pointsOfPolygon);
                closestPoint = pointsOfPolygon[closestVertexId];

                // Remove the closest point:
                pointsOfPolygon.Remove(closestVertexId);

                // Maybe we dont need to calculate for this point because it would be the last
                // point in the dictionary?
                // Find second closest vertex:
                secondClosestVertexId = findClosestPoint(closestPoint, ref pointsOfPolygon);
                secondClosestPoint = pointsOfPolygon[secondClosestVertexId];

                // Make the second triangle:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = closestPoint;
                trianglePoints[2] = secondClosestPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);

                // Make the third triangle:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = prevSecondClosestPoint;
                trianglePoints[2] = secondClosestPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);
            }
            else if (numberOfVertices == 6)
            {
                // Get any key:
                vertexId = pointsOfPolygon.Keys.First();
                point = pointsOfPolygon[vertexId];

                // Remove that key from the dictionary:
                pointsOfPolygon.Remove(vertexId);

                // Find closest vertex:
                closestVertexId = findClosestPoint(point, ref pointsOfPolygon);
                closestPoint = pointsOfPolygon[closestVertexId];

                // Remove the closest point:
                pointsOfPolygon.Remove(closestVertexId);

                // Find second closest vertex:
                secondClosestVertexId = findClosestPoint(closestPoint, ref pointsOfPolygon);
                secondClosestPoint = pointsOfPolygon[secondClosestVertexId];

                // Remove the second closest point:
                pointsOfPolygon.Remove(secondClosestVertexId);

                // Make the first triangle:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = closestPoint;
                trianglePoints[2] = secondClosestPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);

                // Save the second closest point for later triangle:
                Point3D prevSecondClosestPoint = new Point3D(secondClosestPoint.X, secondClosestPoint.Y,
                                                            secondClosestPoint.Z);

                // Find the next set of closest points:
                // Find closest vertex:
                closestVertexId = findClosestPoint(point, ref pointsOfPolygon);
                closestPoint = pointsOfPolygon[closestVertexId];

                // Remove the closest point:
                pointsOfPolygon.Remove(closestVertexId);
                
                // Find second closest vertex:
                secondClosestVertexId = findClosestPoint(closestPoint, ref pointsOfPolygon);
                secondClosestPoint = pointsOfPolygon[secondClosestVertexId];

                // Remove the second closest point:
                pointsOfPolygon.Remove(secondClosestVertexId);

                // Make the second triangle:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = closestPoint;
                trianglePoints[2] = secondClosestPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);
                
                // Get last point:
                Point3D lastPoint = pointsOfPolygon[pointsOfPolygon.Keys.First()];

                // Make the third triangle:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = secondClosestPoint;
                trianglePoints[2] = lastPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);

                // Make the fourth triangle:
                trianglePoints = new Point3D[3];
                trianglePoints[0] = point;
                trianglePoints[1] = prevSecondClosestPoint;
                trianglePoints[2] = lastPoint;
                cubeTriangles[cubeIndex].Add(triangleIndex, trianglePoints);
                planeTrianglesList.Add(triangleIndex++, trianglePoints);
            }
*/
        }

        private int findClosestTriangleVertex(Point3D[] pointArr, Point3D point)
        {
            double distance1, distance2, distance3;
            double X, Y, Z;

            X = point.X - pointArr[0].X;
            Y = point.Y - pointArr[0].Y;
            Z = point.Z - pointArr[0].Z;

            distance1 = Math.Sqrt(X * X + Y * Y + Z * Z);

            X = point.X - pointArr[1].X;
            Y = point.Y - pointArr[1].Y;
            Z = point.Z - pointArr[1].Z;

            distance2 = Math.Sqrt(X * X + Y * Y + Z * Z);

            X = point.X - pointArr[2].X;
            Y = point.Y - pointArr[2].Y;
            Z = point.Z - pointArr[2].Z;

            distance3 = Math.Sqrt(X * X + Y * Y + Z * Z);

            if (distance1 < distance2 && distance1 < distance3)
            {
                return 0;
            }
            else
            {
                if (distance2 < distance3)
                {
                    return 1;
                }
                else
                {
                    return 2;
                }
            }
        }

        private Point3D mergeTwoPoints(Point3D point1, Point3D point2)
        {
            Point3D mergedPoint = new Point3D();
            double X, Y, Z;

            X = (point1.X - point2.X) / 2;
            Y = (point1.Y - point2.Y) / 2;
            Z = (point1.Z - point2.Z) / 2;

            mergedPoint = new Point3D(point2.X + X, point2.Y + Y, point2.Z + Z);

            return mergedPoint;
        }

        /// <summary>
        /// Extract vertices and triangles from their designated files.
        /// </summary>
        /// <param name="triangles"></param>
        /// <param name="vertices"></param>
        public void extractMeshDataFromFiles()
        {
            triangles = new List<string>();
            vertices = new List<string>();
            
            string path = Path.GetDirectoryName(Path.GetDirectoryName(System.IO.Directory.GetCurrentDirectory()));
            path += "/Data Files/";

            // Get triangle information from file:
            path += MainWindow.example3DModelTrianglesName;
            string line;

            StreamReader file =
            new StreamReader(path);
            while ((line = file.ReadLine()) != null)
            {
                triangles.Add(line);
            }

            file.Close();
            
            // Get verticees information from file:
            path = Path.GetDirectoryName(Path.GetDirectoryName(System.IO.Directory.GetCurrentDirectory()));
            path += "/Data Files/";
            path += MainWindow.example3DModelVerticeesName;

            StreamReader file2 =
            new StreamReader(path);
            while ((line = file2.ReadLine()) != null)
            {
                vertices.Add(line);
            }

            file2.Close();
        }

        /// <summary>
        /// Extract vertices and triangles from their designated files.
        /// </summary>
        /// <param name="triangles"></param>
        /// <param name="vertices"></param>
        public void extractDepthDataFromFiles()
        {
            string path = Path.GetDirectoryName(Path.GetDirectoryName(System.IO.Directory.GetCurrentDirectory()));
            path += "/Data Files/";
            path += MainWindow.rawDepthDataFileName;

            string line;
            int i = 0, limit = 512 * 424;

            StreamReader file =
            new StreamReader(path);
            while ((line = file.ReadLine()) != null && i != limit)
            {
                depthDataFromFile[i++] = Double.Parse(line);
            }

            file.Close();
        }
        

        /// <summary>
        /// Extract vertices and triangles from their designated files.
        /// </summary>
        /// <param name="triangles"></param>
        /// <param name="vertices"></param>
        public void extractSavedPointCloud()
        {
            cubeSize = Double.Parse(MainWindow.cubeStructureSize);

            string path = Path.GetDirectoryName(Path.GetDirectoryName(System.IO.Directory.GetCurrentDirectory()));
            path += "/Data Files/";
            path += MainWindow.pointCloudFileName;

            string line;
            string[] splitParts;

            StreamReader file =
            new StreamReader(path);
            while ((line = file.ReadLine()) != null)
            {
                splitParts = line.Split(',');
                pointCloudVertices.Add(new Point3D(-1 * Double.Parse(splitParts[0]),
                    -1 * Double.Parse(splitParts[1]), -1 * Double.Parse(splitParts[2])));
                /*
                pointCloudVertices.Add(new Point3D(-1 * Double.Parse(splitParts[0]),
                    -1 * Double.Parse(splitParts[1]), -1 * Double.Parse(splitParts[2])));
                */
            }

            file.Close();
        }


        public void saveRawDepthToFile()
        {
            throw new NotImplementedException();
        }

        public void savePointCloudToFile()
        {
            int limit = vertices.Count;
            int k = 0;
            Byte[] info;
            double X = 0.0;
            double Y = 0.0;
            double Z = 0.0;
            string[] gridLimitsStr = new string[6];

            string path = Path.GetDirectoryName(Path.GetDirectoryName(System.IO.Directory.GetCurrentDirectory()));
            path += "/Data Files/";
            path += "/pointCloudSingleSave.txt";

            using (FileStream fs = File.Open(path, FileMode.Open, FileAccess.Write, FileShare.None))
            {
                foreach(Point3D point in pointCloudVertices)
                {
                    info = new UTF8Encoding(true).GetBytes(point.X.ToString() + "," + point.Y.ToString() +
                        "," + point.Z.ToString() + "\n");
                    fs.Write(info, 0, info.Length);
                }
            }
        }


        public void createPointCloudActualMeshRealsense()
        {
            Dictionary<int, int> columnList;
            int newPointIndex = 0;
            
            for (int i = 0; i < RealsenseHeight; i++)    
            {
                columnList = new Dictionary<int, int>();
                for (int j = 0; j < RealsenseWidth; j++)     
                {
                    pointCloudIndexed.Add(newPointIndex, pointCloudVertices[(i * RealsenseWidth) + j]);
                    columnList.Add(j, newPointIndex++);
                }
                indexedPoints.Add(i, columnList);
            }
            
        }




        public void runPlaneTest()
        {
            char splitter = ' ';
            Stopwatch sWatch = new Stopwatch();
            sWatch.Reset();
            sWatch.Start();

            string vertecesLine = "";
            string[] vertexLineSeperated = new string[3];

            Point3D vertexPointTmp = new Point3D();

            int vertexIndex = 0;

            List<Vertex> verticeesList = new List<Vertex>();

            // Get the verticees:
            foreach (string vertexLine in vertices)
            {
                vertexLineSeperated = vertexLine.Split(splitter);
                vertexPointTmp = new Point3D(Double.Parse(vertexLineSeperated[0]), Double.Parse(vertexLineSeperated[1]),
                    Double.Parse(vertexLineSeperated[2]));
                verticeesList.Add(new Vertex(vertexIndex++, vertexPointTmp));
            }

            // Create the initial triangle:
            Vertex A = verticeesList[0];
            Vertex B = verticeesList[1];
            Vertex C = verticeesList[2];
            
            Point3D P1 = new Point3D(-12.614800, -4.068210, -68.850000);
            Point3D P2 = new Point3D(-12.260100, -4.057870, -68.675000);
            Point3D P3 = new Point3D(-12.439500, -4.063780, -68.775000);

            A.vertexPosition = P1;
            B.vertexPosition = P2;
            C.vertexPosition = P3;
            
            //verticeesList.Remove(A);
            //verticeesList.Remove(B);
            //verticeesList.Remove(C);

            double dAx = 0;
            double dAy = 0;
            double dAz = 0;

            double dBx = 0;
            double dBy = 0;
            double dBz = 0;

            double dCx = 0;
            double dCy = 0;
            double dCz = 0;

            double distA = 0;
            double distB = 0;
            double distC = 0;

            int closestPoint = 0;

            int accumulatorA = 0;
            int accumulatorB = 0;
            int accumulatorC = 0;

            verticeesList = new List<Vertex>();
            verticeesList.Add(new Vertex(1, new Point3D(-12.081200, -4.051960, -68.575000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.790400, -4.072640, -68.925000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.067900, -4.208430, -68.500000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.246700, -4.214570, -68.600000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.421400, -4.219180, -68.675000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.601000, -4.225320, -68.775000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.776500, -4.229930, -68.850000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.952300, -4.234540, -68.925000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.045900, -4.361360, -68.375000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.228800, -4.369330, -68.500000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.403300, -4.374110, -68.575000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.582700, -4.380490, -68.675000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.762600, -4.386870, -68.775000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.938200, -4.391650, -68.850000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.028300, -4.515350, -68.275000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.206500, -4.521960, -68.375000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.385200, -4.528570, -68.475000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.564400, -4.535190, -68.575000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.744000, -4.541800, -68.675000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.924100, -4.548420, -68.775000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.006300, -4.667160, -68.150000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.188700, -4.675720, -68.275000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.371700, -4.684280, -68.400000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.550600, -4.691130, -68.500000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.734700, -4.699690, -68.625000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.914700, -4.706540, -68.725000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.175300, -4.830780, -68.200000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.358100, -4.839630, -68.325000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.541500, -4.848490, -68.450000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.720800, -4.855570, -68.550000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.905300, -4.864430, -68.675000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.157400, -4.983660, -68.100000)));
            verticeesList.Add(new Vertex(1, new Point3D(-12.344500, -4.994630, -68.250000)));

            foreach (Vertex vert in verticeesList)
            {
                // Calculate distances between triangle points:
                dAx = vert.vertexPosition.X - A.vertexPosition.X;
                dAy = vert.vertexPosition.Y - A.vertexPosition.Y;
                dAz = vert.vertexPosition.Z - A.vertexPosition.Z;

                dBx = vert.vertexPosition.X - B.vertexPosition.X;
                dBy = vert.vertexPosition.Y - B.vertexPosition.Y;
                dBz = vert.vertexPosition.Z - B.vertexPosition.Z;

                dCx = vert.vertexPosition.X - C.vertexPosition.X;
                dCy = vert.vertexPosition.Y - C.vertexPosition.Y;
                dCz = vert.vertexPosition.Z - C.vertexPosition.Z;

                distA = dAx * dAx + dAy * dAy + dAz * dAz;
                distB = dBx * dBx + dBy * dBy + dBz * dBz;
                distC = dCx * dCx + dCy * dCy + dCz * dCz;

                cubeInformationTextBox.Text += "P: " + vert.vertexPosition.X.ToString("n6") + "," +
                           vert.vertexPosition.Y.ToString("n6") + "," + vert.vertexPosition.Z.ToString("n6") + "\n";

                // Find the closest triangle point:
                if (distA < distB && distA < distC)
                {
                    closestPoint = 1;
                    accumulatorA++;

                    dAx /= 2;
                    dAy /= 2;
                    dAz /= 2;

                    dAx /= accumulatorA;
                    dAy /= accumulatorA;
                    dAz /= accumulatorA;

                    A.vertexPosition.X += dAx;
                    A.vertexPosition.Y += dAy;
                    A.vertexPosition.Z += dAz;

                    cubeInformationTextBox.Text += "A: " + A.vertexPosition.X.ToString("n6") + "," +
                           A.vertexPosition.Y.ToString("n6") + "," + A.vertexPosition.Z.ToString("n6") + "\n";
                } else
                {
                    if(distB < distC)
                    {
                        closestPoint = 2;
                        accumulatorB++;

                        dBx /= 2;
                        dBy /= 2;
                        dBz /= 2;

                        dBx /= accumulatorB;
                        dBy /= accumulatorB;
                        dBz /= accumulatorB;

                        B.vertexPosition.X += dBx;
                        B.vertexPosition.Y += dBy;
                        B.vertexPosition.Z += dBz;

                        cubeInformationTextBox.Text += "B: " + B.vertexPosition.X.ToString("n6") + "," +
                           B.vertexPosition.Y.ToString("n6") + "," + B.vertexPosition.Z.ToString("n6") + "\n";
                    } else
                    {
                        closestPoint = 3;
                        accumulatorC++;

                        dCx /= 2;
                        dCy /= 2;
                        dCz /= 2;

                        dCx /= accumulatorC;
                        dCy /= accumulatorC;
                        dCz /= accumulatorC;

                        C.vertexPosition.X += dCx;
                        C.vertexPosition.Y += dCy;
                        C.vertexPosition.Z += dCz;

                        cubeInformationTextBox.Text += "C: " + C.vertexPosition.X.ToString("n6") + "," +
                           C.vertexPosition.Y.ToString("n6") + "," + C.vertexPosition.Z.ToString("n6") + "\n";
                    }
                }
            }

            Vector3D normalVector = Vector3D.CrossProduct(
                new Vector3D(A.vertexPosition.X - B.vertexPosition.X,
                             A.vertexPosition.Y - B.vertexPosition.Y,
                             A.vertexPosition.Z - B.vertexPosition.Z),
                new Vector3D(A.vertexPosition.X - C.vertexPosition.X,
                             A.vertexPosition.Y - C.vertexPosition.Y,
                             A.vertexPosition.Z - C.vertexPosition.Z));

            normalVector.Normalize();
            informationTextBlock.Text = normalVector.ToString() + "\n";

            informationTextBlock.Text += "A Normallleee: " + A.vertexPosition.X.ToString("n6") + "," +
                   A.vertexPosition.Y.ToString("n6") + "," + A.vertexPosition.Z.ToString("n6") + "\n";
            informationTextBlock.Text += "B Normallleee: " + B.vertexPosition.X.ToString("n6") + "," +
                   B.vertexPosition.Y.ToString("n6") + "," + B.vertexPosition.Z.ToString("n6") + "\n";
            informationTextBlock.Text += "C Normallleee: " + C.vertexPosition.X.ToString("n6") + "," +
                   C.vertexPosition.Y.ToString("n6") + "," + C.vertexPosition.Z.ToString("n6") + "\n";

            vertices = new List<string>();
            triangles = new List<string>();

            vertices.Add(A.vertexPosition.X.ToString() + " " + A.vertexPosition.Y.ToString() + " " + A.vertexPosition.Z.ToString());
            vertices.Add(B.vertexPosition.X.ToString() + " " + B.vertexPosition.Y.ToString() + " " + B.vertexPosition.Z.ToString());
            vertices.Add(C.vertexPosition.X.ToString() + " " + C.vertexPosition.Y.ToString() + " " + C.vertexPosition.Z.ToString());

            triangles.Add("1/1/1 2/2/1 3/3/2");
            
            informationTextBlock.Text += "Count: " + verticeesList.Count.ToString() + "\n";
            informationTextBlock.Text += "Time: " + sWatch.ElapsedMilliseconds.ToString() + "\n";
        }
        
    }
}
