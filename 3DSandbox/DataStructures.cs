using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace _3DSandbox
{
    public enum EdgesOfCube { AB, BC, CD, DA, EF, FG, GH, HE, AE, BF, CG, DH }
    public enum ConnectOfCube{
                                AB, BC, CD, DA, EF, FG, GH, HE, AE, BF, CG, DH,
                                ABC, BCD, CDA, DAB, EFG, FGH, GHE, HEF,
                                LEFT, BACK, RIGHT, FRONT, TOP, BOTTOM,
                                NONE
                             }

    public enum TriangleUseType { CUBE_PLANE_ACTUAL, CUBE_PLANE_CONNECTOR, REGION_OUTLINE_CANDIDATE }
    public enum TriangleAccesabilityType { CANDIDATE, UNACCESSABLE, ACCESSABLE }

    public enum CubeUseType { CUBE_PLANE_ACTUAL, CUBE_PLANE_CONNECTOR, REGION_OUTLINE_CANDIDATE, REGION_OUTLINE, REGION_FILL, REGION_NONE }
    public enum CubeAccesabilityType { CANDIDATE, UNACCESSABLE, ACCESSABLE }

    public class Vertex
    {
        public int vertexId;
        public Point3D vertexPosition;

        public Vertex(int id, Point3D point)
        {
            vertexId = id;
            vertexPosition = point;
        }

        public Vertex(Vertex otherVertex)
        {
            this.vertexId = otherVertex.vertexId;
            this.vertexPosition = new Point3D(otherVertex.vertexPosition.X,
                otherVertex.vertexPosition.Y, otherVertex.vertexPosition.Z);
        }
        
        public override string ToString()
        {
            return "Id: " + vertexId.ToString() + " at: (" + vertexPosition.X.ToString("n6") + ", "
                + vertexPosition.Y.ToString("n6") + ", " + vertexPosition.Z.ToString("n6") + ")";
        }

    }

    public class Triangle
    {
        public int triangleId;
        public Vertex vertex1, vertex2, vertex3;
        public Vector3D normalVector;

        public TriangleUseType useType;
        public TriangleAccesabilityType accessabilityType;

        public Triangle(int id, Vertex vertex1, Vertex vertex2, Vertex vertex3)
        {
            triangleId = id;
            this.vertex1 = vertex1;
            this.vertex2 = vertex2;
            this.vertex3 = vertex3;
        }

        public Triangle(int id, Point3D point1, Point3D point2, Point3D point3)
        {
            triangleId = id;

            this.vertex1 = new Vertex(1, point1);
            this.vertex2 = new Vertex(2, point2);
            this.vertex3 = new Vertex(3, point3);
        }

        public Triangle(int id, Point3D[] points)
        {
            triangleId = id;

            this.vertex1 = new Vertex(1, points[0]);
            this.vertex2 = new Vertex(2, points[1]);
            this.vertex3 = new Vertex(3, points[2]);
        }

        public override string ToString()
        {
            return "Id: " + triangleId.ToString() + "\n"
                + "\t" + "Vertices: \n" 
                + "\t" + vertex1.ToString() + "\n" 
                + "\t" + vertex2.ToString() + "\n" 
                + "\t" + vertex3.ToString() + "\n"
                + "\t" + "Normal Vector: (" + normalVector.X.ToString("n6") + ", "
                    + normalVector.Y.ToString("n6") + ", " + normalVector.Z.ToString("n6") + ")" + "\n"
                + "\t" + "Use Type: " + useType.ToString() + "\n"
                + "\t" + "Accessability Type: " + accessabilityType.ToString();
        }

    }

    public class Cube
    {
        public string cubeId;

        public double xFloor;
        public double xCeiling;
        public double yFloor;
        public double yCeiling;
        public double zFloor;
        public double zCeiling;

        public double planeEquationConstant;
        public Vector3D planeEquationNormalVector;
        public Point3D[] planeTrianglePoints;

        
        public bool hasPlane = false;

        public bool[] haveEdgesMerged = new bool[12]{ false, false, false, false, false, false,
            false, false, false, false, false, false};

        /// <summary>
        /// The normal vector of the whole cube which is determined by merging all triangle normals.
        /// </summary>
        public Vector3D cubeNormalVector;

        public CubeUseType useType;
        public CubeAccesabilityType accessabilityType;
        
        /// <summary>
        /// List of all verticees that happen to be at the cross section of the cube and its plane.
        /// </summary>
        public Dictionary<int, Vertex> cubeCrossSectionVertices;

        /// <summary>
        /// List of all cross section verticees by edge type.
        /// </summary>
        public Dictionary<EdgesOfCube, Vertex> crossSectionVerticeesByEdge;

        /// <summary>
        /// The neighbors of the cube.
        /// </summary>
        public Dictionary<string, Cube> neighbors;

        /// <summary>
        /// List of all neighbors of the cube by connection type.
        /// </summary>
        public Dictionary<ConnectOfCube, string> neighborsConnectionType;

        /// <summary>
        /// These are the triangles created from point cloud.
        /// </summary>
        public Dictionary<int, Triangle> triangles;

        /// <summary>
        /// List of normal vectors for all triangles occupying the cube.
        /// </summary>
        public Dictionary<int, Vector3D> triangleNormalVectors;

        /// <summary>
        /// List of all triangles midpoints which are determined by merging the 3 points.
        /// </summary>
        public Dictionary<int, Point3D> triangleMergePoints;

        public Cube(string cubeId, double xFloor, double xCeiling, double yFloor, double yCeiling,
                    double zFloor, double zCeiling)
        {
            this.cubeId = cubeId;

            this.xFloor = xFloor;
            this.xCeiling = xCeiling;
            this.yFloor = yFloor;
            this.yCeiling = yCeiling;
            this.zFloor = zFloor;
            this.zCeiling = zCeiling;

            cubeCrossSectionVertices = new Dictionary<int, Vertex>();
            triangles = new Dictionary<int, Triangle>();
            neighbors = new Dictionary<string, Cube>();
            neighborsConnectionType = new Dictionary<ConnectOfCube, string>();
        }

    }
    
}
