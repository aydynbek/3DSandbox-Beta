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

        public Vector3D averageNormalVector;
        
        public Dictionary<int, Vertex> vertices;
        public Dictionary<EdgesOfCube, Vertex> edges;
        public Dictionary<int, Triangle> triangles;
        public Dictionary<string, Cube> neighbors;
        public Dictionary<ConnectOfCube, string> neighborsConnectionType;

        /*---------*/
        public Dictionary<int, Vector3D> actualMeshTriangleNormalVectors;
        public Dictionary<int, Point3D> actualMeshTrianglePoint;

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

            vertices = new Dictionary<int, Vertex>();
            triangles = new Dictionary<int, Triangle>();
            neighbors = new Dictionary<string, Cube>();
            neighborsConnectionType = new Dictionary<ConnectOfCube, string>();
        }

    }
    
}
