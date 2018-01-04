using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace _3DSandbox
{
    public static class MathAncillary
    {
        public static Vector3D rotateVectorAboutAxis(Vector3D vector, Vector3D axis, double angle, bool isUsingRadians)
        {
            if (isUsingRadians)
            {
                return Vector3D.Multiply(vector, Math.Cos(angle))
               + Vector3D.Multiply((Vector3D.CrossProduct(axis, vector)), Math.Sin(angle))
               + Vector3D.Multiply(axis, (Vector3D.DotProduct(axis, vector) * (1 - Math.Cos(angle))));
            }
            else
            {
                return Vector3D.Multiply(vector, Math.Cos(Math.PI * (angle / 180)))
               + Vector3D.Multiply((Vector3D.CrossProduct(axis, vector)), Math.Sin(Math.PI * (angle / 180)))
               + Vector3D.Multiply(axis, (Vector3D.DotProduct(axis, vector) * (1 - Math.Cos(Math.PI * (angle / 180)))));
            }

        }

        public static Vector3D getNormalVectorOfTriangle(Vertex vertex1, Vertex vertex2, Vertex vertex3)
        {
            Vector3D normalVector = new Vector3D();
            Vector3D vector1 = new Vector3D(vertex2.vertexPosition.X - vertex1.vertexPosition.X,
                                            vertex2.vertexPosition.Y - vertex1.vertexPosition.Y,
                                            vertex2.vertexPosition.Z - vertex1.vertexPosition.Z);
            Vector3D vector2 = new Vector3D(vertex3.vertexPosition.X - vertex1.vertexPosition.X,
                                            vertex3.vertexPosition.Y - vertex1.vertexPosition.Y,
                                            vertex3.vertexPosition.Z - vertex1.vertexPosition.Z);

            normalVector = Vector3D.CrossProduct(vector1, vector2);

            if(normalVector.Y < 0)
            {
                normalVector.X *= -1;
                normalVector.Y *= -1;
                normalVector.Z *= -1;
            } 

            return normalVector;
        }

        public static Vector3D getNormalVectorOfTriangle(Point3D point1, Point3D point2, Point3D point3)
        {
            Vector3D normalVector = new Vector3D();
            Vector3D vector1 = new Vector3D(point2.X - point1.X,
                                            point2.Y - point1.Y,
                                            point2.Z - point1.Z);
            vector1 = Vector3D.Divide(vector1, vector1.Length);

            Vector3D vector2 = new Vector3D(point3.X - point1.X,
                                            point3.Y - point1.Y,
                                            point3.Z - point1.Z);
            vector2 = Vector3D.Divide(vector2, vector2.Length);

            normalVector = Vector3D.CrossProduct(vector1, vector2);

            if (normalVector.X < 0)
            {
                //normalVector.X *= -1;
                //normalVector.Y *= -1;
                //normalVector.Z *= -1;
            }

            normalVector = Vector3D.Divide(normalVector, normalVector.Length);
            return normalVector;
        }

    }
}
