using System;
using System.Collections.Generic;
using System.Text;
using Jitter2D.LinearMath;
using Microsoft.Xna.Framework;

namespace CollisionDemo
{
    public sealed class Conversion
    {
        public static JVector ToJitterVector(Vector2 vector)
        {
            return new JVector(vector.X, vector.Y);
        }

        public static Matrix ToXNAMatrix(JMatrix matrix)
        {
            return new Matrix(matrix.M11,
                            matrix.M12,
                            matrix.M13,
                            0.0f,
                            matrix.M21,
                            matrix.M22,
                            matrix.M23,
                            0.0f,
                            matrix.M31,
                            matrix.M32,
                            matrix.M33,
                            0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
        }

        public static JMatrix ToJitterMatrix(Matrix matrix)
        {
            JMatrix result;
            result.M11 = matrix.M11;
            result.M12 = matrix.M12;
            result.M13 = matrix.M13;
            result.M21 = matrix.M21;
            result.M22 = matrix.M22;
            result.M23 = matrix.M23;
            result.M31 = matrix.M31;
            result.M32 = matrix.M32;
            result.M33 = matrix.M33;
            return result;

        }

        public static Vector2 ToXNAVector2(JVector vector)
        {
            return new Vector2(vector.X, vector.Y);
        }

        public static Vector3 ToXNAVector3(JVector vector)
        {
            return new Vector3(vector.X, vector.Y, 0.0f);
        }
    }
}
