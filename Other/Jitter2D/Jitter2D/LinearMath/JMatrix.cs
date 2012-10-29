/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;
#endregion

namespace Jitter2D.LinearMath
{

    /// <summary>
    /// 3x3 Matrix. Member of the math namespace, so every method
    /// has it's 'by reference' equivalent to speed up time critical
    /// math operations.
    /// </summary>
    public struct JMatrix
    {
        /// <summary>
        /// M11
        /// </summary>
        public float M11; // 1st row vector
        /// <summary>
        /// M12
        /// </summary>
        public float M12;
        /// <summary>
        /// M21
        /// </summary>
        public float M21; // 2nd row vector
        /// <summary>
        /// M22
        /// </summary>
        public float M22;

        internal static JMatrix InternalIdentity;

        /// <summary>
        /// Identity matrix.
        /// </summary>
        public static readonly JMatrix Identity;
        public static readonly JMatrix Zero;

        static JMatrix()
        {
            Zero = new JMatrix();

            Identity = new JMatrix();
            Identity.M11 = 1.0f;
            Identity.M22 = 1.0f;

            InternalIdentity = Identity;
        }

        
        public static JMatrix CreateRotationZ(float radians)
        {
            JMatrix matrix;
            float num2 = (float)Math.Cos((double)radians);
            float num = (float)Math.Sin((double)radians);
            matrix.M11 = num2;
            matrix.M12 = num;
            matrix.M21 = -num;
            matrix.M22 = num2;
            return matrix;
        }


        public static void CreateRotationZ(float radians, out JMatrix result)
        {
            float num2 = (float)Math.Cos((double)radians);
            float num = (float)Math.Sin((double)radians);
            result.M11 = num2;
            result.M12 = num;
            result.M21 = -num;
            result.M22 = num2;
        }

        /// <summary>
        /// Initializes a new instance of the matrix structure.
        /// </summary>
        /// <param name="m11">m11</param>
        /// <param name="m12">m12</param>
        /// <param name="m21">m21</param>
        /// <param name="m22">m22</param>
        #region public JMatrix(float m11, float m12, float m13, float m21, float m22, float m23,float m31, float m32, float m33)
        public JMatrix(float m11, float m12, float m21, float m22)
        {
            this.M11 = m11;
            this.M12 = m12;
            this.M21 = m21;
            this.M22 = m22;
        }
        #endregion

        /// <summary>
        /// Multiply two matrices. Notice: matrix multiplication is not commutative.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <returns>The product of both matrices.</returns>
        #region public static JMatrix Multiply(JMatrix matrix1, JMatrix matrix2)
        public static JMatrix Multiply(JMatrix matrix1, JMatrix matrix2)
        {
            JMatrix result;
            JMatrix.Multiply(ref matrix1, ref matrix2, out result);
            return result;
        }

        /// <summary>
        /// Multiply two matrices. Notice: matrix multiplication is not commutative.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <param name="result">The product of both matrices.</param>
        public static void Multiply(ref JMatrix matrix1, ref JMatrix matrix2, out JMatrix result)
        {
            float num0 = (matrix1.M11 * matrix2.M11) + (matrix1.M12 * matrix2.M21);
            float num1 = (matrix1.M11 * matrix2.M12) + (matrix1.M12 * matrix2.M22);
            float num3 = (matrix1.M21 * matrix2.M11) + (matrix1.M22 * matrix2.M21);
            float num4 = (matrix1.M21 * matrix2.M12) + (matrix1.M22 * matrix2.M22);

            result.M11 = num0;
            result.M12 = num1;
            result.M21 = num3;
            result.M22 = num4;
        }
        #endregion

        /// <summary>
        /// Matrices are added.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <returns>The sum of both matrices.</returns>
        #region public static JMatrix Add(JMatrix matrix1, JMatrix matrix2)
        public static JMatrix Add(JMatrix matrix1, JMatrix matrix2)
        {
            JMatrix result;
            JMatrix.Add(ref matrix1, ref matrix2, out result);
            return result;
        }

        /// <summary>
        /// Matrices are added.
        /// </summary>
        /// <param name="matrix1">The first matrix.</param>
        /// <param name="matrix2">The second matrix.</param>
        /// <param name="result">The sum of both matrices.</param>
        public static void Add(ref JMatrix matrix1, ref JMatrix matrix2, out JMatrix result)
        {
            result.M11 = matrix1.M11 + matrix2.M11;
            result.M12 = matrix1.M12 + matrix2.M12;
            result.M21 = matrix1.M21 + matrix2.M21;
            result.M22 = matrix1.M22 + matrix2.M22;
        }
        #endregion

        /// <summary>
        /// Calculates the inverse of a give matrix.
        /// </summary>
        /// <param name="matrix">The matrix to invert.</param>
        /// <returns>The inverted JMatrix.</returns>
        #region public static JMatrix Inverse(JMatrix matrix)
        public static JMatrix Inverse(JMatrix matrix)
        {
            JMatrix result;
            JMatrix.Inverse(ref matrix, out result);
            return result;
        }

        public float Determinant()
        {
            return M11 * M22 - M12 * M21;
        }

        public static void Invert(ref JMatrix matrix, out JMatrix result)
        {
            float determinantInverse = 1f / matrix.Determinant();

            result.M11 = matrix.M22 * determinantInverse;
            result.M12 = -matrix.M12 * determinantInverse;
            result.M21 = matrix.M11 * determinantInverse;
            result.M22 = -matrix.M21 * determinantInverse;
        }

        /// <summary>
        /// Calculates the inverse of a give matrix.
        /// </summary>
        /// <param name="matrix">The matrix to invert.</param>
        /// <param name="result">The inverted JMatrix.</param>
        public static void Inverse(ref JMatrix matrix, out JMatrix result)
        {
            float det = matrix.M11 * matrix.M22 - matrix.M12 * matrix.M21;
            float determinantInverse = 1f / det;

            result.M11 = matrix.M22 * determinantInverse;
            result.M12 = -matrix.M12 * determinantInverse;
            result.M21 = matrix.M11 * determinantInverse;
            result.M22 = -matrix.M21 * determinantInverse;
        }
        #endregion

        /// <summary>
        /// Multiply a matrix by a scalefactor.
        /// </summary>
        /// <param name="matrix1">The matrix.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <returns>A JMatrix multiplied by the scale factor.</returns>
        #region public static JMatrix Multiply(JMatrix matrix1, float scaleFactor)
        public static JMatrix Multiply(JMatrix matrix1, float scaleFactor)
        {
            JMatrix result;
            JMatrix.Multiply(ref matrix1, scaleFactor, out result);
            return result;
        }

        /// <summary>
        /// Multiply a matrix by a scalefactor.
        /// </summary>
        /// <param name="matrix1">The matrix.</param>
        /// <param name="scaleFactor">The scale factor.</param>
        /// <param name="result">A JMatrix multiplied by the scale factor.</param>
        public static void Multiply(ref JMatrix matrix1, float scaleFactor, out JMatrix result)
        {
            float num = scaleFactor;
            result.M11 = matrix1.M11 * num;
            result.M12 = matrix1.M12 * num;
            result.M21 = matrix1.M21 * num;
            result.M22 = matrix1.M22 * num;
        }
        #endregion

        
        /// <summary>
        /// Creates the transposed matrix.
        /// </summary>
        /// <param name="matrix">The matrix which should be transposed.</param>
        /// <returns>The transposed JMatrix.</returns>
        #region public static JMatrix Transpose(JMatrix matrix)
        public static JMatrix Transpose(JMatrix matrix)
        {
            JMatrix result;
            JMatrix.Transpose(ref matrix, out result);
            return result;
        }

        /// <summary>
        /// Creates the transposed matrix.
        /// </summary>
        /// <param name="matrix">The matrix which should be transposed.</param>
        /// <param name="result">The transposed JMatrix.</param>
        public static void Transpose(ref JMatrix matrix, out JMatrix result)
        {
            result.M11 = matrix.M11;
            result.M12 = matrix.M21;
            result.M21 = matrix.M12;
            result.M22 = matrix.M22;
        }
        #endregion

        /// <summary>
        /// Multiplies two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The product of both values.</returns>
        #region public static JMatrix operator *(JMatrix value1,JMatrix value2)
        public static JMatrix operator *(JMatrix value1, JMatrix value2)
        {
            JMatrix result; JMatrix.Multiply(ref value1, ref value2, out result);
            return result;
        }
        #endregion

        public static JMatrix operator ^(JMatrix matrix1, JMatrix value2)
        {
            JMatrix result;
            var matrix2 = JMatrix.Transpose(value2);
            float num0 = (matrix1.M11 * matrix2.M11) + (matrix1.M12 * matrix2.M21);
            float num1 = (matrix1.M11 * matrix2.M12) + (matrix1.M12 * matrix2.M22); 
            float num3 = (matrix1.M21 * matrix2.M11) + (matrix1.M22 * matrix2.M21);
            float num4 = (matrix1.M21 * matrix2.M12) + (matrix1.M22 * matrix2.M22);

            result.M11 = num0;
            result.M12 = num1;
            result.M21 = num3;
            result.M22 = num4;
            return result;
        }


        public float Trace()
        {
            return this.M11 + this.M22;
        }

        /// <summary>
        /// Adds two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The sum of both values.</returns>
        #region public static JMatrix operator +(JMatrix value1, JMatrix value2)
        public static JMatrix operator +(JMatrix value1, JMatrix value2)
        {
            JMatrix result; JMatrix.Add(ref value1, ref value2, out result);
            return result;
        }
        #endregion

        /// <summary>
        /// Subtracts two matrices.
        /// </summary>
        /// <param name="value1">The first matrix.</param>
        /// <param name="value2">The second matrix.</param>
        /// <returns>The difference of both values.</returns>
        #region public static JMatrix operator -(JMatrix value1, JMatrix value2)
        public static JMatrix operator -(JMatrix value1, JMatrix value2)
        {
            JMatrix result; JMatrix.Multiply(ref value2, -1.0f, out value2);
            JMatrix.Add(ref value1, ref value2, out result);
            return result;
        }
        #endregion
    }
}
