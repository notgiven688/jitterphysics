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

using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
#endregion

namespace Jitter.LinearMath
{

    /// <summary>
    /// Fast but dirty convex hull creation.
    /// advanced convex hull creation: http://www.qhull.org
    /// </summary>
    public static class JConvexHull
    {
        #region public enum Approximation
        public enum Approximation
        {
            Level1 = 1,
            Level2 = 2,
            Level3 = 3,
            Level4 = 4,
            Level5 = 5,
            Level6 = 6,
            Level7 = 7,
            Level8 = 8,
            Level9 = 9,
            Level10 = 10,
            Level15 = 15,
            Level20 = 20
        }
        #endregion

        public static int[] Build(List<JVector> pointCloud, Approximation factor)
        {
            List<int> allIndices = new List<int>();

            int iterations = (int)factor;

            Random rnd = new Random();

            for (int i = 0; i < iterations; i++)
            {
                for (int e = 0; e < iterations; e++)
                {
                    for (int k = 0; k < iterations; k++)
                    {
                        JMatrix rot = JMatrix.CreateRotationX(JMath.PiOver2 / iterations * i) *
                         JMatrix.CreateRotationY(JMath.PiOver2 / iterations * e) *
                         JMatrix.CreateRotationZ(JMath.PiOver2 / iterations * k);

                        JVector vec0 = JVector.Transform(JVector.Right, rot);
                        JVector vec1 = JVector.Transform(JVector.Up, rot);
                        JVector vec2 = JVector.Transform(JVector.Forward, rot);

                        int[] indices = FindExtremePoints(pointCloud, ref vec0, ref vec1, ref vec2);
                        allIndices.AddRange(indices);
                    }
                }
            }

            // this,

            allIndices.Sort();

            for (int i = 1; i < allIndices.Count; i++)
            {
                if (allIndices[i - 1] == allIndices[i])
                { allIndices.RemoveAt(i - 1); i--; }
            }

            return allIndices.ToArray();

            // or using 3.5 extensions
            // return allIndices.Distinct().ToArray();
        }

        private static int[] FindExtremePoints(List<JVector> points,
            ref JVector dirX, ref JVector dirY, ref JVector dirZ)
        {
            int[] indices = new int[6];
            float[] current = new float[6];

            JVector point; float value;

            for (int i = 0; i < points.Count; i++)
            {
                point = points[i];

                value = JVector.Dot(ref point, ref dirX);
                if (value > current[0]) { current[0] = value; indices[0] = i; }
                if (value < current[1]) { current[1] = value; indices[1] = i; }

                value = JVector.Dot(ref point, ref dirY);
                if (value > current[2]) { current[2] = value; indices[2] = i; }
                if (value < current[3]) { current[3] = value; indices[3] = i; }

                value = JVector.Dot(ref point, ref dirZ);
                if (value > current[4]) { current[4] = value; indices[4] = i; }
                if (value < current[5]) { current[5] = value; indices[5] = i; }
            }

            return indices;
        }
    }
}
