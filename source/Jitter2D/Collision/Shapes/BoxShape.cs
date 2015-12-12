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

using Jitter2D.Dynamics;
using Jitter2D.LinearMath;
using Jitter2D.Collision.Shapes;
#endregion

namespace Jitter2D.Collision.Shapes
{

    /// <summary>
    /// A <see cref="Shape"/> representing a box.
    /// </summary>
    public class BoxShape : Shape
    {
        private JVector size = JVector.Zero;

        /// <summary>
        /// The side length of the box.
        /// </summary>
        public JVector Size
        {
            get { return size; }
            set { size = value; UpdateShape(); }
        }

        /// <summary>
        /// Creates a new instance of the BoxShape class.
        /// </summary>
        /// <param name="size">The size of the box.</param>
        public BoxShape(JVector size)
        {
            this.size = size;
            this.type = ShapeType.Box;
            this.UpdateShape();
        }

        /// <summary>
        /// Creates a new instance of the BoxShape class.
        /// </summary>
        /// <param name="length">The width of the box.</param>
        /// <param name="height">The height of the box.</param>
        public BoxShape(float width, float height)
        {
            this.size.X = width;
            this.size.Y = height;
            this.type = ShapeType.Box;
            this.UpdateShape();
        }

        internal JVector halfSize = JVector.Zero;

        // local axes for SAT
        internal JVector xAxis = JVector.Left;
        internal JVector yAxis = JVector.Up;

        /// <summary>
        /// Returns true if the point is inside the box's local space.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns>True if the point is inside the box.</returns>
        public override bool PointInsideLocal(JVector point)
        {
            if (point.X > -halfSize.X && point.X < halfSize.X)
                if (point.Y > -halfSize.Y && point.Y < halfSize.Y)
                    return true;
            return false;
        }

        /// <summary>
        /// Should return true if the point is inside the box's world space.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="position">World position of box.</param>
        /// <param name="orientation">World orientation of box.</param>
        /// <returns>True if the point is inside the shape.</returns>
        public override bool PointInsideWorld(JVector point, JVector position, JMatrix orientation)
        {
            // move point into local space
            throw new NotImplementedException();
        }

        /// <summary>
        /// This method uses the <see cref="ISupportMappable"/> implementation
        /// to calculate the local bounding box, the mass, geometric center and 
        /// the inertia of the shape. In custom shapes this method should be overidden
        /// to compute this values faster.
        /// </summary>
        public override void UpdateShape()
        {
            this.halfSize = size * 0.5f;
            base.UpdateShape();
        }

        // this should only be called if this shape is colliding
        public override void UpdateAxes(float orientation)
        {
            this.xAxis = JVector.Transform(JVector.Left, JMatrix.CreateRotationZ(orientation));
            this.yAxis = JVector.Transform(JVector.Up, JMatrix.CreateRotationZ(orientation));
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(ref float orientation, out JBBox box)
        {
            JMatrix xForm = JMatrix.CreateRotationZ(-orientation);
            JMatrix abs; JMath.Absolute(ref xForm, out abs);
            JVector temp;
            JVector.Transform(ref halfSize, ref abs, out temp);

            box.Max = temp;
            JVector.Negate(ref temp, out box.Min);
        }

        /// <summary>
        /// This method uses the <see cref="ISupportMappable"/> implementation
        /// to calculate the local bounding box, the mass, geometric center and 
        /// the inertia of the shape. In custom shapes this method should be overidden
        /// to compute this values faster.
        /// </summary>
        public override void CalculateMassInertia()
        {
            mass = size.X * size.Y;
            inertia = mass * (this.Size.X * this.Size.X + this.Size.Y * this.Size.Y) / 12.0f;

            this.geomCen = JVector.Zero;
        }

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            result.X = (float)Math.Sign(direction.X) * halfSize.X;
            result.Y = (float)Math.Sign(direction.Y) * halfSize.Y;
        }

        public JVector GetCorner(int i)
        {
            switch (i)
            {
                case 0:
                    return new JVector(-halfSize.X, -halfSize.Y);
                case 1:
                    return new JVector(halfSize.X, -halfSize.Y);
                case 2:
                    return new JVector(halfSize.X, halfSize.Y);
                case 3:
                    return new JVector(-halfSize.X, halfSize.Y);
                default:
                    throw new IndexOutOfRangeException();
            }
        }

        /// <summary>
        /// Gets up to two supports points if the given direction is within a tolerance of being parallel to the normal of the edge formed by the supports. Otherwise it just returns a single support.
        /// </summary>
        internal int FindSupportPoints(ref JVector direction, ref JVector PA, ref JMatrix OA, out JVector[] S)
        {
            // init S
            S = new JVector[2];
            // transform the normal into object space
            JVector N = JVector.TransposedTransform(direction, OA);
            // find dots
            float a = this.GetCorner(0) * N;
            float b = this.GetCorner(1) * N;
            float c = this.GetCorner(2) * N;
            float d = this.GetCorner(3) * N;
            // find min
            float min = JMath.Min(a, b);
            min = JMath.Min(c, min);
            min = JMath.Min(d, min);

            // now min should be right
            int Snum = 0;
            const float threshold = 1.0E-3f;

            if (a < min + threshold)
                S[Snum++] = JVector.Transform(this.GetCorner(0), OA) + PA;

            if (b < min + threshold)
                S[Snum++] = JVector.Transform(this.GetCorner(1), OA) + PA;

            if (c < min + threshold)
                S[Snum++] = JVector.Transform(this.GetCorner(2), OA) + PA;

            if (d < min + threshold)
                S[Snum++] = JVector.Transform(this.GetCorner(3), OA) + PA;
                
            return Snum;
        }


        // TODO - this is just ridiculous, for a box we should only be doing 2 dot products here!
        //        one for xAxis and one for yAxis then if ...
        //----------------------------------------------------------------------------------------------- 
        // find the support points of a convex shape along a direction
        //----------------------------------------------------------------------------------------------- 
        //internal int FindSupportPointsOLD(ref JVector N, ref JVector PA, ref JMatrix OA, out JVector[] S)
        //{
        //    S = new JVector[2];

        //    JVector Norm = JVector.TransposedTransform(N, OA);

        //    float[] d = new float[4];      // 32 points max?
        //    float dmin;
        //    dmin = d[0] = this.GetCorner(0) * Norm;

        //    for (int i = 1; i < 4; i++)
        //    {
        //        d[i] = this.GetCorner(i) * Norm;

        //        if (d[i] < dmin)
        //        {
        //            dmin = d[i];
        //        }
        //    }

        //    // we will limit the number of support points to only 2. 
        //    // if we have more than 2 support points, we take the extremums.
        //    int Snum = 0;
        //    const float threshold = 1.0E-3f;
        //    float[] s = new float[2];
        //    bool sign = false;

        //    JVector Perp = new JVector(-Norm.Y, Norm.X);

        //    for (int i = 0; i < 4; i++)
        //    {
        //        if (d[i] < dmin + threshold)
        //        {
        //            JVector Contact = JVector.Transform(this.GetCorner(i), OA) + PA;//Transform(A[i], PA, VA, OA, t);

        //            float c = Contact * Perp;

        //            if (Snum < 2)
        //            {
        //                s[Snum] = c;
        //                S[Snum] = Contact;
        //                Snum++;

        //                if (Snum > 1)
        //                {
        //                    sign = (s[1] > s[0]);
        //                }
        //            }
        //            else
        //            {
        //                float min = (sign) ? s[0] : s[1];
        //                float max = (sign) ? s[1] : s[0];
        //                JVector Min = (sign) ? S[0] : S[1];
        //                JVector Max = (sign) ? S[1] : S[0];

        //                if (c < min)
        //                {
        //                    min = c;
        //                    Min = Contact;
        //                }
        //                else if (c > max)
        //                {
        //                    max = c;
        //                    Max = Contact;
        //                }
        //            }
        //        }
        //    }
        //    return Snum;
        //}
    }
}
