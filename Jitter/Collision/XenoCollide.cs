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

namespace Jitter.Collision
{

    /// <summary>
    /// The implementation of the ISupportMappable interface defines the form
    /// of a shape. <seealso cref="GJKCollide"/> <seealso cref="XenoCollide"/>
    /// </summary>
    public interface ISupportMappable
    {
        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        void SupportMapping(ref JVector direction, out JVector result);

        /// <summary>
        /// The center of the SupportMap.
        /// </summary>
        /// <param name="center"></param>
        void SupportCenter(out JVector center);
    }

    /// <summary>
    /// Implementation of the XenoCollide Algorithm by Gary Snethen. 
    /// Narrowphase collision detection highly optimized for C#.
    /// http://xenocollide.snethen.com/
    /// </summary>
    public sealed class XenoCollide
    {

        private const float CollideEpsilon = 1e-4f;
        private const int MaximumIterations = 34;

        private static void SupportMapTransformed(ISupportMappable support,
            ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        {
            // THIS IS *THE* HIGH FREQUENCY CODE OF THE COLLLISION PART OF THE ENGINE

            result.X = ((direction.X * orientation.M11) + (direction.Y * orientation.M12)) + (direction.Z * orientation.M13);
            result.Y = ((direction.X * orientation.M21) + (direction.Y * orientation.M22)) + (direction.Z * orientation.M23);
            result.Z = ((direction.X * orientation.M31) + (direction.Y * orientation.M32)) + (direction.Z * orientation.M33);

            support.SupportMapping(ref result, out result);

            float x = ((result.X * orientation.M11) + (result.Y * orientation.M21)) + (result.Z * orientation.M31);
            float y = ((result.X * orientation.M12) + (result.Y * orientation.M22)) + (result.Z * orientation.M32);
            float z = ((result.X * orientation.M13) + (result.Y * orientation.M23)) + (result.Z * orientation.M33);

            result.X = position.X + x;
            result.Y = position.Y + y;
            result.Z = position.Z + z;
        }

        /// <summary>
        /// Checks two shapes for collisions.
        /// </summary>
        /// <param name="support1">The SupportMappable implementation of the first shape to test.</param>
        /// <param name="support2">The SupportMappable implementation of the seconds shape to test.</param>
        /// <param name="orientation1">The orientation of the first shape.</param>
        /// <param name="orientation2">The orientation of the second shape.</param>
        /// <param name="position1">The position of the first shape.</param>
        /// <param name="position2">The position of the second shape</param>
        /// <param name="point">The pointin world coordinates, where collision occur.</param>
        /// <param name="normal">The normal pointing from body2 to body1.</param>
        /// <param name="penetration">Estimated penetration depth of the collision.</param>
        /// <returns>Returns true if there is a collision, false otherwise.</returns>
        public static bool Detect(ISupportMappable support1, ISupportMappable support2, ref JMatrix orientation1,
             ref JMatrix orientation2, ref JVector position1, ref JVector position2,
             out JVector point, out JVector normal, out float penetration)
        {
            // Used variables
            JVector temp1, temp2;
            JVector v01, v02, v0;
            JVector v11, v12, v1;
            JVector v21, v22, v2;
            JVector v31, v32, v3;
            JVector v41, v42, v4;
            JVector mn;

            // Initialization of the output
            point = normal = JVector.Zero;
            penetration = 0.0f;

            //JVector right = JVector.Right;

            // Get the center of shape1 in world coordinates -> v01
            support1.SupportCenter(out v01);
            JVector.Transform(ref v01, ref orientation1, out v01);
            JVector.Add(ref position1, ref v01, out v01);

            // Get the center of shape2 in world coordinates -> v02
            support2.SupportCenter(out v02);
            JVector.Transform(ref v02, ref orientation2, out v02);
            JVector.Add(ref position2, ref v02, out v02);

            // v0 is the center of the minkowski difference
            JVector.Subtract(ref v02, ref v01, out v0);

            // Avoid case where centers overlap -- any direction is fine in this case
            if (v0.IsNearlyZero()) v0 = new JVector(0.00001f, 0, 0);

            // v1 = support in direction of origin
            mn = v0;
            JVector.Negate(ref v0, out normal);

            SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v11);
            SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v12);
            JVector.Subtract(ref v12, ref v11, out v1);

            if (JVector.Dot(ref v1, ref normal) <= 0.0f) return false;

            // v2 = support perpendicular to v1,v0
            JVector.Cross(ref v1, ref v0, out normal);

            if (normal.IsNearlyZero())
            {
                JVector.Subtract(ref v1, ref v0, out normal);

                normal.Normalize();

                point = v11;
                JVector.Add(ref point, ref v12, out point);
                JVector.Multiply(ref point, 0.5f, out point);

                JVector.Subtract(ref v12, ref v11, out temp1);
                penetration = JVector.Dot(ref temp1, ref normal);

                //point = v11;
                //point2 = v12;
                return true;
            }

            JVector.Negate(ref normal, out mn);
            SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v21);
            SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v22);
            JVector.Subtract(ref v22, ref v21, out v2);

            if (JVector.Dot(ref v2, ref normal) <= 0.0f) return false;

            // Determine whether origin is on + or - side of plane (v1,v0,v2)
            JVector.Subtract(ref v1, ref v0, out temp1);
            JVector.Subtract(ref v2, ref v0, out temp2);
            JVector.Cross(ref temp1, ref temp2, out normal);

            float dist = JVector.Dot(ref normal, ref v0);

            // If the origin is on the - side of the plane, reverse the direction of the plane
            if (dist > 0.0f)
            {
                JVector.Swap(ref v1, ref v2);
                JVector.Swap(ref v11, ref v21);
                JVector.Swap(ref v12, ref v22);
                JVector.Negate(ref normal, out normal);
            }


            int phase2 = 0;
            int phase1 = 0;
            bool hit = false;

            // Phase One: Identify a portal
            while (true)
            {
                if (phase1 > MaximumIterations) return false;

                phase1++;

                // Obtain the support point in a direction perpendicular to the existing plane
                // Note: This point is guaranteed to lie off the plane
                JVector.Negate(ref normal, out mn);
                SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v31);
                SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v32);
                JVector.Subtract(ref v32, ref v31, out v3);

                if (JVector.Dot(ref v3, ref normal) <= 0.0f)
                {
                    return false;
                }

                // If origin is outside (v1,v0,v3), then eliminate v2 and loop
                JVector.Cross(ref v1, ref v3, out temp1);
                if (JVector.Dot(ref temp1, ref v0) < 0.0f)
                {
                    v2 = v3;
                    v21 = v31;
                    v22 = v32;
                    JVector.Subtract(ref v1, ref v0, out temp1);
                    JVector.Subtract(ref v3, ref v0, out temp2);
                    JVector.Cross(ref temp1, ref temp2, out normal);
                    continue;
                }

                // If origin is outside (v3,v0,v2), then eliminate v1 and loop
                JVector.Cross(ref v3, ref v2, out temp1);
                if (JVector.Dot(ref temp1, ref v0) < 0.0f)
                {
                    v1 = v3;
                    v11 = v31;
                    v12 = v32;
                    JVector.Subtract(ref v3, ref v0, out temp1);
                    JVector.Subtract(ref v2, ref v0, out temp2);
                    JVector.Cross(ref temp1, ref temp2, out normal);
                    continue;
                }

                // Phase Two: Refine the portal
                // We are now inside of a wedge...
                while (true)
                {
                    phase2++;

                    // Compute normal of the wedge face
                    JVector.Subtract(ref v2, ref v1, out temp1);
                    JVector.Subtract(ref v3, ref v1, out temp2);
                    JVector.Cross(ref temp1, ref temp2, out normal);

                    // Can this happen???  Can it be handled more cleanly?
                    if (normal.IsNearlyZero()) return true;

                    normal.Normalize();

                    // Compute distance from origin to wedge face
                    float d = JVector.Dot(ref normal, ref v1);


                    // If the origin is inside the wedge, we have a hit
                    if (d >= 0 && !hit)
                    {
                        // HIT!!!
                        hit = true;
                    }

                    // Find the support point in the direction of the wedge face
                    JVector.Negate(ref normal, out mn);
                    SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v41);
                    SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v42);
                    JVector.Subtract(ref v42, ref v41, out v4);

                    JVector.Subtract(ref v4, ref v3, out temp1);
                    float delta = JVector.Dot(ref temp1, ref normal);
                    penetration = JVector.Dot(ref v4, ref normal);

                    // If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex, then we can terminate
                    if (delta <= CollideEpsilon || penetration <= 0.0f || phase2 > MaximumIterations)
                    {

                        if (hit)
                        {
                            JVector.Cross(ref v1, ref v2, out temp1);
                            float b0 = JVector.Dot(ref temp1, ref v3);
                            JVector.Cross(ref v3, ref v2, out temp1);
                            float b1 = JVector.Dot(ref temp1, ref v0);
                            JVector.Cross(ref v0, ref v1, out temp1);
                            float b2 = JVector.Dot(ref temp1, ref v3);
                            JVector.Cross(ref v2, ref v1, out temp1);
                            float b3 = JVector.Dot(ref temp1, ref v0);

                            float sum = b0 + b1 + b2 + b3;

                            if (sum <= 0)
                            {
                                b0 = 0;
                                JVector.Cross(ref v2, ref v3, out temp1);
                                b1 = JVector.Dot(ref temp1, ref normal);
                                JVector.Cross(ref v3, ref v1, out temp1);
                                b2 = JVector.Dot(ref temp1, ref normal);
                                JVector.Cross(ref v1, ref v2, out temp1);
                                b3 = JVector.Dot(ref temp1, ref normal);

                                sum = b1 + b2 + b3;
                            }

                            float inv = 1.0f / sum;

                            JVector.Multiply(ref v01, b0, out point);
                            JVector.Multiply(ref v11, b1, out temp1);
                            JVector.Add(ref point, ref temp1, out point);
                            JVector.Multiply(ref v21, b2, out temp1);
                            JVector.Add(ref point, ref temp1, out point);
                            JVector.Multiply(ref v31, b3, out temp1);
                            JVector.Add(ref point, ref temp1, out point);

                            JVector.Multiply(ref v02, b0, out temp2);
                            JVector.Add(ref temp2, ref point, out point);
                            JVector.Multiply(ref v12, b1, out temp1);
                            JVector.Add(ref point, ref temp1, out point);
                            JVector.Multiply(ref v22, b2, out temp1);
                            JVector.Add(ref point, ref temp1, out point);
                            JVector.Multiply(ref v32, b3, out temp1);
                            JVector.Add(ref point, ref temp1, out point);

                            JVector.Multiply(ref point, inv * 0.5f, out point);

                        }

                        // Compute the barycentric coordinates of the origin
                        return hit;
                    }

                    //// Compute the tetrahedron dividing face (v4,v0,v1)
                    //JVector.Cross(ref v4, ref v1, out temp1);
                    //float d1 = JVector.Dot(ref temp1, ref v0);


                    //// Compute the tetrahedron dividing face (v4,v0,v2)
                    //JVector.Cross(ref v4, ref v2, out temp1);
                    //float d2 = JVector.Dot(ref temp1, ref v0);


                    // Compute the tetrahedron dividing face (v4,v0,v3)
                    JVector.Cross(ref v4, ref v0, out temp1);
                    float dot = JVector.Dot(ref temp1, ref v1);

                    if (dot >= 0.0f)
                    {
                        dot = JVector.Dot(ref temp1, ref v2);

                        if (dot >= 0.0f)
                        {
                            // Inside d1 & inside d2 ==> eliminate v1
                            v1 = v4;
                            v11 = v41;
                            v12 = v42;
                        }
                        else
                        {
                            // Inside d1 & outside d2 ==> eliminate v3
                            v3 = v4;
                            v31 = v41;
                            v32 = v42;
                        }
                    }
                    else
                    {
                        dot = JVector.Dot(ref temp1, ref v3);

                        if (dot >= 0.0f)
                        {
                            // Outside d1 & inside d3 ==> eliminate v2
                            v2 = v4;
                            v21 = v41;
                            v22 = v42;
                        }
                        else
                        {
                            // Outside d1 & outside d3 ==> eliminate v1
                            v1 = v4;
                            v11 = v41;
                            v12 = v42;
                        }
                    }


                }
            }

        }

    }
}