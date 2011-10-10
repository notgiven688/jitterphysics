using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter2D.LinearMath;

namespace Jitter2D.Collision
{

    public sealed class XenoCollide
    {
        private const float CollideEpsilon = 1e-4f;
        private const int MaximumIterations = 10;

        #region private static void SupportMapTransformed(ISupportMappable support, ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        public static void SupportMapTransformed(ISupportMappable support,
            ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        {
            // THIS IS *THE* HIGH FREQUENCY CODE OF THE COLLLISION PART OF THE ENGINE
            result.X = ((direction.X * orientation.M11) + (direction.Y * orientation.M12));
            result.Y = ((direction.X * orientation.M21) + (direction.Y * orientation.M22));

            support.SupportMapping(ref result, out result);

            float x = ((result.X * orientation.M11) + (result.Y * orientation.M21));
            float y = ((result.X * orientation.M12) + (result.Y * orientation.M22));

            result.X = position.X + x;
            result.Y = position.Y + y;
        }
        #endregion

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
            JVector temp1;
            JVector v01, v02, v0;
            JVector v11, v12, v1;
            JVector v21, v22, v2;
            JVector v31, v32, v3;
            JVector mn;

            // Initialization of the output
            point = normal = JVector.Zero;
            penetration = 0.0f;

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
            if (v0.IsNearlyZero()) v0 = new JVector(0.00001f, 0);

            // v1 = support in direction of origin
            mn = v0;
            JVector.Negate(ref v0, out normal);

            SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v11);
            SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v12);
            JVector.Subtract(ref v12, ref v11, out v1);

            if (JVector.Dot(ref v1, ref normal) <= 0.0f) return false;

            // v2 = support perpendicular to v1,v0
            normal = OutsidePortal(v1, v0);

            JVector.Negate(ref normal, out mn);
            SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v21);
            SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v22);
            JVector.Subtract(ref v22, ref v21, out v2);

            if (JVector.Dot(ref v2, ref normal) <= 0.0f) return false;

            // phase two: portal refinement
            int maxIterations = 0;

            while (true)
            {
                // find normal direction
                if (!IntersectPortal(v0, v2, v1))
                    normal = InsidePortal(v2, v1);
                else
                    // origin ray crosses the portal
                    normal = OutsidePortal(v2, v1);

                // obtain the next support point
                JVector.Negate(ref normal, out mn);
                SupportMapTransformed(support1, ref orientation1, ref position1, ref mn, out v31);
                SupportMapTransformed(support2, ref orientation2, ref position2, ref normal, out v32);
                JVector.Subtract(ref v32, ref v31, out v3);

                if (JVector.Dot(v3, normal) <= 0)
                {
                    JVector ab = v3 - v2;
                    float t = -(JVector.Dot(v2, ab)) / (JVector.Dot(ab, ab));
                    normal = (v2 + (t * ab));
                    return false;
                }

                // Portal lies on the outside edge of the Minkowski Hull.
                // Return contact information
                if (JVector.Dot((v3 - v2), normal) <= CollideEpsilon || ++maxIterations > MaximumIterations)
                {
                    JVector ab = v2 - v1;
                    float t = JVector.Dot(JVector.Negate(v1), ab);
                    if (t <= 0.0f)
                    {
                        t = 0.0f;
                        normal = v1;
                    }
                    else
                    {
                        float denom = JVector.Dot(ab, ab);
                        if (t >= denom)
                        {
                            normal = v2;
                            t = 1.0f;
                        }
                        else
                        {
                            t /= denom;
                            normal = v1 + t * ab;
                        }
                    }

                    float s = 1 - t;

                    point = s * v12 + t * v22;
                    var point2 = s * v11 + t * v21;

                    // this  causes a sq root = bad!
                    penetration = -JVector.Dot(normal, v1);

                    return true;
                }

                // if origin is inside (v1, v0, v3), refine portal
                if (OriginInTriangle(v0, v1, v3))
                {
                    v2 = v3;
                    v21 = v31;
                    v22 = v32;
                    continue;
                }
                // if origin is inside (v3, v0, v2), refine portal
                else if (OriginInTriangle(v0, v2, v3))
                {
                    v1 = v3;
                    v11 = v31;
                    v12 = v32;
                    continue;
                }
                return false;
            }
        }

        public static JVector InsidePortal(JVector v1, JVector v2)
        {
            // Perp-dot product
            float dir = v1.X * v2.Y - v1.Y * v2.X;

            if (dir > float.Epsilon)
            {
                // rotate left
                JVector v = new JVector(v1.X - v2.X, v1.Y - v2.Y);
                v = new JVector(-v.Y, v.X);
                return v;
            }
            else
            {
                // rotate right
                JVector v = new JVector(v1.X - v2.X, v1.Y - v2.Y);
                v = new JVector(v.Y, -v.X);
                return v;
            }
        }

        public static bool IntersectPortal(JVector v0, JVector v1, JVector v2)
        {
            JVector a = JVector.Zero;
            JVector b = v0;
            JVector c = v1;
            JVector d = v2;

            float a1 = (a.X - d.X) * (b.Y - d.Y) - (a.Y - d.Y) * (b.X - d.X);
            float a2 = (a.X - c.X) * (b.Y - c.Y) - (a.Y - c.Y) * (b.X - c.X);

            if (a1 != 0.0f && a2 != 0.0f && a1 * a2 < 0.0f)
            {
                float a3 = (c.X - a.X) * (d.Y - a.Y) - (c.Y - a.Y) * (d.X - a.X);
                float a4 = a3 + a2 - a1;
                if (a3 != 0.0f && a4 != 0.0f && a3 * a4 < 0.0f) return true;
            }
            // segments not intersecting
            return false;
        }

        public static bool OriginInTriangle(JVector a, JVector b, JVector c)
        {
            float pab = JVector.Cross(JVector.Negate(a), b - a);
            float pbc = JVector.Cross(JVector.Negate(b), c - b);

            if (!SameSign(pab, pbc)) return false;

            float pca = JVector.Cross(JVector.Negate(c), a - c);

            if (!SameSign(pab, pca)) return false;

            return true;
        }

        private static bool SameSign(float a, float b)
        {
            int x = Math.Sign(a);
            int y = Math.Sign(b);
            //if (x != 0 && y != 0)
            if (x == y)
                return true;
            return false;
        }

        public static JVector OutsidePortal(JVector v1, JVector v2)
        {
            // Perp-dot product
            float dir = v1.X * v2.Y - v1.Y * v2.X;

            if (dir < float.Epsilon)
            {
                // rotate left
                JVector v = new JVector(v1.X - v2.X, v1.Y - v2.Y);
                v = new JVector(-v.Y, v.X);
                return v;
            }
            else
            {
                // rotate right
                JVector v = new JVector(v1.X - v2.X, v1.Y - v2.Y);
                v = new JVector(v.Y, -v.X);
                return v;
            }
        }
    }
}
