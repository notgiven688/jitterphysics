using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter2D.Collision.Shapes;
using Jitter2D.LinearMath;

namespace Jitter2D.Collision
{
    public class Collision
    {
        public static bool CircleCircleTest(JVector centerA, float radiusA, JVector centerB, float radiusB, out JVector pointA, out JVector pointB, out JVector normal, out JVector penetrationVector, out float distance)
        {
            // ||A-B|| - (r1+r2) < 0
            float d = JVector.Distance(centerA, centerB);
            float r = radiusA + radiusB;
            
            distance = d - r;

            normal = (centerA - centerB) / d;

            penetrationVector = normal * distance;

            // calculate closest 2 points
            pointA = JVector.Negate(normal) * radiusA + centerA;
            pointB = normal * radiusB + centerB;

            if (distance < 0.0f)
                return true;
            else
                return false;
        }

        /*
         *  Minkowski Portal Refinement 2D collision detection
         */

        public static bool Detect(IDebugDrawer drawer, ISupportMappable support1, ISupportMappable support2,
            ref float orientation1, ref float orientation2,
            ref JVector position1, ref JVector position2,
            out JVector point, out JVector normal, out float penetration, out int iterations)
        {
            // Used variables
            JVector temp1, temp2;
            JVector v01, v02, v0;
            JVector v11, v12, v1;
            JVector v21, v22, v2;
            JVector v31, v32, v3;
            JVector mn;

            // Initialization of the output
            point = normal = JVector.Zero;
            penetration = 0.0f;
            iterations = 1;

            JMatrix m1 = JMatrix.CreateRotationZ(orientation1) * JMatrix.CreateTranslation(position1);
            JMatrix m2 = JMatrix.CreateRotationZ(orientation2) * JMatrix.CreateTranslation(position2);

            JMatrix o1 = JMatrix.CreateRotationZ(orientation1);
            JMatrix o2 = JMatrix.CreateRotationZ(orientation2);

#if DEBUG   // draw shapes and minkowski difference
            JVector dir = JVector.Up;
            JVector dir2 = JVector.Down;
            JVector u = JVector.Zero;
            JVector a, b, c;

            for (int i = -1; i <= 36; i++)
            {
                SupportMapTransformed(support1, ref o1, ref position1, ref dir2, out b);
                SupportMapTransformed(support2, ref o2, ref position2, ref dir, out c);
                JVector.Subtract(ref c, ref b, out a);

                dir = JVector.Transform(dir, JMatrix.CreateRotationZ(0.0174532925f * 10f));
                dir2 = JVector.Negate(dir);
                if (i >= 0)
                {
                    drawer.SetColor(1, 1, 1, 0.5f);
                    drawer.DrawLine(a, u);
                }
                u = a;
            }
            
#endif

            // Step 1:  We obtain a point that we know lies somewhere deep within B–A. We can obtain such a point by subtracting any point 
            //              deep within A from any point deep within B. The geometric centers of A and B are excellent choices.

            // Get the center of shape1 in world coordinates -> v01
            support1.SupportCenter(out v01);
            JVector.Transform(ref v01, ref m1, out v01);

            // Get the center of shape2 in world coordinates -> v02
            support1.SupportCenter(out v02);
            JVector.Transform(ref v02, ref m2, out v02);

            // v0 is the center of the minkowski difference
            JVector.Subtract(ref v02, ref v01, out v0);

#if DEBUG   // draw v0
            drawer.DrawPoint(v0);
#endif

            // Avoid case where centers overlap -- any direction is fine in this case
            if (v0.IsNearlyZero()) v0 = new JVector(0.00001f, 0);

            // Step 2:  We construct a normal that originates at the interior point and points towards the origin. We then find the support point in the direction of this ray.
            mn = v0;    // mn = normal towards v0
            JVector.Negate(ref v0, out normal);     // normal = normal towards origin from v0

#if DEBUG   // draw normal's
            drawer.SetColor(0.5f, 0.5f, 0, 1);
            drawer.DrawLine(JVector.Zero, normal * 1f);
            drawer.SetColor(0f, 0.5f, 0.5f, 1);
            drawer.DrawLine(JVector.Zero, mn * 1f);
#endif

            SupportMapTransformed(support1, ref o1, ref position1, ref mn, out v11);
            SupportMapTransformed(support2, ref o2, ref position2, ref normal, out v12);
            JVector.Subtract(ref v12, ref v11, out v1);

#if DEBUG   // draw transformed supports
            drawer.DrawPoint(v1);
#endif

            // Step 3:  We construct a ray that is perpendicular to the line between the support just discovered and the interior point. There are two choices for this ray, 
            //              one for each side of the line segment. We choose the ray that lies on the same side of the segment as the origin. We use this ray to find a second 
            //              support point on the surface of B–A.

            //JVector perp = new JVector(-normal.Y, normal.X);
            //normal = perp;
            normal = OutsidePortal(v0, v1);
            mn = JVector.Negate(normal);

#if DEBUG   // draw normal's
            drawer.SetColor(0.5f, 0.5f, 1, 1);
            drawer.DrawLine(JVector.Zero, normal * 1f);
            drawer.SetColor(1f, 0.5f, 1f, 1);
            drawer.DrawLine(JVector.Zero, mn * 1f);
#endif

            SupportMapTransformed(support1, ref o1, ref position1, ref mn, out v21);
            SupportMapTransformed(support2, ref o2, ref position2, ref normal, out v22);
            JVector.Subtract(ref v22, ref v21, out v2);

#if DEBUG   // draw transformed supports
            drawer.DrawPoint(v2);
            drawer.SetColor(1, 1, 1, 0.1f);
            drawer.DrawTriangle(v0, v1, v2);
#endif
            if (JVector.Dot(ref v2, ref normal) <= 0.0f) return false;

            // phase two: portal refinement
            int maxIterations = 0;

            while (true)
            {
                iterations++;

#if DEBUG   // draw transformed supports
                drawer.DrawPoint(v2);
                drawer.SetColor(0, 1, 1, 0.25f);
                drawer.DrawTriangle(v0, v1, v2);
#endif

                // find normal direction
                if (!IntersectPortal(v0, v2, v1))
                    normal = InsidePortal(v2, v1);
                else
                    // origin ray crosses the portal
                    normal = OutsidePortal(v2, v1);

                // obtain the next support point
                JVector.Negate(ref normal, out mn);
                SupportMapTransformed(support1, ref o1, ref position1, ref mn, out v31);
                SupportMapTransformed(support2, ref o2, ref position2, ref normal, out v32);
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
                if (JVector.Dot((v3 - v2), normal) <= 0.00000001f || ++maxIterations > 10)
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

                    point = s * v11 + t * v21;
                    var point2 = s * v12 + t * v22;

                    penetration = JVector.Dot(point, normal);
#if DEBUG
                    drawer.DrawPoint(point);
                    drawer.DrawPoint(point2);
                   drawer.SetColor(0, 0, 0, 1f);
                   //drawer.DrawLine(point, point + (normal * 2f));
#endif

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

        private static void SupportMapTransformed(ISupportMappable support, ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        {
            // THIS IS *THE* HIGH FREQUENCY CODE OF THE COLLLISION PART OF THE ENGINE
            // so once we have it working well...inline it!
            // rotate the direction vector into the shapes space
            JVector.TransposedTransform(ref direction, ref orientation, out result);
            // get the support in the given direction
            JVector s; support.SupportMapping(ref result, out s);
            // transform the support into world space
            result = JVector.Transform(s, orientation) + position;

#if DEBUG// debugging
            if (float.IsNaN(result.X) || float.IsNaN(result.Y))
                throw new NotFiniteNumberException();
#endif
        }
    }
}
