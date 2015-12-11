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
        // These tests are fully discrete, meaning no motion is considered, this makes them very fast, but they can easily miss
        // collisions on fast moving or spinning shapes.
        // 
        // Some of these tests do not return a boolean. This is because braches can be very costly and I try to leave it up to the 
        // user on when to branch. Use the distance variable to determine if two shapes are colliding. If distance is positive
        // then there is no collision, if negative then the shapes overlap.
        #region Discrete Tests

        /// <summary>
        /// Discrete Box vs Box test. Very fast. No contact info. NOTE: ensure UpdateAxes is called for each BoxShape prior.
        /// </summary>
        /// <param name="A">BoxShape A.</param>
        /// <param name="PA">BoxShape A's position.</param>
        /// <param name="B">BoxShape B.</param>
        /// <param name="PB">BoxShape B's position.</param>
        /// <returns></returns>
        public static bool BoxBoxTest(ref BoxShape A, ref JVector PA, ref BoxShape B, ref JVector PB)
        {
            JVector T = PB - PA;

            // cache scaled local axes
            JVector Ax = A.halfSize.X * A.xAxis;
            JVector Ay = A.halfSize.Y * A.yAxis;
            JVector Bx = B.halfSize.X * B.xAxis;
            JVector By = B.halfSize.Y * B.yAxis;

            // axis to test
            JVector L = A.xAxis;
            float TL = Math.Abs(T * L);

            float a = Math.Abs((Ax) * L) + Math.Abs((Ay) * L);
            float b = Math.Abs((Bx) * L) + Math.Abs((By) * L);

            if (TL > (a + b))
                return false;

            // axis to test
            L = A.yAxis;
            TL = Math.Abs(T * L);

            a = Math.Abs((Ax) * L) + Math.Abs((Ay) * L);
            b = Math.Abs((Bx) * L) + Math.Abs((By) * L);

            if (TL > (a + b))
                return false;

            // axis to test
            L = B.xAxis;
            TL = Math.Abs(T * L);

            a = Math.Abs((Ax) * L) + Math.Abs((Ay) * L);
            b = Math.Abs((Bx) * L) + Math.Abs((By) * L);

            if (TL > (a + b))
                return false;

            // axis to test
            L = B.yAxis;
            TL = Math.Abs(T * L);

            a = Math.Abs((Ax) * L) + Math.Abs((Ay) * L);
            b = Math.Abs((Bx) * L) + Math.Abs((By) * L);

            if (TL > (a + b))
                return false;

            // if all axes overlap collision exists
            return true;
        }

       /// <summary>
        /// Discrete Box vs Box test. Very fast. Generates contact info. NOTE: ensure UpdateAxes is called for each BoxShape prior.
       /// </summary>
       /// <param name="A">BoxShape A.</param>
       /// <param name="PA">BoxShape A's position.</param>
       /// <param name="OA">BoxShape A's orientation.</param>
       /// <param name="B">BoxShape B.</param>
       /// <param name="PB">BoxShape B's position.</param>
       /// <param name="OB">BoxShape B's orientation.</param>
       /// <param name="normal">Normal of collision.</param>
       /// <param name="t">Amount of penetration.</param>
       /// <param name="CA">Contacts found on BoxShape A.</param>
        /// <param name="CB">Contacts found on BoxShape B.</param>
       /// <param name="NumContacts">Number of contacts found.</param>
       /// <returns>True if collision exists.</returns>
        public static bool BoxBoxTestContact(ref BoxShape A, ref JVector PA, ref JMatrix OA,
            ref BoxShape B, ref JVector PB, ref JMatrix OB,
            out JVector normal, out float t, out JVector[] CA, out JVector[] CB, out int NumContacts)
        {
            float overlap = 0;
            normal = A.xAxis;
            t = 0.0f;
            // TODO in future to save a few bytes of garbage make these
            // SA1, SA2 and SB1, SB2 (no arrays)
            JVector[] SA = new JVector[2], SB = new JVector[2];
            int NumSA = 0, NumSB = 0;

            CA = new JVector[2]; CB = new JVector[2];
            NumContacts = 0;

            JVector T = PB - PA;

            // cache scaled local axes
            JVector Ax = A.halfSize.X * A.xAxis;
            JVector Ay = A.halfSize.Y * A.yAxis;
            JVector Bx = B.halfSize.X * B.xAxis;
            JVector By = B.halfSize.Y * B.yAxis;

            // axis to test
            JVector L = A.xAxis;
            float TL = Math.Abs(T * L);

            float a = Math.Abs((Ax) * L) + Math.Abs((Ay) * L);
            float b = Math.Abs((Bx) * L) + Math.Abs((By) * L);

            if (TL > (a + b))
                return false;

            // cache overlap
            // just set first overlap
            overlap = TL - (b + a);

            // axis to test
            L = A.yAxis;
            TL = Math.Abs(T * L);

            a = Math.Abs((Ax) * L) + Math.Abs((Ay) * L);
            b = Math.Abs((Bx) * L) + Math.Abs((By) * L);

            if (TL > (a + b))
                return false;

            // cache overlap
            var o = TL - (b + a);
            if (o > overlap)
            {
                overlap = o;
                normal = A.yAxis;
            }

            // axis to test
            L = B.xAxis;
            TL = Math.Abs(T * L);

            a = Math.Abs((Ax) * L) + Math.Abs((Ay) * L);
            b = Math.Abs((Bx) * L) + Math.Abs((By) * L);

            if (TL > (a + b))
                return false;

            // cache overlap
            o = TL - (b + a);
            if (o > overlap)
            {
                overlap = o;
                normal = B.xAxis;
            }

            // axis to test
            L = B.yAxis;
            TL = Math.Abs(T * L);

            a = Math.Abs((Ax) * L) + Math.Abs((Ay) * L);
            b = Math.Abs((Bx) * L) + Math.Abs((By) * L);

            if (TL > (a + b))
                return false;

            // cache overlap
            o = TL - (b + a);
            if (o > overlap)
            {
                overlap = o;
                normal = B.yAxis;
            }

            // make sure the polygons gets pushed away from each other.
            if (normal * T < 0.0f)
                normal = -normal;

            t = overlap;

            // now to find a contact point or edge!
            var mn = -normal;

            NumSA = A.FindSupportPoints(ref mn, ref PA, ref OA, out SA);
            NumSB = B.FindSupportPoints(ref normal, ref PB, ref OB, out SB);

            // now refine contact points from support points
            // edge/vertex case
            if (NumSA == 2 && NumSB == 1)
            {
                ProjectPointOnLine(ref SB[0], ref SA[0], ref SA[1], out CA[NumContacts]);
                CB[NumContacts] = SB[0];
                NumContacts++;
                return true;
            }
            // vertex/edge case
            if (NumSA == 1 && NumSB == 2)
            {
                ProjectPointOnLine(ref SA[0], ref SB[0], ref SB[1], out CB[NumContacts]);
                CA[NumContacts] = SA[0];
                NumContacts++;
                return true;
            }
            // edge/edge case
            if (NumSA == 2 && NumSB == 2)
            {
                // clip contacts
                JVector perp = new JVector(-normal.Y, normal.X);
                // project first 2 contacts to axis perpendicular to normal
                float min0 = SA[0] * perp;
                float max0 = min0;
                float min1 = SB[0] * perp;
                float max1 = min1;
                // project next point from A
                max0 = SA[1] * perp;

                if (max0 < min0)
                {
                    JMath.Swapf(ref min0, ref max0);
                    JVector.Swap(ref SA[0], ref SA[1]);
                }

                max1 = SB[1] * perp;

                if (max1 < min1)
                {
                    JMath.Swapf(ref min1, ref max1);
                    JVector.Swap(ref SB[0], ref SB[1]);
                }

                if (min0 > min1)
                {
                    JVector Pseg;
                    ProjectPointOnLine(ref SA[0], ref SB[0], ref SB[1], out Pseg);
                    CA[NumContacts] = SA[0];
                    CB[NumContacts] = Pseg;
                    NumContacts++;
                }
                else
                {
                    JVector Pseg;
                    ProjectPointOnLine(ref SB[0], ref SA[0], ref SA[1], out Pseg);
                    CA[NumContacts] = Pseg;
                    CB[NumContacts] = SB[0];
                    NumContacts++;
                }

                if (max0 < max1)
                {
                    JVector Pseg;
                    ProjectPointOnLine(ref SA[1], ref SB[0], ref SB[1], out Pseg);
                    CA[NumContacts] = SA[1];
                    CB[NumContacts] = Pseg;
                    NumContacts++;
                }
                else
                {
                    JVector Pseg;
                    ProjectPointOnLine(ref SB[1], ref SA[0], ref SA[1], out Pseg);
                    CA[NumContacts] = Pseg;
                    CB[NumContacts] = SB[1];
                    NumContacts++;
                }

                return true;
            }

            // if all axes overlap collision exists
            return true;
        }

        /// <summary>
        /// Discrete Circle vs Circle test. Very fast. Generates contact info.
        /// NOTE: check distance for collisions. If negative then a collision has occurred.
        /// This is done to remove all branches from this test and leave it to the user to decide when to branch.
        /// </summary>
        public static void CircleCircleTest(JVector centerA, float radiusA, JVector centerB, float radiusB, out JVector pointA, out JVector pointB, out JVector normal, out float distance)
        {
            // ||A-B|| - (r1+r2) < 0
            float d = JVector.DistanceSquared(centerA, centerB);
            float r = (radiusA + radiusB);
            r *= r;

            distance = d - r;

            normal = (centerA - centerB) / d;
            normal.Normalize();

            // calculate closest 2 points
            pointA = JVector.Negate(normal) * radiusA + centerA;
            pointB = normal * radiusB + centerB;
        }

        #endregion

        #region Continuous Tests



        #endregion

        #region Toolbox

        //----------------------------------------------------------------------------------------------- 
        // Find closest point on a segment to a vertex
        //----------------------------------------------------------------------------------------------- 
        internal static bool ProjectPointOnSegment(JVector V, JVector A, JVector B, out JVector W, out float t)
        {
            JVector AV = V - A;
            JVector AB = B - A;
            t = (AV * AB) / (AB * AB);

            if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;

            W = A + t * AB;

            return true;
        }

        // this version avoids all branches, but doesn't clamp the vector to the segment
        // When clamping isn't needed this saves expensive branches
        internal static void ProjectPointOnLine(ref JVector V, ref JVector A, ref JVector B, out JVector W)
        {
            JVector AV = V - A;
            JVector AB = B - A;
            float t = (AV * AB) / (AB * AB);

            W = A + t * AB;
        }

        #endregion


        internal static bool CircleBoxTest(ref CircleShape A, ref JVector PA, ref BoxShape B, ref JVector PB, ref JMatrix OB)
        {
            // find vertex closest to circles center
            // move circle into boxes space
            var pa = JVector.TransposedTransform(PA - PB, OB);
            // find normal from box to circles center
            var axis = pa;
            JVector closestVertex;
            // find closest vertex
            B.SupportMapping(ref axis, out closestVertex);
            // do a SAT test on that axis

            // axis to test
            JVector T = pa - closestVertex;
            float TL = Math.Abs(T * axis);

            float a = Math.Abs(pa * axis);
            float b = Math.Abs(closestVertex * axis);

            if (TL > (a + b + A.Radius))
                return false;

            return true;
        }

        internal static bool CircleCapsuleTest(JVector centerA, float radiusA, JVector centerB, JVector axis, float length, float radiusB, out JVector pointA, out JVector pointB, out JVector normal, out float distance)
        {
            // get capsule endpoints
            var p0 = centerB - axis * (length * 0.5f);
            var p1 = centerB + axis * (length * 0.5f);

            // get vector from endpoint to circle
            var D = centerA - p0;

            // project vector onto axis and clamp
            var d = JVector.Dot(D, axis);
            d = JMath.Clamp(d, 0, length);

            // get point on axis
            var R = p0 + axis * d;

            // distance
            var b = Math.Abs((centerA - R).Length());

            normal = (centerA - R) / b;
            
            // calculate closest 2 points
            var RH = JVector.Normalize(centerA - R);

            pointA = JVector.Negate(RH) * radiusA + centerA;
            pointB = RH * radiusB + R;

            normal.Negate();

            distance = b - (radiusA + radiusB);

            // 
            if (b < radiusA + radiusB)
                return true;
            return false;
        }
    }
}
