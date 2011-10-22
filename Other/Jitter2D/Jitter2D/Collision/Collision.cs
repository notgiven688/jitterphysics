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
        public static bool CircleCircleTest(JVector centerA, float radiusA, JVector centerB, float radiusB, out JVector pointA, out JVector pointB, out JVector normal, out float distance)
        {
            // ||A-B|| - (r1+r2) < 0
            float d = JVector.Distance(centerA, centerB);
            float r = radiusA + radiusB;
            
            distance = d - r;

            normal = (centerA - centerB) / d;

            //penetrationVector = normal * distance;

            // calculate closest 2 points
            pointA = JVector.Negate(normal) * radiusA + centerA;
            pointB = normal * radiusB + centerB;

            if (distance < 0.0f)
                return true;
            else
                return false;
        }

        public static bool CircleCapsuleTest(JVector centerA, float radiusA, JVector centerB, JVector axis, float length, float radiusB, out JVector pointA, out JVector pointB, out JVector normal, out float distance)
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
