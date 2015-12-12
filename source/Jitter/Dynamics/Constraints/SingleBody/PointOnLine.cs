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

namespace Jitter.Dynamics.Constraints.SingleBody
{

    /// <summary>
    /// </summary>
    public class PointOnLine : Constraint
    {
        private JVector localAnchor1;
        private JVector r1;

        private JVector lineNormal = JVector.Right;
        private JVector anchor;

        private float biasFactor = 0.5f;
        private float softness = 0.0f;

        /// <summary>
        /// Initializes a new instance of the WorldLineConstraint.
        /// </summary>
        /// <param name="body">The body of the constraint.</param>
        /// <param name="localAnchor">The anchor point on the body in local (body)
        /// coordinates.</param>
        /// <param name="lineDirection">The axis defining the line in world space.</param>/param>
        public PointOnLine(RigidBody body, JVector localAnchor, JVector lineDirection)
            : base(body, null)
        {
            if (lineDirection.LengthSquared() == 0.0f)
                throw new ArgumentException("Line direction can't be zero", "lineDirection");

            localAnchor1 = localAnchor;
            this.anchor = body.position + JVector.Transform(localAnchor, body.orientation);

            this.lineNormal = lineDirection;
            this.lineNormal.Normalize();
        }

        /// <summary>
        /// The anchor point of the body in world space.
        /// </summary>
        public JVector Anchor { get { return anchor; } set { anchor = value; } }

        /// <summary>
        /// The axis defining the line of the constraint.
        /// </summary>
        public JVector Axis { get { return lineNormal; } set { lineNormal = value; lineNormal.Normalize(); } }

        /// <summary>
        /// Defines how big the applied impulses can get.
        /// </summary>
        public float Softness { get { return softness; } set { softness = value; } }

        /// <summary>
        /// Defines how big the applied impulses can get which correct errors.
        /// </summary>
        public float BiasFactor { get { return biasFactor; } set { biasFactor = value; } }

        float effectiveMass = 0.0f;
        float accumulatedImpulse = 0.0f;
        float bias;
        float softnessOverDt;

        JVector[] jacobian = new JVector[2];

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The simulation timestep</param>
        public override void PrepareForIteration(float timestep)
        {
            JVector.Transform(ref localAnchor1, ref body1.orientation, out r1);

            JVector p1, dp;
            JVector.Add(ref body1.position, ref r1, out p1);

            JVector.Subtract(ref p1, ref anchor, out dp);

            JVector l = lineNormal;

            JVector t = (p1 - anchor) % l;
            if (t.LengthSquared() != 0.0f) t.Normalize();
            t = t % l;

            jacobian[0] = t;
            jacobian[1] = r1 % t;

            effectiveMass = body1.inverseMass
                + JVector.Transform(jacobian[1], body1.invInertiaWorld) * jacobian[1];

            softnessOverDt = softness / timestep;
            effectiveMass += softnessOverDt;

            if (effectiveMass != 0) effectiveMass = 1.0f / effectiveMass;

            bias = -(l % (p1 - anchor)).Length() * biasFactor * (1.0f / timestep);

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                body1.angularVelocity += JVector.Transform(accumulatedImpulse * jacobian[1], body1.invInertiaWorld);
            }

        }

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public override void Iterate()
        {
            float jv =
                body1.linearVelocity * jacobian[0] +
                body1.angularVelocity * jacobian[1];

            float softnessScalar = accumulatedImpulse * softnessOverDt;

            float lambda = -effectiveMass * (jv + bias + softnessScalar);

            accumulatedImpulse += lambda;

            if (!body1.isStatic)
            {
                body1.linearVelocity += body1.inverseMass * lambda * jacobian[0];
                body1.angularVelocity += JVector.Transform(lambda * jacobian[1], body1.invInertiaWorld);
            }
        }

        public override void DebugDraw(IDebugDrawer drawer)
        {
            drawer.DrawLine(anchor - lineNormal * 50.0f, anchor + lineNormal * 50.0f);
            drawer.DrawLine(body1.position, body1.position + r1);
        }

    }
}
