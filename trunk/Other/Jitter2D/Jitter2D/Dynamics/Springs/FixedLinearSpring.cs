using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter2D.LinearMath;

namespace Jitter2D.Dynamics.Springs
{
    public class FixedLinearSpring : Spring
    {
        public bool IsOnlyPull { get; set; }

        public JVector LocalAnchor { get; set; }

        public JVector WorldAnchor { get; set; }

        public RigidBody Body { get; set; }

        public float Length { get; set; }

        public float SpringConstant { get; set; }

        public float DampingConstant { get; set; }

        public float SpringError { get; set; }


        public FixedLinearSpring(RigidBody body, JVector localAnchor, JVector worldAnchor, float springConstant, float dampingConstant)
        {
            Body = body;
            LocalAnchor = localAnchor;
            WorldAnchor = worldAnchor;
            SpringConstant = springConstant;
            DampingConstant = dampingConstant;
            Length = (worldAnchor - Body.LocalToWorld(localAnchor)).Length();
        }

        public override void Update(float timestep)
        {
            if (Body.IsStaticOrInactive)
                return;

            var worldBodyAnchor = Body.LocalToWorld(LocalAnchor);

            var difference = worldBodyAnchor - WorldAnchor;
            var differenceMag = difference.Length();

            if (IsOnlyPull)
            {
                if (differenceMag < Length)
                    return;
            }

            if (JMath.IsNearlyZero(differenceMag))
                return;

            var diffNormal = JVector.Normalize(difference);

            SpringError = differenceMag - Length;

            var springForce = SpringConstant * SpringError;

            var force = diffNormal * -springForce;

            if (!force.IsNearlyZero())
            {
                Body.AddForce(force, worldBodyAnchor);
            }
        }

        public override void DebugDraw(IDebugDrawer debugDrawer)
        {
            debugDrawer.DrawLine(Body.LocalToWorld(LocalAnchor), WorldAnchor);
        }
    }
}
