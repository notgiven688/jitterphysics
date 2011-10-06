using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Dynamics.Joints;

namespace JitterDemo.Scenes
{
    public class PrismaticJointTest : Scene
    {

        public PrismaticJointTest(JitterDemo demo)
            : base(demo)
        {


        }

        public override void Build()
        {
            AddGround();

            RigidBody body1 = new RigidBody(new BoxShape(1, 1, 1));
            RigidBody body2 = new RigidBody(new BoxShape(1, 1, 1));

            body1.Position = new JVector(0, 7, 0);
            body2.Position = new JVector(0, 4, 0);

            // add a prismatic joint.
            // the minimum allowed distance is 3
            // the maximum allowed distance is also 3
            // => the body should be fixed on the slider
            PrismaticJoint pj = new PrismaticJoint(Demo.World, body1, body2, 3, 3);

            // but we set very heigh softness (1.0f) to the minimum distance
            // so we have something like a suspension effect.
            pj.MaximumDistanceConstraint.Softness = 0.0f;
            pj.MinimumDistanceConstraint.Softness = 1.0f;
            pj.Activate();

            Demo.World.AddBody(body1);
            Demo.World.AddBody(body2);
        }

    }
}
