using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter2D.Collision.Shapes;
using Jitter2D.LinearMath;
using Jitter2D.Dynamics;
using Microsoft.Xna.Framework;
using Jitter2D;
using Jitter2D.Dynamics.Constraints;
using Jitter2D.Dynamics.Joints;

namespace JitterDemo.Scenes
{

    public class EmptyScene : Scene
    {

        public EmptyScene(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround(true);

            ground.Material.DynamicFriction = 0;
            ground.Material.StaticFriction = 0;

            RigidBody body = new RigidBody(new BoxShape(3f, 1f));
            body.Position = new JVector(0, 0);
            body.Orientation = 0;
            Demo.World.AddBody(body);
            body.Material.Restitution = 0.0f;
            body.Material.DynamicFriction = 1;
            body.Material.StaticFriction = 1;

            RigidBody body2 = new RigidBody(new BoxShape(1f, 3f));
            body2.Position = new JVector(10, 0);
            body2.Orientation = 0;
            Demo.World.AddBody(body2);
            body2.Material.Restitution = 0.0f;
            body2.Material.DynamicFriction = 1;
            body2.Material.StaticFriction = 1;

            //FixedAngle rev = new FixedAngle(body, body2);
            //rev.Behavior = Distance.DistanceBehavior.LimitMaximumDistance;
            //Demo.World.AddConstraint(rev);

            WeldJoint wj = new WeldJoint(Demo.World, body, body2);
            wj.Activate();
        }
    }


}
