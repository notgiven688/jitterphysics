using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter;
using Microsoft.Xna.Framework;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Dynamics.Constraints;
using Jitter.Dynamics.Joints;

namespace JitterDemo.Scenes
{
    class Ragdoll : Scene
    {

        public Ragdoll(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            //RigidBody body = new RigidBody(new BoxShape(JVector.One * 3));
            //body.Position = new JVector(0, 5, 0);
            //body.UseUserMassProperties(JMatrix.Zero, 1f, true);
            //Demo.World.AddBody(body);

            for (int i = 3; i < 8; i++)
            {
                for (int e = 3; e < 8; e++)
                {
                    BuildRagdoll(Demo.World, new JVector(i * 6 - 25, 5, e * 6 - 25));
                }
            }

        }


        public void BuildRagdoll(World world, JVector position)
        {
            // the torso
            RigidBody torso = new RigidBody(new BoxShape(1.5f, 3, 0.5f));
            torso.Position = position;

            // the head
            RigidBody head = new RigidBody(new SphereShape(0.5f));
            head.Position = position + new JVector(0, 2.1f, 0);

            // connect head and torso
            PointPointDistance headTorso = new PointPointDistance(head, torso,
                position + new JVector(0, 1.6f, 0), position + new JVector(0, 1.5f, 0));

            RigidBody arm1 = new RigidBody(new CapsuleShape(0.8f, 0.2f));
            arm1.Position = position + new JVector(1.0f, 0.75f, 0);

            RigidBody arm2 = new RigidBody(new CapsuleShape(0.8f, 0.2f));
            arm2.Position = position + new JVector(-1.0f, 0.75f, 0);

            RigidBody lowerarm1 = new RigidBody(new CapsuleShape(0.6f, 0.2f));
            lowerarm1.Position = position + new JVector(1.0f, -0.45f, 0);

            RigidBody lowerarm2 = new RigidBody(new CapsuleShape(0.6f, 0.2f));
            lowerarm2.Position = position + new JVector(-1.0f, -0.45f, 0);

            PointOnPoint arm1torso = new PointOnPoint(arm1, torso, position + new JVector(0.9f, 1.4f, 0));
            PointOnPoint arm2torso = new PointOnPoint(arm2, torso, position + new JVector(-0.9f, 1.4f, 0));

            HingeJoint arm1Hinge = new HingeJoint(world, arm1, lowerarm1, position + new JVector(1.0f, 0.05f, 0), JVector.Right);
            HingeJoint arm2Hinge = new HingeJoint(world, arm2, lowerarm2, position + new JVector(-1.0f, 0.05f, 0), JVector.Right);

            RigidBody leg1 = new RigidBody(new CapsuleShape(1.0f, 0.3f));
            leg1.Position = position + new JVector(-0.5f, -2.4f, 0);

            RigidBody leg2 = new RigidBody(new CapsuleShape(1.0f, 0.3f));
            leg2.Position = position + new JVector(0.5f, -2.4f, 0);

            PointOnPoint leg1torso = new PointOnPoint(leg1, torso, position + new JVector(-0.5f, -1.6f, 0));
            PointOnPoint leg2torso = new PointOnPoint(leg2, torso, position + new JVector(+0.5f, -1.6f, 0));

            RigidBody lowerleg1 = new RigidBody(new CapsuleShape(0.8f, 0.3f));
            lowerleg1.Position = position + new JVector(-0.5f, -4.0f, 0);

            RigidBody lowerleg2 = new RigidBody(new CapsuleShape(0.8f, 0.3f));
            lowerleg2.Position = position + new JVector(+0.5f, -4.0f, 0);

            HingeJoint leg1Hinge = new HingeJoint(world, leg1, lowerleg1, position + new JVector(-0.5f, -3.35f, 0), JVector.Right);
            HingeJoint leg2Hinge = new HingeJoint(world, leg2, lowerleg2, position + new JVector(0.5f, -3.35f, 0), JVector.Right);

            lowerleg1.IsActive = false;
            lowerleg2.IsActive = false;
            leg1.IsActive = false;
            leg2.IsActive = false;
            head.IsActive = false;
            torso.IsActive = false;
            arm1.IsActive = false;
            arm2.IsActive = false;
            lowerarm1.IsActive = false;
            lowerarm2.IsActive = false;

            world.AddBody(head); world.AddBody(torso);
            world.AddBody(arm1); world.AddBody(arm2);
            world.AddBody(lowerarm1); world.AddBody(lowerarm2);
            world.AddBody(leg1); world.AddBody(leg2);
            world.AddBody(lowerleg1); world.AddBody(lowerleg2);

            arm1Hinge.Activate(); arm2Hinge.Activate();
            leg1Hinge.Activate(); leg2Hinge.Activate();

            world.AddConstraint(headTorso);
            world.AddConstraint(arm1torso);
            world.AddConstraint(arm2torso);
            world.AddConstraint(leg1torso);
            world.AddConstraint(leg2torso);
        }

    }
}