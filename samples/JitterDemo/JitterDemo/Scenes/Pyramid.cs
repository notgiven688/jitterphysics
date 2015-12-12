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

namespace JitterDemo.Scenes
{
    class Pyramid : Scene
    {
        public Pyramid(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            for (int i = 0; i < 30; i++)
            {
                for (int e = i; e < 30; e++)
                {
                    RigidBody body = new RigidBody(new BoxShape(new JVector(1.0f, 1.0f, 1.0f)));
                    body.Position = new JVector((e - i * 0.5f) * 1.01f + 7, 0.5f + i * 1.0f, 3.0f);
                    Demo.World.AddBody(body);
                    //body.IsParticle = true;
                    //body.AffectedByGravity = false;
                    body.Material.Restitution = 0.0f;
                }
            }

            //BoxShape shape = new BoxShape(JVector.One);

            //for (int i = 0; i < 20; i++)
            //{
            //    for (int e = 0; e < 20; e++)
            //    {
            //        for (int k = 0; k < 20; k++)
            //        {
            //            RigidBody b = new RigidBody(shape);
            //            Demo.World.AddBody(b);
            //            b.Position = new JVector(i, e, k) * 2.0f;
            //            b.AffectedByGravity = false;
            //        }
            //    }
            //}

        }

    }
}