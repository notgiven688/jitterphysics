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
    class Rope : Scene
    {

        public Rope(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            RigidBody last = null;

            for (int i = 0; i < 12; i++)
            {
                RigidBody body = new RigidBody(new BoxShape(JVector.One));
                body.Position = new JVector(i * 1.5f-20, 0.5f, 0);

                JVector jpos2 = body.Position;

                Demo.World.AddBody(body);
                body.Update();

                if (last != null)
                {
                    JVector jpos3 = last.Position;

                    JVector dif; JVector.Subtract(ref jpos2, ref jpos3, out dif);
                    JVector.Multiply(ref dif, 0.5f, out dif);
                    JVector.Subtract(ref jpos2, ref dif, out dif);

                    Constraint cons = new PointOnPoint(last, body, dif);
                    Demo.World.AddConstraint(cons);
                }

                last = body;
            }
           
        }


    }
}