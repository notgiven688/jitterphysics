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
            AddGround();

            //for (int i = 0; i < 10; i++)
            //{
            //    for (int e = i; e < 10; e++)
            //    {
            //        RigidBody body = new RigidBody(new BoxShape(new JVector(1.0f, 1.0f)));
            //        body.Position = new JVector((e - i * 0.5f) * 2.01f - 10f, 0.5f + i * 2.0f - 9);
            //        body.Orientation = 1;
            //        Demo.World.AddBody(body);
            //        //body.IsParticle = true;
            //        //body.AffectedByGravity = false;
            //        body.Material.Restitution = 0.0f;
            //    }
            //}
        }
    }


}
