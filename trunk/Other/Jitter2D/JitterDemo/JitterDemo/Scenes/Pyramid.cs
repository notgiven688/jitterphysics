using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter2D;
using Microsoft.Xna.Framework;
using Jitter2D.Collision.Shapes;
using Jitter2D.Dynamics;
using Jitter2D.LinearMath;
using Jitter2D.Dynamics.Constraints;

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
            AddGround(true);

            var width = 1f;
            var height = 1f;
            var horizontal_spacing = 0.1f;
            var veritcal_spacing = 0.1f;

            for (int i = 0; i < 10; i++)
            {
                for (int e = i; e < 10; e++)
                {
                    RigidBody body = new RigidBody(new BoxShape(new JVector(width, height)));
                    body.Position = new JVector((e - i * 0.5f) * (width + horizontal_spacing) - ((width + horizontal_spacing) * 5), (height + veritcal_spacing * 0.5f) + i * height + 0.26f);
                    Demo.World.AddBody(body);
                    body.AffectedByGravity = true;
                    body.Material.Restitution = 0.0f;
                }
            }

        }

    }
}