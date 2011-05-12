using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter;
using Microsoft.Xna.Framework;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;

namespace JitterDemo.Scenes
{
    class Restitution : Scene
    {

        public Restitution(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            for (int i = 0; i < 11; i++)
            {
                RigidBody box = new RigidBody(new BoxShape(JVector.One));
                this.Demo.World.AddBody(box);
                JVector boxPos = new JVector(-15 + i * 3 + 5, 5, 0);

                box.Position = boxPos;
                box.IsStatic = true;

                RigidBody sphere = new RigidBody(new SphereShape(0.5f));
                this.Demo.World.AddBody(sphere);

                sphere.Position = boxPos + JVector.Up * 30;
               
                // set restitution
                sphere.Material.Restitution = box.Material.Restitution = i / 10.0f;

                sphere.Damping = RigidBody.DampingType.Angular;
            }

         
        }


        public override void Destroy()
        {
            RemoveGround();
            Demo.World.Clear();
        }


    }
}