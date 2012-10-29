using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Jitter2D.Dynamics;
using Jitter2D.LinearMath;
using Jitter2D.Collision.Shapes;
//using JitterDemo.Vehicle;

namespace JitterDemo.Scenes
{
    public abstract class Scene
    {
        public JitterDemo Demo { get; private set; }

        public Scene(JitterDemo demo)
        {
            this.Demo = demo;
        }

        public abstract void Build();

        //private QuadDrawer quadDrawer = null;
        protected RigidBody ground = null;
        //protected CarObject car = null;

        public void AddGround(bool solid)
        {
            if (solid)
            {
                ground = new RigidBody(new BoxShape(3600, 1f));
                //ground.SetMassProperties(float.MaxValue, float.MaxValue, false);
                ground.Position = new JVector(0, 0);
                ground.IsStatic = true; 
                Demo.World.AddBody(ground);
                ground.Material.DynamicFriction = 1.0f;
                ground.Material.StaticFriction = 1.0f;
                ground.Material.Restitution = 0.0f;
            }
            else
            {
                List<JVector> groundLine = new List<JVector> { new JVector(-40f, 0f), new JVector(40f, 0f) };

                ground = new RigidBody(new BoxShape(new JVector(1.0f, 1.0f)));
                ground.Position = new JVector(0, -10);
                ground.Orientation = 0;
                ground.IsStatic = true;
                ground.SetMassProperties(float.MaxValue, float.MaxValue, false);
                Demo.World.AddBody(ground);
                ground.Material.DynamicFriction = 1.0f;
                ground.Material.Restitution = 0.0f;
            }
        }

        public void RemoveGround()
        {
            Demo.World.RemoveBody(ground);
        }

        public virtual void Draw() { }

    }
}
