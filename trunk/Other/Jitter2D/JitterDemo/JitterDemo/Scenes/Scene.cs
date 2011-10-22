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

        public void AddGround()
        {
            for (int i = 0; i < 20; i++)
            {
                ground = new RigidBody(new CircleShape(2));
                ground.SetMassProperties(float.MaxValue, float.MaxValue, false);
                ground.Position = new JVector(i * 4f - 20f, -10);
                //ground.Tag = BodyTag.DontDrawMe;
                ground.IsStatic = true; Demo.World.AddBody(ground);
                //ground.Restitution = 1.0f;
                ground.Material.DynamicFriction = 1.0f;
            }

            //quadDrawer = new QuadDrawer(Demo, 100);
            //Demo.Components.Add(quadDrawer);
        }

        public void RemoveGround()
        {
            Demo.World.RemoveBody(ground);
            //Demo.Components.Remove(quadDrawer);
           // quadDrawer.Dispose();
        }

        public void AddCar(JVector position)
        {
            //car = new CarObject(Demo);
            //this.Demo.Components.Add(car);

            //car.carBody.Position = position;
        }

        public void RemoveCar()
        {
            //Demo.World.RemoveBody(car.carBody);
            //Demo.Components.Remove(quadDrawer);
            //Demo.Components.Remove(car);
        }


        public virtual void Draw() { }

    }
}
