using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Collision.Shapes;
using Jitter.LinearMath;
using Jitter.Dynamics;

namespace JitterDemo.Scenes
{
    public class EmptyScene : Scene
    {

        public EmptyScene(JitterDemo demo)
            : base(demo)
        {
        }

        Random rnd = new Random();

        public override void Build()
        {
            AddGround();

            //List<CompoundShape.TransformedShape> subshapes = new List<CompoundShape.TransformedShape>();

            //for (int i = 0; i < 20; i++)
            //{
            //    SphereShape ss = new SphereShape(0.3f);
            //    subshapes.Add(new CompoundShape.TransformedShape(ss, JMatrix.Identity, new JVector((float)rnd.Next(50) / 15.0f, (float)rnd.Next(50) / 15.0f, (float)rnd.Next(50) / 15.0f)));
            //}

            //CompoundShape cs = new CompoundShape(subshapes);

            //RigidBody body1 = new Jitter.Dynamics.RigidBody(cs);
            //RigidBody body2 = new Jitter.Dynamics.RigidBody(cs);

            //body2.Position = new JVector(0, 10, 0);

            //this.Demo.World.AddBody(body1);
            //this.Demo.World.AddBody(body2);

        }

        public override void Destroy()
        {
            RemoveGround();
        }
    }
}
