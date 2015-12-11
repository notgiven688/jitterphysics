using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Forces;
using Jitter.Dynamics.Constraints;

namespace JitterDemo.Scenes
{
    public class Cloth : Scene
    {

        public Cloth(JitterDemo demo)
            : base(demo)
        {


        }

        public override void Build()
        {

            AddGround();

            // we need some of them!
            Demo.World.SetIterations(5);

            PseudoCloth pc = new PseudoCloth(Demo.World, 20,20, 0.5f);

            BoxShape boxShape = new BoxShape(JVector.One);

            RigidBody[] boxes = new RigidBody[4];

            int size = 19;

            for(int i=0;i<4;i++)
            {
                boxes[i] = new RigidBody(boxShape);
                boxes[i].Position = new JVector(i % 2 == 0 ? 10.0f : -0.5f, 10.5f, (i < 2) ? 10.0f : -0.5f);
               // Demo.World.AddBody(boxes[i]);



                if (i == 0)
                {

                    pc.GetCorner(size, size).IsStatic = true;
                }
                else if (i == 1)
                {

                    pc.GetCorner(size, 0).IsStatic = true;
                }
                else if (i == 2)
                {

                    pc.GetCorner(0, size).IsStatic = true;
                }
                else if (i == 3)
                {
                 
                   pc.GetCorner(0, 0).IsStatic = true;
                }

                boxes[i].IsStatic = true;
            }

            RigidBody sphereBody = new RigidBody(new SphereShape(2.0f));
            Demo.World.AddBody(sphereBody);
            sphereBody.Mass = 10.0f;
            sphereBody.Position = new JVector(5, 20, 5);

            //ConvexHullObject2 obj2 = new ConvexHullObject2(this.Demo);
            //Demo.Components.Add(obj2);

            //obj2.body.Position = new JVector(5, 30, 5);
            //Demo.World.AddBody(obj2.body);

        }

        public override void Destroy()
        {
            RemoveGround();
            this.Demo.World.Clear();
        }

    }
}
