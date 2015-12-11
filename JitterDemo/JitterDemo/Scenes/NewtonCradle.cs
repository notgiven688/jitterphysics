using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Dynamics.Constraints;

namespace JitterDemo.Scenes
{
    public class NewtonCradle : Scene
    {

        public NewtonCradle(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            this.Demo.World.Solver = Jitter.World.SolverType.Sequential;

            AddGround();

            RigidBody boxb = new RigidBody(new BoxShape(7,1,2));
            boxb.Position = new JVector(3.0f,12,0);
            this.Demo.World.AddBody(boxb);
            boxb.Tag = BodyTag.DontDrawMe;

            boxb.IsStatic = true;

            this.Demo.World.Solver = Jitter.World.SolverType.Sequential;
            //this.Demo.World.SetDampingFactors(1.0f, 1.0f);

            SphereShape shape = new SphereShape(0.501f);

            for (int i = 0; i < 7; i++)
            {
                RigidBody body = new RigidBody(shape);
                body.Position = new JVector(i, 6, 0);

                DistanceConstraint dc1 = new DistanceConstraint(boxb, body, body.Position + JVector.Up * 6 + JVector.Backward * 5 + JVector.Down * 0.5f, body.Position);
                dc1.Softness = 1.0f;

                DistanceConstraint dc2 = new DistanceConstraint(boxb, body, body.Position + JVector.Up * 6 + JVector.Forward * 5 + JVector.Down * 0.5f, body.Position);
                dc2.Softness = 1.0f;

                dc1.BiasFactor = dc2.BiasFactor = 0.8f;

                dc1.IsMaxDistance = dc2.IsMaxDistance = false;

                this.Demo.World.AddBody(body);
                this.Demo.World.AddConstraint(dc1);
                this.Demo.World.AddConstraint(dc2);

                body.Restitution = 1.0f;
                body.StaticFriction = 1.0f;

              //  this.Demo.World.SetDampingFactors(1.0f, 1.0f);
            }

            //for (int i = 0; i < 5; i++)
            //{
            //    RigidBody sBody = new RigidBody(new SphereShape(0.5f));
            //    sBody.Position = new JVector(0, 0.5f, i);
            //    this.Demo.World.AddBody(sBody);
            //    sBody.Restitution = 1.0f;
            //    sBody.Friction = 0.0f;
            //}

            //for (int i = 0; i < 3; i++)
            //{
            //    RigidBody sBody = new RigidBody(new SphereShape(0.5f));
            //    sBody.Position = new JVector(0, 0.5f, 10 + i);
            //    this.Demo.World.AddBody(sBody);
            //    sBody.LinearVelocity = JVector.Forward * 3;
            //    sBody.Restitution = 1.0f;
            //    sBody.Friction = 0.0f;
            //}

      

            //this.Demo.World.SetDampingFactors(1, 1);


        }

        public override void Destroy()
        {
            this.Demo.World.Solver = Jitter.World.SolverType.Simultaneous;

            RemoveGround();
            this.Demo.World.Clear();
        }

    }
}
