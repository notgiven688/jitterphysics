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
    class Tower : Scene
    {

        public Tower(JitterDemo demo)
            : base(demo)
        {
        }

        public override void Build()
        {
            AddGround();

            World world = Demo.World;

            Matrix halfRotationStep = Matrix.CreateRotationY(MathHelper.Pi * 2.0f / 24.0f);
            Matrix fullRotationStep = halfRotationStep * halfRotationStep;
            Matrix orientation = Matrix.Identity;

            BoxShape shape = new BoxShape(2, 1, 1);

            //for (int i = 0; i < 15; i++)
            //{
            //    for (int e = 0; e < 15; e++)
            //    {
            //        for (int k = 0; k < 15; k++)
            //        {
            //            RigidBody b = new RigidBody(shape);
            //            Demo.World.AddBody(b);
            //            b.Position = new JVector(i, e, k) * 4.0f;
            //            b.AffectedByGravity = true;
            //        }
            //    }
            //}

            for (int e = 0; e < 40; e++)
            {
                orientation *= halfRotationStep;

                for (int i = 0; i < 12; i++)
                {
                    Vector3 position = Vector3.Transform(
                        new Vector3(0, 0.5f + e, 6.5f), orientation);

                    RigidBody body = new RigidBody(shape);
                    body.Orientation = Conversion.ToJitterMatrix(orientation);
                    body.Position = Conversion.ToJitterVector(position);

                    world.AddBody(body);

                    orientation *= fullRotationStep;
                }
            }

            //for (int e = 0; e < 40; e++)
            //{
            //    orientation *= halfRotationStep;

            //    for (int i = 0; i < 12; i++)
            //    {
            //        Vector3 position = Vector3.Transform(
            //            new Vector3(0, 0.5f + e, 6.5f), orientation);

            //        RigidBody body = new RigidBody(shape);
            //        body.Orientation = Conversion.ToJitterMatrix(orientation);
            //        body.Position = Conversion.ToJitterVector(position) + new JVector(20,0,0);

            //        world.AddBody(body);

            //        orientation *= fullRotationStep;
            //    }
            //}

            //for (int e = 0; e < 40; e++)
            //{
            //    orientation *= halfRotationStep;

            //    for (int i = 0; i < 12; i++)
            //    {
            //        Vector3 position = Vector3.Transform(
            //            new Vector3(0, 0.5f + e, 6.5f), orientation);

            //        RigidBody body = new RigidBody(shape);
            //        body.Orientation = Conversion.ToJitterMatrix(orientation);
            //        body.Position = Conversion.ToJitterVector(position) + new JVector(00, 0, -20); ;

            //        world.AddBody(body);

            //        orientation *= fullRotationStep;
            //    }
            //}

            //for (int e = 0; e < 40; e++)
            //{
            //    orientation *= halfRotationStep;

            //    for (int i = 0; i < 12; i++)
            //    {
            //        Vector3 position = Vector3.Transform(
            //            new Vector3(0, 0.5f + e, 6.5f), orientation);

            //        RigidBody body = new RigidBody(shape);
            //        body.Orientation = Conversion.ToJitterMatrix(orientation);
            //        body.Position = Conversion.ToJitterVector(position) + new JVector(20, 0, -20);

            //        world.AddBody(body);

            //        orientation *= fullRotationStep;
            //    }
            //}






            //for (int i = 0; i < 15; i++)
            //{
            //    for (int k = 0; k < 15; k++)
            //    {
            //        for (int l = 0; l < 15; l++)
            //        {
            //            RigidBody body = new RigidBody(new BoxShape(1, 1, 1));
            //            this.Demo.World.AddBody(body);
            //            body.AffectedByGravity = false;
            //            body.Position = new JVector(i, k, l) * 2.0f;
            //            body.Restitution = 1.0f;
            //            body.Damping = RigidBody.DampingType.None;
            //            body.IsActive = false;
            //        }
            //    }
            //}


            ground.Material.StaticFriction = 1.0f;
            ground.Material.KineticFriction = 1.0f;
        }


    }
}