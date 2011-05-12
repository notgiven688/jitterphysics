using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Dynamics;
using Jitter.Collision.Shapes;
using Jitter.LinearMath;
using Jitter.Dynamics.Constraints;

namespace Jitter.Forces
{
    public class PseudoCloth
    {


        public List<Constraint> constraints = new List<Constraint>();

        public class PseudoClothBody : RigidBody
        {
            public PseudoClothBody(float sphereRadius) : base(new SphereShape(sphereRadius)) { }
        }

        int sizeX, sizeY;
        float scale;

        World world;

        PseudoClothBody[] bodies;

        public PseudoCloth(World world, int sizeX, int sizeY, float scale)
        {
            bodies = new PseudoClothBody[sizeX * sizeY];

            for (int i = 0; i < sizeX; i++)
            {
                for (int e = 0; e < sizeX; e++)
                {
                    bodies[i + e * sizeY] = new PseudoClothBody(0.1f);
                    bodies[i + e * sizeY].Position = new JVector(i * scale, 0, e * scale) + JVector.Up * 10.0f;
                    bodies[i + e * sizeY].StaticFriction =0.5f;
                    bodies[i + e * sizeY].DynamicFriction = 0.5f;
                    bodies[i + e * sizeY].Mass = 0.1f;
                    world.AddBody(bodies[i + e * sizeY]);
                }
            }

            world.CollisionSystem.PassedBroadphase += new Collision.PassedBroadphaseHandler(CollisionSystem_PassedBroadphase);
            world.PostStep += new WorldStep(world_PostStep);

            this.world = world;

            for (int i = 0; i < sizeX; i++)
            {
                for (int e = 0; e < sizeY; e++)
                {
                    if (i + 1 < sizeX)
                    {
                        AddDistance(e * sizeY + i, (i + 1) + e * sizeY);
                        // (i,e) and (i+1,e)
                    }

                    if (e + 1 < sizeY)
                    {
                        AddDistance(e * sizeY + i, ((e + 1) * sizeY) + i);
                        // (e,i) and (e+1,i)

                    }

                    if( (i + 1 < sizeX) && (e + 1 < sizeY))
                    {
                        AddDistance(e * sizeY + i, ((e + 1) * sizeY) +( i+1));
                    }


                    if ((i > 0) && (e + 1 < sizeY))
                    {
                        AddDistance(e * sizeY + i, ((e + 1) * sizeY) + (i - 1));
                    }


                }
            }

            this.sizeX = sizeX;
            this.sizeY = sizeY;
            this.scale = scale;
            
        }

        void world_PostStep(float timeStep)
        {
            CheckConstraints();
        }

        public RigidBody GetCorner(int e,int i)
        {
            return bodies[e * sizeY + i];
        }

       

        private void AddDistance(int p1, int p2)
        {
            DistanceConstraint dc = new DistanceConstraint(bodies[p1], bodies[p2], bodies[p1].position, bodies[p2].position);
            dc.Softness = 2f;
            dc.BiasFactor = 0.1f;
            world.AddConstraint(dc);
            this.constraints.Add(dc);
        }

        public void CheckConstraints()
        {
            foreach (Constraint c in constraints)
            {
                if ((c as DistanceConstraint).AppliedImpulse.Length() > 1.8f)
                {
                    world.constraints.Remove(c);
                }
            }
        }






        private bool CollisionSystem_PassedBroadphase(RigidBody body1, RigidBody body2)
        {
            // prevent PseudoClothBody,PseudoClothBody collisions
            return !(body1 is PseudoClothBody && body2 is PseudoClothBody);
        }







    }
}
