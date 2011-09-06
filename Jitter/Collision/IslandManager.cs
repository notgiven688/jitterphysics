using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Dynamics;
using Jitter.Dynamics.Constraints;

namespace Jitter.Collision
{
    public class IslandManager : List<CollisionIsland>
    {

        // static bodies dont get anything
        // what to remove when body gets static?

        public void ConstraintCreated(Constraint constraint)
        {
            RigidBody body1 = constraint.body1;
            RigidBody body2 = constraint.body2;

            // constraints - body2 can be null
            if ((body2 == null || body2.isStatic) && !body1.isStatic)
            {
                if (body1.island == null)
                {
                    body1.island = CollisionIsland.Pool.GetNew();
                    body1.island.bodies.Add(body1);
                    this.Add(body1.island);
                }

                body1.island.constraints.Add(constraint);
                body1.constraints.Add(constraint);
            }
            else if (body1.isStatic && !body2.isStatic)
            {
                if (body2.island == null)
                {
                    body2.island = CollisionIsland.Pool.GetNew();
                    body2.island.bodies.Add(body1);
                    this.Add(body2.island);
                }

                body2.island.constraints.Add(constraint);
                body2.constraints.Add(constraint);
            }
            else if (!body1.isStatic && !body2.isStatic)
            {
                // both are !static
                MergeIslands(body1, body2);

                body1.island.constraints.Add(constraint);

                body1.bodies.Add(body2);
                body1.constraints.Add(constraint);

                body2.bodies.Add(body1);
                body2.constraints.Add(constraint);
            }
    
        }

        public void ConstraintRemoved(Constraint constraint)
        {
            RigidBody body1 = constraint.body1;
            RigidBody body2 = constraint.body2;

            if (body2 != null)
            {
                body1.bodies.Remove(body2);
                body2.bodies.Remove(body1);
            }

            body1.constraints.Remove(constraint);
            if (body2 != null) body2.constraints.Remove(constraint);


            if ((body2 == null || body2.isStatic) && !body1.isStatic)
            {
                // only body2 is static
                body1.island.constraints.Remove(constraint);
            }
            else if (body1.isStatic && !body2.isStatic)
            {
                // only body1 is static
                body2.island.constraints.Remove(constraint);
            }
            else if (!body1.isStatic && !body2.isStatic)
            {
                // TODO: dont know in which island the arbiter is.. try to remove from both
                body1.island.constraints.Remove(constraint);

                // both are !static
                SplitIslands(body1, body2);
            }
        }

        public void ArbiterCreated(Arbiter arbiter)
        {
            RigidBody body1 = arbiter.body1;
            RigidBody body2 = arbiter.body2;

            // dont accept arbiter between static-static.
            if (body1.isStatic && body2.isStatic) return;

            if (body1.isStatic)
            {
                // only body1 is static
                if (body2.island == null)
                {
                    body2.island = CollisionIsland.Pool.GetNew();
                    body2.island.bodies.Add(body2);
                    this.Add(body2.island);
                }

                body2.island.arbiter.Add(arbiter);
                body2.arbiter.Add(arbiter);
            }
            else if (body2.isStatic)
            {
                // only body2 is static
                if (arbiter.body1.island == null)
                {
                    body1.island = CollisionIsland.Pool.GetNew();
                    body1.island.bodies.Add(body1);
                    this.Add(body1.island);
                }

                body1.island.arbiter.Add(arbiter);
                body1.arbiter.Add(arbiter);
            }
            else
            {
                // both are !static
                MergeIslands(body1, body2);

                body1.island.arbiter.Add(arbiter);

                body1.bodies.Add(body2);
                body1.arbiter.Add(arbiter);

                body2.bodies.Add(body1);
                body2.arbiter.Add(arbiter);
            }
        }

        public void ArbiterRemoved(Arbiter arbiter)
        {
            RigidBody body1 = arbiter.body1;
            RigidBody body2 = arbiter.body2;

            body1.bodies.Remove(body2);
            body2.bodies.Remove(body1);

            body1.arbiter.Remove(arbiter);
            body2.arbiter.Remove(arbiter);

            // dont accept arbiter between static-static.
            if (body1.isStatic && body2.isStatic) return;

            if (body1.isStatic)
            {
                // only body1 is static
                body2.island.arbiter.Remove(arbiter);
            }
            else if (body2.isStatic)
            {
                // only body2 is static
                body1.island.arbiter.Remove(arbiter);
            }
            else
            {
                // TODO: dont know in which island the arbiter is.. try to remove from both
                body1.island.arbiter.Remove(arbiter);

                // both are !static
                SplitIslands(body1, body2);
            }
        }


        private Queue<RigidBody> leftSearchQueue = new Queue<RigidBody>();
        private Queue<RigidBody> rightSearchQueue = new Queue<RigidBody>();

        private List<RigidBody> visitedBodiesLeft = new List<RigidBody>();
        private List<RigidBody> visitedBodiesRight = new List<RigidBody>();

        // Boths bodies must be !static - so they must have an island
        private void SplitIslands(RigidBody body0, RigidBody body1)
        {
            System.Diagnostics.Debug.Assert(body0.island == body1.island,
                "Two bodies connected by arbiter but not in the same island");

            leftSearchQueue.Enqueue(body0);
            rightSearchQueue.Enqueue(body1);

            visitedBodiesLeft.Add(body0);
            visitedBodiesRight.Add(body1);

            body0.marker = 1;
            body1.marker = 2;

            while (leftSearchQueue.Count > 0 && rightSearchQueue.Count > 0)
            {
                RigidBody currentNode = leftSearchQueue.Dequeue();
                if (!currentNode.isStatic)
                {
                    for (int i = 0; i < currentNode.bodies.Count; i++)
                    {
                        RigidBody connectedNode = currentNode.bodies[i];

                        if (connectedNode.marker == 0)
                        {
                            leftSearchQueue.Enqueue(connectedNode);
                            visitedBodiesLeft.Add(connectedNode);
                            currentNode.marker = 1;
                        }
                        else if (connectedNode.marker == 2)
                        {
                            leftSearchQueue.Clear();
                            rightSearchQueue.Clear();
                            goto ResetSearchStates;
                        }
                    }
                }

                currentNode = rightSearchQueue.Dequeue();
                if (!currentNode.isStatic)
                {

                    for (int i = 0; i < currentNode.bodies.Count; i++)
                    {
                        RigidBody connectedNode = currentNode.bodies[i];

                        if (connectedNode.marker == 0)
                        {
                            rightSearchQueue.Enqueue(connectedNode);
                            visitedBodiesRight.Add(connectedNode);
                            currentNode.marker = 2;
                        }
                        else if (connectedNode.marker == 1)
                        {
                            leftSearchQueue.Clear();
                            rightSearchQueue.Clear();
                            goto ResetSearchStates;
                        }
                    }
                }
            }

            CollisionIsland island = CollisionIsland.Pool.GetNew();
            this.Add(island);

            if (leftSearchQueue.Count == 0)
            {
                for (int i = 0; i < visitedBodiesLeft.Count; i++)
                {
                    RigidBody body = visitedBodiesLeft[i];

                    body1.island.bodies.Remove(body);
                    island.bodies.Add(body);

                    foreach (Arbiter a in body.arbiter)
                    { body1.island.arbiter.Remove(a); island.arbiter.Add(a); }
                    foreach (Constraint c in body.constraints)
                    { body1.island.constraints.Remove(c); island.constraints.Add(c); }

                    body.island = island;
                }

                rightSearchQueue.Clear();
            }
            else if (rightSearchQueue.Count == 0)
            {
                for (int i = 0; i < visitedBodiesRight.Count; i++)
                {
                    RigidBody body = visitedBodiesRight[i];

                    body0.island.bodies.Remove(body);
                    island.bodies.Add(body);

                    foreach (Arbiter a in body.arbiter)
                    { body0.island.arbiter.Remove(a); island.arbiter.Add(a); }
                    foreach (Constraint c in body.constraints)
                    { body0.island.constraints.Remove(c); island.constraints.Add(c); }

                    body.island = island;
                }

                leftSearchQueue.Clear();
            }

        ResetSearchStates:

            for (int i = 0; i < visitedBodiesLeft.Count; i++)
            {
                visitedBodiesLeft[i].marker = 0;
            }

            for (int i = 0; i < visitedBodiesRight.Count; i++)
            {
                visitedBodiesRight[i].marker = 0;
            }

            visitedBodiesLeft.Clear();
            visitedBodiesRight.Clear();
        }

        // Boths bodies must be !static
        private void MergeIslands(RigidBody body0, RigidBody body1)
        {
            if (body0.island != body1.island)
            {
                // both bodies are in different islands
                // so we can merge them
                if (body0.island == null)
                {
                    // one island is null
                    body0.island = body1.island;
                    body0.island.bodies.Add(body0);
                }
                else if (body1.island == null)
                {
                    // one island is null
                    body1.island = body0.island;
                    body1.island.bodies.Add(body1);
                }
                else
                {
                    // both islands are different,
                    // merge smaller into larger

                    RigidBody smallIslandOwner, largeIslandOwner;

                    if (body0.island.bodies.Count > body1.island.bodies.Count)
                    {
                        smallIslandOwner = body1;
                        largeIslandOwner = body0;
                    }
                    else
                    {
                        smallIslandOwner = body0;
                        largeIslandOwner = body1;
                    }

                    CollisionIsland giveBackIsland = smallIslandOwner.island;

                    CollisionIsland.Pool.GiveBack(giveBackIsland);
                    this.Remove(giveBackIsland);

                    foreach (RigidBody b in giveBackIsland.bodies)
                    {
                        b.island = largeIslandOwner.island;
                        largeIslandOwner.island.bodies.Add(b);
                    }

                    foreach (Arbiter a in giveBackIsland.arbiter)
                    {
                        largeIslandOwner.island.arbiter.Add(a);
                    }

                    foreach (Constraint c in giveBackIsland.constraints)
                    {
                        largeIslandOwner.island.constraints.Add(c);
                    }

                    giveBackIsland.ClearLists();
                }

            }
            else if (body0.island == null)
            {
                // both are null
                CollisionIsland island = CollisionIsland.Pool.GetNew();
                body0.island = body1.island = island;

                body0.island.bodies.Add(body0);
                body0.island.bodies.Add(body1);

                this.Add(island);
            }

        }




    }
}
