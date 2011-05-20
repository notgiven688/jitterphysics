/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;

using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using Jitter.Collision;
using Jitter.Dynamics.Constraints;
#endregion

namespace Jitter
{

    /// <summary>
    /// </summary>
    /// <param name="timeStep">The timestep of the step.</param>
    public delegate void WorldStep(float timeStep);

    /// <summary>
    /// This class brings 'dynamics' and 'collisions' together. It handles
    /// all bodies and constraints.
    /// </summary>
    public class World
    {

        /// <summary>
        /// If a contact exceeds this breakThreshold it
        /// gets removed from the <see cref="ContactList"/> of an <see cref="Arbiter"/>.
        /// </summary>
        private ContactSettings contactSettings = new ContactSettings();

        private float inactiveAngularThresholdSq = 0.1f;
        private float inactiveLinearThresholdSq = 0.1f;
        private float deactivationTime = 2f;

        private float angularDamping = 0.85f;
        private float linearDamping = 0.85f;

        private int contactIterations = 10;
        private int smallIterations = 4;
        private float timestep = 0.0f;

        private List<CollisionIsland> islands = new List<CollisionIsland>();

        internal List<RigidBody> rigidBodies = new List<RigidBody>();
        internal List<Constraint> constraints = new List<Constraint>();

        /// <summary>
        /// Gets the <see cref="CollisionSystem"/> used
        /// to detect collisions.
        /// </summary>
        public CollisionSystem CollisionSystem {  set; get; }

        /// <summary>
        /// It's a good idea to add force and torque here.
        /// </summary>
        public event WorldStep PreStep;

        /// <summary>
        /// Called at the end of every step.
        /// </summary>
        public event WorldStep PostStep;

        /// <summary>
        /// Holds a list of <see cref="Arbiter"/>. All currently
        /// active arbiter in the <see cref="World"/> are stored in this map.
        /// </summary>
        public ArbiterMap ArbiterMap { get { return arbiterMap; } }
        private ArbiterMap arbiterMap;

        private Stack<Arbiter> garbageArbiterStack = new Stack<Arbiter>();

        private JVector gravity = new JVector(0, -9.81f, 0);

        public ContactSettings ContactSettings { get { return contactSettings; } }

        /// <summary>
        /// Gets a read only collection of the <see cref="Jitter.Dynamics.RigidBody"/> objects managed by
        /// this class.
        /// </summary>
        public ReadOnlyCollection<RigidBody> RigidBodies { get { return readOnlyRigidBodies; } }
        private ReadOnlyCollection<RigidBody> readOnlyRigidBodies;

        /// <summary>
        /// Gets a read only collection of the <see cref="Jitter.Dynamics.Constraints.Constraint"/> objects managed by
        /// this class.
        /// </summary>
        public ReadOnlyCollection<Constraint> Constraints { get { return readOnlyConstraints; } }
        private ReadOnlyCollection<Constraint> readOnlyConstraints;

        /// <summary>
        /// Gets a read only collection of the <see cref="Jitter.Collision.CollisionIsland"/> objects managed by
        /// this class.
        /// </summary>
        public ReadOnlyCollection<CollisionIsland> Islands { get { return readOnlyIslands; } }
        private ReadOnlyCollection<CollisionIsland> readOnlyIslands;

        private Action<object> arbiterCallback;
        private Action<object> integrateCallback;

        /// <summary>
        /// Create a new instance of the <see cref="World"/> class.
        /// </summary>
        /// <param name="collision">The collisionSystem which is used to detect
        /// collisions. See for example: <see cref="CollisionSystemSAP"/>
        /// or <see cref="CollisionSystemBrute"/>.
        /// </param>
        public World(CollisionSystem collision)
        {
            if (collision == null)
                throw new ArgumentNullException("The CollisionSystem can't be null.", "collision");

            ThreadManager.InitializeInstance();

            arbiterCallback = new Action<object>(ArbiterCallback);
            integrateCallback = new Action<object>(IntegrateCallback);

            this.constraints = new List<Constraint>();

            // Create the readonly wrappers
            this.readOnlyRigidBodies = rigidBodies.AsReadOnly();
            this.readOnlyConstraints = constraints.AsReadOnly();
            this.readOnlyIslands = islands.AsReadOnly();

            this.CollisionSystem = collision;
            this.CollisionSystem.CollisionDetected += new CollisionDetectedHandler(CollisionDetected);

            this.arbiterMap = new ArbiterMap();

            AllowDeactivation = true;
        }

        /// <summary>
        /// In Jitter many objects get added to stacks after they were used.
        /// If a new object is needed the old object gets removed from the stack
        /// and is reused. This saves some time and also garbage collections.
        /// Calling this method removes all cached objects from all
        /// stacks.
        /// </summary>
        public void ResetResourcePools()
        {
            CollisionIsland.Pool.ResetResourcePool();
            Arbiter.Pool.ResetResourcePool();
            Contact.Pool.ResetResourcePool();
            JBBox.CornersPool.ResetResourcePool();
        }

        /// <summary>
        /// Removes all objects from the world and removes all memory cached objects.
        /// </summary>
        public void Clear()
        {
            // remove bodies from collision system
            foreach (RigidBody body in rigidBodies)
                CollisionSystem.RemoveBody(body);

            // remove bodies from the world
            rigidBodies.Clear();

            // remove constraints
            constraints.Clear();

            // remove all islands
            islands.Clear();

            // delete the arbiters
            arbiterMap.Clear();

            ResetResourcePools();
        }

        /// <summary>
        /// Gets or sets the gravity in this <see cref="World"/>. The default gravity
        /// is (0,-9.81,0)
        /// </summary>
        public JVector Gravity { get { return gravity; } set { gravity = value; } }

        /// <summary>
        /// Global sets or gets if a body is able to be temporarily deactivated by the engine to
        /// safe computation time. Use <see cref="SetInactivityThreshold"/> to set parameters
        /// of the deactivation process.
        /// </summary>
        public bool AllowDeactivation { get; set; }

        /// <summary>
        /// Every computation <see cref="Step"/> the angular and linear velocity 
        /// of a <see cref="RigidBody"/> gets multiplied by this value.
        /// </summary>
        /// <param name="angularDamping">The factor multiplied with the angular velocity.
        /// The default value is 0.85.</param>
        /// <param name="linearDamping">The factor multiplied with the linear velocity.
        /// The default value is 0.85</param>
        public void SetDampingFactors(float angularDamping, float linearDamping)
        {
            if (angularDamping < 0.0f || angularDamping > 1.0f)
                throw new ArgumentException("Angular damping factor has to be between 0.0 and 1.0", "angularDamping");

            if (linearDamping < 0.0f || linearDamping > 1.0f)
                throw new ArgumentException("Linear damping factor has to be between 0.0 and 1.0", "linearDamping");

            this.angularDamping = angularDamping;
            this.linearDamping = linearDamping;
        }

        /// <summary>
        /// Sets parameters for the <see cref="RigidBody"/> deactivation process.
        /// If the bodies angular velocity is less than the angular velocity threshold
        /// and its linear velocity is lower then the linear velocity threshold for a 
        /// specific time the body gets deactivated. A body can be reactivated by setting
        /// <see cref="RigidBody.IsActive"/> to true. A body gets also automatically
        /// reactivated if another moving object hits it or the <see cref="CollisionIsland"/>
        /// the object is in gets activated.
        /// </summary>
        /// <param name="angularVelocity">The threshold value for the angular velocity. The default value
        /// is 0.1.</param>
        /// <param name="linearVelocity">The threshold value for the linear velocity. The default value
        /// is 0.1</param>
        /// <param name="time">The threshold value for the time in seconds. The default value is 2.</param>
        public void SetInactivityThreshold(float angularVelocity, float linearVelocity, float time)
        {
            if (angularVelocity < 0.0f) throw new ArgumentException("Angular velocity threshold has to " +
                 "be larger than zero", "angularVelocity");

            if (linearVelocity < 0.0f) throw new ArgumentException("Linear velocity threshold has to " +
                "be larger than zero", "linearVelocity");

            if (time < 0.0f) throw new ArgumentException("Deactivation time threshold has to " +
                "be larger than zero", "time");

            this.inactiveAngularThresholdSq = angularVelocity * angularVelocity;
            this.inactiveLinearThresholdSq = linearVelocity * linearVelocity;
            this.deactivationTime = time;
        }

        /// <summary>
        /// Jitter uses an iterativ approach to solve collisions and contacts. You can set the number of
        /// iterations Jitter should do. In general the more iterations the more stable a simulation gets
        /// but also costs computation time.
        /// </summary>
        /// <param name="iterations">The number of contact iterations. Default value 10.</param>
        /// <param name="smallIterations">The number of contact iteration used for smaller (two and three constraint) systems. Default value 4.</param>
        /// <remarks>The number of iterations for collision and contact should be between 3 - 30.
        /// More iterations means more stability and also a longer calculation time.</remarks>
        public void SetIterations(int iterations, int smallIterations)
        {
            if (iterations < 1) throw new ArgumentException("The number of collision " +
                 "iterations has to be larger than zero", "iterations");

            if (smallIterations < 1) throw new ArgumentException("The number of collision " +
                "iterations has to be larger than zero", "smallIterations");

            this.contactIterations = iterations;
            this.smallIterations = smallIterations;
        }

        /// <summary>
        /// Removes a <see cref="RigidBody"/> from the world.
        /// </summary>
        /// <param name="body">The body which should be removed.</param>
        /// <returns>Returns false if the body could not be removed from the world.</returns>
        public bool RemoveBody(RigidBody body)
        {
            // Its very important to clean up, after removing a body

            // remove the body from the world list
            int index = rigidBodies.BinarySearch(body);
            if (index < 0) return false;
            else { rigidBodies.RemoveAt(index); }

            // remove the body from the collision system
            CollisionSystem.RemoveBody(body);

            // remove the island
            if (body.island != null && body.island.bodies.Count == 1)
            {
                islands.Remove(body.island);
                CollisionIsland.Pool.GiveBack(body.island);
            }

            body.island = null;

            // remove all arbiters and contacts connected with this body
            List<Arbiter> orphanArbiters = new List<Arbiter>();
            foreach (Arbiter arbiter in arbiterMap.Values)
            {
                if (arbiter.body1 == body || arbiter.body2 == body)
                    orphanArbiters.Add(arbiter);
            }

            foreach (Arbiter arbiter in orphanArbiters)
            {
                // Give the contacts of the arbiter back
                int contactCount = arbiter.contactList.Count;
                for (int e = 0; e < contactCount; e++)
                {
                    Contact.Pool.GiveBack(arbiter.contactList[e]);
                }

                arbiter.contactList.Clear();

                // Give the arbiter back
                Arbiter.Pool.GiveBack(arbiter);
                arbiterMap.Remove(arbiter);
            }

            // remove all constraints connected with this body
            List<Constraint> orphanConstraints = new List<Constraint>();

            foreach (Constraint constraint in constraints)
            {
                if (constraint.body1 == body || constraint.body2 == body)
                    orphanConstraints.Add(constraint);
            }

            foreach (Constraint constraint in orphanConstraints)
            {
                constraints.Remove(constraint);
            }

            return true;
        }

        /// <summary>
        /// Adds a <see cref="RigidBody"/> to the world.
        /// </summary>
        /// <param name="body">The body which should be added.</param>
        public void AddBody(RigidBody body)
        {
            if (body == null) throw new ArgumentNullException("body", "body can't be null.");

            int index = rigidBodies.BinarySearch(body);
            if (index < 0)
            {
                body.Update();
                body.island = null;
                rigidBodies.Insert(~index, body);
                this.CollisionSystem.AddBody(body);
            }
            else throw new ArgumentException("The body was already added to the world.", "body");
        }

        /// <summary>
        /// Add a <see cref="Constraint"/> to the world. Fast, O(log(N)).
        /// </summary>
        /// <param name="constraint">The constraint which should be added.</param>
        /// <returns>True if the constraint was successfully removed.</returns>
        public bool RemoveConstraint(Constraint constraint)
        {
            int index = constraints.BinarySearch(constraint);
            if (index < 0) return false;
            else { constraints.RemoveAt(index); return true; }
        }

        /// <summary>
        /// Add a <see cref="Constraint"/> to the world.  Fast, O(log(N)).
        /// </summary>
        /// <param name="constraint">The constraint which should be removed.</param>
        public void AddConstraint(Constraint constraint)
        {
            int index = constraints.BinarySearch(constraint);
            if (index < 0) constraints.Insert(~index, constraint);
            else throw new ArgumentException("The constraint was already added to the world.", "constraint");
        }

        private float currentLinearDampFactor = 1.0f;
        private float currentAngularDampFactor = 1.0f;

#if(!WINDOWS_PHONE)
        Stopwatch sw = new Stopwatch();

        public enum DebugType { CollisionDetect, BuildIslands, HandleArbiter, UpdateContacts,
            PreStep, DeactivateBodies, IntegrateForces, Integrate, PostStep, Num }
        
        /// <summary>
        /// Time in ms for every part of the <see cref="Step"/> method.
        /// </summary>
        /// <example>int time = DebugTimes[(int)DebugType.CollisionDetect] gives
        /// the amount of time spent on collision detection during the last <see cref="Step"/>.
        /// </example>
        private double[] debugTimes = new double[(int)DebugType.Num];
        public double[] DebugTimes { get { return debugTimes; } }
#endif

        /// <summary>
        /// Integrates the whole world a timestep further in time.
        /// </summary>
        /// <param name="timestep">The timestep in seconds. 
        /// It should be small as possible to keep the simulation stable.
        /// The physics simulation shouldn't run slower than 60fps.
        /// (timestep=1/60).</param>
        /// <param name="multithread">If true the engine uses several threads to
        /// integrate the world. This is faster on multicore CPUs.</param>
        public void Step(float timestep, bool multithread)
        {
            this.timestep = timestep;

            // yeah! nothing to do!
            if (timestep == 0.0f) return;

            // throw exception if the timestep is smaller zero.
            if (timestep < 0.0f) throw new ArgumentException("The timestep can't be negative.", "timestep");

            // Calculate this
            currentAngularDampFactor = (float)Math.Pow(angularDamping, timestep);
            currentLinearDampFactor = (float)Math.Pow(linearDamping, timestep);

#if(WINDOWS_PHONE)
            if (this.PreStep != null) PreStep(timestep);
            for (int i = 0; i < rigidBodies.Count; i++) rigidBodies[i].PreStep();

            UpdateContacts();
            CollisionSystem.Detect(multithread);
            BuildIslands();
            CheckDeactivation();
            IntegrateForces();
            HandleArbiter(contactIterations, multithread);
            Integrate(multithread);

            for (int i = 0; i < rigidBodies.Count; i++) rigidBodies[i].PostStep();
            if (this.PostStep != null) PostStep(timestep);
#else
            sw.Reset(); sw.Start();
            if (this.PreStep != null) PreStep(timestep);
            for (int i = 0; i < rigidBodies.Count; i++) rigidBodies[i].PreStep();
            sw.Stop(); debugTimes[(int)DebugType.PreStep] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            UpdateContacts();
            sw.Stop(); debugTimes[(int)DebugType.UpdateContacts] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            CollisionSystem.Detect(multithread);
            sw.Stop(); debugTimes[(int)DebugType.CollisionDetect] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            BuildIslands();
            sw.Stop(); debugTimes[(int)DebugType.BuildIslands] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            CheckDeactivation();
            sw.Stop(); debugTimes[(int)DebugType.DeactivateBodies] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            IntegrateForces();
            sw.Stop(); debugTimes[(int)DebugType.IntegrateForces] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            HandleArbiter(contactIterations, multithread);
            sw.Stop(); debugTimes[(int)DebugType.HandleArbiter] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            Integrate(multithread);
            sw.Stop(); debugTimes[(int)DebugType.Integrate] = sw.Elapsed.TotalMilliseconds;

            sw.Reset(); sw.Start();
            for (int i = 0; i < rigidBodies.Count; i++) rigidBodies[i].PostStep();
            if (this.PostStep != null) PostStep(timestep);
            sw.Stop(); debugTimes[(int)DebugType.PostStep] = sw.Elapsed.TotalMilliseconds;
#endif
        }

        private float accumulatedTime = 0.0f;

        /// <summary>
        /// Integrates the whole world several fixed timestep further in time.
        /// </summary>
        /// <param name="totalTime">The time to integrate.</param>
        /// <param name="timestep">The timestep in seconds. 
        /// It should be small as possible to keep the simulation stable.
        /// The physics simulation shouldn't run slower than 60fps.
        /// (timestep=1/60).</param>
        /// <param name="multithread">If true the engine uses several threads to
        /// integrate the world. This is faster on multicore CPUs.</param>
        /// <param name="maxSteps">The maximum number of substeps. After that Jitter gives up
        /// to keep up with the given totalTime.</param>
        public void Step(float totalTime, bool multithread, float timestep, int maxSteps)
        {
            int counter = 0;
            accumulatedTime += totalTime;

            while (accumulatedTime > timestep)
            {
                Step(timestep, multithread);

                accumulatedTime -= timestep;
                counter++;

                if (counter > maxSteps)
                {
                    // okay, okay... we can't keep up
                    accumulatedTime = 0.0f;
                    break;
                }
            }

        }

        private void UpdateArbiterContacts(Arbiter arbiter)
        {
            if (arbiter.contactList.Count == 0)
            {
                lock (garbageArbiterStack) { garbageArbiterStack.Push(arbiter); }
                return;
            }

            for (int i = arbiter.contactList.Count - 1; i >= 0; i--)
            {
                Contact c = arbiter.contactList[i];
                c.UpdatePosition();

                if (c.penetration < -contactSettings.breakThreshold)
                {
                    Contact.Pool.GiveBack(c);
                    arbiter.contactList.RemoveAt(i);
                    continue;
                }
                else
                {
                    JVector diff; JVector.Subtract(ref c.p1, ref c.p2, out diff);
                    float distance = JVector.Dot(ref diff, ref c.normal);

                    diff = diff - distance * c.normal;
                    distance = diff.LengthSquared();

                    //HACK!! * 100
                    if (distance > contactSettings.breakThreshold * contactSettings.breakThreshold * 100.0f)
                    {
                        Contact.Pool.GiveBack(c);
                        arbiter.contactList.RemoveAt(i);
                        continue;
                    }
                }

            }
        }

        private void UpdateContacts()
        {
            foreach (Arbiter arbiter in arbiterMap.Values)
            {
                UpdateArbiterContacts(arbiter);
            }

            while (garbageArbiterStack.Count > 0)
            {
                Arbiter arbiter = garbageArbiterStack.Pop();
                Arbiter.Pool.GiveBack(arbiter);
                arbiterMap.Remove(arbiter);
            }

        }

        #region private void ArbiterCallback(object obj)
        private void ArbiterCallback(object obj)
        {
            CollisionIsland island = obj as CollisionIsland;

            int thisIterations;
            if (island.bodies.Count + island.constraints.Count > 3) thisIterations = contactIterations;
            else thisIterations = smallIterations;

            for (int i = -1; i < thisIterations; i++)
            {
                // Contact and Collision
                foreach (Arbiter arbiter in island.arbiter)
                {
                    int contactCount = arbiter.contactList.Count;
                    for (int e = 0; e < contactCount; e++)
                    {
                        if (i == -1) arbiter.contactList[e].PrepareForIteration(timestep);
                        else arbiter.contactList[e].Iterate();
                    }
                }

                // Constraints
                foreach (Constraint c in island.constraints)
                {
                    if (c.body1 != null && !c.body1.IsActive && c.body2 != null && !c.body2.IsActive)
                        continue;

                    if (i == -1) c.PrepareForIteration(timestep);
                    else c.Iterate();
                }

            }
        }
        #endregion

        private void HandleArbiter(int iterations, bool multiThreaded)
        {
            if (multiThreaded)
            {
                for (int i = 0; i < islands.Count; i++)
                {
                    if(islands[i].IsActive()) ThreadManager.internalInstance.AddTask(arbiterCallback, islands[i]);
                }

                ThreadManager.internalInstance.Execute();
            }
            else
            {
                for (int i = 0; i < islands.Count; i++)
                {
                    if (islands[i].IsActive()) arbiterCallback(islands[i]);
                }

            }
        }

        private void IntegrateForces()
        {
            for (int i = 0; i < rigidBodies.Count; i++)
            {
                RigidBody body = rigidBodies[i];

                if (!body.isStatic && body.IsActive)
                {
                    JVector temp;
                    JVector.Multiply(ref body.force, body.inverseMass * timestep, out temp);
                    JVector.Add(ref temp, ref body.linearVelocity, out body.linearVelocity);

                    JVector.Multiply(ref body.torque, timestep, out temp);
                    JVector.Transform(ref temp, ref body.invInertiaWorld, out temp);
                    JVector.Add(ref temp, ref body.angularVelocity, out body.angularVelocity);

                    if (body.affectedByGravity)
                    {
                        JVector.Multiply(ref gravity, timestep, out temp);
                        JVector.Add(ref body.linearVelocity, ref temp, out body.linearVelocity);
                    }
                }

                body.force.MakeZero();
                body.torque.MakeZero();

            }
        }

        #region private void IntegrateCallback(object obj)
        private void IntegrateCallback(object obj)
        {
            RigidBody body = obj as RigidBody;

            JVector temp;
            JVector.Multiply(ref body.linearVelocity, timestep, out temp);
            JVector.Add(ref temp, ref body.position, out body.position);

            //exponential map
            JVector axis;
            float angle = body.angularVelocity.Length();

            if (angle < 0.001f)
            {
                // use Taylor's expansions of sync function
                axis = body.angularVelocity * (0.5f * timestep - (timestep * timestep * timestep) * (0.020833333333f) * angle * angle);
                JVector.Multiply(ref body.angularVelocity, (0.5f * timestep - (timestep * timestep * timestep) * (0.020833333333f) * angle * angle), out axis);
            }
            else
            {
                // sync(fAngle) = sin(c*fAngle)/t
                JVector.Multiply(ref body.angularVelocity, ((float)Math.Sin(0.5f * angle * timestep) / angle), out axis);
            }

            JQuaternion dorn = new JQuaternion(axis.X, axis.Y, axis.Z, (float)Math.Cos(angle * timestep * 0.5f));
            JQuaternion ornA; JQuaternion.CreateFromMatrix(ref body.orientation, out ornA);

            JQuaternion.Multiply(ref dorn, ref ornA, out dorn);

            dorn.Normalize(); JMatrix.CreateFromQuaternion(ref dorn, out body.orientation);

            if ((body.Damping & RigidBody.DampingType.Linear) != 0)
                JVector.Multiply(ref body.linearVelocity, currentLinearDampFactor, out body.linearVelocity);

            if ((body.Damping & RigidBody.DampingType.Angular) != 0)
                JVector.Multiply(ref body.angularVelocity, currentAngularDampFactor, out body.angularVelocity);

            body.Update();
        }
        #endregion

        private void Integrate(bool multithread)
        {
            if (multithread)
            {
                for (int i = 0; i < rigidBodies.Count; i++)
                {
                    if (rigidBodies[i].isStatic || !rigidBodies[i].IsActive) continue;
                    ThreadManager.internalInstance.AddTask(integrateCallback, rigidBodies[i]);
                }

                ThreadManager.internalInstance.Execute();
            }
            else
            {
                for (int i = 0; i < rigidBodies.Count; i++)
                {
                    if (rigidBodies[i].isStatic || !rigidBodies[i].IsActive) continue;
                    integrateCallback(rigidBodies[i]);
                }
            }
        }

        private void CollisionDetected(RigidBody body1, RigidBody body2, JVector point1, JVector point2, JVector normal, float penetration)
        {
            Arbiter arbiter;

            lock (arbiterMap)
            {
                if (!arbiterMap.LookUpArbiter(body1, body2, out arbiter))
                {
                    arbiter = Arbiter.Pool.GetNew();
                    arbiter.body1 = body1; arbiter.body2 = body2;
                    arbiterMap.Add(new ArbiterKey(body1, body2), arbiter);
                }
            }

            if (arbiter.body1 == body1)
            {
                JVector.Negate(ref normal, out normal);
                arbiter.AddContact(point1, point2, normal, penetration, contactSettings);
            }
            else if(arbiter.body1 == body2)
            {
                arbiter.AddContact(point2, point1, normal, penetration, contactSettings);
            }

        }

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
                    islands.Remove(giveBackIsland);

                    int count = giveBackIsland.bodies.Count;
                    for (int i = 0; i < count; i++)
                    {
                        giveBackIsland.bodies[i].island = largeIslandOwner.island;
                        largeIslandOwner.island.bodies.Add(giveBackIsland.bodies[i]);
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

                islands.Add(island);
            }

        }

        private void BuildIslands()
        {
            for (int i = 0; i < islands.Count; i++)
            {
                if (islands[i].bodies.Count == 1 && !islands[i].bodies[0].isStatic)
                {
                    // keep islands which contain only one body.
                    islands[i].constraints.Clear();
                    islands[i].arbiter.Clear();
                }
                else if (!islands[i].IsActive())
                {
                    // keep islands which are inactive
                    islands[i].constraints.Clear();
                    islands[i].arbiter.Clear();
                }
                else
                {
                    // remove all others
                    foreach (RigidBody body in islands[i].bodies) body.island = null;

                    islands[i].constraints.Clear();
                    islands[i].arbiter.Clear();
                    islands[i].bodies.Clear();

                    CollisionIsland.Pool.GiveBack(islands[i]);

                    islands.RemoveAt(i); i--;
                }
            }

            // Connect islands by arbiters
            foreach (Arbiter arbiter in arbiterMap.Values)
            {
                RigidBody body0 = arbiter.body1;
                RigidBody body1 = arbiter.body2;

                if (body0.isStatic || body1.isStatic) continue;

                MergeIslands(body0, body1);
            }

            // Connect islands by constraints
            int count = constraints.Count;
            for (int i = 0; i < count; i++)
            {
                Constraint constraint = constraints[i];

                RigidBody body0 = constraint.body1;
                RigidBody body1 = constraint.body2;

                if (body0 == null || body1 == null) continue;

                if (body0.isStatic && body1.isStatic) continue;

                MergeIslands(body0, body1);
            }

            // every single body gets an island
            count = rigidBodies.Count;
            for (int i = 0; i < count; i++)
            {
                RigidBody body = rigidBodies[i];

                if (body.isStatic) continue;

                if (body.island == null)
                {
                    body.island = CollisionIsland.Pool.GetNew();
                    body.island.bodies.Add(body);

                    islands.Add(body.island);
                }
            }


            #region Add Constraints to Collision Islands
            count = constraints.Count;
            for (int i = 0; i < count; i++)
            {
                Constraint constraint = constraints[i];

                if (constraint.body1 != null && constraint.body1.CollisionIsland != null)
                    constraint.body1.island.constraints.Add(constraint);
            }
            #endregion

            #region Add Arbiters to Collision Islands
            foreach (Arbiter arbiter in arbiterMap.Values)
            {
                // that must be the same islands OR one or both are null
                CollisionIsland island1 = arbiter.body1.island;
                CollisionIsland island2 = arbiter.body2.island;

                if (island1 != null)
                {
                    island1.arbiter.Add(arbiter);
                }
                else if (island2 != null)
                {
                    island2.arbiter.Add(arbiter);
                }
            }
            #endregion

        }

        private void CheckDeactivation()
        {
            // A body deactivation DOESN'T kill the contacts - they are stored in
            // the arbitermap within the arbiters. So, waking up ist STABLE - old
            // contacts are reused. Also the collisionislands build every frame (based 
            // on the contacts) keep the same.

            foreach (CollisionIsland island in islands)
            {
                bool deactivateIsland = true;

                // global allowdeactivation
                if (!this.AllowDeactivation) deactivateIsland = false;
                else
                {
                    foreach (RigidBody body in island.bodies)
                    {
                        // body allowdeactivation
                        if (body.AllowDeactivation && (body.angularVelocity.LengthSquared() < inactiveAngularThresholdSq &&
                        (body.linearVelocity.LengthSquared() < inactiveLinearThresholdSq)))
                        {
                            body.inactiveTime += timestep;
                            if (body.inactiveTime < deactivationTime)
                                deactivateIsland = false;
                        }
                        else
                        {
                            body.inactiveTime = 0.0f;
                            deactivateIsland = false;
                        }
                    }
                }

                foreach (RigidBody body in island.bodies)
                {
                    body.IsActive = !deactivateIsland;
                }
            }
        }

    }
}
