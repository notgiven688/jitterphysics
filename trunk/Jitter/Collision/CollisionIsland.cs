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
using System.Threading;

using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using Jitter.Dynamics.Constraints;
using System.Collections.ObjectModel;
#endregion

namespace Jitter.Collision
{
    /// <summary>
    /// Holds a list of bodies which are in contact with each other.
    /// </summary>
    public class CollisionIsland
    {
        internal List<RigidBody> bodies = new List<RigidBody>();
        internal List<Arbiter> arbiter = new List<Arbiter>();
        internal List<Constraint> constraints = new List<Constraint>();

        /// <summary>
        /// Gets a read only list of <see cref="RigidBody"/> which are in contact with each other.
        /// </summary>
        public ReadOnlyCollection<RigidBody> Bodies { get { return readOnlyBodies; } }
        private ReadOnlyCollection<RigidBody> readOnlyBodies;

        /// <summary>
        /// Gets a read only list of <see cref="Arbiter"/> which are involved in this island.
        /// </summary>
        public ReadOnlyCollection<Arbiter> Arbiter { get { return readOnlyArbiter; } }
        private ReadOnlyCollection<Arbiter> readOnlyArbiter;

        /// <summary>
        /// Gets a read only list of <see cref="Constraint"/> which are involved in this island.
        /// </summary>
        public ReadOnlyCollection<Constraint> Constraints { get { return readOnlyConstraints; } }
        private ReadOnlyCollection<Constraint> readOnlyConstraints;

        /// <summary>
        /// A CollisionIsland pool.
        /// </summary>
        public static ResourcePool<CollisionIsland> Pool = new ResourcePool<CollisionIsland>();

        private static int instanceCount = 0;
        private int instance;


        /// <summary>
        /// Constructor of CollisionIsland class.
        /// </summary>
        public CollisionIsland()
        {
            // Prepare readonly wrappers.
            readOnlyConstraints = constraints.AsReadOnly();
            readOnlyBodies = bodies.AsReadOnly();
            readOnlyArbiter = arbiter.AsReadOnly();

            instance = Interlocked.Increment(ref instanceCount);
        }

        /// <summary>
        /// Whether the island is active or not.
        /// </summary>
        /// <returns>Returns true if the island is active, otherwise false.</returns>
        /// <seealso cref="RigidBody.IsActive"/>
        public bool IsActive()
        {
            return bodies[0].IsActive;
        }

        /// <summary>
        /// Sets the status of every body in this island to active or inactive.
        /// </summary>
        /// <param name="active">If true the island gets activated, if false it
        /// gets deactivated. </param>
        /// <seealso cref="RigidBody.IsActive"/>
        public void SetStatus(bool active)
        {
            foreach (RigidBody body in bodies)
            {
                body.IsActive = active;
                if (active && !body.IsActive) body.inactiveTime = 0.0f;
            }

        }

        internal void ClearLists()
        {
            arbiter.Clear();
            bodies.Clear();
            constraints.Clear();
        }

    }
}
