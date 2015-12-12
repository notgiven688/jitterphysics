using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace Jitter2D.Dynamics.Constraints
{
    public interface IConstraint
    {
        void PrepareForIteration(float timestep);
        void Iterate();

        RigidBody Body1 { get; }

        /// <summary>
        /// Gets the second body. Can be null.
        /// </summary>
        RigidBody Body2 { get; }
    }

    /// <summary>
    /// A constraints forces a body to behave in a specific way.
    /// </summary>
    public abstract class Constraint : IConstraint, IDebugDrawable, IComparable<Constraint>
    {
        internal RigidBody body1;
        internal RigidBody body2;

        /// <summary>
        /// Gets the first body. Can be null.
        /// </summary>
        public RigidBody Body1 { get { return body1; } }

        /// <summary>
        /// Gets the second body. Can be null.
        /// </summary>
        public RigidBody Body2 { get { return body2; } }

        private static int instanceCount = 0;
        private int instance;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="body1">The first body which should get constrained. Can be null.</param>
        /// <param name="body2">The second body which should get constrained. Can be null.</param>
        public Constraint(RigidBody body1, RigidBody body2)
        {
            this.body1 = body1;
            this.body2 = body2;

            instance = Interlocked.Increment(ref instanceCount);

            // calling body.update does not hurt
            // if the user set orientations all
            // inverse orientations etc. get also
            // recalculated.
            if (body1 != null) body1.Update();
            if (body2 != null) body2.Update();
        }

        /// <summary>
        /// Called once before iteration starts.
        /// </summary>
        /// <param name="timestep">The simulation timestep</param>
        public abstract void PrepareForIteration(float timestep);

        /// <summary>
        /// Iteratively solve this constraint.
        /// </summary>
        public abstract void Iterate();


        public int CompareTo(Constraint other)
        {
            if (other.instance < this.instance) return -1;
            else if (other.instance > this.instance) return 1;
            else return 0;
        }

        public virtual void DebugDraw(IDebugDrawer drawer)
        {
            //throw new NotImplementedException();
        }
    }
}
