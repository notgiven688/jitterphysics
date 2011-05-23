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
using Jitter.Collision;
#endregion

namespace Jitter.Dynamics
{

    public interface IBroadphaseEntity
    {
        JBBox BoundingBox { get; }
        int BroadphaseTag { get; set; }
        bool IsStaticOrInactive();
    }

    /// <summary>
    /// The RigidBody class.
    /// </summary>
    public class RigidBody : IBroadphaseEntity, IEquatable<RigidBody>, IComparable<RigidBody>
    {
        [Flags]
        public enum DampingType { None = 0x00, Angular = 0x01, Linear = 0x02 }

        internal JMatrix inertia;
        internal JMatrix invInertia;

        internal JMatrix invInertiaWorld;
        internal JMatrix orientation;
        internal JMatrix invOrientation;
        internal JVector position;
        internal JVector linearVelocity;
        internal JVector angularVelocity;

        internal Material material;

        internal JBBox boundingBox;

        internal float inactiveTime = 0.0f;

        internal bool isActive = true;
        internal bool isStatic = false;
        internal bool affectedByGravity = true;

        internal CollisionIsland island;
        internal float inverseMass;

        internal JVector force, torque;

        

        private int hashCode;

        internal int internalIndex = 0;

        private ShapeUpdatedHandler updatedHandler;

        internal bool multigrid = false;

        /// <summary>
        /// Calculates a hashcode for this RigidBody.
        /// The hashcode should be unique as possible
        /// for every body.
        /// </summary>
        /// <returns>The hashcode.</returns>
        public override int GetHashCode()
        {
            return hashCode;
        }

        /// <summary>
        /// If set to false the body will never be deactived by the
        /// world.
        /// </summary>
        public bool AllowDeactivation { get; set; }

        /// <summary>
        /// The axis aligned bounding box of the body.
        /// </summary>
        public JBBox BoundingBox { get { return boundingBox; } }


        public RigidBody(Shape shape)
            : this(shape, new Material())
        {
        }

        /// <summary>
        /// Initializes a new instance of the RigidBody class.
        /// </summary>
        /// <param name="shape">The shape of the body.</param>
        public RigidBody(Shape shape, Material material)
        {
            instance = Interlocked.Increment(ref instanceCount);
            hashCode = CalculateHash(instance);

            this.Shape = shape;
            orientation = JMatrix.Identity;

            updatedHandler = new ShapeUpdatedHandler(ShapeUpdated);

            this.Shape.ShapeUpdated += updatedHandler;

            SetMassProperties();

            this.material = material;

            AllowDeactivation = true;

            Update();
        }

        private static int instanceCount = 0;
        private int instance;

        private int CalculateHash(int a)
        {
            a = (a ^ 61) ^ (a >> 16);
            a = a + (a << 3);
            a = a ^ (a >> 4);
            a = a * 0x27d4eb2d;
            a = a ^ (a >> 15);
            return a;
        }

        /// <summary>
        /// Gets the current collision island the body is in.
        /// </summary>
        public CollisionIsland CollisionIsland { get { return this.island; } }

        /// <summary>
        /// If set to false the velocity is set to zero,
        /// the body gets immediately freezed.
        /// </summary>
        public bool IsActive
        {
            get 
            {
                return isActive;
            }
            set
            {
                if (!isActive && value)
                {
                    // if inactive and should be active
                    inactiveTime = 0.0f;
                }
                else if (isActive && !value)
                {
                    // if active and should be inactive
                    inactiveTime = float.PositiveInfinity;
                    this.angularVelocity.MakeZero();
                    this.linearVelocity.MakeZero();
                }

                isActive = value;
            }
        }

        /// <summary>
        /// Applies an impulse on the center of the body. Changing
        /// linear velocity.
        /// </summary>
        /// <param name="impulse">Impulse direction and magnitude.</param>
        public void ApplyImpulse(JVector impulse)
        {
            if (this.isStatic)
                throw new InvalidOperationException("Can't apply an impulse to a static body.");

            JVector temp;
            JVector.Multiply(ref impulse, inverseMass, out temp);
            JVector.Add(ref linearVelocity, ref temp, out linearVelocity);
        }

        /// <summary>
        /// Applies an impulse on the specific position. Changing linear
        /// and angular velocity.
        /// </summary>
        /// <param name="impulse">Impulse direction and magnitude.</param>
        /// <param name="relativePosition">The position where the impulse gets applied
        /// in Body coordinate frame.</param>
        public void ApplyImpulse(JVector impulse, JVector relativePosition)
        {
            if (this.isStatic)
                throw new InvalidOperationException("Can't apply an impulse to a static body.");

            JVector temp;
            JVector.Multiply(ref impulse, inverseMass, out temp);
            JVector.Add(ref linearVelocity, ref temp, out linearVelocity);

            JVector.Cross(ref relativePosition, ref impulse, out temp);
            JVector.Transform(ref temp, ref invInertiaWorld, out temp);
            JVector.Add(ref angularVelocity, ref temp, out angularVelocity);
        }

        /// <summary>
        /// Adds a force to the center of the body. The force gets applied
        /// the next time <see cref="World.Step"/> is called. The 'impact'
        /// of the force depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="force">The force to add next <see cref="World.Step"/>.</param>
        public void AddForce(JVector force)
        {
            JVector.Add(ref force, ref this.force, out this.force);
        }

        /// <summary>
        /// Adds a force to the center of the body. The force gets applied
        /// the next time <see cref="World.Step"/> is called. The 'impact'
        /// of the force depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="force">The force to add next <see cref="World.Step"/>.</param>
        /// <param name="pos">The position where the force is applied.</param>
        public void AddForce(JVector force, JVector pos)
        {
            JVector.Add(ref this.force, ref force, out this.force);
            JVector.Subtract(ref pos, ref this.position, out pos);
            JVector.Cross(ref pos, ref force, out pos);
            JVector.Add(ref pos, ref this.torque, out this.torque);
        }

        /// <summary>
        /// Adds torque to the body. The torque gets applied
        /// the next time <see cref="World.Step"/> is called. The 'impact'
        /// of the torque depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="torque">The torque to add next <see cref="World.Step"/>.</param>
        public void AddTorque(JVector torque)
        {
            JVector.Add(ref torque, ref this.torque, out this.torque);
        }

        private bool useShapeMassProperties = true;

        /// <summary>
        /// By calling this method the shape inertia and mass is used.
        /// </summary>
        public void SetMassProperties()
        {
            this.inertia = Shape.inertia;
            JMatrix.Inverse(ref inertia, out invInertia);
            this.inverseMass = 1.0f / Shape.mass;
            useShapeMassProperties = true;
        }

        /// <summary>
        /// The engine used the given values for inertia and mass and ignores
        /// the shape mass properties.
        /// </summary>
        /// <param name="inertia">The inertia/inverse inertia of the untransformed object.</param>
        /// <param name="mass">The mass/inverse mass of the object.</param>
        /// <param name="setAsInverseValues">Sets the InverseInertia and the InverseMass
        /// to this values.</param>
        public void SetMassProperties(JMatrix inertia, float mass, bool setAsInverseValues)
        {
            if (setAsInverseValues)
            {
                this.invInertia = inertia;
                JMatrix.Inverse(ref inertia, out this.inertia);
                this.inverseMass = mass;
            }
            else
            {
                this.inertia = inertia;
                JMatrix.Inverse(ref inertia, out this.invInertia);
                this.inverseMass = 1.0f / mass;
            }

            useShapeMassProperties = false;
            Update();
        }

        private void ShapeUpdated()
        {
            if (useShapeMassProperties) SetMassProperties();
            Update();
        }

        /// <summary>
        /// Allows to set a user defined value to the body.
        /// </summary>
        public object Tag { get; set; }

        /// <summary>
        /// The shape the body is using.
        /// </summary>
        public Shape Shape 
        {
            get { return shape; } 
            set 
            {
                // deregister update event
                if(shape != null) shape.ShapeUpdated -= updatedHandler;

                // register new event
                shape = value; 
                shape.ShapeUpdated += new ShapeUpdatedHandler(ShapeUpdated); 
            } 
        }

        private Shape shape;

        #region Properties

        private DampingType damping = DampingType.Angular | DampingType.Linear;

        public DampingType Damping { get { return damping; } set { damping = value; } }

        public Material Material { get { return material; } set { material = value; } }

        /// <summary>
        /// The inertia currently used for this body.
        /// </summary>
        public JMatrix Inertia { get { return inertia; } }

        /// <summary>
        /// The inverse inertia currently used for this body.
        /// </summary>
        public JMatrix InverseInertia { get { return invInertia; } }

        /// <summary>
        /// The velocity of the body.
        /// </summary>
        public JVector LinearVelocity
        {
            get { return linearVelocity; }
            set 
            { 
                if (this.isStatic) 
                    throw new InvalidOperationException("Can't set a velocity to a static body.");
                linearVelocity = value;
            }
        }

        // TODO: check here is static!
        /// <summary>
        /// The angular velocity of the body.
        /// </summary>
        public JVector AngularVelocity
        {
            get { return angularVelocity; }
            set
            {
                if (this.isStatic)
                    throw new InvalidOperationException("Can't set a velocity to a static body.");
                angularVelocity = value;
            }
        }

        /// <summary>
        /// The current position of the body.
        /// </summary>
        public JVector Position
        {
            get { return position; }
            set { position = value ; Update(); }
        }

        /// <summary>
        /// The current oriention of the body.
        /// </summary>
        public JMatrix Orientation
        {
            get { return orientation; }
            set { orientation = value; Update(); }
        }

        /// <summary>
        /// If set to true the body can't be moved.
        /// </summary>
        public bool IsStatic
        {
            get
            {
                return isStatic;
            }
            set
            {
                isStatic = value;
                if (isStatic)
                {
                    this.angularVelocity.MakeZero();
                    this.linearVelocity.MakeZero();
                }
            }
        }

        public bool AffectedByGravity { get { return affectedByGravity; } set { affectedByGravity = value; } }

        /// <summary>
        /// The inverse inertia tensor in world space.
        /// </summary>
        public JMatrix InverseInertiaWorld
        {
            get
            {
                return invInertiaWorld;
            }
        }

        /// <summary>
        /// Setting the mass automatically scales the inertia.
        /// To set the mass indepedently from the mass use SetMassProperties.
        /// </summary>
        public float Mass
        {
            get { return 1.0f / inverseMass; }
            set 
            {
                if (value <= 0.0f) throw new ArgumentException("Mass can't be less or equal zero.");

                // scale inertia
                JMatrix.Multiply(ref Shape.inertia, value / Shape.mass, out inertia);
                JMatrix.Inverse(ref inertia, out invInertia);

                inverseMass = 1.0f / value;
            }
        }

        #endregion

        /// <summary>
        /// Recalculates the axis aligned bounding box and the inertia
        /// values in world space.
        /// </summary>
        public virtual void Update()
        {
            // Given: Orientation, Inertia

            JMatrix.Transpose(ref orientation, out invOrientation);
            this.Shape.GetBoundingBox(ref orientation, out boundingBox);
            JVector.Add(ref boundingBox.Min, ref this.position, out boundingBox.Min);
            JVector.Add(ref boundingBox.Max, ref this.position, out boundingBox.Max);

            if (!isStatic)
            {
                JMatrix.Multiply(ref invOrientation, ref invInertia, out invInertiaWorld);
                JMatrix.Multiply(ref invInertiaWorld, ref orientation, out invInertiaWorld);
            }
        }

        public bool Equals(RigidBody other)
        {
            return (other.instance == this.instance);
        }

        public int CompareTo(RigidBody other)
        {
            if (other.instance < this.instance) return -1;
            else if (other.instance > this.instance) return 1;
            else return 0;
        }


        public bool IsStaticOrInactive()
        {
            return (!this.isActive || this.isStatic);
        }

        public int BroadphaseTag { get; set; }


        public virtual void PreStep()
        {
            //
        }

        public virtual void PostStep()
        {
            //
        }
    }
}
