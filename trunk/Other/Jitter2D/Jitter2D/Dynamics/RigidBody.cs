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
using Jitter2D.LinearMath;
using Jitter2D.Collision.Shapes;
using Jitter2D.Collision;
using Jitter2D.Dynamics.Constraints;
using Jitter2D.DataStructures;
#endregion

namespace Jitter2D.Dynamics
{


    public enum RigidBodyIndex
    {
        RigidBody1, RigidBody2
    }

    /// <summary>
    /// The RigidBody class.
    /// </summary>
    public class RigidBody : IBroadphaseEntity, IDebugDrawable, IEquatable<RigidBody>, IComparable<RigidBody>
    {
        [Flags]
        public enum DampingType { None = 0x00, Angular = 0x01, Linear = 0x02 }

        internal float inertia;
        internal float invInertia;

        internal float orientation;
        internal float invOrientation;
        internal JVector position;
        internal JVector linearVelocity;
        internal float angularVelocity;

        internal Material material;

        internal JBBox boundingBox;

        internal float inactiveTime = 0.0f;

        internal bool isActive = true;
        internal bool isStatic = false;
        internal bool affectedByGravity = true;

        internal CollisionIsland island;
        internal float inverseMass;

        internal JVector force;
        internal float torque;

        private int hashCode;

        internal int internalIndex = 0;

        private ShapeUpdatedHandler updatedHandler;

        internal List<RigidBody> connections = new List<RigidBody>();

        internal HashSet<Arbiter> arbiters = new HashSet<Arbiter>();
        internal HashSet<Constraint> constraints = new HashSet<Constraint>();

        private ReadOnlyHashset<Arbiter> readOnlyArbiters;
        private ReadOnlyHashset<Constraint> readOnlyConstraints;

        internal int marker = 0;


        public RigidBody(Shape shape)
            : this(shape, new Material(), false)
        {
        }

        internal bool isParticle = false;

        /// <summary>
        /// If true, the body as no angular movement.
        /// </summary>
        public bool IsParticle { get { return isParticle; } }

        /// <summary>
        /// Initializes a new instance of the RigidBody class.
        /// </summary>
        /// <param name="shape">The shape of the body.</param>
        public RigidBody(Shape shape, Material material)
            : this(shape, material, false)
        {
        }

        /// <summary>
        /// Initializes a new instance of the RigidBody class.
        /// </summary>
        /// <param name="shape">The shape of the body.</param>
        /// <param name="isParticle">If set to true the body doesn't rotate. 
        /// Also contacts are only solved for the linear motion part.</param>
        public RigidBody(Shape shape, Material material, bool isParticle)
        {
            readOnlyArbiters = new ReadOnlyHashset<Arbiter>(arbiters);
            readOnlyConstraints = new ReadOnlyHashset<Constraint>(constraints);

            instance = Interlocked.Increment(ref instanceCount);
            hashCode = CalculateHash(instance);

            this.Shape = shape;
            orientation = 0.0f;

            if (!isParticle)
            {
                updatedHandler = new ShapeUpdatedHandler(ShapeUpdated);
                this.Shape.ShapeUpdated += updatedHandler;
                SetMassProperties();
            }
            else
            {
                this.inertia = 0.0f;
                this.invInertia = 0.0f;
                this.invOrientation = this.orientation = 0.0f;
                inverseMass = 1.0f;
            }

            this.material = material;

            AllowDeactivation = true;

            this.isParticle = isParticle;

            Update();
        }

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

        public ReadOnlyHashset<Arbiter> Arbiters { get { return readOnlyArbiters; } }
        public ReadOnlyHashset<Constraint> Constraints { get { return readOnlyConstraints; } }

        /// <summary>
        /// If set to false the body will never be deactivated by the
        /// world.
        /// </summary>
        public bool AllowDeactivation { get; set; }

        public bool EnableSpeculativeContacts { get; set; }

        /// <summary>
        /// The axis aligned bounding box of the body.
        /// </summary>
        public JBBox BoundingBox { get { return boundingBox; } }


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
                    this.angularVelocity = 0.0f;
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

            // 3D version
            //JVector.Cross(ref relativePosition, ref impulse, out temp);
            //JVector.Transform(ref temp, ref invInertiaWorld, out temp);
            //JVector.Add(ref angularVelocity, ref temp, out angularVelocity);

            float temp1; JVector.Cross(ref relativePosition, ref impulse, out temp1);
            this.angularVelocity += this.invInertia * temp1;
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

            // 3d version
            //JVector.Cross(ref pos, ref force, out pos);
            //JVector.Add(ref pos, ref this.torque, out this.torque);

            //body->t += cpvcross(r, force);

            this.torque += JVector.Cross(pos, force);
        }

        /// <summary>
        /// Returns the torque which acts this timestep on the body.
        /// </summary>
        public float Torque { get { return torque; } }

        /// <summary>
        /// Returns the force which acts this timestep on the body.
        /// </summary>
        public JVector Force { get { return force; } }

        /// <summary>
        /// Adds torque to the body. The torque gets applied
        /// the next time <see cref="World.Step"/> is called. The 'impact'
        /// of the torque depends on the time it is applied to a body - so
        /// the timestep influences the energy added to the body.
        /// </summary>
        /// <param name="torque">The torque to add next <see cref="World.Step"/>.</param>
        public void AddTorque(float torque)
        {
            this.torque += torque;
        }

        protected bool useShapeMassProperties = true;

        /// <summary>
        /// By calling this method the shape inertia and mass is used.
        /// </summary>
        public void SetMassProperties()
        {
            this.inertia = Shape.inertia;
            this.invInertia = 1.0f / this.inertia;
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
        public void SetMassProperties(float inertia, float mass, bool setAsInverseValues)
        {
            if (setAsInverseValues)
            {
                if (!isParticle)
                {
                    this.invInertia = inertia;
                    this.invInertia = 1.0f / this.inertia;
                }
                this.inverseMass = mass;
            }
            else
            {
                if (!isParticle)
                {
                    this.inertia = inertia;
                    this.invInertia = 1.0f / this.inertia;
                }
                this.inverseMass = 1.0f / mass;
            }

            useShapeMassProperties = false;
            Update();
        }

        private void ShapeUpdated()
        {
            if (useShapeMassProperties) SetMassProperties();
            Update();
            UpdateHullData();
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
                if (shape != null) shape.ShapeUpdated -= updatedHandler;

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
        public float Inertia { get { return inertia; } }

        /// <summary>
        /// The inverse inertia currently used for this body.
        /// </summary>
        public float InverseInertia { get { return invInertia; } }

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

        /// <summary>
        /// The angular velocity of the body.
        /// </summary>
        public float AngularVelocity
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
            set { position = value; Update(); }
        }

        /// <summary>
        /// The current orientation of the body.
        /// </summary>
        public float Orientation
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
                if (value && !isStatic)
                {
                    if (island != null)
                        island.islandManager.MakeBodyStatic(this);

                    this.angularVelocity = 0.0f;
                    this.linearVelocity.MakeZero();
                }
                isStatic = value;
            }
        }

        public bool AffectedByGravity { get { return affectedByGravity; } set { affectedByGravity = value; } }


        /// <summary>
        /// Setting the mass automatically scales the inertia.
        /// To set the mass independently from the mass use SetMassProperties.
        /// </summary>
        public float Mass
        {
            get { return 1.0f / inverseMass; }
            set
            {
                if (value <= 0.0f) throw new ArgumentException("Mass can't be less or equal zero.");

                // scale inertia
                if (!isParticle)
                {
                    //JMatrix.Multiply(ref Shape.inertia, value / Shape.mass, out inertia);
                    this.invInertia = 1.0f / this.inertia;
                }

                inverseMass = 1.0f / value;
            }
        }

        #endregion

        internal JVector sweptDirection = JVector.Zero;

        public void SweptExpandBoundingBox(float timestep)
        {
            sweptDirection = linearVelocity * timestep;

            if (sweptDirection.X < 0.0f)
            {
                boundingBox.Min.X += sweptDirection.X;
            }
            else
            {
                boundingBox.Max.X += sweptDirection.X;
            }

            if (sweptDirection.Y < 0.0f)
            {
                boundingBox.Min.Y += sweptDirection.Y;
            }
            else
            {
                boundingBox.Max.Y += sweptDirection.Y;
            }
        }

        /// <summary>
        /// Recalculates the axis aligned bounding box and the inertia
        /// values in world space.
        /// </summary>
        public virtual void Update()
        {
            // particles don't rotate
            if (isParticle)
            {
                this.inertia = 0.0f;
                this.invInertia = 0.0f;
                this.invOrientation = this.orientation = 0.0f;
                this.boundingBox = shape.boundingBox;
                JVector.Add(ref boundingBox.Min, ref this.position, out boundingBox.Min);
                JVector.Add(ref boundingBox.Max, ref this.position, out boundingBox.Max);

                angularVelocity = 0.0f;
            }
            else
            {
                // Given: Orientation, Inertia
                // 3d version
                //JMatrix.Transpose(ref orientation, out invOrientation);
                invOrientation = -orientation;
                this.Shape.GetBoundingBox(ref orientation, out boundingBox);
                JVector.Add(ref boundingBox.Min, ref this.position, out boundingBox.Min);
                JVector.Add(ref boundingBox.Max, ref this.position, out boundingBox.Max);

                if (!isStatic)
                {
                    // 3d version
                    //JMatrix.Multiply(ref invOrientation, ref invInertia, out invInertiaWorld);
                    //JMatrix.Multiply(ref invInertiaWorld, ref orientation, out invInertiaWorld);

                    // we don't need a world inverse inertia
                }
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

        public int BroadphaseTag { get; set; }


        public virtual void PreStep()
        {
            //
        }

        public virtual void PostStep()
        {
            //
        }


        public bool IsStaticOrInactive
        {
            get { return (!this.isActive || this.isStatic); }
        }

        private bool enableDebugDraw = false;
        public bool EnableDebugDraw
        {
            get { return enableDebugDraw; }
            set
            {
                enableDebugDraw = value;
                UpdateHullData();
            }
        }

        private List<JVector> hullPoints = new List<JVector>();

        private void UpdateHullData()
        {
            hullPoints.Clear();

            if (enableDebugDraw) shape.MakeHull(ref hullPoints, 3);
        }

        // this method is extremely brute force, only use for debugging!
        public void DebugDraw(IDebugDrawer drawer)
        {
            JMatrix o1 = JMatrix.CreateRotationZ(orientation);
            JVector dir = JVector.Up;
            JVector u = JVector.Zero;
            JVector a;

            for (int i = -1; i <= 36; i++)
            {
                JVector.TransposedTransform(ref dir, ref o1, out a);
                // get the support in the given direction
                JVector s; this.shape.SupportMapping(ref a, out s);
                // transform the support into world space
                a = JVector.Transform(s, o1) + position;

                dir = JVector.Transform(dir, JMatrix.CreateRotationZ(0.0174532925f * 10f));

                if (i >= 0)
                {
                    if (isStatic)
                        drawer.SetColor(0.25f, 0.85f, 0.25f, 1);
                    else if (isActive)
                        drawer.SetColor(0.85f, 0.85f, 0.85f, 1);
                    else
                        drawer.SetColor(0.65f, 0.65f, 0.65f, 1);
                    drawer.DrawTriangle(a, u, this.position);
                    drawer.SetColor(0,0,0, 1);
                    drawer.DrawLine(a, u);
                }
                u = a;
            }

            JMatrix xForm = JMatrix.CreateRotationZ(orientation);

            drawer.SetColor(1, 0, 0, 1);
            drawer.DrawLine(position + JVector.Transform(JVector.Left * 0.25f, xForm), position + JVector.Transform(JVector.Zero, xForm));
            drawer.SetColor(0, 1, 0, 1);
            drawer.DrawLine(position + JVector.Transform(JVector.Up * 0.25f, xForm), position + JVector.Transform(JVector.Zero, xForm));
        }
    }
}
