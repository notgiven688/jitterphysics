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
#define JITTER_CONTACT_SOLVER
//#define SPEC_CONTACT_SOLVER
//#define BOX2D_CONTACT_SOLVER

#region Using Statements
using System;
using System.Collections.Generic;

using Jitter2D.Dynamics;
using Jitter2D.LinearMath;
using Jitter2D.Collision.Shapes;
using Jitter2D.Dynamics.Constraints;
#endregion

namespace Jitter2D.Dynamics
{

    #region public class ContactSettings
    public class ContactSettings
    {
        public enum MaterialCoefficientMixingType { TakeMaximum, TakeMinimum, UseAverage }

        internal float maximumBias = 10.0f;
        internal float bias = 0.25f;
        internal float minVelocity = 0.001f;
        internal float allowedPenetration = 0.01f;
        internal float breakThreshold = 0.01f;

        internal MaterialCoefficientMixingType materialMode = MaterialCoefficientMixingType.UseAverage;

        public float MaximumBias { get { return maximumBias; } set { maximumBias = value; } }

        public float BiasFactor { get { return bias; } set { bias = value; } }

        public float MinimumVelocity { get { return minVelocity; } set { minVelocity = value; } }

        public float AllowedPenetration { get { return allowedPenetration; } set { allowedPenetration = value; } }

        public float BreakThreshold { get { return breakThreshold; } set { breakThreshold = value; } }

        public MaterialCoefficientMixingType MaterialCoefficientMixing { get { return materialMode; } set { materialMode = value; } }
    }
    #endregion


    /// <summary>
    /// </summary>
    public class Contact : IConstraint
    {
        private ContactSettings settings;

        internal RigidBody body1, body2;

        internal JVector normal, tangent;

        internal JVector realRelPos1, realRelPos2;
        internal JVector relativePos1, relativePos2;
        internal JVector p1, p2;

        internal float accumulatedNormalImpulse = 0.0f;
        internal float accumulatedTangentImpulse = 0.0f;

        internal float penetration = 0.0f;
        internal float initialPen = 0.0f;

        private float staticFriction, dynamicFriction, restitution;
        private float friction = 0.0f;

        private float massNormal = 0.0f, massTangent = 0.0f;
        private float restitutionBias = 0.0f;

        private bool newContact = false;

        private bool treatBody1AsStatic = false;
        private bool treatBody2AsStatic = false;


        bool body1IsMassPoint; bool body2IsMassPoint;

        float lostSpeculativeBounce = 0.0f;
        float speculativeVelocity = 0.0f;

        /// <summary>
        /// A contact resource pool.
        /// </summary>
        public static readonly ResourcePool<Contact> Pool =
            new ResourcePool<Contact>();

        private float lastTimeStep = float.PositiveInfinity;

        #region Properties
        public float Restitution
        {
            get { return restitution; }
            set { restitution = value; }
        }

        public float StaticFriction
        {
            get { return staticFriction; }
            set { staticFriction = value; }
        }

        public float DynamicFriction
        {
            get { return dynamicFriction; }
            set { dynamicFriction = value; }
        }

        /// <summary>
        /// The first body involved in the contact.
        /// </summary>
        public RigidBody Body1 { get { return body1; } }

        /// <summary>
        /// The second body involved in the contact.
        /// </summary>
        public RigidBody Body2 { get { return body2; } }

        /// <summary>
        /// The penetration of the contact.
        /// </summary>
        public float Penetration { get { return penetration; } }

        /// <summary>
        /// The collision position in world space of body1.
        /// </summary>
        public JVector Position1 { get { return p1; } }

        /// <summary>
        /// The collision position in world space of body2.
        /// </summary>
        public JVector Position2 { get { return p2; } }

        /// <summary>
        /// The contact tangent.
        /// </summary>
        public JVector Tangent { get { return tangent; } }

        /// <summary>
        /// The contact normal.
        /// </summary>
        public JVector Normal { get { return normal; } }
        #endregion

        /// <summary>
        /// Calculates relative velocity of body contact points on the bodies.
        /// </summary>
        /// <param name="relVel">The relative velocity of body contact points on the bodies.</param>
        public JVector CalculateRelativeVelocity()
        {
            throw new NotImplementedException();
            /*
            float x, y, z;

            x = (body2.angularVelocity.Y * relativePos2.Z) - (body2.angularVelocity.Z * relativePos2.Y) + body2.linearVelocity.X;
            y = (body2.angularVelocity.Z * relativePos2.X) - (body2.angularVelocity.X * relativePos2.Z) + body2.linearVelocity.Y;
            z = (body2.angularVelocity.X * relativePos2.Y) - (body2.angularVelocity.Y * relativePos2.X) + body2.linearVelocity.Z;

            JVector relVel;
            relVel.X = x - (body1.angularVelocity.Y * relativePos1.Z) + (body1.angularVelocity.Z * relativePos1.Y) - body1.linearVelocity.X;
            relVel.Y = y - (body1.angularVelocity.Z * relativePos1.X) + (body1.angularVelocity.X * relativePos1.Z) - body1.linearVelocity.Y;
            relVel.Z = z - (body1.angularVelocity.X * relativePos1.Y) + (body1.angularVelocity.Y * relativePos1.X) - body1.linearVelocity.Z;

            return relVel;
             * */
        }

        /// <summary>
        /// Solves the contact iteratively.
        /// </summary>
        public void Iterate()
        {
            // if both bodies are static then were done
            if (treatBody1AsStatic && treatBody2AsStatic) return;

            float dvx, dvy;

            // get relative linear velocity
            dvx = body2.linearVelocity.X - body1.linearVelocity.X;
            dvy = body2.linearVelocity.Y - body1.linearVelocity.Y;

            // if body1 isn't a particle
            if (!body1IsMassPoint)
            {
                // add in the angular velocity
                dvx = dvx - -body1.angularVelocity * relativePos1.Y;
                dvy = dvy - body1.angularVelocity * relativePos1.X;
            }

            // if body2 isn't a particle
            if (!body2IsMassPoint)
            {
                // add in the angular velocity
                dvx = dvx + -body2.angularVelocity * relativePos2.Y;
                dvy = dvy + body2.angularVelocity * relativePos2.X;
            }

            // this gets us some performance
            if (dvx * dvx + dvy * dvy < settings.minVelocity * settings.minVelocity)
            { return; }

            // find the normal impulse
            float vn = normal.X * dvx + normal.Y * dvy;
            float normalImpulse = massNormal * (-vn + restitutionBias + speculativeVelocity);

            // clamp the accumulated impulse
            float oldNormalImpulse = accumulatedNormalImpulse;
            accumulatedNormalImpulse = oldNormalImpulse + normalImpulse;
            if (accumulatedNormalImpulse < 0.0f) accumulatedNormalImpulse = 0.0f;
            normalImpulse = accumulatedNormalImpulse - oldNormalImpulse;

            // find the tangent impulse
            float vt = dvx * tangent.X + dvy * tangent.Y;
            float maxTangentImpulse = friction * accumulatedNormalImpulse;
            float tangentImpulse = massTangent * (-vt);

            // clamp the accumulated impulse
            float oldTangentImpulse = accumulatedTangentImpulse;
            accumulatedTangentImpulse = oldTangentImpulse + tangentImpulse;
            if (accumulatedTangentImpulse < -maxTangentImpulse) accumulatedTangentImpulse = -maxTangentImpulse;
            else if (accumulatedTangentImpulse > maxTangentImpulse) accumulatedTangentImpulse = maxTangentImpulse;
            tangentImpulse = accumulatedTangentImpulse - oldTangentImpulse;

            // Apply contact impulse
            JVector impulse;
            // combine normal and tangent impulses into one impulse
            impulse.X = normal.X * normalImpulse + tangent.X * tangentImpulse;
            impulse.Y = normal.Y * normalImpulse + tangent.Y * tangentImpulse;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);

                if (!body1IsMassPoint)
                {
                    body1.angularVelocity -= body1.invInertia * (relativePos1.X * impulse.Y - relativePos1.Y * impulse.X);
                }
            }

            if (!treatBody2AsStatic)
            {
                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);

                if (!body2IsMassPoint)
                {
                    body2.angularVelocity += body2.invInertia * (relativePos2.X * impulse.Y - relativePos2.Y * impulse.X);
                }
            }
        }

        public float AppliedNormalImpulse { get { return accumulatedNormalImpulse; } }
        public float AppliedTangentImpulse { get { return accumulatedTangentImpulse; } }

        /// <summary>
        /// The points in world space gets recalculated by transforming the
        /// local coordinates. Also new penetration depth is estimated.
        /// </summary>
        public void UpdatePosition()
        {
            if (body1IsMassPoint)
            {
                JVector.Add(ref realRelPos1, ref body1.position, out p1);
            }
            else
            {
                JMatrix o1 = JMatrix.CreateRotationZ(body1.orientation);
                JVector.Transform(ref realRelPos1, ref o1, out p1);
                JVector.Add(ref p1, ref body1.position, out p1);
            }

            if (body2IsMassPoint)
            {
                JVector.Add(ref realRelPos2, ref body2.position, out p2);
            }
            else
            {
                JMatrix o2 = JMatrix.CreateRotationZ(body2.orientation);
                JVector.Transform(ref realRelPos2, ref o2, out p2);
                JVector.Add(ref p2, ref body2.position, out p2);
            }


            JVector dist; JVector.Subtract(ref p1, ref p2, out dist);
            penetration = JVector.Dot(ref dist, ref normal);
        }

        /// <summary>
        /// An impulse is applied an both contact points.
        /// </summary>
        /// <param name="impulse">The impulse to apply.</param>
        public void ApplyImpulse(ref JVector impulse)
        {
            #region INLINE - HighFrequency

            throw new NotImplementedException();

            #endregion
        }

        public void ApplyImpulse(JVector impulse)
        {
            #region INLINE - HighFrequency
            
            throw new NotImplementedException();

            #endregion
        }

        /// <summary>
        /// PrepareForIteration has to be called before <see cref="Iterate"/>.
        /// </summary>
        /// <param name="timestep">The timestep of the simulation.</param>
        public void PrepareForIteration(float timestep)
        {
            float dvx, dvy;

            // relative velocity at contact point
            dvx = body2.linearVelocity.X + (-body2.angularVelocity * relativePos2.Y);
            dvy = body2.linearVelocity.Y + (body2.angularVelocity * relativePos2.X);

            dvx = dvx - body1.linearVelocity.X + (-body1.angularVelocity * relativePos1.Y);
            dvy = dvy - body1.linearVelocity.Y + (body1.angularVelocity * relativePos1.X);

            float kNormal = 0.0f;

            float rantra = 0.0f;

            // if body1 isn't static
            if (!treatBody1AsStatic)
            {
                // add it's mass to the mass normal
                kNormal += body1.inverseMass;

                // if body1 isn't a mass point (particle) 
                if (!body1IsMassPoint)
                {
                    // 
                    rantra = relativePos1.X * normal.Y - relativePos1.Y * normal.X;
                }
            }

            float rbntrb = 0.0f;

            // if body2 isn't static
            if (!treatBody2AsStatic)
            {
                // add it's mass to the mass normal
                kNormal += body2.inverseMass;

                // if body1 isn't a mass point (particle)
                if (!body2IsMassPoint)
                {
                    //
                    rbntrb = relativePos2.X * normal.Y - relativePos2.Y * normal.X;
                }
            }

            // compute overall mass normal
            if (!treatBody1AsStatic) kNormal += body1.invInertia * (rantra * rantra);
            if (!treatBody2AsStatic) kNormal += body2.invInertia * (rbntrb * rbntrb);

            massNormal = 1.0f / kNormal;

            tangent.X = -normal.Y;
            tangent.Y = normal.X;

            float kTangent = 0.0f;

            if (treatBody1AsStatic) rantra = 0.0f;
            else
            {
                kTangent += body1.inverseMass;

                if (!body1IsMassPoint)
                {
                    // 
                    rantra = relativePos1.X * tangent.Y - relativePos1.Y * tangent.X;
                }

            }

            if (treatBody2AsStatic) rbntrb = 0.0f;
            else
            {
                kTangent += body2.inverseMass;

                if (!body2IsMassPoint)
                {
                    rantra = relativePos2.X * tangent.Y - relativePos2.Y * tangent.X;
                }
            }

            // compute overall mass tangent
            if (!treatBody1AsStatic) kTangent += body1.invInertia * (rantra * rantra);
            if (!treatBody2AsStatic) kTangent += body2.invInertia * (rbntrb * rbntrb);

            massTangent = 1.0f / kTangent;

            restitutionBias = lostSpeculativeBounce;

            speculativeVelocity = 0.0f;

            float relNormalVel = JVector.Dot(normal, body2.linearVelocity + JVector.Cross(body2.angularVelocity, relativePos2) - body1.linearVelocity + JVector.Cross(body1.angularVelocity, relativePos1));

            if (Penetration > settings.allowedPenetration)
            {
                restitutionBias = settings.bias * (1.0f / timestep) * JMath.Max(0.0f, Penetration - settings.allowedPenetration);
                restitutionBias = JMath.Clamp(restitutionBias, 0.0f, settings.maximumBias);
                //  body1IsMassPoint = body2IsMassPoint = false;
            }


            float timeStepRatio = timestep / lastTimeStep;
            accumulatedNormalImpulse *= timeStepRatio;
            accumulatedTangentImpulse *= timeStepRatio;

            {
                // Static/Dynamic friction
                float relTangentVel = -(tangent.X * dvx + tangent.Y * dvy);
                float tangentImpulse = massTangent * relTangentVel;
                float maxTangentImpulse = -staticFriction * accumulatedNormalImpulse;

                if (tangentImpulse < maxTangentImpulse) friction = dynamicFriction;
                else friction = staticFriction;
            }

            JVector impulse;

            // Simultaneous solving and restitution is simply not possible
            // so fake it a bit by just applying restitution impulse when there
            // is a new contact.
            if (relNormalVel < -1.0f && newContact)
            {
                restitutionBias = Math.Max(-restitution * relNormalVel, restitutionBias);
            }

            // Speculative Contacts!
            // if the penetration is negative (which means the bodies are not already in contact, but they will
            // be in the future) we store the current bounce bias in the variable 'lostSpeculativeBounce'
            // and apply it the next frame, when the speculative contact was already solved.
            if (penetration < -settings.allowedPenetration)
            {
                speculativeVelocity = penetration / timestep;

                lostSpeculativeBounce = restitutionBias;
                restitutionBias = 0.0f;
            }
            else
            {
                lostSpeculativeBounce = 0.0f;
            }

            // warm start

            impulse.X = normal.X * accumulatedNormalImpulse + tangent.X * accumulatedTangentImpulse;
            impulse.Y = normal.Y * accumulatedNormalImpulse + tangent.Y * accumulatedTangentImpulse;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);

                if (!body1IsMassPoint)
                {
                    body1.angularVelocity -= body1.invInertia * (relativePos1.X * impulse.Y - relativePos1.Y * impulse.X);
                }
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);

                if (!body2IsMassPoint)
                {
                    body2.angularVelocity += body2.invInertia * (relativePos2.X * impulse.Y - relativePos2.Y * impulse.X);
                }
            }
           
            lastTimeStep = timestep;

            newContact = false;
        }

        public void TreatBodyAsStatic(RigidBodyIndex index)
        {
            if (index == RigidBodyIndex.RigidBody1) treatBody1AsStatic = true;
            else treatBody2AsStatic = true;
        }


        /// <summary>
        /// Initializes a contact.
        /// </summary>
        /// <param name="body1">The first body.</param>
        /// <param name="body2">The second body.</param>
        /// <param name="point1">The collision point in worldspace</param>
        /// <param name="point2">The collision point in worldspace</param>
        /// <param name="n">The normal pointing to body2.</param>
        /// <param name="penetration">The estimated penetration depth.</param>
        public void Initialize(RigidBody body1, RigidBody body2, ref JVector point1, ref JVector point2, ref JVector n,
            float penetration, bool newContact, ContactSettings settings)
        {
            this.body1 = body1; this.body2 = body2;
            this.normal = n; normal.Normalize();
            this.p1 = point1; this.p2 = point2;

            this.newContact = newContact;

            JVector.Subtract(ref p1, ref body1.position, out relativePos1);
            JVector.Subtract(ref p2, ref body2.position, out relativePos2);
            JMatrix o1 = JMatrix.CreateRotationZ(body1.invOrientation);
            JMatrix o2 = JMatrix.CreateRotationZ(body2.invOrientation);
            JVector.Transform(ref relativePos1, ref o1, out realRelPos1);
            JVector.Transform(ref relativePos2, ref o2, out realRelPos2);

            this.initialPen = penetration;
            this.penetration = penetration;

            body1IsMassPoint = body1.isParticle;
            body2IsMassPoint = body2.isParticle;

            // Material Properties
            if (newContact)
            {
                treatBody1AsStatic = body1.isStatic;
                treatBody2AsStatic = body2.isStatic;

                accumulatedNormalImpulse = 0.0f;
                accumulatedTangentImpulse = 0.0f;

                lostSpeculativeBounce = 0.0f;

                switch (settings.MaterialCoefficientMixing)
                {
                    case ContactSettings.MaterialCoefficientMixingType.TakeMaximum:
                        staticFriction = JMath.Max(body1.material.staticFriction, body2.material.staticFriction);
                        dynamicFriction = JMath.Max(body1.material.kineticFriction, body2.material.kineticFriction);
                        restitution = JMath.Max(body1.material.restitution, body2.material.restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.TakeMinimum:
                        staticFriction = JMath.Min(body1.material.staticFriction, body2.material.staticFriction);
                        dynamicFriction = JMath.Min(body1.material.kineticFriction, body2.material.kineticFriction);
                        restitution = JMath.Min(body1.material.restitution, body2.material.restitution);
                        break;
                    case ContactSettings.MaterialCoefficientMixingType.UseAverage:
                        staticFriction = (body1.material.staticFriction + body2.material.staticFriction) / 2.0f;
                        dynamicFriction = (body1.material.kineticFriction + body2.material.kineticFriction) / 2.0f;
                        restitution = (body1.material.restitution + body2.material.restitution) / 2.0f;
                        break;
                }

            }
            this.settings = settings;
        }
    }
}
