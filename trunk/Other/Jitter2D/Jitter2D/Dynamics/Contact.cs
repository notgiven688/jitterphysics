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
        internal JVector p1, p2, m_ra, m_rb;
        internal float m_invDenom, m_invDenomTan;

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
            if (treatBody1AsStatic && treatBody2AsStatic) return;

            #region Jitter Version
            ////body1.linearVelocity = JVector.Zero;
            ////body2.linearVelocity = JVector.Zero;
            ////return;

            //if (treatBody1AsStatic && treatBody2AsStatic) return;

            //float dvx, dvy;//, dvz;

            //dvx = body2.linearVelocity.X - body1.linearVelocity.X;
            //dvy = body2.linearVelocity.Y - body1.linearVelocity.Y;

            //if (!body1IsMassPoint)
            //{
            //    //dvx = dvx - (body1.angularVelocity.Y * relativePos1.Z) + (body1.angularVelocity.Z * relativePos1.Y);
            //    //dvy = dvy - (body1.angularVelocity.Z * relativePos1.X) + (body1.angularVelocity.X * relativePos1.Z);
            //    //dvz = dvz - (body1.angularVelocity.X * relativePos1.Y) + (body1.angularVelocity.Y * relativePos1.X);
            //    // left in the 0 * 0 for readability
            //    dvx = dvx - (0 * 0) + (body1.angularVelocity * relativePos1.Y);
            //    dvy = dvy - (body1.angularVelocity * relativePos1.X) + (0 * 0);
            //}

            //if (!body2IsMassPoint)
            //{
            //    //dvx = dvx + (body2.angularVelocity.Y * relativePos2.Z) - (body2.angularVelocity.Z * relativePos2.Y);
            //    //dvy = dvy + (body2.angularVelocity.Z * relativePos2.X) - (body2.angularVelocity.X * relativePos2.Z);
            //    //dvz = dvz + (body2.angularVelocity.X * relativePos2.Y) - (body2.angularVelocity.Y * relativePos2.X);
            //    // left in the 0 * 0 for readability
            //    dvx = dvx + (0 * 0) - (body2.angularVelocity * relativePos2.Y);
            //    dvy = dvy + (body2.angularVelocity * relativePos2.X) - (0 * 0);
            //}

            //// this gets us some performance
            //if (dvx * dvx + dvy * dvy < settings.minVelocity * settings.minVelocity)
            //{ return; }

            //float vn = normal.X * dvx + normal.Y * dvy;
            //float normalImpulse = massNormal * (-vn + restitutionBias + speculativeVelocity);

            //float oldNormalImpulse = accumulatedNormalImpulse;
            //accumulatedNormalImpulse = oldNormalImpulse + normalImpulse;
            //if (accumulatedNormalImpulse < 0.0f) accumulatedNormalImpulse = 0.0f;
            //normalImpulse = accumulatedNormalImpulse - oldNormalImpulse;

            //float vt = dvx * tangent.X + dvy * tangent.Y;
            //float maxTangentImpulse = friction * accumulatedNormalImpulse;
            //float tangentImpulse = massTangent * (-vt);

            //float oldTangentImpulse = accumulatedTangentImpulse;
            //accumulatedTangentImpulse = oldTangentImpulse + tangentImpulse;
            //if (accumulatedTangentImpulse < -maxTangentImpulse) accumulatedTangentImpulse = -maxTangentImpulse;
            //else if (accumulatedTangentImpulse > maxTangentImpulse) accumulatedTangentImpulse = maxTangentImpulse;

            //tangentImpulse = accumulatedTangentImpulse - oldTangentImpulse;

            //// Apply contact impulse
            //JVector impulse;
            //impulse.X = normal.X * normalImpulse + tangent.X * tangentImpulse;
            //impulse.Y = normal.Y * normalImpulse + tangent.Y * tangentImpulse;
            ////impulse.Z = normal.Z * normalImpulse + tangent.Z * tangentImpulse;

            //if (!treatBody1AsStatic)
            //{
            //    body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
            //    body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
            //    //body1.linearVelocity.Z -= (impulse.Z * body1.inverseMass);

            //    if (!body1IsMassPoint)
            //    {
            //        float num0, num1, num2;
            //        //num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
            //        //num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
            //        num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;
            //        /*
            //        float num3 =
            //            (((num0 * body1.invInertiaWorld.M11) +
            //            (num1 * body1.invInertiaWorld.M21)) +
            //            (num2 * body1.invInertiaWorld.M31));
            //        float num4 =
            //            (((num0 * body1.invInertiaWorld.M12) +
            //            (num1 * body1.invInertiaWorld.M22)) +
            //            (num2 * body1.invInertiaWorld.M32));
            //        float num5 =
            //            (((num0 * body1.invInertiaWorld.M13) +
            //            (num1 * body1.invInertiaWorld.M23)) +
            //            (num2 * body1.invInertiaWorld.M33));

            //        body1.angularVelocity.X -= num3;
            //        body1.angularVelocity.Y -= num4;
            //         * */
            //        body1.angularVelocity -= num2 * body1.invInertia;
            //    }
            //}

            //if (!treatBody2AsStatic)
            //{

            //    body2.linearVelocity.X += (impulse.X * body2.inverseMass);
            //    body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
            //    //body2.linearVelocity.Z += (impulse.Z * body2.inverseMass);

            //    if (!body2IsMassPoint)
            //    {

            //        float num2;
            //        //num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
            //        //num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
            //        num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;
            //        /*
            //        float num3 =
            //            (((num0 * body2.invInertiaWorld.M11) +
            //            (num1 * body2.invInertiaWorld.M21)) +
            //            (num2 * body2.invInertiaWorld.M31));
            //        float num4 =
            //            (((num0 * body2.invInertiaWorld.M12) +
            //            (num1 * body2.invInertiaWorld.M22)) +
            //            (num2 * body2.invInertiaWorld.M32));
            //        float num5 =
            //            (((num0 * body2.invInertiaWorld.M13) +
            //            (num1 * body2.invInertiaWorld.M23)) +
            //            (num2 * body2.invInertiaWorld.M33));

            //        body2.angularVelocity.X += num3;
            //        body2.angularVelocity.Y += num4;
            //         * */
            //        body2.angularVelocity += num2 * body2.invInertia;

            //    }
            //}
            #endregion
            #region Box2d Lite Version
            // Relative velocity at contact
            JVector dv = body2.linearVelocity + JVector.Cross(body2.angularVelocity, relativePos2) - body1.linearVelocity - JVector.Cross(body1.angularVelocity, relativePos1);

            // Compute normal impulse
            float vn = JVector.Dot(dv, normal);

            float dPn = massNormal * (-vn + restitutionBias);

            // Clamp the accumulated impulse
            float Pn0 = accumulatedNormalImpulse;
            accumulatedNormalImpulse = Math.Max(Pn0 + dPn, 0.0f);
            dPn = accumulatedNormalImpulse - Pn0;

            // Apply contact impulse
            JVector Pn = dPn * normal;

            if (!body1.isStatic)
            {
                body1.linearVelocity -= body1.inverseMass * Pn;
                body1.angularVelocity -= body1.invInertia * JVector.Cross(relativePos1, Pn);
            }
            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * Pn;
                body2.angularVelocity += body2.invInertia * JVector.Cross(relativePos2, Pn);
            }

            // Relative velocity at contact
            dv = body2.linearVelocity + JVector.Cross(body2.angularVelocity, relativePos2) - body1.linearVelocity - JVector.Cross(body1.angularVelocity, relativePos1);

            JVector tangent = JVector.Cross(normal, 1.0f);
            float vt = JVector.Dot(dv, tangent);
            float dPt = massTangent * (-vt);

            // Compute friction impulse
            float maxPt = friction * accumulatedNormalImpulse;

            // Clamp friction
            float oldTangentImpulse = accumulatedTangentImpulse;
            accumulatedTangentImpulse = JMath.Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
            dPt = accumulatedTangentImpulse - oldTangentImpulse;

            // Apply contact impulse
            JVector Pt = dPt * tangent;
            if (!body1.isStatic)
            {
                body1.linearVelocity -= body1.inverseMass * Pt;
                body1.angularVelocity -= body1.invInertia * JVector.Cross(relativePos1, Pt);
            }
            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * Pt;
                body2.angularVelocity += body2.invInertia * JVector.Cross(relativePos2, Pt);
            }

            #endregion

            #region Speculative Contacts Version

            //// get all of relative normal velocity
            //JVector dv = body2.linearVelocity + m_rb * body2.angularVelocity - body1.linearVelocity + m_ra * body1.angularVelocity;
            //float relNv = JVector.Dot(dv, normal);

            //// get tangential velocity
            //tangent = normal.PerpR();
            //float tanV = JVector.Dot(dv, tangent);

            //float remove = relNv + penetration/ lastTimeStep;

            ////if (remove < 0.0f)
            //{
            //    float mag = remove * m_invDenom;
            //    float newImpulse = Math.Min(mag + accumulatedNormalImpulse, 0);
            //    float change = newImpulse - accumulatedNormalImpulse;

            //    JVector imp = normal * mag;

            //    // apply impulse
            //    if (!body1.isStatic)
            //    {
            //        body1.linearVelocity += imp * body1.inverseMass;
            //        if (!body1.isParticle)
            //            body1.angularVelocity += JVector.Dot(imp, m_ra) * body1.invInertia;
            //    }

            //    if (!body2.isStatic)
            //    {
            //        body2.linearVelocity -= imp * body2.inverseMass;
            //        if (!body2.isParticle)
            //            body2.angularVelocity -= JVector.Dot(imp, m_rb) * body2.invInertia;
            //    }

            //    accumulatedNormalImpulse = newImpulse;

            //    float kFriction = 0;

            //    float absMag = Math.Abs(mag) * kFriction;

            //    // friction
            //    mag = tanV * m_invDenomTan;
            //    newImpulse = JMath.Clamp(mag + accumulatedTangentImpulse, -absMag, absMag);
            //    change = newImpulse - accumulatedTangentImpulse;
            //    imp = tangent * mag;

            //    // apply impulse
            //    if (!body1.isStatic)
            //    {
            //        body1.linearVelocity += imp * body1.inverseMass;
            //        if (!body1.isParticle)
            //            body1.angularVelocity += JVector.Dot(imp, m_ra) * body1.invInertia;
            //    }

            //    if (!body2.isStatic)
            //    {
            //        body2.linearVelocity -= imp * body2.inverseMass;
            //        if (!body2.isParticle)
            //            body2.angularVelocity -= JVector.Dot(imp, m_rb) * body2.invInertia;
            //    }

            //    accumulatedTangentImpulse = newImpulse;
            //}

            #endregion
        }

        public float AppliedNormalImpulse { get { return accumulatedNormalImpulse; } }
        public float AppliedTangentImpulse { get { return accumulatedTangentImpulse; } }

        /// <summary>
        /// The points in wolrd space gets recalculated by transforming the
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
            //JVector temp;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
                //body1.linearVelocity.Z -= (impulse.Z * body1.inverseMass);

                float num0, num1, num2;
                //num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                //num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;
                /*
                float num3 =
                    (((num0 * body1.invInertiaWorld.M11) +
                    (num1 * body1.invInertiaWorld.M21)) +
                    (num2 * body1.invInertiaWorld.M31));
                float num4 =
                    (((num0 * body1.invInertiaWorld.M12) +
                    (num1 * body1.invInertiaWorld.M22)) +
                    (num2 * body1.invInertiaWorld.M32));
                float num5 =
                    (((num0 * body1.invInertiaWorld.M13) +
                    (num1 * body1.invInertiaWorld.M23)) +
                    (num2 * body1.invInertiaWorld.M33));

                body1.angularVelocity.X -= num3;
                body1.angularVelocity.Y -= num4;
                 * */
                body1.angularVelocity -= num2 * body1.invInertia;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
                //body2.linearVelocity.Z += (impulse.Z * body2.inverseMass);

                float num0, num1, num2;
                //num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                //num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;
                /*
                float num3 =
                    (((num0 * body2.invInertiaWorld.M11) +
                    (num1 * body2.invInertiaWorld.M21)) +
                    (num2 * body2.invInertiaWorld.M31));
                float num4 =
                    (((num0 * body2.invInertiaWorld.M12) +
                    (num1 * body2.invInertiaWorld.M22)) +
                    (num2 * body2.invInertiaWorld.M32));
                float num5 =
                    (((num0 * body2.invInertiaWorld.M13) +
                    (num1 * body2.invInertiaWorld.M23)) +
                    (num2 * body2.invInertiaWorld.M33));

                body2.angularVelocity.X += num3;
                body2.angularVelocity.Y += num4;
                 * */
                body2.angularVelocity += num2 * body2.invInertia;
            }


            #endregion
        }

        public void ApplyImpulse(JVector impulse)
        {
            #region INLINE - HighFrequency
            //JVector temp;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
                //body1.linearVelocity.Z -= (impulse.Z * body1.inverseMass);

                float num0, num1, num2;
                //num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
                //num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
                num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;
                /*
                float num3 =
                    (((num0 * body1.invInertiaWorld.M11) +
                    (num1 * body1.invInertiaWorld.M21)) +
                    (num2 * body1.invInertiaWorld.M31));
                float num4 =
                    (((num0 * body1.invInertiaWorld.M12) +
                    (num1 * body1.invInertiaWorld.M22)) +
                    (num2 * body1.invInertiaWorld.M32));
                float num5 =
                    (((num0 * body1.invInertiaWorld.M13) +
                    (num1 * body1.invInertiaWorld.M23)) +
                    (num2 * body1.invInertiaWorld.M33));

                body1.angularVelocity.X -= num3;
                body1.angularVelocity.Y -= num4;
                 * */
                body1.angularVelocity -= num2 * body1.invInertia;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
                //body2.linearVelocity.Z += (impulse.Z * body2.inverseMass);

                float num0, num1, num2;
                //num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
                //num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
                num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;
                /*
                float num3 =
                    (((num0 * body2.invInertiaWorld.M11) +
                    (num1 * body2.invInertiaWorld.M21)) +
                    (num2 * body2.invInertiaWorld.M31));
                float num4 =
                    (((num0 * body2.invInertiaWorld.M12) +
                    (num1 * body2.invInertiaWorld.M22)) +
                    (num2 * body2.invInertiaWorld.M32));
                float num5 =
                    (((num0 * body2.invInertiaWorld.M13) +
                    (num1 * body2.invInertiaWorld.M23)) +
                    (num2 * body2.invInertiaWorld.M33));

                body2.angularVelocity.X += num3;
                body2.angularVelocity.Y += num4;
                 */
                body2.angularVelocity += num2 * body2.invInertia;
            }


            #endregion
        }

        /// <summary>
        /// PrepareForIteration has to be called before <see cref="Iterate"/>.
        /// </summary>
        /// <param name="timestep">The timestep of the simulation.</param>
        public void PrepareForIteration(float timestep)
        {

            #region Jitter Version
            //float dvx, dvy, dvz;

            //dvx = (0 * 0) - (body2.angularVelocity * relativePos2.Y) + body2.linearVelocity.X;
            //dvy = (body2.angularVelocity * relativePos2.X) - (0 * 0) + body2.linearVelocity.Y;

            //dvx = dvx - (0 * 0) + (body1.angularVelocity * relativePos1.Y) - body1.linearVelocity.X;
            //dvy = dvy - (body1.angularVelocity * relativePos1.X) + (0 * 0) - body1.linearVelocity.Y;
            
            //float kNormal = 0.0f;

            ////JVector rantra = JVector.Zero;
            //float rantra = 0.0f;
            //if (!treatBody1AsStatic)
            //{
            //    kNormal += body1.inverseMass;

            //    if (!body1IsMassPoint)
            //    {
            //        // I really don't know what to do here, I think in 2D that rantra should be scalar?
            //        // JVector.Cross(ref relativePos1, ref normal, out rantra);
            //        //rantra.X = (relativePos1.Y * normal.Z) - (relativePos1.Z * normal.Y);
            //        //rantra.Y = (relativePos1.Z * normal.X) - (relativePos1.X * normal.Z);
            //        //rantra = (relativePos1.X * normal.Y) - (relativePos1.Y * normal.X);

            //        rantra = relativePos1 * normal;

            //        // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
            //        //float num0 = ((rantra.X * body1.invInertiaWorld.M11) + (rantra.Y * body1.invInertiaWorld.M21)) + (rantra.Z * body1.invInertiaWorld.M31);
            //        //float num1 = ((rantra.X * body1.invInertiaWorld.M12) + (rantra.Y * body1.invInertiaWorld.M22)) + (rantra.Z * body1.invInertiaWorld.M32);
            //        //float num2 = ((rantra.X * body1.invInertiaWorld.M13) + (rantra.Y * body1.invInertiaWorld.M23)) + (rantra.Z * body1.invInertiaWorld.M33);

            //        //rantra.X = num0; rantra.Y = num1; rantra.Z = num2;

            //        // this doesn't make sense in 2D
            //        //JVector.Cross(ref rantra, ref relativePos1, out rantra);
            //        //num0 = (rantra.Y * relativePos1.Z) - (rantra.Z * relativePos1.Y);
            //        //num1 = (rantra.Z * relativePos1.X) - (rantra.X * relativePos1.Z);
            //        //num2 = (rantra.X * relativePos1.Y) - (rantra.Y * relativePos1.X);

            //        //rantra.X = num0; rantra.Y = num1; rantra.Z = num2;
            //    }
            //}

            ////JVector rbntrb = JVector.Zero;
            //float rbntrb = 0.0f;
            //if (!treatBody2AsStatic)
            //{
            //    kNormal += body2.inverseMass;

            //    if (!body2IsMassPoint)
            //    {
            //        // I really don't know what to do here, I think in 2D that rbntrb should be scalar?
            //        // JVector.Cross(ref relativePos1, ref normal, out rbntrb);
            //        //rbntrb.X = (relativePos2.Y * normal.Z) - (relativePos2.Z * normal.Y);
            //        //rbntrb.Y = (relativePos2.Z * normal.X) - (relativePos2.X * normal.Z);
            //        //rbntrb = (relativePos2.X * normal.Y) - (relativePos2.Y * normal.X);

            //        rbntrb = relativePos2 * normal;

            //        // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
            //        //float num0 = ((rbntrb.X * body2.invInertiaWorld.M11) + (rbntrb.Y * body2.invInertiaWorld.M21)) + (rbntrb.Z * body2.invInertiaWorld.M31);
            //        //float num1 = ((rbntrb.X * body2.invInertiaWorld.M12) + (rbntrb.Y * body2.invInertiaWorld.M22)) + (rbntrb.Z * body2.invInertiaWorld.M32);
            //        //float num2 = ((rbntrb.X * body2.invInertiaWorld.M13) + (rbntrb.Y * body2.invInertiaWorld.M23)) + (rbntrb.Z * body2.invInertiaWorld.M33);

            //        //rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;

            //        //JVector.Cross(ref rantra, ref relativePos1, out rantra);
            //        //num0 = (rbntrb.Y * relativePos2.Z) - (rbntrb.Z * relativePos2.Y);
            //        //num1 = (rbntrb.Z * relativePos2.X) - (rbntrb.X * relativePos2.Z);
            //        //num2 = (rbntrb.X * relativePos2.Y) - (rbntrb.Y * relativePos2.X);

            //        //rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;
            //    }
            //}
            // // this doesn't make sense
            //if (!treatBody1AsStatic) kNormal += body1.invInertia * relativePos1.LengthSquared() - rantra * rantra;
            //if (!treatBody2AsStatic) kNormal += body2.invInertia * relativePos2.LengthSquared() - rbntrb * rbntrb;

            //massNormal = 1.0f / kNormal;

            ////float num = dvx * normal.X + dvy * normal.Y;

            //tangent.X = -normal.Y;
            //tangent.Y = normal.X;

            ////num = tangent.X * tangent.X + tangent.Y * tangent.Y;

            ////if (num != 0.0f)
            ////{
            ////    num = (float)Math.Sqrt(num);
            ////    tangent.X /= num;
            ////    tangent.Y /= num;
            ////}

            //float kTangent = 0.0f;

            //if (treatBody1AsStatic) rantra = 0.0f;
            //else
            //{
            //    kTangent += body1.inverseMass;

            //    if (!body1IsMassPoint)
            //    {
            //        // again i have no idea how this applies to 2D
            //        // JVector.Cross(ref relativePos1, ref normal, out rantra);
            //        //rantra.X = (relativePos1.Y * tangent.Z) - (relativePos1.Z * tangent.Y);
            //        //rantra.Y = (relativePos1.Z * tangent.X) - (relativePos1.X * tangent.Z);

            //        rantra = relativePos1 * tangent;

            //        // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
            //        //float num0 = ((rantra.X * body1.invInertiaWorld.M11) + (rantra.Y * body1.invInertiaWorld.M21)) + (rantra.Z * body1.invInertiaWorld.M31);
            //        //float num1 = ((rantra.X * body1.invInertiaWorld.M12) + (rantra.Y * body1.invInertiaWorld.M22)) + (rantra.Z * body1.invInertiaWorld.M32);
            //        //float num2 = ((rantra.X * body1.invInertiaWorld.M13) + (rantra.Y * body1.invInertiaWorld.M23)) + (rantra.Z * body1.invInertiaWorld.M33);

            //        //rantra.X = num0; rantra.Y = num1; rantra.Z = num2;

            //        //JVector.Cross(ref rantra, ref relativePos1, out rantra);
            //        //num0 = (rantra.Y * relativePos1.Z) - (rantra.Z * relativePos1.Y);
            //        //num1 = (rantra.Z * relativePos1.X) - (rantra.X * relativePos1.Z);
            //        //num2 = (rantra.X * relativePos1.Y) - (rantra.Y * relativePos1.X);

            //        //rantra.X = num0; rantra.Y = num1; rantra.Z = num2;
            //    }

            //}

            //if (treatBody2AsStatic) rbntrb = 0.0f;
            //else
            //{
            //    kTangent += body2.inverseMass;

            //    if (!body2IsMassPoint)
            //    {
            //        // JVector.Cross(ref relativePos1, ref normal, out rantra);
            //        //rbntrb.X = (relativePos2.Y * tangent.Z) - (relativePos2.Z * tangent.Y);
            //        //rbntrb.Y = (relativePos2.Z * tangent.X) - (relativePos2.X * tangent.Z);

            //        rantra = relativePos1 * tangent;

            //        // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
            //        //float num0 = ((rbntrb.X * body2.invInertiaWorld.M11) + (rbntrb.Y * body2.invInertiaWorld.M21)) + (rbntrb.Z * body2.invInertiaWorld.M31);
            //        //float num1 = ((rbntrb.X * body2.invInertiaWorld.M12) + (rbntrb.Y * body2.invInertiaWorld.M22)) + (rbntrb.Z * body2.invInertiaWorld.M32);
            //        //float num2 = ((rbntrb.X * body2.invInertiaWorld.M13) + (rbntrb.Y * body2.invInertiaWorld.M23)) + (rbntrb.Z * body2.invInertiaWorld.M33);

            //        //rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;

            //        //JVector.Cross(ref rantra, ref relativePos1, out rantra);
            //        //num0 = (rbntrb.Y * relativePos2.Z) - (rbntrb.Z * relativePos2.Y);
            //        //num1 = (rbntrb.Z * relativePos2.X) - (rbntrb.X * relativePos2.Z);
            //        //num2 = (rbntrb.X * relativePos2.Y) - (rbntrb.Y * relativePos2.X);

            //        //rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;
            //    }
            //}

            //// ?
            ////if (!treatBody1AsStatic) kTangent += JVector.Dot(ref rantra, ref tangent);
            ////if (!treatBody2AsStatic) kTangent += JVector.Dot(ref rbntrb, ref tangent);
            //massTangent = 1.0f / kTangent;

            //restitutionBias = lostSpeculativeBounce;

            //speculativeVelocity = 0.0f;

            //float relNormalVel = normal.X * dvx + normal.Y * dvy; //JVector.Dot(ref normal, ref dv);

            //if (Penetration > settings.allowedPenetration)
            //{
            //    restitutionBias = settings.bias * (1.0f / timestep) * JMath.Max(0.0f, Penetration - settings.allowedPenetration);
            //    restitutionBias = JMath.Clamp(restitutionBias, 0.0f, settings.maximumBias);
            //    //  body1IsMassPoint = body2IsMassPoint = false;
            //}


            //float timeStepRatio = timestep / lastTimeStep;
            //accumulatedNormalImpulse *= timeStepRatio;
            //accumulatedTangentImpulse *= timeStepRatio;

            //{
            //    // Static/Dynamic friction
            //    float relTangentVel = -(tangent.X * dvx + tangent.Y * dvy);
            //    float tangentImpulse = massTangent * relTangentVel;
            //    float maxTangentImpulse = -staticFriction * accumulatedNormalImpulse;

            //    if (tangentImpulse < maxTangentImpulse) friction = dynamicFriction;
            //    else friction = staticFriction;
            //}

            //JVector impulse;

            //// Simultaneous solving and restitution is simply not possible
            //// so fake it a bit by just applying restitution impulse when there
            //// is a new contact.
            //if (relNormalVel < -1.0f && newContact)
            //{
            //    restitutionBias = Math.Max(-restitution * relNormalVel, restitutionBias);
            //}

            //// Speculative Contacts!
            //// if the penetration is negative (which means the bodies are not already in contact, but they will
            //// be in the future) we store the current bounce bias in the variable 'lostSpeculativeBounce'
            //// and apply it the next frame, when the speculative contact was already solved.
            //if (penetration < -settings.allowedPenetration)
            //{
            //    speculativeVelocity = penetration / timestep;

            //    lostSpeculativeBounce = restitutionBias;
            //    restitutionBias = 0.0f;
            //}
            //else
            //{
            //    lostSpeculativeBounce = 0.0f;
            //}

            //impulse.X = normal.X * accumulatedNormalImpulse + tangent.X * accumulatedTangentImpulse;
            //impulse.Y = normal.Y * accumulatedNormalImpulse + tangent.Y * accumulatedTangentImpulse;
            ////impulse.Z = normal.Z * accumulatedNormalImpulse + tangent.Z * accumulatedTangentImpulse;

            //if (!treatBody1AsStatic)
            //{
            //    //body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
            //    //body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
            //    //body1.linearVelocity.Z -= (impulse.Z * body1.inverseMass);

            //    if (!body1IsMassPoint)
            //    {
            //        float num0, num1, num2;
            //        //num0 = relativePos1.Y * impulse.Z - relativePos1.Z * impulse.Y;
            //        //num1 = relativePos1.Z * impulse.X - relativePos1.X * impulse.Z;
            //        num2 = relativePos1.X * impulse.Y - relativePos1.Y * impulse.X;
            //        /*
            //        float num3 =
            //            (((num0 * body1.invInertiaWorld.M11) +
            //            (num1 * body1.invInertiaWorld.M21)) +
            //            (num2 * body1.invInertiaWorld.M31));
            //        float num4 =
            //            (((num0 * body1.invInertiaWorld.M12) +
            //            (num1 * body1.invInertiaWorld.M22)) +
            //            (num2 * body1.invInertiaWorld.M32));
            //        float num5 =
            //            (((num0 * body1.invInertiaWorld.M13) +
            //            (num1 * body1.invInertiaWorld.M23)) +
            //            (num2 * body1.invInertiaWorld.M33));

            //        body1.angularVelocity.X -= num3;
            //        body1.angularVelocity.Y -= num4;
            //         * */
            //        //body1.angularVelocity -= num2 * body1.invInertia;

            //    }
            //}

            //if (!treatBody2AsStatic)
            //{

            //    //body2.linearVelocity.X += (impulse.X * body2.inverseMass);
            //    //body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
            //    //body2.linearVelocity.Z += (impulse.Z * body2.inverseMass);

            //    if (!body2IsMassPoint)
            //    {

            //        float num0, num1, num2;
            //        //num0 = relativePos2.Y * impulse.Z - relativePos2.Z * impulse.Y;
            //        //num1 = relativePos2.Z * impulse.X - relativePos2.X * impulse.Z;
            //        num2 = relativePos2.X * impulse.Y - relativePos2.Y * impulse.X;
            //        /*
            //        float num3 =
            //            (((num0 * body2.invInertiaWorld.M11) +
            //            (num1 * body2.invInertiaWorld.M21)) +
            //            (num2 * body2.invInertiaWorld.M31));
            //        float num4 =
            //            (((num0 * body2.invInertiaWorld.M12) +
            //            (num1 * body2.invInertiaWorld.M22)) +
            //            (num2 * body2.invInertiaWorld.M32));
            //        float num5 =
            //            (((num0 * body2.invInertiaWorld.M13) +
            //            (num1 * body2.invInertiaWorld.M23)) +
            //            (num2 * body2.invInertiaWorld.M33));

            //        body2.angularVelocity.X += num3;
            //        body2.angularVelocity.Y += num4;
            //         * */
            //        //body2.angularVelocity += num2 * body2.invInertia;
            //    }
            //}
            #endregion

            #region Box2d Lite Version

            float rn1 = JVector.Dot(this.relativePos1, this.normal);
            float rn2 = JVector.Dot(this.relativePos2, this.normal);
            float kNormal = body1.inverseMass + body2.inverseMass;
            kNormal += body1.invInertia * (JVector.Dot(relativePos1, relativePos1) - rn1 * rn1) + body2.invInertia * (JVector.Dot(relativePos2, relativePos2) - rn2 * rn2);
            massNormal = 1.0f / kNormal;

            JVector tangent = JVector.Cross(normal, 1.0f);
            float rt1 = JVector.Dot(relativePos1, tangent);
            float rt2 = JVector.Dot(relativePos2, tangent);
            float kTangent = body1.inverseMass + body2.inverseMass;
            kTangent += body1.invInertia * (JVector.Dot(relativePos1, relativePos1) - rt1 * rt1) + body2.invInertia * (JVector.Dot(relativePos2, relativePos2) - rt2 * rt2);
            massTangent = 1.0f / kTangent;

            float k_biasFactor = 0.2f;
            float k_allowedPenetration = 0.01f;

            this.restitutionBias = -k_biasFactor * (1f / timestep) * Math.Min(0.0f, penetration + k_allowedPenetration);

            // Relative velocity at contact
            JVector dv = body2.linearVelocity + JVector.Cross(body2.angularVelocity, relativePos2) - body1.linearVelocity - JVector.Cross(body1.angularVelocity, relativePos1);

            // Static/Dynamic friction
            float relTangentVel = -(tangent.X * dv.X + tangent.Y * dv.Y);
            float tangentImpulse = massTangent * relTangentVel;
            float maxTangentImpulse = -staticFriction * accumulatedNormalImpulse;

            if (tangentImpulse < maxTangentImpulse) friction = dynamicFriction;
            else friction = staticFriction;

            // Apply normal + friction impulse
            JVector P = this.accumulatedNormalImpulse * normal + accumulatedTangentImpulse * tangent;
            if (!body1.isStatic)
            {
                body1.linearVelocity -= body1.inverseMass * P;
                body1.angularVelocity -= body1.invInertia * JVector.Cross(relativePos1, P);
            }
            if (!body2.isStatic)
            {
                body2.linearVelocity += body2.inverseMass * P;
                body2.angularVelocity += body2.invInertia * JVector.Cross(relativePos2, P);
            }
            #endregion

            #region Speculative Contacts Version

            //// calculate radius arms
            //m_ra = (p1 - body1.position).PerpR();
            //m_rb = (p2 - body2.position).PerpR();

            //// compute denominator in impulse equation
            //float a = body1.inverseMass;
            //float b = body2.inverseMass;
            //float ran = JVector.Dot(m_ra, normal);
            //float rbn = JVector.Dot(m_rb, normal);
            //float c = ran * ran * body1.invInertia;
            //float d = rbn * rbn * body2.invInertia;

            //m_invDenom = 1 / (a + b + c + d);

            //JVector tangent = normal.PerpR();
            //ran = JVector.Dot(m_ra, tangent);
            //rbn = JVector.Dot(m_rb, tangent);
            //c = ran * ran * body1.invInertia;
            //d = rbn * rbn * body2.invInertia;

            //m_invDenomTan = 1 / (a + b + c + d);


            #endregion

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
