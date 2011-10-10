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
    /// <summary>
    /// </summary>
    public class Contact2 : IConstraint
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

        /// <summary>
        /// A contact resource pool.
        /// </summary>
        public static readonly ResourcePool<Contact2> Pool =
            new ResourcePool<Contact2>();

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
            // if the bodies are static...ignore them
            if (treatBody1AsStatic && treatBody2AsStatic) return;

            float dvx, dvy;

            dvx = body2.linearVelocity.X - body1.linearVelocity.X;
            dvy = body2.linearVelocity.Y - body1.linearVelocity.Y;
           
            if (!body1IsParticle)
            {
                dvx = dvx - (0.0f * 0.0f) + (body1.angularVelocity * relativePos1.Y);
                dvy = dvy - (body1.angularVelocity * relativePos1.X) + (0.0f * 0.0f);
                }

            if (!body2IsParticle)
            {
                dvx = dvx + (0.0f * 0.0f) - (body2.angularVelocity * relativePos2.Y);
                dvy = dvy + (body2.angularVelocity * relativePos2.X) - (0.0f * 0.0f);
            }

            // this gets us some performance
            if (dvx * dvx + dvy * dvy < settings.minVelocity * settings.minVelocity)
            { return; }

            float vn = normal.X * dvx + normal.Y * dvy;
            float normalImpulse = massNormal * (-vn + restitutionBias);

            float oldNormalImpulse = accumulatedNormalImpulse;
            accumulatedNormalImpulse = oldNormalImpulse + normalImpulse;
            if (accumulatedNormalImpulse < 0.0f) accumulatedNormalImpulse = 0.0f;
            normalImpulse = accumulatedNormalImpulse - oldNormalImpulse;

            float vt = dvx * tangent.X + dvy * tangent.Y;
            float maxTangentImpulse = friction * accumulatedNormalImpulse;
            float tangentImpulse = massTangent * (-vt);

            float oldTangentImpulse = accumulatedTangentImpulse;
            accumulatedTangentImpulse = oldTangentImpulse + tangentImpulse;
            if (accumulatedTangentImpulse < -maxTangentImpulse) accumulatedTangentImpulse = -maxTangentImpulse;
            else if (accumulatedTangentImpulse > maxTangentImpulse) accumulatedTangentImpulse = maxTangentImpulse;

            tangentImpulse = accumulatedTangentImpulse - oldTangentImpulse;

            // Apply contact impulse
            JVector impulse;
            impulse.X = normal.X * normalImpulse + tangent.X * tangentImpulse;
            impulse.Y = normal.Y * normalImpulse + tangent.Y * tangentImpulse;

            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);

                if (!body1IsParticle)
                {
                    float num2;
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
                    */
                    //body1.angularVelocity.X -= num3;
                    //body1.angularVelocity.Y -= num4;
                    // is this correct?
                    body1.angularVelocity -= num2 * body1.invInertia;
                }
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);

                if (!body2IsParticle)
                {

                    float num2;
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
            
            if (body1IsParticle)
            {
                JVector.Add(ref realRelPos1, ref body1.position, out p1);
            }
            else
            {
                JMatrix xForm = JMatrix.CreateRotationZ(body1.invOrientation);
                JVector.Transform(ref realRelPos1, ref xForm, out p1);
                JVector.Add(ref p1, ref body1.position, out p1);
            }

            if (body2IsParticle)
            {
                JVector.Add(ref realRelPos2, ref body2.position, out p2);
            }
            else
            {
                JMatrix xForm = JMatrix.CreateRotationZ(body2.invOrientation);
                JVector.Transform(ref realRelPos2, ref xForm, out p2);
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
                //body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                //body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);

                float num2;
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
                //body1.angularVelocity -= num2 * body1.invInertia;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);
                
                float num2;
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
                //body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                //body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);

                float num2;
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
                //body1.angularVelocity -= num2 * body1.invInertia;
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);

                float num2;
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

        /// <summary>
        /// PrepareForIteration has to be called before <see cref="Iterate"/>.
        /// </summary>
        /// <param name="timestep">The timestep of the simulation.</param>
        public void PrepareForIteration(float timestep)
        {
            float dvx, dvy;

            dvx = (body2.angularVelocity * relativePos2.Y) + body2.linearVelocity.X;
            dvy = (body2.angularVelocity * relativePos2.X) + body2.linearVelocity.Y;

            dvx = dvx - (0.0f * 0.0f) + (body1.angularVelocity * relativePos1.Y) - body1.linearVelocity.X;
            dvy = dvy - (body1.angularVelocity * relativePos1.X) + (0.0f * 0.0f) - body1.linearVelocity.Y;
            
            float kNormal = 0.0f;

            float rantra = 0.0f;
            if (!treatBody1AsStatic)
            {
                kNormal += body1.inverseMass;

                if (!body1IsParticle)
                {

                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    //rantra.X = (relativePos1.Y * normal.Z) - (relativePos1.Z * normal.Y);
                    //rantra.Y = (relativePos1.Z * normal.X) - (relativePos1.X * normal.Z);
                    rantra = (relativePos1.X * normal.Y) - (relativePos1.Y * normal.X);

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    //float num0 = ((rantra.X * body1.invInertiaWorld.M11) + (rantra.Y * body1.invInertiaWorld.M21)) + (rantra.Z * body1.invInertiaWorld.M31);
                    //float num1 = ((rantra.X * body1.invInertiaWorld.M12) + (rantra.Y * body1.invInertiaWorld.M22)) + (rantra.Z * body1.invInertiaWorld.M32);
                    //float num2 = ((rantra.X * body1.invInertiaWorld.M13) + (rantra.Y * body1.invInertiaWorld.M23)) + (rantra.Z * body1.invInertiaWorld.M33);

                    //rantra.X = num0; rantra.Y = num1; rantra.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    //num0 = (rantra.Y * relativePos1.Z) - (rantra.Z * relativePos1.Y);
                    //num1 = (rantra.Z * relativePos1.X) - (rantra.X * relativePos1.Z);
                    //num2 = (rantra.X * relativePos1.Y) - (rantra.Y * relativePos1.X);

                    //rantra.X = num0; rantra.Y = num1; rantra.Z = num2;
                }
            }

            float rbntrb = 0.0f;
            if (!treatBody2AsStatic)
            {
                kNormal += body2.inverseMass;

                if (!body2IsParticle)
                {

                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    //rbntrb.X = (relativePos2.Y * normal.Z) - (relativePos2.Z * normal.Y);
                    //rbntrb.Y = (relativePos2.Z * normal.X) - (relativePos2.X * normal.Z);
                    rbntrb = (relativePos2.X * normal.Y) - (relativePos2.Y * normal.X);

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    //float num0 = ((rbntrb.X * body2.invInertiaWorld.M11) + (rbntrb.Y * body2.invInertiaWorld.M21)) + (rbntrb.Z * body2.invInertiaWorld.M31);
                    //float num1 = ((rbntrb.X * body2.invInertiaWorld.M12) + (rbntrb.Y * body2.invInertiaWorld.M22)) + (rbntrb.Z * body2.invInertiaWorld.M32);
                    //float num2 = ((rbntrb.X * body2.invInertiaWorld.M13) + (rbntrb.Y * body2.invInertiaWorld.M23)) + (rbntrb.Z * body2.invInertiaWorld.M33);

                    //rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    //num0 = (rbntrb.Y * relativePos2.Z) - (rbntrb.Z * relativePos2.Y);
                    //num1 = (rbntrb.Z * relativePos2.X) - (rbntrb.X * relativePos2.Z);
                    //num2 = (rbntrb.X * relativePos2.Y) - (rbntrb.Y * relativePos2.X);

                    //rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;
                }
            }

            // is this right?
            if (!treatBody1AsStatic) kNormal += body1.inverseMass + body1.invInertia * rantra;//        rantra.X * normal.X + rantra.Y * normal.Y + rantra.Z * normal.Z;
            if (!treatBody2AsStatic) kNormal += body2.inverseMass + body2.invInertia * rbntrb;//        rbntrb.X * normal.X + rbntrb.Y * normal.Y + rbntrb.Z * normal.Z;

            massNormal = 1.0f / kNormal;

            float num = dvx * normal.X + dvy * normal.Y;

            tangent.X = dvx - normal.X * num;
            tangent.Y = dvy - normal.Y * num;

            num = tangent.X * tangent.X + tangent.Y * tangent.Y;

            if (num != 0.0f)
            {
                num = (float)Math.Sqrt(num);
                tangent.X /= num;
                tangent.Y /= num;
            }

            float kTangent = 0.0f;

            if (treatBody1AsStatic) rantra = 0.0f;
            else
            {
                kTangent += body1.inverseMass;

                if (!body1IsParticle)
                {
                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    //rantra.X = (relativePos1.Y * tangent.Z) - (relativePos1.Z * tangent.Y);
                    //rantra.Y = (relativePos1.Z * tangent.X) - (relativePos1.X * tangent.Z);
                    rantra = (relativePos1.X * tangent.Y) - (relativePos1.Y * tangent.X) * body1.invInertia;

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    //float num0 = ((rantra.X * body1.invInertiaWorld.M11) + (rantra.Y * body1.invInertiaWorld.M21)) + (rantra.Z * body1.invInertiaWorld.M31);
                    //float num1 = ((rantra.X * body1.invInertiaWorld.M12) + (rantra.Y * body1.invInertiaWorld.M22)) + (rantra.Z * body1.invInertiaWorld.M32);
                    //float num2 = ((rantra.X * body1.invInertiaWorld.M13) + (rantra.Y * body1.invInertiaWorld.M23)) + (rantra.Z * body1.invInertiaWorld.M33);

                    //rantra.X = num0; rantra.Y = num1; rantra.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    //num0 = (rantra.Y * relativePos1.Z) - (rantra.Z * relativePos1.Y);
                    //num1 = (rantra.Z * relativePos1.X) - (rantra.X * relativePos1.Z);
                    //num2 = (rantra.X * relativePos1.Y) - (rantra.Y * relativePos1.X);

                    //rantra.X = num0; rantra.Y = num1; rantra.Z = num2;
                }

            }

            if (treatBody2AsStatic) rbntrb = 0.0f;
            else
            {
                kTangent += body2.inverseMass;

                if (!body2IsParticle)
                {
                    // JVector.Cross(ref relativePos1, ref normal, out rantra);
                    //rbntrb.X = (relativePos2.Y * tangent.Z) - (relativePos2.Z * tangent.Y);
                    //rbntrb.Y = (relativePos2.Z * tangent.X) - (relativePos2.X * tangent.Z);
                    rbntrb = (relativePos2.X * tangent.Y) - (relativePos2.Y * tangent.X) * body2.invInertia;

                    // JVector.Transform(ref rantra, ref body1.invInertiaWorld, out rantra);
                    //float num0 = ((rbntrb.X * body2.invInertiaWorld.M11) + (rbntrb.Y * body2.invInertiaWorld.M21)) + (rbntrb.Z * body2.invInertiaWorld.M31);
                    //float num1 = ((rbntrb.X * body2.invInertiaWorld.M12) + (rbntrb.Y * body2.invInertiaWorld.M22)) + (rbntrb.Z * body2.invInertiaWorld.M32);
                    //float num2 = ((rbntrb.X * body2.invInertiaWorld.M13) + (rbntrb.Y * body2.invInertiaWorld.M23)) + (rbntrb.Z * body2.invInertiaWorld.M33);

                    //rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;

                    //JVector.Cross(ref rantra, ref relativePos1, out rantra);
                    //num0 = (rbntrb.Y * relativePos2.Z) - (rbntrb.Z * relativePos2.Y);
                    //num1 = (rbntrb.Z * relativePos2.X) - (rbntrb.X * relativePos2.Z);
                    //num2 = (rbntrb.X * relativePos2.Y) - (rbntrb.Y * relativePos2.X);

                    //rbntrb.X = num0; rbntrb.Y = num1; rbntrb.Z = num2;
                }
            }
            // is this right?
            if (!treatBody1AsStatic) kTangent += body1.inverseMass + body1.invInertia * rantra;// JVector.Dot(ref rantra, ref tangent);
            if (!treatBody2AsStatic) kTangent += body2.inverseMass + body2.invInertia * rbntrb;// JVector.Dot(ref rbntrb, ref tangent);
            massTangent = 1.0f / kTangent;

            restitutionBias = 0.0f;

            float relNormalVel = normal.X * dvx + normal.Y * dvy; //JVector.Dot(ref normal, ref dv);

            if (Penetration > settings.allowedPenetration)
            {
                //restitutionBias = settings.bias * (1.0f / timestep) * JMath.Max(0.0f, Penetration - settings.allowedPenetration);
                //restitutionBias = JMath.Clamp(restitutionBias, 0.0f, settings.maximumBias);
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

            // Simultaneos solving and restitution is simply not possible
            // so fake it a bit by just applying restitution impulse when there
            // is a new contact.
            if (relNormalVel < -1.0f && newContact)
            {
                restitutionBias = Math.Max(-restitution * relNormalVel, restitutionBias);
            }

            impulse.X = normal.X * accumulatedNormalImpulse + tangent.X * accumulatedTangentImpulse;
            impulse.Y = normal.Y * accumulatedNormalImpulse + tangent.Y * accumulatedTangentImpulse;
            
            if (!treatBody1AsStatic)
            {
                body1.linearVelocity.X -= (impulse.X * body1.inverseMass);
                body1.linearVelocity.Y -= (impulse.Y * body1.inverseMass);
                
                if (!body1IsParticle)
                {
                    float  num2;
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
            }

            if (!treatBody2AsStatic)
            {

                body2.linearVelocity.X += (impulse.X * body2.inverseMass);
                body2.linearVelocity.Y += (impulse.Y * body2.inverseMass);

                if (!body2IsParticle)
                {

                    float num2;
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
            }

            lastTimeStep = timestep;

            newContact = false;
        }

        public void TreatBodyAsStatic(RigidBodyIndex index)
        {
            if (index == RigidBodyIndex.RigidBody1) treatBody1AsStatic = true;
            else treatBody2AsStatic = true;
        }


        bool body1IsParticle; bool body2IsParticle;

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
            JMatrix xForm = JMatrix.CreateRotationZ(-body1.orientation);
            JVector.Transform(ref relativePos1, ref xForm, out realRelPos1);
            xForm = JMatrix.CreateRotationZ(-body2.orientation);
            JVector.Transform(ref relativePos2, ref xForm, out realRelPos2);

            this.initialPen = penetration;
            this.penetration = penetration;

            body1IsParticle = body1.isParticle;
            body2IsParticle = body2.isParticle;

            // Material Properties
            if (newContact)
            {
                treatBody1AsStatic = body1.isStatic;
                treatBody2AsStatic = body2.isStatic;

                accumulatedNormalImpulse = 0.0f;
                accumulatedTangentImpulse = 0.0f;

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
