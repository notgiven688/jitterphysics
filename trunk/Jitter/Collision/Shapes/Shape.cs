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

using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
#endregion

namespace Jitter.Collision.Shapes
{

    /// <summary>
    /// Gets called when a shape changes one of the parameters.
    /// For example the size of a box is changed.
    /// </summary>
    public delegate void ShapeUpdatedHandler();

    /// <summary>
    /// Represents the collision part of the RigidBody. A shape is mainly definied through it's supportmap.
    /// Shapes represent convex objects. Inherited classes have to overwrite the supportmap function.
    /// To implement you own shape: derive a class from <see cref="Shape"/>, implement the support map function
    /// and call 'UpdateShape' within the constructor. GeometricCenter, Mass, BoundingBox and Inertia is calculated numerically
    /// based on your SupportMap implementation.
    /// </summary>
    public abstract class Shape : ISupportMappable
    {

        // internal values so we can access them fast  without calling properties.
        internal JMatrix inertia = JMatrix.Identity;
        internal float mass = 1.0f;

        internal JBBox boundingBox = JBBox.LargeBox;
        internal JVector geomCen = JVector.Zero;

        /// <summary>
        /// Gets called when the shape changes one of the parameters.
        /// </summary>
        public event ShapeUpdatedHandler ShapeUpdated;

        /// <summary>
        /// Creates a new instance of a shape.
        /// </summary>
        public Shape()
        {
        }

        /// <summary>
        /// Returns the inertia of the untransformed shape.
        /// </summary>
        public JMatrix Inertia { get { return inertia; } protected set { inertia = value; } }


        /// <summary>
        /// Gets the mass of the shape. This is the volume. (density = 1)
        /// </summary>
        public float Mass { get { return mass; } protected set { mass = value; } }

        /// <summary>
        /// Informs all listener that the shape changed.
        /// </summary>
        protected void RaiseShapeUpdated()
        {
            if (ShapeUpdated != null) ShapeUpdated();
        }

        /// <summary>
        /// The untransformed axis aligned bounding box of the shape.
        /// </summary>
        public JBBox BoundingBox { get { return boundingBox; } }


        /// <summary>
        /// Uses the supportMapping to calculate the bounding box. Should be overidden
        /// to make this faster.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The resulting axis aligned bounding box.</param>
        public virtual void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            // I don't think that this can be done faster.
            // 6 is the minimum number of SupportMap calls.

            JVector vec = JVector.Zero;

            vec.Set(orientation.M11, orientation.M21, orientation.M31);
            SupportMapping(ref vec, out vec);
            box.Max.X = orientation.M11 * vec.X + orientation.M21 * vec.Y + orientation.M31 * vec.Z;

            vec.Set(orientation.M12, orientation.M22, orientation.M32);
            SupportMapping(ref vec, out vec);
            box.Max.Y = orientation.M12 * vec.X + orientation.M22 * vec.Y + orientation.M32 * vec.Z;

            vec.Set(orientation.M13, orientation.M23, orientation.M33);
            SupportMapping(ref vec, out vec);
            box.Max.Z = orientation.M13 * vec.X + orientation.M23 * vec.Y + orientation.M33 * vec.Z;

            vec.Set(-orientation.M11, -orientation.M21, -orientation.M31);
            SupportMapping(ref vec, out vec);
            box.Min.X = orientation.M11 * vec.X + orientation.M21 * vec.Y + orientation.M31 * vec.Z;

            vec.Set(-orientation.M12, -orientation.M22, -orientation.M32);
            SupportMapping(ref vec, out vec);
            box.Min.Y = orientation.M12 * vec.X + orientation.M22 * vec.Y + orientation.M32 * vec.Z;

            vec.Set(-orientation.M13, -orientation.M23, -orientation.M33);
            SupportMapping(ref vec, out vec);
            box.Min.Z = orientation.M13 * vec.X + orientation.M23 * vec.Y + orientation.M33 * vec.Z;
        }

        /// <summary>
        /// This method uses the <see cref="ISupportMappable"/> implementation
        /// to calculate the local bounding box, the mass, geometric center and 
        /// the inertia of the shape. In custom shapes this method should be overidden
        /// to compute this values faster.
        /// </summary>
        public virtual void UpdateShape()
        {
            GetBoundingBox(ref JMatrix.InternalIdentity, out boundingBox);

            CalculateMassInertia();
            RaiseShapeUpdated();
        }
        
        /// <summary>
        /// Calculated the inertia of the shape relative to the center of mass.
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="centerOfMass"></param>
        /// <param name="inertia"></param>
        /// <returns></returns>
        #region  public static float CalculateMassInertia(Shape shape, out JVector centerOfMass, out JMatrix inertia)
        public static float CalculateMassInertia(Shape shape, out JVector centerOfMass,
            out JMatrix inertia)
        {
            // TODO: do something smarter here ...

            if (shape is Multishape) throw new ArgumentException("Can't calculate inertia of multishapes.","shape");

            float mass = 0.0f;

            List<JVector> massPoints = new List<JVector>();
            JVector testVector;

            int subdivision = 25;

            JVector diff; JVector.Subtract(ref shape.boundingBox.Max, ref shape.boundingBox.Min, out diff);

            if (diff.IsNearlyZero())
                throw new InvalidOperationException("BoundingBox volume of the shape is zero. Try to call CalculateLocalBoundingBox first.");

            for (int i = 0; i < subdivision; i++)
            {
                for (int e = 0; e < subdivision; e++)
                {
                    for (int k = 0; k < subdivision; k++)
                    {
                        testVector.X = shape.boundingBox.Min.X + (diff.X / (float)subdivision) * ((float)i);
                        testVector.Y = shape.boundingBox.Min.Y + (diff.Y / (float)subdivision) * ((float)e);
                        testVector.Z = shape.boundingBox.Min.Z + (diff.Z / (float)subdivision) * ((float)k);

                        if (GJKCollide.Pointcast(shape, ref JMatrix.InternalIdentity, ref JVector.InternalZero, ref testVector))
                        {
                            massPoints.Add(testVector);
                        }
                    }
                }
            }

            float m11 = 0.0f; float m22 = 0.0f; float m33 = 0.0f;
            float m21 = 0.0f; float m31 = 0.0f; float m32 = 0.0f;

            float x = 0.0f; float y = 0.0f; float z = 0.0f;

            for (int i = 0; i < massPoints.Count; i++)
            {
                x += massPoints[i].X;
                y += massPoints[i].Y;
                z += massPoints[i].Z;

                m11 += (massPoints[i].Y * massPoints[i].Y + massPoints[i].Z * massPoints[i].Z);
                m22 += (massPoints[i].X * massPoints[i].X + massPoints[i].Z * massPoints[i].Z);
                m33 += (massPoints[i].X * massPoints[i].X + massPoints[i].Y * massPoints[i].Y);
                m21 += -massPoints[i].Y * massPoints[i].X;
                m31 += -massPoints[i].Z * massPoints[i].X;
                m32 += -massPoints[i].Z * massPoints[i].Y;
            }

            m11 /= (float)massPoints.Count;
            m22 /= (float)massPoints.Count;
            m33 /= (float)massPoints.Count;
            m21 /= (float)massPoints.Count;
            m31 /= (float)massPoints.Count;
            m32 /= (float)massPoints.Count;

            x /= (float)massPoints.Count;
            y /= (float)massPoints.Count;
            z /= (float)massPoints.Count;

            inertia = JMatrix.Identity;
            inertia.M11 = m11;
            inertia.M22 = m22;
            inertia.M33 = m33;
            inertia.M21 = inertia.M12 = m21;
            inertia.M31 = inertia.M13 = m31;
            inertia.M32 = inertia.M23 = m32;

            mass = (diff.X * diff.Y * diff.Z) * ((float)massPoints.Count / ((float)(subdivision * subdivision * subdivision)));
            JMatrix.Multiply(ref inertia, mass, out inertia);

            centerOfMass.X = x;
            centerOfMass.Y = y;
            centerOfMass.Z = z;

            // now translate the inertia by the center of mass

            JMatrix t = new JMatrix(
                mass * (y * y + z * z), -mass * x * y, -mass * x * z,
                -mass * y * x, mass * (z * z + x * x), -mass * y * z,
                -mass * z * x, -mass * z * y, mass * (x * x + y * y));

            JMatrix.Add(ref inertia, ref t, out inertia);

            return mass;
        }
        #endregion
        
        /// <summary>
        /// Numerically calculates the inertia, mass and geometric center of the shape.
        /// This gets a good value for "normal" shapes. The algorithm isn't very accurate
        /// for very flat shapes. 
        /// </summary>
        public virtual void CalculateMassInertia()
        {
            this.mass = Shape.CalculateMassInertia(this, out geomCen, out inertia);
        }

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public abstract void SupportMapping(ref JVector direction, out JVector result);

        /// <summary>
        /// The center of the SupportMap.
        /// </summary>
        /// <param name="geomCenter">The center of the SupportMap.</param>
        public void SupportCenter(out JVector geomCenter)
        {
            geomCenter = this.geomCen;
        }


        public virtual void AddToDebugDrawList(List<JVector> lineList, List<JVector> pointList)
        {
            //
        }
    }
}
