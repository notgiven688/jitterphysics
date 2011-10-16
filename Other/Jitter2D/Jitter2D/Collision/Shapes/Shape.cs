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
#endregion

namespace Jitter2D.Collision.Shapes
{

    /// <summary>
    /// Gets called when a shape changes one of the parameters.
    /// For example the size of a box is changed.
    /// </summary>
    public delegate void ShapeUpdatedHandler();

    /// <summary>
    /// Represents the collision part of the RigidBody. A shape is mainly defined through it's supportmap.
    /// Shapes represent convex objects. Inherited classes have to overwrite the supportmap function.
    /// To implement you own shape: derive a class from <see cref="Shape"/>, implement the support map function
    /// and call 'UpdateShape' within the constructor. GeometricCenter, Mass, BoundingBox and Inertia is calculated numerically
    /// based on your SupportMap implementation.
    /// </summary>
    public abstract class Shape : ISupportMappable
    {

        // internal values so we can access them fast  without calling properties.
        internal float inertia = 0.0f;
        internal float density = 1.0f;
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
        public float Inertia { get { return inertia; } protected set { inertia = value; } }

        /// <summary>
        /// Gets or sets the density.
        /// </summary>
        /// <value>The density.</value>
        public float Density
        {
            get { return density; }
            set
            {
                density = value;
            }
        }


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
        /// Hull making.
        /// </summary>
        /// <remarks>Based/Completely from http://www.xbdev.net/physics/MinkowskiDifference/index.php
        /// I don't (100%) see why this should always work.
        /// </remarks>
        /// <param name="triangleList"></param>
        /// <param name="generationThreshold"></param>
        public virtual void MakeHull(ref List<JVector> triangleList, int generationThreshold)
        {
           
        }

        /// <summary>
        /// Uses the supportMapping to calculate the bounding box. Should be overidden
        /// to make this faster.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The resulting axis aligned bounding box.</param>
        public virtual void GetBoundingBox(ref float rotation, out JBBox box)
        {
            JVector vec = JVector.Zero;

            JMatrix orientation = JMatrix.CreateRotationZ(rotation);

            vec.Set(orientation.M11, orientation.M21);
            SupportMapping(ref vec, out vec);
            box.Max.X = orientation.M11 * vec.X + orientation.M21 * vec.Y + orientation.M31;

            vec.Set(orientation.M12, orientation.M22);
            SupportMapping(ref vec, out vec);
            box.Max.Y = orientation.M12 * vec.X + orientation.M22 * vec.Y + orientation.M32;

            vec.Set(-orientation.M11, -orientation.M21);
            SupportMapping(ref vec, out vec);
            box.Min.X = orientation.M11 * vec.X + orientation.M21 * vec.Y + orientation.M31;

            vec.Set(-orientation.M12, -orientation.M22);
            SupportMapping(ref vec, out vec);
            box.Min.Y = orientation.M12 * vec.X + orientation.M22 * vec.Y + orientation.M32;
        }

        /// <summary>
        /// This method uses the <see cref="ISupportMappable"/> implementation
        /// to calculate the local bounding box, the mass, geometric center and 
        /// the inertia of the shape. In custom shapes this method should be overidden
        /// to compute this values faster.
        /// </summary>
        public virtual void UpdateShape()
        {
            //GetBoundingBox(ref 0.0f, out boundingBox);

            CalculateMassInertia();
            RaiseShapeUpdated();
        }

        /// <summary>
        /// Calculates the inertia of the shape relative to the center of mass.
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="centerOfMass"></param>
        /// <param name="inertia">Returns the inertia relative to the center of mass, not to the origin</param>
        /// <returns></returns>
        #region  public static float CalculateMassInertia(Shape shape, out JVector centerOfMass, out JMatrix inertia)
        public static float CalculateMassInertia(Shape shape, out JVector centerOfMass,
            out JMatrix inertia)
        {
            throw new NotImplementedException();
        }
        #endregion

        /// <summary>
        /// Numerically calculates the inertia, mass and geometric center of the shape.
        /// This gets a good value for "normal" shapes. The algorithm isn't very accurate
        /// for very flat shapes. 
        /// </summary>
        public virtual void CalculateMassInertia()
        {
            //this.mass = Shape.CalculateMassInertia(this, out geomCen, out inertia);
            mass = 5;
            inertia = 10;
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

    }
}
