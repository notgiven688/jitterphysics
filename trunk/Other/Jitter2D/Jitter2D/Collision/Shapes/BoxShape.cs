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
    /// A <see cref="Shape"/> representing a box.
    /// </summary>
    public class BoxShape : Shape
    {
        private JVector size = JVector.Zero;

        /// <summary>
        /// The sidelength of the box.
        /// </summary>
        public JVector Size
        {
            get { return size; }
            set { size = value; UpdateShape(); }
        }

        /// <summary>
        /// Creates a new instance of the BoxShape class.
        /// </summary>
        /// <param name="size">The size of the box.</param>
        public BoxShape(JVector size)
        {
            this.size = size;
            this.UpdateShape();
        }

        /// <summary>
        /// Creates a new instance of the BoxShape class.
        /// </summary>
        /// <param name="length">The width of the box.</param>
        /// <param name="height">The height of the box.</param>
        public BoxShape(float width, float height)
        {
            this.size.X = width;
            this.size.Y = height;
            this.UpdateShape();
        }

        private JVector halfSize = JVector.Zero;

        /// <summary>
        /// This method uses the <see cref="ISupportMappable"/> implementation
        /// to calculate the local bounding box, the mass, geometric center and 
        /// the inertia of the shape. In custom shapes this method should be overidden
        /// to compute this values faster.
        /// </summary>
        public override void UpdateShape()
        {
            this.halfSize = size * 0.5f;
            base.UpdateShape();
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(ref float orientation, out JBBox box)
        {
            JMatrix xForm = JMatrix.CreateRotationZ(-orientation);
            JMatrix abs; JMath.Absolute(ref xForm, out abs);
            JVector temp;
            JVector.Transform(ref halfSize, ref abs, out temp);

            box.Max = temp;
            JVector.Negate(ref temp, out box.Min);
        }

        /// <summary>
        /// This method uses the <see cref="ISupportMappable"/> implementation
        /// to calculate the local bounding box, the mass, geometric center and 
        /// the inertia of the shape. In custom shapes this method should be overidden
        /// to compute this values faster.
        /// </summary>
        public override void CalculateMassInertia()
        {
            mass = 10;

            inertia = 1;

            this.geomCen = JVector.Zero;
        }

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            result.X = (float)Math.Sign(direction.X) * halfSize.X;
            result.Y = (float)Math.Sign(direction.Y) * halfSize.Y;
        }
    }
}
