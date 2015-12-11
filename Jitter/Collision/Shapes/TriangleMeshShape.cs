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
    /// A <see cref="Shape"/> representing a triangleMesh.
    /// </summary>
    public class TriangleMeshShape : Multishape
    {
        private List<int> potentialTriangles = new List<int>();
        private Octree octree = null;

        private float sphericalExpansion = 0.05f;

        /// <summary>
        /// Expands the triangles by the specified amount.
        /// This stabilizes collision detection for flat shapes.
        /// </summary>
        public float SphericalExpansion 
        { 
            get { return sphericalExpansion; } 
            set { sphericalExpansion = value; } 
        }

        /// <summary>
        /// Creates a new istance if the TriangleMeshShape class.
        /// </summary>
        /// <param name="octree">The octree which holds the triangles
        /// of a mesh.</param>
        public TriangleMeshShape(Octree octree)
        {
            this.octree = octree;
            UpdateShape();
        }

        internal TriangleMeshShape() { }

 
        protected override Multishape CreateWorkingClone()
        {
            TriangleMeshShape clone = new TriangleMeshShape(this.octree);
            clone.sphericalExpansion = this.sphericalExpansion;
            return clone;
        }


        /// <summary>
        /// Passes a axis aligned bounding box to the shape where collision
        /// could occour.
        /// </summary>
        /// <param name="box">The bounding box where collision could occur.</param>
        /// <returns>The upper index with which <see cref="SetCurrentShape"/> can be 
        /// called.</returns>
        public override int Prepare(ref JBBox box)
        {
            potentialTriangles.Clear();

            #region Expand Spherical
            JBBox exp = box;

            exp.Min.X -= sphericalExpansion;
            exp.Min.Y -= sphericalExpansion;
            exp.Min.Z -= sphericalExpansion;
            exp.Max.X += sphericalExpansion;
            exp.Max.Y += sphericalExpansion;
            exp.Max.Z += sphericalExpansion;
            #endregion

            octree.GetTrianglesIntersectingtAABox(potentialTriangles, ref exp);

            return potentialTriangles.Count;
        }

        public override void MakeHull(ref List<JVector> triangleList, int generationThreshold)
        {
            JBBox large = JBBox.LargeBox;

            List<int> indices = new List<int>();
            octree.GetTrianglesIntersectingtAABox(indices, ref large);

            for (int i = 0; i < indices.Count; i++)
            {
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I0));
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I1));
                triangleList.Add(octree.GetVertex(octree.GetTriangleVertexIndex(i).I2));
            }

        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="rayOrigin"></param>
        /// <param name="rayDelta"></param>
        /// <returns></returns>
        public override int Prepare(ref JVector rayOrigin, ref JVector rayDelta)
        {
            potentialTriangles.Clear();

            #region Expand Spherical
            JVector expDelta;
            JVector.Normalize(ref rayDelta, out expDelta);
            expDelta = rayDelta + expDelta * sphericalExpansion;
            #endregion

            octree.GetTrianglesIntersectingRay(potentialTriangles, rayOrigin, expDelta);

            return potentialTriangles.Count;
        }

        JVector[] vecs = new JVector[3];

        /// <summary>
        /// SupportMapping. Finds the point in the shape furthest away from the given direction.
        /// Imagine a plane with a normal in the search direction. Now move the plane along the normal
        /// until the plane does not intersect the shape. The last intersection point is the result.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <param name="result">The result.</param>
        public override void SupportMapping(ref JVector direction, out JVector result)
        {
            JVector exp;
            JVector.Normalize(ref direction, out exp);
            exp *= sphericalExpansion;

            float min = JVector.Dot(ref vecs[0], ref direction);
            int minIndex = 0;
            float dot = JVector.Dot(ref vecs[1], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 1;
            }
            dot = JVector.Dot(ref vecs[2], ref direction);
            if (dot > min)
            {
                min = dot;
                minIndex = 2;
            }

            result = vecs[minIndex] + exp;
        }

        /// <summary>
        /// Gets the axis aligned bounding box of the orientated shape. This includes
        /// the whole shape.
        /// </summary>
        /// <param name="orientation">The orientation of the shape.</param>
        /// <param name="box">The axis aligned bounding box of the shape.</param>
        public override void GetBoundingBox(ref JMatrix orientation, out JBBox box)
        {
            box = octree.rootNodeBox;

            #region Expand Spherical
            box.Min.X -= sphericalExpansion;
            box.Min.Y -= sphericalExpansion;
            box.Min.Z -= sphericalExpansion;
            box.Max.X += sphericalExpansion;
            box.Max.Y += sphericalExpansion;
            box.Max.Z += sphericalExpansion;
            #endregion

            box.Transform(ref orientation);
        }

        private bool flipNormal = false;
        public bool FlipNormals { get { return flipNormal; } set { flipNormal = value; } }

        /// <summary>
        /// Sets the current shape. First <see cref="Prepare"/> has to be called.
        /// After SetCurrentShape the shape immitates another shape.
        /// </summary>
        /// <param name="index"></param>
        public override void SetCurrentShape(int index)
        {
            vecs[0] = octree.GetVertex(octree.tris[potentialTriangles[index]].I0);
            vecs[1] = octree.GetVertex(octree.tris[potentialTriangles[index]].I1);
            vecs[2] = octree.GetVertex(octree.tris[potentialTriangles[index]].I2);

            JVector sum = vecs[0];
            JVector.Add(ref sum, ref vecs[1], out sum);
            JVector.Add(ref sum, ref vecs[2], out sum);
            JVector.Multiply(ref sum, 1.0f / 3.0f, out sum);

      
            geomCen = sum;

            JVector.Subtract(ref vecs[1], ref vecs[0], out sum);
            JVector.Subtract(ref vecs[2], ref vecs[0], out normal);
            JVector.Cross(ref sum, ref normal, out normal);

            if (flipNormal) normal.Negate();
        }

        private JVector normal = JVector.Up;

        public void CollisionNormal(out JVector normal)
        {
            normal = this.normal;
        }
    }

}
