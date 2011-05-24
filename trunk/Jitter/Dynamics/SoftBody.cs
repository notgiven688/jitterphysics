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
using System.Text;

using Jitter.Dynamics.Constraints;
using Jitter.Dynamics;
using Jitter.Collision.Shapes;
using Jitter.LinearMath;
using Jitter.Collision;
using System.Collections.ObjectModel;
#endregion

namespace Jitter.Dynamics
{
    public partial class SoftBody : IBroadphaseEntity
    {
        #region public class Spring : Constraint
        public class Spring : Constraint
        {
            public enum DistanceBehavior
            {
                LimitDistance,
                LimitMaximumDistance,
                LimitMinimumDistance,
            }

            private float biasFactor = 0.1f;
            private float softness = 0.01f;
            private float distance;

            private DistanceBehavior behavior = DistanceBehavior.LimitDistance;

            /// <summary>
            /// Initializes a new instance of the DistanceConstraint class.
            /// </summary>
            /// <param name="body1">The first body.</param>
            /// <param name="body2">The second body.</param>
            /// <param name="anchor1">The anchor point of the first body in world space. 
            /// The distance is given by the initial distance between both anchor points.</param>
            /// <param name="anchor2">The anchor point of the second body in world space.
            /// The distance is given by the initial distance between both anchor points.</param>
            public Spring(RigidBody body1, RigidBody body2)
                : base(body1, body2)
            {
                distance = (body1.position - body2.position).Length();
            }

            public float AppliedImpulse { get { return accumulatedImpulse; } }

            /// <summary>
            /// 
            /// </summary>
            public float Distance { get { return distance; } set { distance = value; } }

            /// <summary>
            /// 
            /// </summary>
            public DistanceBehavior Behavior { get { return behavior; } set { behavior = value; } }

            /// <summary>
            /// Defines how big the applied impulses can get.
            /// </summary>
            public float Softness { get { return softness; } set { softness = value; } }

            /// <summary>
            /// Defines how big the applied impulses can get which correct errors.
            /// </summary>
            public float BiasFactor { get { return biasFactor; } set { biasFactor = value; } }

            float effectiveMass = 0.0f;
            float accumulatedImpulse = 0.0f;
            float bias;
            float softnessOverDt;

            JVector[] jacobian = new JVector[2];

            bool skipConstraint = false;

            float myCounter = 0.0f;

            /// <summary>
            /// Called once before iteration starts.
            /// </summary>
            /// <param name="timestep">The 5simulation timestep</param>
            public override void PrepareForIteration(float timestep)
            {
                JVector dp;
                JVector.Subtract(ref body2.position, ref body1.position, out dp);

                float deltaLength = dp.Length() - distance;

                if (behavior == DistanceBehavior.LimitMaximumDistance && deltaLength <= 0.0f)
                {
                    skipConstraint = true;
                }
                else if (behavior == DistanceBehavior.LimitMinimumDistance && deltaLength >= 0.0f)
                {
                    skipConstraint = true;
                }
                else
                {
                    skipConstraint = false;

                    JVector n = dp;
                    if (n.LengthSquared() != 0.0f) n.Normalize();

                    jacobian[0] = -1.0f * n;
                    //jacobian[1] = -1.0f * (r1 % n);
                    jacobian[1] = 1.0f * n;
                    //jacobian[3] = (r2 % n);

                    effectiveMass = body1.inverseMass + body2.inverseMass;

                    softnessOverDt = softness / timestep;
                    effectiveMass += softnessOverDt;

                    effectiveMass = 1.0f / effectiveMass;

                    bias = deltaLength * biasFactor * (1.0f / timestep);

                    if (!body1.isStatic)
                    {
                        body1.linearVelocity += body1.inverseMass * accumulatedImpulse * jacobian[0];
                    }

                    if (!body2.isStatic)
                    {
                        body2.linearVelocity += body2.inverseMass * accumulatedImpulse * jacobian[1];
                    }
                }

            }

            /// <summary>
            /// Iteratively solve this constraint.
            /// </summary>
            public override void Iterate()
            {
                if (skipConstraint) return;

                float jv = JVector.Dot(ref body1.linearVelocity, ref jacobian[0]);
                jv += JVector.Dot(ref body2.linearVelocity, ref jacobian[1]);

                float softnessScalar = accumulatedImpulse * softnessOverDt;

                float lambda = -effectiveMass * (jv + bias + softnessScalar);

                if (behavior == DistanceBehavior.LimitMinimumDistance)
                {
                    float previousAccumulatedImpulse = accumulatedImpulse;
                    accumulatedImpulse = JMath.Max(accumulatedImpulse + lambda, 0);
                    lambda = accumulatedImpulse - previousAccumulatedImpulse;
                }
                else if (behavior == DistanceBehavior.LimitMaximumDistance)
                {
                    float previousAccumulatedImpulse = accumulatedImpulse;
                    accumulatedImpulse = JMath.Min(accumulatedImpulse + lambda, 0);
                    lambda = accumulatedImpulse - previousAccumulatedImpulse;
                }
                else
                {
                    accumulatedImpulse += lambda;
                }

                JVector temp;

                if (!body1.isStatic)
                {
                    JVector.Multiply(ref jacobian[0], lambda * body1.inverseMass, out temp);
                    JVector.Add(ref temp, ref body1.linearVelocity, out body1.linearVelocity);
                }

                if (!body2.isStatic)
                {
                    JVector.Multiply(ref jacobian[1], lambda * body2.inverseMass, out temp);
                    JVector.Add(ref temp, ref body2.linearVelocity, out body2.linearVelocity);
                }
            }


            /// <summary>
            /// This method is used to debug draw the constraints.
            /// </summary>
            /// <param name="lineList">A list of <see cref="JVector"/> to which lines (definied trough two points)
            /// are added.</param>
            /// <param name="pointList">A list of <see cref="JVector"/> defining points.</param>
            public override void AddToDebugDrawList(List<JVector> lineList, List<JVector> pointList)
            {
                lineList.Add(body1.position);
                lineList.Add(body2.position);
            }
        }
        #endregion

        #region public class MassPoint : RigidBody
        public class MassPoint : RigidBody
        {
            private const float sphereSize = 0.1f;

            private static Shape sphereShape = new SphereShape(sphereSize);

            public MassPoint(Material material) : base(sphereShape, material) { }

            public override void Update()
            {
                this.inertia = JMatrix.Zero;
                this.invInertia = this.invInertiaWorld = JMatrix.Zero;
                this.invOrientation = this.orientation = JMatrix.Identity;

                this.boundingBox.Min = new JVector(-sphereSize) + position;
                this.boundingBox.Max = new JVector(sphereSize) + position;

                angularVelocity.MakeZero();
            }

            // Setting angular velocity or inertia does not have any effect.

            public new void SetMassProperties(JMatrix inertia, float mass, bool setAsInverseValues)
            {
                if (setAsInverseValues) inverseMass = mass;
                else inverseMass = 1.0f / mass;
            }

            public new void SetMassProperties() { }

            public new float Mass
            {
                get { return 1.0f / inverseMass; }
                set { inverseMass = 1.0f / value; }
            }

            public new JVector AngularVelocity
            {
                get { return angularVelocity; }
                set { }
            }


        }
        #endregion

        #region public class Triangle : ISupportMappable
        public class Triangle : ISupportMappable
        {
            private SoftBody owner;

            public SoftBody Owner { get { return owner; } }

            internal JBBox boundingBox;
            internal int dynamicTreeID;
            internal TriangleVertexIndices indices;

            public JBBox BoundingBox { get { return boundingBox; } }
            public int DynamicTreeID { get { return dynamicTreeID; } }

            public TriangleVertexIndices Indices { get { return indices; } }

            public MassPoint VertexBody1 { get { return owner.points[indices.I0]; } }
            public MassPoint VertexBody2 { get { return owner.points[indices.I1]; } }
            public MassPoint VertexBody3 { get { return owner.points[indices.I2]; } }

            public Triangle(SoftBody owner)
            {
                this.owner = owner;
            }

            public void GetNormal(out JVector normal)
            {
                JVector sum;
                JVector.Subtract(ref owner.points[indices.I1].position, ref owner.points[indices.I0].position, out sum);
                JVector.Subtract(ref owner.points[indices.I2].position, ref owner.points[indices.I0].position, out normal);
                JVector.Cross(ref sum, ref normal, out normal);
            }

            public void UpdateBoundingBox()
            {
                boundingBox = JBBox.SmallBox;
                boundingBox.AddPoint(ref owner.points[indices.I0].position);
                boundingBox.AddPoint(ref owner.points[indices.I1].position);
                boundingBox.AddPoint(ref owner.points[indices.I2].position);

                boundingBox.Min -= new JVector(owner.triangleExpansion);
                boundingBox.Max += new JVector(owner.triangleExpansion);
            }

            public float CalculateArea()
            {
                return ((owner.points[indices.I1].position - owner.points[indices.I0].position) %
                    (owner.points[indices.I2].position - owner.points[indices.I0].position)).Length();
            }

            public void SupportMapping(ref JVector direction, out JVector result)
            {
                JVector exp;
                JVector.Normalize(ref direction, out exp);
                exp *= owner.triangleExpansion;

                float min = JVector.Dot(ref owner.points[indices.I0].position, ref direction);
                float dot = JVector.Dot(ref owner.points[indices.I1].position, ref direction);

                JVector minVertex = owner.points[indices.I0].position;

                if (dot > min)
                {
                    min = dot;
                    minVertex = owner.points[indices.I1].position;
                }
                dot = JVector.Dot(ref owner.points[indices.I2].position, ref direction);
                if (dot > min)
                {
                    min = dot;
                    minVertex = owner.points[indices.I2].position;
                }

                result = minVertex + exp;
            }

            public void SupportCenter(out JVector center)
            {
                center = owner.points[indices.I0].position;
                JVector.Add(ref center, ref owner.points[indices.I1].position, out center);
                JVector.Add(ref center, ref owner.points[indices.I2].position, out center);
                JVector.Multiply(ref center, 1.0f / 3.0f, out center);
            }
        }
        #endregion

        internal Spring[] springs;
        internal MassPoint[] points;

        public Spring[] EdgeSprings { get { return springs; } }
        public MassPoint[] VertexBodies { get { return points; } }

        protected float triangleExpansion = 0.1f;

        public float TriangleExpansion { get { return triangleExpansion; } 
            set { triangleExpansion = value; } }

        private float volume = 1.0f;
        private float mass = 1.0f;

        internal DynamicTree<Triangle> dynamicTree = new DynamicTree<Triangle>();
        public DynamicTree<Triangle> DynamicTree { get { return dynamicTree; } }

        private Material material = new Material();
        public Material Material { get { return material; } }

        JBBox box = new JBBox();

        private List<Triangle> triangles = new List<Triangle>();
        public ReadOnlyCollection<Triangle> Triangles { private set; get; }

        bool active = true;

        public SoftBody(List<TriangleVertexIndices> indices, List<JVector> vertices)
        {
            AddPointsAndSprings(indices, vertices);
            Triangles = triangles.AsReadOnly();
        }

        private float pressure = 0.0f;
        public float Pressure { get { return pressure; } set { pressure = value; } }

        private struct Edge
        {
            public int Index1;
            public int Index2;

            public Edge(int index1, int index2)
            {
                Index1 = index1;
                Index2 = index2;
            }

            public override int GetHashCode()
            {
                return Index1.GetHashCode() + Index2.GetHashCode();
            }

            public override bool Equals(object obj)
            {
                Edge e = (Edge)obj;
                return (e.Index1 == Index1 && e.Index2 == Index2 || e.Index1 == Index2 && e.Index2 == Index1);
            }
        }

        #region AddPressureForces
        public void AddPressureForces(float timeStep)
        {
            if (pressure == 0.0f || volume == 0.0f) return;

            float invVolume = 1.0f / volume;

            foreach (Triangle t in triangles)
            {
                JVector v1 = points[t.indices.I0].position;
                JVector v2 = points[t.indices.I1].position;
                JVector v3 = points[t.indices.I2].position;

                JVector cross = (v3 - v1) % (v2 - v1);
                JVector center = (v1 + v2 + v3) * (1.0f / 3.0f);

                points[t.indices.I0].AddForce(invVolume * cross * pressure);
                points[t.indices.I1].AddForce(invVolume * cross * pressure);
                points[t.indices.I2].AddForce(invVolume * cross * pressure);
            }
        }
        #endregion

        public void Translate(JVector position)
        {
            foreach (MassPoint point in points) point.Position += position;
        }

        public void AddForce(JVector force)
        {
            // TODO
            throw new NotImplementedException();
        }

        public void Rotate(JMatrix orientation, JVector center)
        {
            // TODO
            throw new NotImplementedException();
        }

        public JVector CalculateCenter()
        {
            // TODO
            throw new NotImplementedException();
        }

        private HashSet<Edge> GetEdges(List<TriangleVertexIndices> indices)
        {
            HashSet<Edge> edges = new HashSet<Edge>();

            for (int i = 0; i < indices.Count; i++)
            {
                Edge edge;

                edge = new Edge(indices[i].I0, indices[i].I1);
                if (!edges.Contains(edge)) edges.Add(edge);

                edge = new Edge(indices[i].I1, indices[i].I2);
                if (!edges.Contains(edge)) edges.Add(edge);

                edge = new Edge(indices[i].I2, indices[i].I0);
                if (!edges.Contains(edge)) edges.Add(edge);
            }

            return edges;
        }

        private void AddPointsAndSprings(List<TriangleVertexIndices> indices, List<JVector> vertices)
        {
            points = new MassPoint[vertices.Count];

            for (int i = 0; i < vertices.Count; i++)
            {
                MassPoint point = new MassPoint(material);
                point.Position = vertices[i];
                point.Mass = 0.1f;

                points[i] = point;
            }

            for (int i = 0; i < indices.Count; i++)
            {
                TriangleVertexIndices index = indices[i];
                
                Triangle t = new Triangle(this);

                t.indices = index;
                triangles.Add(t);

                t.boundingBox = JBBox.SmallBox;
                t.boundingBox.AddPoint(points[t.indices.I0].position);
                t.boundingBox.AddPoint(points[t.indices.I1].position);
                t.boundingBox.AddPoint(points[t.indices.I2].position);

                t.dynamicTreeID = dynamicTree.AddProxy(ref t.boundingBox, t);
            }

            HashSet<Edge> edges = GetEdges(indices);

            springs = new Spring[edges.Count];
            int count = 0;

            foreach (Edge edge in edges)
            {
                Spring spring = new Spring(points[edge.Index1], points[edge.Index2]);
                spring.Softness = 0.01f; spring.BiasFactor = 0.1f;

                springs[count] = spring;
                count++;
            }

        }

        public void SetSpringValues(float bias, float softness)
        {
            for (int i = 0; i < springs.Length; i++)
            { springs[i].Softness = softness; springs[i].BiasFactor = bias; }
        }

        public void Update(float timestep)
        {
            active = false;

            foreach (MassPoint point in points)
            {
                if (point.isActive && !point.isStatic) { active = true; break; }
            }

            if(!active) return;

            box = JBBox.SmallBox;
            volume = 0.0f;
            mass = 0.0f;

            foreach (MassPoint point in points)
            {
                mass += point.Mass;
                box.AddPoint(point.position);
            }

            box.Min -= new JVector(TriangleExpansion);
            box.Max += new JVector(TriangleExpansion);

            foreach (Triangle t in triangles)
            {
                // Update bounding box and move proxy in dynamic tree.
                JVector prevCenter = t.boundingBox.Center;
                t.UpdateBoundingBox();

                JVector linVel = t.VertexBody1.linearVelocity + 
                    t.VertexBody2.linearVelocity + 
                    t.VertexBody3.linearVelocity;

                linVel *= 1.0f / 3.0f;

                dynamicTree.MoveProxy(t.dynamicTreeID, ref t.boundingBox, linVel * timestep);

                JVector v1 = points[t.indices.I0].position;
                JVector v2 = points[t.indices.I1].position;
                JVector v3 = points[t.indices.I2].position;

                volume -= ((v2.Y - v1.Y) * (v3.Z - v1.Z) -
                    (v2.Z - v1.Z) * (v3.Y - v1.Y)) * (v1.X + v2.X + v3.X);
            }

            volume /= 6.0f;
        }

        public float Mass
        {
            get
            {
                return mass;
            }
            set
            {
                for (int i = 0; i < points.Length; i++)
                {
                    points[i].Mass = value / points.Length;
                }
            }
        }

        public float Volume { get { return volume; } }

        public JBBox BoundingBox
        {
            get { return box; }
        }

        public int BroadphaseTag { get; set; }

        public object Tag { get; set; }

        public bool IsStaticOrInactive
        {
            get { return !active; }
        }
    }


}


    
