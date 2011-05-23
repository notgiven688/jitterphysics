using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter;
using Microsoft.Xna.Framework;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;
using Microsoft.Xna.Framework.Graphics;
using Jitter.Collision;

namespace JitterDemo.Scenes
{
    class SoftBodyJenga : Scene
    {

        public SoftBodyJenga(JitterDemo demo)
            : base(demo)
        {
        }

        private void RemoveDuplicateVertices(List<TriangleVertexIndices> indices,
                List<JVector> vertices)
        {
            Dictionary<JVector, int> unique = new Dictionary<JVector, int>(vertices.Count);
            Stack<int> tbr = new Stack<int>(vertices.Count / 3);

            // get all unique vertices and their indices
            for (int i = 0; i < vertices.Count; i++)
            {
                if (!unique.ContainsKey(vertices[i]))
                    unique.Add(vertices[i], unique.Count);
                else tbr.Push(i);
            }

            // reconnect indices
            for (int i = 0; i < indices.Count; i++)
            {
                TriangleVertexIndices tvi = indices[i];

                tvi.I0 = unique[vertices[tvi.I0]];
                tvi.I1 = unique[vertices[tvi.I1]];
                tvi.I2 = unique[vertices[tvi.I2]];

                indices[i] = tvi;
            }

            // remove duplicate vertices
            while (tbr.Count > 0) vertices.RemoveAt(tbr.Pop());

            unique.Clear();
        }

        public override void Build()
        {
            AddGround();

            for (int i = 0; i < 15; i++)
            {
                bool even = (i % 2 == 0);

                for (int e = 0; e < 3; e++)
                {
                    JVector size = (even) ? new JVector(1, 1, 3) : new JVector(3, 1, 1);
                    RigidBody body = new RigidBody(new BoxShape(size));
                    body.Position = new JVector(3.0f + (even ? e : 1.0f), i + 0.5f, -5.0f + (even ? 1.0f : e));

                    Demo.World.AddBody(body);
                }

            }

            Model model = this.Demo.Content.Load<Model>("torus");
            List<Vector3> vertices = new List<Vector3>();
            List<TriangleVertexIndices> indices = new List<TriangleVertexIndices>();

            ConvexHullObject.ExtractData(vertices, indices, model);
            List<JVector> jvecs = new List<JVector>();

            foreach (Vector3 vec in vertices) jvecs.Add(Conversion.ToJitterVector(vec) + new JVector(3, 5, 3));

            RemoveDuplicateVertices(indices, jvecs);
            SoftBody softBody = new SoftBody(indices, jvecs);
            SoftBody softBody3 = new SoftBody(indices, jvecs);

            softBody3.TriangleExpansion = softBody.TriangleExpansion = 0.05f;

            softBody3.Translate(new JVector(-4,5,-4));

            softBody.Pressure = 500.0f;
            softBody3.Pressure = 500.0f;

            Demo.World.AddBody(softBody);
            Demo.World.AddBody(softBody3);

            model = this.Demo.Content.Load<Model>("cloth");
            vertices.Clear(); indices.Clear(); jvecs.Clear();

            ConvexHullObject.ExtractData(vertices, indices, model);
            foreach (Vector3 vec in vertices) jvecs.Add(Conversion.ToJitterVector(vec) + new JVector(10, 10, 3));
            RemoveDuplicateVertices(indices, jvecs);

            SoftBody softBody2 = new SoftBody(indices, jvecs);
            softBody2.Pressure = 0.0f;
            Demo.World.AddBody(softBody2);

            for (int i = 2; i < 3; i++)
            {
                softBody2.VertexBodies[i].IsStatic = true;
            }

            for (int i = 124; i < 125; i++)
            {
                softBody2.VertexBodies[i].IsStatic = true;
            }

            for (int i = 234; i < 235; i++)
            {
                softBody2.VertexBodies[i].IsStatic = true;
            }

            for (int i = 356; i < 357; i++)
            {
                softBody2.VertexBodies[i].IsStatic = true;
            }
        }


        public override void Destroy()
        {
            RemoveGround();
            Demo.World.Clear();
        }


    }
}