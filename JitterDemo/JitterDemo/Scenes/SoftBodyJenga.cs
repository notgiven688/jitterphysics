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

            List<TriangleVertexIndices> indices = new List<TriangleVertexIndices>();
            List<JVector> vertices = new List<JVector>();

            ConvexHullObject.ExtractData(vertices, indices, model);
            RemoveDuplicateVertices(indices, vertices);

            for (int i = 0; i < 3; i++)
            {
                SoftBody softBody = new SoftBody(indices, vertices);

                if (i % 2 == 0) softBody.Rotate(JMatrix.CreateRotationY(JMath.PiOver2), JVector.Zero);
                softBody.Translate(new JVector(10, 10 - i * 3, 0));
                softBody.Pressure =1000.0f;
                softBody.SetSpringValues(0.1f, 0.001f);
                //softBody.TriangleExpansion = 0.05f;

                Demo.World.AddBody(softBody);

                if (i == 0)
                {
                    foreach (SoftBody.MassPoint point in softBody.VertexBodies) point.IsStatic = true;
                }
            }

            indices.Clear(); vertices.Clear();
            model = this.Demo.Content.Load<Model>("cloth");

            ConvexHullObject.ExtractData(vertices, indices, model);
            RemoveDuplicateVertices(indices, vertices);

            SoftBody cloth = new SoftBody(indices, vertices);

            cloth.Translate(new JVector(0, 10, 10));

            cloth.VertexBodies[2].IsStatic = true;
            cloth.VertexBodies[124].IsStatic = true;
            cloth.VertexBodies[234].IsStatic = true;
            cloth.VertexBodies[356].IsStatic = true;

            cloth.SetSpringValues(0.3f, 0.01f);

            Demo.World.AddBody(cloth);
        }


        public override void Destroy()
        {
            RemoveGround();
            Demo.World.Clear();
        }


    }
}