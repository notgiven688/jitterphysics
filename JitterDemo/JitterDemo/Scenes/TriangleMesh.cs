using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using JitterDemo.Vehicle;
using Jitter.Collision;

namespace JitterDemo.Scenes
{
    public class TriangleMesh : Scene
    {

        Model model;

        public TriangleMesh(JitterDemo demo)
            : base(demo)
        {
        }

        /// <summary>
        /// Helper Method to get the vertex and index List from the model.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="indices"></param>
        /// <param name="model"></param>
        public void ExtractData(List<Vector3> vertices, List<TriangleVertexIndices> indices, Model model)
        {
            Matrix[] bones_ = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(bones_);
            foreach (ModelMesh mm in model.Meshes)
            {
                Matrix xform = bones_[mm.ParentBone.Index];
                foreach (ModelMeshPart mmp in mm.MeshParts)
                {
                    int offset = vertices.Count;
                    Vector3[] a = new Vector3[mmp.NumVertices];
                    mmp.VertexBuffer.GetData<Vector3>(mmp.VertexOffset * mmp.VertexBuffer.VertexDeclaration.VertexStride,
                        a, 0, mmp.NumVertices, mmp.VertexBuffer.VertexDeclaration.VertexStride);
                    for (int i = 0; i != a.Length; ++i)
                        Vector3.Transform(ref a[i], ref xform, out a[i]);
                    vertices.AddRange(a);

                    if (mmp.IndexBuffer.IndexElementSize != IndexElementSize.SixteenBits)
                        throw new Exception(
                            String.Format("Model uses 32-bit indices, which are not supported."));
                    short[] s = new short[mmp.PrimitiveCount * 3];
                    mmp.IndexBuffer.GetData<short>(mmp.StartIndex * 2, s, 0, mmp.PrimitiveCount * 3);
                    TriangleVertexIndices[] tvi = new TriangleVertexIndices[mmp.PrimitiveCount];
                    for (int i = 0; i != tvi.Length; ++i)
                    {
                        tvi[i].I0 = s[i * 3 + 0] + offset;
                        tvi[i].I1 = s[i * 3 + 1] + offset;
                        tvi[i].I2 = s[i * 3 + 2] + offset;
                    }
                    indices.AddRange(tvi);
                }
            }
        }


        public override void Draw()
        {
            Camera camera = this.Demo.Camera;

            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (BasicEffect effect in mesh.Effects)
                {
                    effect.World = boneTransforms[mesh.ParentBone.Index];
                    effect.View = camera.View;
                    effect.Projection = camera.Projection;

                    effect.EnableDefaultLighting();
                    effect.PreferPerPixelLighting = true;
                }
                mesh.Draw();
            }
        }

        Matrix[] boneTransforms;


        public override void Build()
        {
            model = Demo.Content.Load<Model>("staticmesh");
            boneTransforms = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(boneTransforms);

            List<TriangleVertexIndices> indices = new List<TriangleVertexIndices>();
            List<Vector3> vertices = new List<Vector3>();

            ExtractData(vertices, indices, model);

            List<JVector> jvertices = new List<JVector>(vertices.Count);
            foreach(Vector3 vertex in vertices) jvertices.Add(Conversion.ToJitterVector(vertex));

            Octree octree = new Octree(jvertices, indices);

            TriangleMeshShape tms = new TriangleMeshShape(octree);
            RigidBody body = new RigidBody(tms);
            body.IsStatic = true;
            //body.EnableDebugDraw = true;
            body.Tag = BodyTag.DontDrawMe;

            Demo.World.AddBody(body);

            AddCar(new JVector(-20, 20, 0));
        }


    }
}
