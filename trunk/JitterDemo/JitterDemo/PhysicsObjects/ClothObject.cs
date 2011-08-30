using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Jitter.Dynamics;
using Jitter.LinearMath;

namespace JitterDemo.PhysicsObjects
{
    class ClothObject : DrawableGameComponent
    {
        private Texture2D texture;
        private BasicEffect effect;

        private VertexPositionNormalTexture[] vertices;
        private int[] indices;

        SoftBody cloth = null;

        public ClothObject(Game game, SoftBody cloth)
            : base(game)
        {
            this.cloth = cloth;
        }

        public override void Initialize()
        {
            BuildVertices();
            base.Initialize();
        }

        private void BuildVertices()
        {
            vertices = new VertexPositionNormalTexture[cloth.VertexBodies.Count];
            indices = new int[cloth.Triangles.Count * 3];

            for (int i = 0; i < cloth.Triangles.Count;i++ )
            {
                SoftBody.Triangle t = cloth.Triangles[i];
                indices[3 * i + 0] = t.Indices.I0;
                indices[3 * i + 1] = t.Indices.I1;
                indices[3 * i + 2] = t.Indices.I2;
            }

            UpdatePositionAndNormal();

            int sqrt = (int)Math.Sqrt(vertices.Length);

            for (int i = 0; i < sqrt; i++)
            {
                for (int e = 0; e < sqrt; e++)
                {

                    vertices[i * sqrt + e].TextureCoordinate = new Vector2(1.0f / (float)(sqrt - 1) * (float)i, 1.0f / (float)(sqrt-1) * (float)e);
                }
            }
            


            //vertices[0].Position = Vector3.Forward + Vector3.Left;
            //vertices[0].TextureCoordinate = new Vector2(0.0f, 1.0f);
            //vertices[1].Position = Vector3.Backward + Vector3.Left;
            //vertices[1].TextureCoordinate = new Vector2(0.0f, 0.0f);
            //vertices[2].Position = Vector3.Forward + Vector3.Right;
            //vertices[2].TextureCoordinate = new Vector2(1.0f, 1.0f);
            //vertices[3].Position = Vector3.Backward + Vector3.Right;
            //vertices[3].TextureCoordinate = new Vector2(1.0f, 0.0f);

            //for (int i = 0; i < vertices.Length; i++)
            //{
            //    vertices[i].Normal = Vector3.Up;
            //    vertices[i].Position *= size;
            //    vertices[i].TextureCoordinate *= size;
            //}

            //indices[5] = 0; indices[4] = 1; indices[3] = 2;
            //indices[2] = 2; indices[1] = 1; indices[0] = 3;
        }

        private void UpdatePositionAndNormal()
        {
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i].Position = Conversion.ToXNAVector(cloth.VertexBodies[i].Position);
            }

            //JVector normal;

            //for (int i = 0; i < cloth.Triangles.Count; i++)
            //{
            //    SoftBody.Triangle t = cloth.Triangles[i];
            //    t.GetNormal(out normal); normal.Normalize();

            //    vertices[t.Indices.I0].Normal =
            //        vertices[t.Indices.I1].Normal =
            //            vertices[t.Indices.I2].Normal = Conversion.ToXNAVector(normal);
            //}



            //vertices[i].

            Vector3[] neighbour = new Vector3[4];

            int sqrt = (int)Math.Sqrt(vertices.Length);

            for (int i = 0; i < sqrt; i++)
            {
                for (int e = 0; e < sqrt; e++)
                {
                    Vector3 pos = vertices[i * sqrt + e].Position;

                    if (i > 0) neighbour[0] = vertices[(i - 1) * sqrt + (e + 0)].Position;
                    else neighbour[0] = pos;

                    if (e > 0) neighbour[1] = vertices[(i + 0) * sqrt + (e - 1)].Position;
                    else neighbour[1] = pos;

                    if (i < sqrt - 1) neighbour[2] = vertices[(i + 1) * sqrt + (e + 0)].Position;
                    else neighbour[2] = pos;

                    if (e < sqrt - 1) neighbour[3] = vertices[(i + 0) * sqrt + (e + 1)].Position;
                    else neighbour[3] = pos;

                    Vector3 normal = Vector3.Zero;

                    normal += Vector3.Cross(neighbour[1] - pos, neighbour[0] - pos);
                    normal += Vector3.Cross(neighbour[2] - pos, neighbour[1] - pos);
                    normal += Vector3.Cross(neighbour[3] - pos, neighbour[2] - pos);
                    normal += Vector3.Cross(neighbour[0] - pos, neighbour[3] - pos);
                    normal.Normalize();

                    vertices[i * sqrt + e].Normal = normal;
                }
            }

        }

        public override void Update(GameTime gameTime)
        {
            UpdatePositionAndNormal();
        }

        protected override void LoadContent()
        {
            texture = this.Game.Content.Load<Texture2D>("cloth");
            effect = new BasicEffect(this.GraphicsDevice);
            effect.EnableDefaultLighting();
            effect.SpecularColor = new Vector3(0.1f, 0.1f, 0.1f);

            effect.World = Matrix.Identity;
            effect.TextureEnabled = true;

            effect.Texture = texture;

            base.LoadContent();
        }

        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);
        }

        public override void Draw(GameTime gameTime)
        {
            JitterDemo demo = this.Game as JitterDemo;

            GraphicsDevice.SamplerStates[0] = SamplerState.AnisotropicWrap;
            GraphicsDevice.DepthStencilState = DepthStencilState.Default;

            effect.View = demo.Camera.View;
            effect.Projection = demo.Camera.Projection;

            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();

                GraphicsDevice.DrawUserIndexedPrimitives
                    <VertexPositionNormalTexture>(PrimitiveType.TriangleList,
                    vertices, 0, cloth.VertexBodies.Count,indices, 0, cloth.Triangles.Count);
            }

            base.Draw(gameTime);
        }

    }
}
