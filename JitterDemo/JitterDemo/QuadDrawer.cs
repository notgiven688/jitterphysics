using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JitterDemo
{

    public class QuadDrawer : DrawableGameComponent
    {
        private Texture2D texture;
        private BasicEffect effect;

        private float size = 100.0f;

        private VertexPositionNormalTexture[] vertices;
        private int[] indices;

        public QuadDrawer(Game game, float size)
            : base(game)
        {
            this.size = size;
        }

        public override void Initialize()
        {
            BuildVertices();
            base.Initialize();
        }

        private void BuildVertices()
        {
            vertices = new VertexPositionNormalTexture[4];
            indices = new int[6];

            vertices[0].Position = Vector3.Forward + Vector3.Left;
            vertices[0].TextureCoordinate = new Vector2(0.0f, 1.0f);
            vertices[1].Position = Vector3.Backward + Vector3.Left;
            vertices[1].TextureCoordinate = new Vector2(0.0f, 0.0f);
            vertices[2].Position = Vector3.Forward + Vector3.Right;
            vertices[2].TextureCoordinate = new Vector2(1.0f, 1.0f);
            vertices[3].Position = Vector3.Backward + Vector3.Right;
            vertices[3].TextureCoordinate = new Vector2(1.0f, 0.0f);

            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i].Normal = Vector3.Up;
                vertices[i].Position *= size;
                vertices[i].TextureCoordinate *= size;
            }

            indices[5] = 0; indices[4] = 1; indices[3] = 2;
            indices[2] = 2; indices[1] = 1; indices[0] = 3;
        }

        protected override void LoadContent()
        {
            texture = this.Game.Content.Load<Texture2D>("checker");
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
                    vertices, 0, 4,indices, 0, 2);
            }

            base.Draw(gameTime);
        }
    }
}
