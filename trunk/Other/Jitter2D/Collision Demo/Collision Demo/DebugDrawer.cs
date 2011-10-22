using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Jitter2D.LinearMath;
using Microsoft.Xna.Framework.Content;

namespace CollisionDemo
{

    /// <summary>
    /// Draw axis aligned bounding boxes, points and lines.
    /// </summary>
    public class DebugDrawer : DrawableGameComponent, Jitter2D.IDebugDrawer
    {
        BasicEffect basicEffect;
        SpriteBatch sb;

        Texture2D pointTex;
        List<Vector2> points = new List<Vector2>();

        public DebugDrawer(Game game)
            : base(game)
        {
        }

        public override void Initialize()
        {
            base.Initialize();
            basicEffect = new BasicEffect(this.GraphicsDevice);
            basicEffect.VertexColorEnabled = true;

            sb = new SpriteBatch(this.GraphicsDevice);
        }

        protected override void LoadContent()
        {
            ContentManager cm = new ContentManager(this.Game.Services, "Content");
            pointTex = cm.Load<Texture2D>("Point Blue");
            base.LoadContent();
        }

        public void SetColor(float r, float g, float b, float a)
        {
            Color = new Color(r, g, b, a);
        }

        public void DrawLine(JVector p0, JVector p1, Color color)
        {
            lineIndex += 2;

            if (lineIndex == LineList.Length)
            {
                VertexPositionColor[] temp = new VertexPositionColor[LineList.Length + 50];
                LineList.CopyTo(temp, 0);
                LineList = temp;
            }

            LineList[lineIndex - 2].Color = color;
            LineList[lineIndex - 2].Position = Conversion.ToXNAVector3(p0);

            LineList[lineIndex - 1].Color = color;
            LineList[lineIndex - 1].Position = Conversion.ToXNAVector3(p1);
        }

        public void DrawTriangle(JVector p0, JVector p1, JVector p2, Color color)
        {
            triangleIndex += 3;

            if (triangleIndex == TriangleList.Length)
            {
                VertexPositionColor[] temp = new VertexPositionColor[TriangleList.Length + 300];
                TriangleList.CopyTo(temp, 0);
                TriangleList = temp;
            }

            TriangleList[triangleIndex - 2].Color = color;
            TriangleList[triangleIndex - 2].Position = Conversion.ToXNAVector3(p0);

            TriangleList[triangleIndex - 1].Color = color;
            TriangleList[triangleIndex - 1].Position = Conversion.ToXNAVector3(p1);

            TriangleList[triangleIndex - 3].Color = color;
            TriangleList[triangleIndex - 3].Position = Conversion.ToXNAVector3(p2);
        }

        public void DrawAabb(JVector from, JVector to, Color color)
        {
            DrawLine(new JVector(from.X, from.Y), new JVector(to.X, from.Y), color);
            DrawLine(new JVector(from.X, to.Y), new JVector(to.X, to.Y), color);
            DrawLine(new JVector(to.X, from.Y), new JVector(to.X, to.Y), color);
            DrawLine(new JVector(from.X, from.Y), new JVector(from.X, to.Y), color);
        }

        public VertexPositionColor[] TriangleList = new VertexPositionColor[99];
        public VertexPositionColor[] LineList = new VertexPositionColor[50];

        private int lineIndex = 0;
        private int triangleIndex = 0;

        public override void Draw(GameTime gameTime)
        {
            CollisionDemo demo = Game as CollisionDemo;

            basicEffect.GraphicsDevice.RasterizerState = RasterizerState.CullNone;

            basicEffect.View = demo.Camera.View;
            basicEffect.Projection = demo.Camera.Projection;

            basicEffect.TextureEnabled = false;
            foreach (EffectPass pass in basicEffect.CurrentTechnique.Passes)
            {
                pass.Apply();
                if (triangleIndex > 0)
                    GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                        PrimitiveType.TriangleList, TriangleList, 0, triangleIndex / 3);

                if (lineIndex > 0)
                    GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                        PrimitiveType.LineList, LineList, 0, lineIndex / 2);
            }

            basicEffect.TextureEnabled = true;
            
            sb.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend, null, null, RasterizerState.CullNone, basicEffect);

            foreach (var point in points)
            {
                sb.Draw(pointTex, point, null, Color.White, 0, new Vector2(5, 5), 0.025f, SpriteEffects.None, 0);
            }

            sb.End();

            points.Clear();

            lineIndex = 0;
            triangleIndex = 0;

            base.Draw(gameTime);
        }


        public void DrawLine(JVector start, JVector end)
        {
            DrawLine(start, end, Color);
        }

        public void DrawPoint(JVector pos)
        {
            points.Add(Conversion.ToXNAVector2(pos));
        }

        public Color Color { get; set; }

        public void DrawTriangle(JVector pos1, JVector pos2, JVector pos3)
        {
            DrawTriangle(pos1, pos2, pos3, Color);
        }
    }
}
