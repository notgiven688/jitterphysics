using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Jitter2D.LinearMath;
using Microsoft.Xna.Framework.Content;

namespace JitterDemo
{

    /// <summary>
    /// Draw axis aligned bounding boxes, points and lines.
    /// </summary>
    public class DebugDrawer : DrawableGameComponent, Jitter2D.IDebugDrawer
    {
        private class PointDef
        {
            public Vector2 point;
            public Color color;

            public PointDef(Vector2 vector2, Color color_2)
            {
                this.point = vector2;
                this.color = color_2;
            }
        }

        private class StringDef
        {
            public string text;
            public Vector2 point;
            public Color color;

            public StringDef(string text, Vector2 vector2, Color color_2)
            {
                this.text = text;
                this.point = vector2;
                this.color = color_2;
            }
        }

        BasicEffect basicEffect;
        SpriteBatch sb;

        Texture2D pointTex;
        SpriteFont font;
        List<PointDef> points = new List<PointDef>();
        List<StringDef> strings = new List<StringDef>();

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

        public void DrawString(string text, JVector position)
        {
            strings.Add(new StringDef(text, Conversion.ToXNAVector2(position), Color.Black));
        }

        public void DrawString(string text, JVector position, Color color)
        {
            strings.Add(new StringDef(text, Conversion.ToXNAVector2(position), color));
        }

        public VertexPositionColor[] TriangleList = new VertexPositionColor[99];
        public VertexPositionColor[] LineList = new VertexPositionColor[50];

        private int lineIndex = 0;
        private int triangleIndex = 0;

        public override void Draw(GameTime gameTime)
        {
            JitterDemo demo = Game as JitterDemo;

            basicEffect.View = demo.Camera.View;
            basicEffect.Projection = demo.Camera.Projection;

            basicEffect.TextureEnabled = false;
            foreach (EffectPass pass in basicEffect.CurrentTechnique.Passes)
            {
                pass.Apply();
                if (triangleIndex > 0)
                {
                    int triangles = triangleIndex / 3;
                    int loop = 0;

                    while (triangles > 20000)
                    {
                        int numToDraw = 20000;
                        triangles -= numToDraw;

                        GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                        PrimitiveType.TriangleList, TriangleList, loop * 3 * 20000, numToDraw);

                        loop++;
                    }

                    GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                        PrimitiveType.TriangleList, TriangleList, loop * 3 * 20000, triangles);
                }

                if (lineIndex > 0)
                {
                    int lines = lineIndex / 2;
                    int loop = 0;

                    while (lines > 20000)
                    {
                        int numToDraw = 20000;
                        lines -= numToDraw;

                        GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                            PrimitiveType.LineList, LineList, loop * 2 * 20000, numToDraw);

                        loop++;
                    }
                    GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                           PrimitiveType.LineList, LineList, loop * 2 * 20000, lines);
                }
            }

            basicEffect.TextureEnabled = true;
            
            sb.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend, null, null, RasterizerState.CullNone, basicEffect);

            foreach (var point in points)
            {
                sb.Draw(pointTex, point.point, null, point.color, 0, new Vector2(5, 5), 0.01f, SpriteEffects.None, 0);
            }

            foreach (var s in strings)
            {
                sb.DrawString(font, s.text, s.point, s.color);
            }

            sb.End();

            points.Clear();
            strings.Clear();

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
            points.Add(new PointDef(Conversion.ToXNAVector2(pos), Color));
        }

        public Color Color { get; set; }

        public void DrawTriangle(JVector pos1, JVector pos2, JVector pos3)
        {
            DrawTriangle(pos1, pos2, pos3, Color);
        }
    }
}
