using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JitterDemo
{

    /// <summary>
    /// Draw axis aligned bounding boxes, points and lines.
    /// </summary>
    public class DebugDrawer : DrawableGameComponent
    {
        BasicEffect basicEffect;

        public DebugDrawer(Game game)
            : base(game)
        {
        }

        public override void Initialize()
        {
            base.Initialize();
            basicEffect = new BasicEffect(this.GraphicsDevice);
            basicEffect.VertexColorEnabled = true;
        }

        public void DrawLine(Vector3 p0, Vector3 p1, Color color)
        {
            index += 2;

            if (index == LineList.Length)
            {
                VertexPositionColor[] temp = new VertexPositionColor[LineList.Length + 50];
                LineList.CopyTo(temp, 0);
                LineList = temp;
            }

            LineList[index - 2].Color = color;
            LineList[index - 2].Position = p0;

            LineList[index - 1].Color = color;
            LineList[index - 1].Position = p1;
        }

        private void SetElement(ref Vector3 v, int index, float value)
        {
            if (index == 0)
                v.X = value;
            else if (index == 1)
                v.Y = value;
            else if (index == 2)
                v.Z = value;
            else
                throw new ArgumentOutOfRangeException("index");
        }

        private float GetElement(Vector3 v, int index)
        {
            if (index == 0)
                return v.X;
            if (index == 1)
                return v.Y;
            if (index == 2)
                return v.Z;

            throw new ArgumentOutOfRangeException("index");
        }

        public void DrawAabb(Vector3 from, Vector3 to, Color color)
        {
            Vector3 halfExtents = (to - from) * 0.5f;
            Vector3 center = (to + from) * 0.5f;

            Vector3 edgecoord = new Vector3(1f, 1f, 1f), pa, pb;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    pa = new Vector3(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y,
                        edgecoord.Z * halfExtents.Z);
                    pa += center;

                    int othercoord = j % 3;
                    SetElement(ref edgecoord, othercoord, GetElement(edgecoord, othercoord) * -1f);
                    pb = new Vector3(edgecoord.X * halfExtents.X, edgecoord.Y * halfExtents.Y,
                        edgecoord.Z * halfExtents.Z);
                    pb += center;

                    DrawLine(pa, pb, color);
                }
                edgecoord = new Vector3(-1f, -1f, -1f);
                if (i < 3)
                    SetElement(ref edgecoord, i, GetElement(edgecoord, i) * -1f);
            }
        }

        public VertexPositionColor[] LineList = new VertexPositionColor[50];
        private int index = 0;

        public override void Draw(GameTime gameTime)
        {
            if (index == 0) return;

            JitterDemo demo = Game as JitterDemo;

            basicEffect.View = demo.Camera.View;
            basicEffect.Projection = demo.Camera.Projection;

            foreach (EffectPass pass in basicEffect.CurrentTechnique.Passes)
            {
                pass.Apply();

                GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(
                    PrimitiveType.LineList, LineList, 0, index / 2);
            }

            index = 0;
 
            base.Draw(gameTime);
        }

    }
}
