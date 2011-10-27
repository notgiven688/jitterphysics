using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;

namespace SAT
{
   public class LD
    {
       private static BasicEffect effect;
       private static VertexPositionColor[] vertices;
       private static int vertexCount;

       public LD(GraphicsDevice gd)
       {
           effect = new BasicEffect(gd);
           effect.VertexColorEnabled = true;
           vertices = new VertexPositionColor[ushort.MaxValue];
           vertexCount = 0;
       }

       public static void Draw(Vector2 s, Vector2 e, Color c)
       {
           vertices[vertexCount].Position = new Vector3(s, 0);
           vertices[vertexCount++].Color = c;
           vertices[vertexCount].Position = new Vector3(e, 0);
           vertices[vertexCount++].Color = c;
       }

       public static void Render(Matrix view, Matrix projection)
       {
           if (vertexCount > 1)
           {
               effect.View = view;
               effect.Projection = projection;

               effect.CurrentTechnique.Passes[0].Apply();

               effect.GraphicsDevice.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, vertices, 0, vertexCount / 2);
               vertexCount = 0;
           }
       }
    }
}
