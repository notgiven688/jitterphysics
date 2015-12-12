//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using Microsoft.Xna.Framework.Graphics;
//using Microsoft.Xna.Framework;
//using Jitter.LinearMath;
//using JitterDemo;

//namespace JitterDemo.Primitives3D
//{
//    public class ConvexHullPrimitive : GeometricPrimitive
//    {

//        //public JConvexHull ConvexHull = new JConvexHull();

//        public ConvexHullPrimitive(GraphicsDevice device, List<JVector> pointCloud)
//        {
//            JConvexHull.Build(pointCloud,JConvexHull.Approximation.Level5);

//            int counter = 0;

//            foreach (JConvexHull.Face face in ConvexHull.HullFaces)
//            {
//                this.AddVertex(Conversion.ToXNAVector(pointCloud[face.VertexC]), Conversion.ToXNAVector(face.Normal));
//                this.AddVertex(Conversion.ToXNAVector(pointCloud[face.VertexB]), Conversion.ToXNAVector(face.Normal));
//                this.AddVertex(Conversion.ToXNAVector(pointCloud[face.VertexA]), Conversion.ToXNAVector(face.Normal));

//                this.AddIndex(counter + 0);
//                this.AddIndex(counter + 1);
//                this.AddIndex(counter + 2);

//                counter+=3;
//            }
            

//            this.InitializePrimitive(device);
//        }

//    }
//}
