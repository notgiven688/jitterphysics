#region File Description
//-----------------------------------------------------------------------------
// CapsulePrimitive.cs
//
// Microsoft XNA Community Game Platform
// Copyright (C) Microsoft Corporation. All rights reserved.
//-----------------------------------------------------------------------------
#endregion

#region Using Statements
using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
#endregion

namespace JitterDemo.Primitives3D
{
    /// <summary>
    /// Geometric primitive class for drawing spheres.
    /// </summary>
    public class CapsulePrimitive : GeometricPrimitive
    {
        /// <summary>
        /// Constructs a new sphere primitive, using default settings.
        /// </summary>
        public CapsulePrimitive(GraphicsDevice graphicsDevice)
            : this(graphicsDevice, 1.0f,0.8f, 12)
        {
        }


        /// <summary>
        /// Constructs a new sphere primitive,
        /// with the specified size and tessellation level.
        /// </summary>
        public CapsulePrimitive(GraphicsDevice graphicsDevice,
                               float diameter,float length, int tessellation)
        {
            if (tessellation % 2 != 0)
                throw new ArgumentOutOfRangeException("tessellation should be even");

            int verticalSegments = tessellation;
            int horizontalSegments = tessellation * 2;

            float radius = diameter / 2;

            // Start with a single vertex at the bottom of the sphere.
            AddVertex(Vector3.Down * radius + Vector3.Down * 0.5f * length, Vector3.Down);

            // Create rings of vertices at progressively higher latitudes.
            for (int i = 0; i < verticalSegments - 1; i++)
            {
                float latitude = ((i + 1) * MathHelper.Pi /
                                            verticalSegments) - MathHelper.PiOver2;
                float dy = (float)Math.Sin(latitude);
                float dxz = (float)Math.Cos(latitude);

                bool bla = false;

                if (i > (verticalSegments-2) / 2)
                {
                    bla = true;
                }

                // Create a single ring of vertices at this latitude.
                for (int j = 0; j < horizontalSegments; j++)
                {
                    float longitude = j * MathHelper.TwoPi / horizontalSegments;

                    float dx = (float)Math.Cos(longitude) * dxz;
                    float dz = (float)Math.Sin(longitude) * dxz;

                    Vector3 normal = new Vector3(dx, dy, dz);
                    Vector3 position = normal * radius;

                    if (bla) position += Vector3.Up * 0.5f * length;
                    else position += Vector3.Down * 0.5f * length;

                    AddVertex(position, normal);
                }
            }

            // Finish with a single vertex at the top of the sphere.
            AddVertex(Vector3.Up * radius + Vector3.Up * 0.5f * length, Vector3.Up);

            // Create a fan connecting the bottom vertex to the bottom latitude ring.
            for (int i = 0; i < horizontalSegments; i++)
            {
                AddIndex(0);
                AddIndex(1 + (i + 1) % horizontalSegments);
                AddIndex(1 + i);
            }

            // Fill the sphere body with triangles joining each pair of latitude rings.
            for (int i = 0; i < verticalSegments - 2; i++)
            {
                for (int j = 0; j < horizontalSegments; j++)
                {
                    int nextI = i + 1;
                    int nextJ = (j + 1) % horizontalSegments;

                    AddIndex(1 + i * horizontalSegments + j);
                    AddIndex(1 + i * horizontalSegments + nextJ);
                    AddIndex(1 + nextI * horizontalSegments + j);

                    AddIndex(1 + i * horizontalSegments + nextJ);
                    AddIndex(1 + nextI * horizontalSegments + nextJ);
                    AddIndex(1 + nextI * horizontalSegments + j);
                }
            }

            // Create a fan connecting the top vertex to the top latitude ring.
            for (int i = 0; i < horizontalSegments; i++)
            {
                AddIndex(CurrentVertex - 1);
                AddIndex(CurrentVertex - 2 - (i + 1) % horizontalSegments);
                AddIndex(CurrentVertex - 2 - i);
            }

            InitializePrimitive(graphicsDevice);
        }
    }
}
