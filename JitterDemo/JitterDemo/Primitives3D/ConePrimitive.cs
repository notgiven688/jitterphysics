#region File Description
//-----------------------------------------------------------------------------
// ConePrimitive.cs
//
// Microsoft XNA Community Game Platform
// Copyright (C) Microsoft Corporation. All rights reserved.
//-----------------------------------------------------------------------------
#endregion

#region Using Statements
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using System;
#endregion

namespace JitterDemo.Primitives3D
{
    /// <summary>
    /// Geometric primitive class for drawing cubes.
    /// </summary>
    public class ConePrimitive : GeometricPrimitive
    {
        /// <summary>
        /// Constructs a new cube primitive, using default settings.
        /// </summary>
        public ConePrimitive(GraphicsDevice graphicsDevice)
            : this(graphicsDevice, 1.0f, 1.0f, 32)
        {
        }


        /// <summary>
        /// Constructs a new cube primitive, with the specified size.
        /// </summary>
        public ConePrimitive(GraphicsDevice graphicsDevice, float height, float radius, int tessellation)
        {
            // Create a ring of triangles around the outside of the cylinder.
            AddVertex(Vector3.Up * (2.0f / 3.0f) * height, Vector3.Up);

            for (int i = 0; i < tessellation; i++)
            {
                Vector3 normal = GetCircleVector(i, tessellation);
                AddVertex(normal * radius + (1.0f / 3.0f) * height * Vector3.Down, normal);

                AddIndex(0);
                AddIndex(i);
                AddIndex(i + 1);
            }

            AddIndex(0);
            AddIndex(tessellation);
            AddIndex(1);

            CreateCap(tessellation, (1.0f / 3.0f) * height , radius, Vector3.Down);

            InitializePrimitive(graphicsDevice);
        }

        /// <summary>
        /// Helper method creates a triangle fan to close the ends of the cylinder.
        /// </summary>
        void CreateCap(int tessellation, float height, float radius, Vector3 normal)
        {
            // Create cap indices.
            for (int i = 0; i < tessellation - 2; i++)
            {
                if (normal.Y > 0)
                {
                    AddIndex(CurrentVertex);
                    AddIndex(CurrentVertex + (i + 1) % tessellation);
                    AddIndex(CurrentVertex + (i + 2) % tessellation);
                }
                else
                {
                    AddIndex(CurrentVertex);
                    AddIndex(CurrentVertex + (i + 2) % tessellation);
                    AddIndex(CurrentVertex + (i + 1) % tessellation);
                }
            }

            // Create cap vertices.
            for (int i = 0; i < tessellation; i++)
            {
                Vector3 position = GetCircleVector(i, tessellation) * radius +
                                   normal * height;

                AddVertex(position, normal);
            }
        }


        /// <summary>
        /// Helper method computes a point on a circle.
        /// </summary>
        static Vector3 GetCircleVector(int i, int tessellation)
        {
            float angle = i * MathHelper.TwoPi / tessellation;

            float dx = (float)Math.Cos(angle);
            float dz = (float)Math.Sin(angle);

            return new Vector3(dx, 0, dz);
        }
    }
}
