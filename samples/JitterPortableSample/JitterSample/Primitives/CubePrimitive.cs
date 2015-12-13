using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace JitterSample.Primitives
{
    /// <summary>
    /// Geometric primitive class for drawing cubes.
    /// </summary>
    public class CubePrimitive : GeometricPrimitive
    {
        // A cube has six faces, each one pointing in a different direction.
        private static readonly Vector3[] normals =
        {
            new Vector3(0, 0, 1),
            new Vector3(0, 0, -1),
            new Vector3(1, 0, 0),
            new Vector3(-1, 0, 0),
            new Vector3(0, 1, 0),
            new Vector3(0, -1, 0),
        };

        /// <summary>
        /// Constructs a new cube primitive, with the specified size.
        /// </summary>
        public CubePrimitive(GraphicsDevice graphicsDevice)
        {
            // Create each face in turn.
            foreach (Vector3 normal in normals)
            {
                // Get two vectors perpendicular to the face normal and to each other.
                var side1 = new Vector3(normal.Y, normal.Z, normal.X);
                var side2 = Vector3.Cross(normal, side1);

                // Six indices (two triangles) per face.
                AddIndex(CurrentVertex + 0);
                AddIndex(CurrentVertex + 1);
                AddIndex(CurrentVertex + 2);

                AddIndex(CurrentVertex + 0);
                AddIndex(CurrentVertex + 2);
                AddIndex(CurrentVertex + 3);

                // Four vertices per face.
                AddVertex((normal - side1 - side2) * 0.5f, normal);
                AddVertex((normal - side1 + side2) * 0.5f, normal);
                AddVertex((normal + side1 + side2) * 0.5f, normal);
                AddVertex((normal + side1 - side2) * 0.5f, normal);
            }

            InitializePrimitive(graphicsDevice);
        }
    }
}
