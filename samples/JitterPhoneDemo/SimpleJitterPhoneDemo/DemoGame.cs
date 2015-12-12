#region Using Statements
using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Input.Touch;
using Microsoft.Xna.Framework.Media;

using Primitives3D;

// Add some Jitter namespaces
using Jitter;
using Jitter.Collision;
using Jitter.Dynamics;
using Jitter.Collision.Shapes;
using Jitter.LinearMath;
#endregion

namespace SimpleJitterPhoneDemo
{

    /// <summary>
    /// Simple JitterPhone Demo.
    /// </summary>
    public class DemoGame : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;

        // In this very simple demo we just have cubes.
        // I just used the Microsoft sample "Primitives3D"
        CubePrimitive renderCube;

        // Our reference to the physics world.
        World world;

        public DemoGame()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.PreferredBackBufferWidth = 480;
            graphics.PreferredBackBufferHeight = 800;

            Content.RootDirectory = "Content";

            TargetElapsedTime = TimeSpan.FromTicks(333333);

            // Choose a collision system and create a new world instance.
            CollisionSystemSAP collision = new CollisionSystemSAP();
            world = new World(collision);

            // Call this method which adds some boxes to the
            // emtpy world.
            CreateInitialScene();
        }

        private void CreateInitialScene()
        {
            // Create the shapes
            BoxShape groundShape = new BoxShape(9, 1, 9);
            BoxShape boxShape = new BoxShape(JVector.One);

            // Create rigid bodies from them
            RigidBody groundBody = new RigidBody(groundShape);
            groundBody.IsStatic = true; // making the ground immovable
            world.AddBody(groundBody);  // finally add it to the world

            for (int i = 0; i < 17; i++)
            {
                RigidBody boxBody = new RigidBody(boxShape);         // create a new box
                boxBody.Position = new JVector(0, 0.5f + i * 2.2f, i*0.1f);  // move it
                world.AddBody(boxBody);                              // and add it
            }
           
        }

        protected override void Initialize()
        {
            base.Initialize();
        }

        protected override void LoadContent()
        {
            // Initialize our render cube
            renderCube = new CubePrimitive(graphics.GraphicsDevice);
        }

        protected override void UnloadContent()
        {
            renderCube.Dispose();
        }

        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                this.Exit();

            // not sure if multithreading makes really sense
            // has to be tested with real devices.
            world.Step(1.0f / 60.0f, false);
            world.Step(1.0f / 60.0f, false);

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);
            GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;

            Vector3 cameraPosition = new Vector3(3, 5, 20f);

            float aspect = GraphicsDevice.Viewport.AspectRatio;


            Matrix worldMatrix;
            Matrix viewMatrix = Matrix.CreateLookAt(cameraPosition, Vector3.Zero, Vector3.Up);
            Matrix projectionMatrix = Matrix.CreatePerspectiveFieldOfView(1, aspect, 1, 100);

            // go through all bodies in the world.
            foreach(RigidBody body in world.RigidBodies)
            {
                // convert the current body.Shape to a BoxShape
                BoxShape shape = body.Shape as BoxShape;

                // the cube we want to draw is a unit cube. So by setting
                // the world matrix to a scale matrix with the shape.Size
                // we can draw different sized boxes.
                // => WorldMatrix = Scale * Orientation * Translation
                worldMatrix = Matrix.CreateScale(shape.Size.X, shape.Size.Y, shape.Size.Z) *

                    new Matrix(body.Orientation.M11, body.Orientation.M12, body.Orientation.M13, 0.0f,
                        body.Orientation.M21, body.Orientation.M22, body.Orientation.M23, 0.0f,
                        body.Orientation.M31, body.Orientation.M32, body.Orientation.M33, 0.0f,
                        0, 0, 0, 1) *

                    Matrix.CreateTranslation(body.Position.X, body.Position.Y, body.Position.Z);
                        
                // draw every cube
                Color color = (shape.Mass != 1.0f) ? Color.Gray : Color.Red;
                renderCube.Draw(worldMatrix, viewMatrix, projectionMatrix, color);
            }

            base.Draw(gameTime);
        }
    }
}
