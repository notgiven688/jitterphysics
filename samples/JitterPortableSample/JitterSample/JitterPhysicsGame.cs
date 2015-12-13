using System;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Input.Touch;

using Jitter;
using Jitter.Collision;
using Jitter.Collision.Shapes;
using Jitter.Dynamics;
using Jitter.LinearMath;

using JitterSample.Primitives;

namespace JitterSample
{
    public class JitterPhysicsGame : Game
    {
        private static readonly Color XamarinBlue = new Color(52, 152, 219);
        private static readonly Color XamarinGreen = new Color(119, 208, 101);
        private static readonly Color XamarinPurple = new Color(180, 85, 182);
        private static readonly Color XamarinBlack = new Color(44, 62, 80);

        private GraphicsDeviceManager graphics;
        private SpriteBatch spriteBatch;

        // In this very simple demo we just have cubes.
        private CubePrimitive renderCube;
        // And a nice font
        private SpriteFont font;

        // Our reference to the physics world.
        private World world;

        public JitterPhysicsGame()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.PreferMultiSampling = true;
            graphics.PreferredBackBufferWidth = (int)PreferredSize.X;
            graphics.PreferredBackBufferHeight = (int)PreferredSize.Y;

            Content.RootDirectory = "Content";

            // Choose a collision system and create a new world instance.
            var collision = new CollisionSystemSAP();
            world = new World(collision);

            // Call this method which adds some boxes to the
            // empty world.
            CreateInitialScene();
        }

        public static Vector2 PreferredSize { get; set; }

        private void CreateInitialScene()
        {
            // Create the solid ground
            var groundShape = new BoxShape(9, 1, 9);
            var groundBody = new RigidBody(groundShape);
            groundBody.Position = new JVector(0, -5, 0);
            groundBody.IsStatic = true;
            world.AddBody(groundBody);

            // add the boxes to the world
            for (int i = 0; i < 17; i++)
            {
                CreateCube(i);
            }
        }

        private void CreateCube(int height)
        {
            var boxShape = new BoxShape(JVector.One);
            var boxBody = new RigidBody(boxShape);
            boxBody.Position = new JVector(0, 0.5f + height * 2.2f, height * 0.1f);
            world.AddBody(boxBody);
        }

        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures. 
            spriteBatch = new SpriteBatch(GraphicsDevice);
            // Initialize our render cube
            renderCube = new CubePrimitive(graphics.GraphicsDevice);
            // and then our font
            font = Content.Load<SpriteFont>("font");
        }

        protected override void UnloadContent()
        {
            renderCube.Dispose();
        }

        protected override void Update(GameTime gameTime)
        {
            HandleInput(gameTime);

            // step the physics
            var time = (float)gameTime.ElapsedGameTime.TotalSeconds;
            world.Step(time, true);

            // remove bodies as they get too far down
            foreach (var body in world.RigidBodies.ToArray())
            {
                if (body.Position.Y < -50)
                {
                    world.RemoveBody(body);
                }
            }

            base.Update(gameTime);
        }

        private void HandleInput(GameTime gameTime)
        {
            var gamePadState = GamePad.GetState(PlayerIndex.One);

            var shouldQuit =
                gamePadState.Buttons.Back == ButtonState.Pressed;
            if (shouldQuit)
            {
                Exit();
                return;
            }

            var touchState = TouchPanel.GetState();
            var keyboardState = Keyboard.GetState();

            var isTouching = touchState.Any(t =>
                t.State == TouchLocationState.Pressed ||
                t.State == TouchLocationState.Moved);
            var shouldCreateCube =
                keyboardState.IsKeyDown(Keys.Space) ||
                gamePadState.IsButtonDown(Buttons.A) ||
                isTouching;
            if (shouldCreateCube)
            {
                CreateCube(5);
            }
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(XamarinBlue);
            GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;

            var cameraPosition = new Vector3(3, 5, 20f);
            var aspect = GraphicsDevice.Viewport.AspectRatio;
            var viewMatrix = Matrix.CreateLookAt(cameraPosition, Vector3.Zero, Vector3.Up);
            var projectionMatrix = Matrix.CreatePerspectiveFieldOfView(1, aspect, 1, 100);

            // go through all bodies in the world.
            foreach (RigidBody body in world.RigidBodies)
            {
                // convert the current body.Shape to a BoxShape
                var shape = body.Shape as BoxShape;

                // the cube we want to draw is a unit cube. So by setting
                // the world matrix to a scale matrix with the shape.Size
                // we can draw different sized boxes.
                // => WorldMatrix = Scale * Orientation * Translation
                var scale = Matrix.CreateScale(shape.Size.X, shape.Size.Y, shape.Size.Z);
                var orientation = new Matrix(
                    body.Orientation.M11, body.Orientation.M12, body.Orientation.M13, 0.0f,
                    body.Orientation.M21, body.Orientation.M22, body.Orientation.M23, 0.0f,
                    body.Orientation.M31, body.Orientation.M32, body.Orientation.M33, 0.0f,
                    0, 0, 0, 1);
                var translation = Matrix.CreateTranslation(body.Position.X, body.Position.Y, body.Position.Z);
                var worldMatrix = scale * orientation * translation;

                var color = (shape.Mass != 1.0f) ? XamarinPurple : XamarinGreen;

                // draw every cube
                renderCube.Draw(worldMatrix, viewMatrix, projectionMatrix, color);
            }

            // calculate the text position
            var titleSafeArea = GraphicsDevice.Viewport.TitleSafeArea;
            var width = titleSafeArea.Width * 0.75f;
            var left = (titleSafeArea.Width - width) / 2f + titleSafeArea.X;
            var top = Math.Max(titleSafeArea.Y, left);
            var hudLocation = new Vector2(left, top);
            // calculate the text size
            string text = "tap to create more cubes";
            var textWidth = font.MeasureString(text).X;
            var textScale = width / textWidth;

            // draw the HUD
            spriteBatch.Begin(SpriteSortMode.Immediate);
            spriteBatch.DrawString(font, text, hudLocation, XamarinBlack, 0f, Vector2.Zero, new Vector2(textScale), SpriteEffects.None, 0f);
            spriteBatch.End();

            base.Draw(gameTime);
        }
    }
}
