using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using Jitter2D.Collision;
using Jitter2D.Collision.Shapes;
using Jitter2D.LinearMath;
using Jitter2D.Dynamics;
using System.Diagnostics;

namespace CollisionDemo
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class CollisionDemo : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        public DebugDrawer DebugDrawer;
        public Camera Camera;

        RigidBody body1 = new RigidBody(new CircleShape(2));
        RigidBody body2 = new RigidBody(new CapsuleShape(3, 1));

        JVector point, normal;
        float penetration;
        bool hit;
        int iterations;

        SpriteFont font;
        Stopwatch sw = new Stopwatch();
        long ticks;

        public CollisionDemo()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.PreferMultiSampling = true;

            Content.RootDirectory = "Content";

            graphics.PreferredBackBufferHeight = 720;
            graphics.PreferredBackBufferWidth = 1280;
        }

        protected override void Initialize()
        {
            DebugDrawer = new DebugDrawer(this);
            this.Components.Add(DebugDrawer);

            Camera = new Camera(this);
            this.Components.Add(Camera);

            body2.Position = new JVector(4, 2);

            base.Initialize();
        }

        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);

            font = Content.Load<SpriteFont>("font1");
        }

        protected override void Update(GameTime gameTime)
        {
            // Allows the game to exit
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                this.Exit();

            KeyboardState keys = Keyboard.GetState();
            JVector moveVector = JVector.Zero;
            float amountOfMovement = 0.05f;

            if (keys.IsKeyDown(Keys.Right))
                moveVector.X += amountOfMovement;
            if (keys.IsKeyDown(Keys.Left))
                moveVector.X -= amountOfMovement;
            if (keys.IsKeyDown(Keys.Down))
                moveVector.Y -= amountOfMovement;
            if (keys.IsKeyDown(Keys.Up))
                moveVector.Y += amountOfMovement;

            body1.Position += moveVector;

            body1.Orientation += 0.001f;
            body2.Orientation -= 0.001f;

            var c1 = body1.Shape as CircleShape;
            var c2 = body2.Shape as CapsuleShape;

            var o = new JVector((float)-Math.Sin(body2.Orientation), (float)Math.Cos(body2.Orientation));

            JVector point1, point2;

            sw.Start();
            hit = Collision.CircleCapsuleTest(body1.Position, c1.Radius, body2.Position, o, c2.Length, c2.Radius, out point1, out point2, out normal, out penetration);
            sw.Stop();

            ticks = sw.ElapsedTicks;
            sw.Reset();

            DebugDrawer.DrawLine(point1, point1 + normal);

            DebugDrawer.DrawPoint(point2);
            DebugDrawer.DrawPoint(point1);
            DebugDrawer.Color = Color.Red;

            DebugDrawer.Color = Color.Black;
            DebugDrawer.DrawLine(JVector.Up, JVector.Down);
            DebugDrawer.DrawLine(JVector.Left, JVector.Right);

            body1.DebugDraw(DebugDrawer);
            body2.DebugDraw(DebugDrawer);
            
            if (hit)
            {
                var oldPosition = body1.Position;

                body1.Position += normal * penetration;
                body1.DebugDraw(DebugDrawer);
                body1.Position = oldPosition;
            }
            
            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            int line = 1;

            spriteBatch.Begin();

            spriteBatch.DrawString(font, "Collided: " + hit.ToString(), new Vector2(10, line++ * 20), Color.Black);
            spriteBatch.DrawString(font, "Penetration: " + penetration.ToString(), new Vector2(10, line++ * 20), Color.Black);
            spriteBatch.DrawString(font, "MPR Iterations: " + iterations.ToString(), new Vector2(10, line++ * 20), Color.Black);
            spriteBatch.DrawString(font, "MPR Ticks: " + ticks.ToString(), new Vector2(10, line++ * 20), Color.Black);

            spriteBatch.End();

            base.Draw(gameTime);
        }
    }
}
