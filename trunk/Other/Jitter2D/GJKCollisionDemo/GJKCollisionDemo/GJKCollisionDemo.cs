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
using System;

namespace GJKCollisionDemo
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class GJKCollisionDemo : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        public DebugDrawer DebugDrawer;
        public Camera Camera;
        KeyboardState oldState;

        RigidBody body1 = new RigidBody(new CircleShape(2.5f));
        RigidBody body2 = new RigidBody(new CapsuleShape(2, 0.75f));

        JVector point, normal;
        float penetration;
        bool hit;
        int shapeType = 0;

        SpriteFont font;
        Stopwatch sw = new Stopwatch();
        long ticks;

        public GJKCollisionDemo()
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

            body1.Position = new JVector(0, 2);
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

            if (keys.IsKeyDown(Keys.OemPlus) && oldState.IsKeyUp(Keys.OemPlus))
                GJKCollide.MaxIterations++;
            if (keys.IsKeyDown(Keys.OemMinus) && oldState.IsKeyUp(Keys.OemMinus))
                GJKCollide.MaxIterations--;

            GJKCollide.MaxIterations = (int)JMath.Clamp(GJKCollide.MaxIterations, 0, 25);

            bool changeShape = false;
            if (keys.IsKeyDown(Keys.D1) && oldState.IsKeyUp(Keys.D1))
            {
                shapeType++;
                changeShape = true;
            }
            if (keys.IsKeyDown(Keys.D2) && oldState.IsKeyUp(Keys.D2))
            {
                shapeType--;
                changeShape = true;
            }

            shapeType = (int)JMath.Clamp(shapeType, 0, 2);
            if (changeShape)
            {
                Random r = new Random();
                switch (shapeType)
                {
                    case 0:     // circle
                        body1 = new RigidBody(new CircleShape((float)r.NextDouble() * 3f));
                        break;
                    case 1:      // capsule
                        body1 = new RigidBody(new CapsuleShape((float)r.NextDouble() * 3f, (float)r.NextDouble() * 1f));
                        break;
                    case 2:     // box
                        body1 = new RigidBody(new BoxShape((float)r.NextDouble() * 3f, (float)r.NextDouble() * 3f));
                        break;
                }
            }
            

            body1.Orientation += 0.005f;
            body2.Orientation -= 0.005f;

            JMatrix o1 = JMatrix.CreateRotationZ(body1.Orientation);
            JMatrix o2 = JMatrix.CreateRotationZ(body2.Orientation);
            JVector pos1 = body1.Position;
            JVector pos2 = body2.Position;

            JVector point2;

            sw.Start();
            hit = GJKCollide.ClosestPoints(body1.Shape, body2.Shape, ref o1, ref o2, ref pos1, ref pos2, out point, out point2, out normal);
            sw.Stop();

            penetration = JVector.Distance(point, point2);

            ticks = sw.ElapsedTicks;
            sw.Reset();

            DebugDrawer.DrawPoint(point2);
            DebugDrawer.DrawPoint(point);
            DebugDrawer.Color = Color.Red;

            DebugDrawer.DrawLine(JVector.Zero, normal);

            DebugDrawer.Color = Color.Black;
            DebugDrawer.DrawLine(JVector.Up, JVector.Down);
            DebugDrawer.DrawLine(JVector.Left, JVector.Right);

            body1.DebugDraw(DebugDrawer);
            body2.DebugDraw(DebugDrawer);

            oldState = keys;

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            int line = 1;

            spriteBatch.Begin();

            spriteBatch.DrawString(font, "Collided: " + hit.ToString(), new Vector2(10, line++ * 20), Color.Black);
            spriteBatch.DrawString(font, "Penetration: " + penetration.ToString(), new Vector2(10, line++ * 20), Color.Black);
            spriteBatch.DrawString(font, "GJK Iterations: " + GJKCollide.IterationsTaken.ToString(), new Vector2(10, line++ * 20), Color.Black);
            spriteBatch.DrawString(font, "GJK Max Iterations: " + GJKCollide.MaxIterations.ToString(), new Vector2(10, line++ * 20), Color.Black);
            spriteBatch.DrawString(font, "GJK Ticks: " + ticks.ToString(), new Vector2(10, line++ * 20), Color.Black);

            spriteBatch.End();

            base.Draw(gameTime);
        }
    }
}
