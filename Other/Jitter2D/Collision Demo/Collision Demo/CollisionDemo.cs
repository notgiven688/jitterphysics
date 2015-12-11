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
using Jitter2D.Collision.Narrowphase;

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

        JVector point, normal;
        float penetration;
        bool hit, collide;
        int iterations;

        SpriteFont font;
        Stopwatch sw = new Stopwatch();
        long ticks;

        // A
        private BoxShape A;
        private JVector PA;
        private JVector VA;
        private float OA;
        private List<JVector> AV;
        
        // B
        private BoxShape B;
        private JVector PB;
        private JVector VB;
        private float OB;
        private List<JVector> BV;
        
        public CollisionDemo()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.PreferMultiSampling = true;

            Content.RootDirectory = "Content";

            IsFixedTimeStep = false;
            graphics.SynchronizeWithVerticalRetrace = false;

            graphics.PreferredBackBufferHeight = 550;
            graphics.PreferredBackBufferWidth = 600;
        }

        protected override void Initialize()
        {
            DebugDrawer = new DebugDrawer(this);
            this.Components.Add(DebugDrawer);

            Camera = new Camera(this);
            this.Components.Add(Camera);

            A = new BoxShape(4, 1);
            PA = new JVector(-1, 2);
            VA = new JVector(0, 0);
            OA = MathHelper.ToRadians(0);

            B = new BoxShape(1, 4);
            PB = new JVector(-3, 0);
            VB = new JVector(0, 0);
            OB = 0;

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

            PB += moveVector;
            OA = (float)gameTime.TotalGameTime.TotalSeconds * 0.1f;
            OB = (float)gameTime.TotalGameTime.TotalSeconds * -0.1f;

            DrawBox(A, PA, OA, Color.Blue * 0.25f);
            DrawBox(B, PB, OB, Color.Green * 0.25f);

            float t = 0.0f;
            JVector[] CA = new JVector[2], CB = new JVector[2];
            int NumContacts = 0;


            sw.Start();

            JMatrix OAM = JMatrix.CreateRotationZ(OA);
            JMatrix OBM = JMatrix.CreateRotationZ(OB);
           
            for (int i = 0; i < 1; i++)
            {
                A.UpdateAxes(OA);
                B.UpdateAxes(OB);

                //hit = Collision.BoxBoxTest(ref A, ref PA, ref B, ref PB);
                //AV = new List<JVector> { A.GetCorner(0), A.GetCorner(1), A.GetCorner(2), A.GetCorner(3) };
                //BV = new List<JVector> { B.GetCorner(0), B.GetCorner(1), B.GetCorner(2), B.GetCorner(3) };

                //hit = SAT.Collide(ref AV, ref PA, ref VA, ref OAM,
                //    ref BV, ref PB, ref VB, ref OBM,
                //    ref normal, ref t);

                //if (hit)
                //{
                //    SAT.FindContacts(ref AV, ref PA, ref VA, ref OAM,
                //        ref BV, ref PB, ref VB, ref OBM,
                //        ref normal, t, out CA, out CB, out NumContacts);

                //    normal.Normalize();
                //}

                hit = Collision.BoxBoxTestContact(ref A, ref PA, ref OAM, ref B, ref PB, ref OBM,
                    out normal, out t, out CA, out CB, out NumContacts);

                penetration = t;
                iterations = NumContacts;
            }

            sw.Stop();
            ticks = sw.ElapsedTicks / 1;
            sw.Reset();

            if (hit)
            {
                //DrawBox(A, PA + normal * (t * 0.5f), OA, Color.Blue);
                //DrawBox(B, PB - normal * (t * 0.5f), OB, Color.Green);

                for (int i = 0; i < NumContacts; i++)
                {
                    DebugDrawer.DrawPoint(CA[i]);// + normal * (t * 0.5f));
                    DebugDrawer.DrawPoint(CB[i]);// - normal * (t * 0.5f));
                }
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
            spriteBatch.DrawString(font, "Contacts: " + iterations.ToString(), new Vector2(10, line++ * 20), Color.Black);
            spriteBatch.DrawString(font, "Ticks: " + ticks.ToString(), new Vector2(10, line++ * 20), Color.Black);

            spriteBatch.End();

            base.Draw(gameTime);
        }

        void DrawBox(BoxShape box, JVector pos, float orientation, Color color)
        {
            List<JVector> poly = new List<JVector> 
            { 
                new JVector(-box.Size.X * 0.5f, -box.Size.Y * 0.5f), 
                new JVector(box.Size.X * 0.5f, -box.Size.Y * 0.5f), 
                new JVector(box.Size.X * 0.5f, box.Size.Y * 0.5f), 
                new JVector(-box.Size.X * 0.5f,box.Size.Y * 0.5f) 
            };

            DrawPoly(poly, pos, JMatrix.CreateRotationZ(orientation), color);
        }

        void DrawPoly(List<JVector> poly, JVector pos, JMatrix o, Color color)
        {
            for (int i = 0; i < poly.Count - 1; i++)
            {
                JVector a = JVector.Transform(poly[i], o * JMatrix.CreateTranslation(pos));
                JVector b = JVector.Transform(poly[i + 1], o * JMatrix.CreateTranslation(pos));
                DebugDrawer.DrawLine(a, b, color);
            }
            JVector c = JVector.Transform(poly[0], o * JMatrix.CreateTranslation(pos));
            JVector d = JVector.Transform(poly[poly.Count - 1], o * JMatrix.CreateTranslation(pos));
            DebugDrawer.DrawLine(c, d, color);
        }

        List<JVector> BuildBlob(int iNumVertices, float radius)
        {
            List<JVector> axVertices = new List<JVector>(iNumVertices);

            float a = (float)Math.PI / iNumVertices;
            float da = (float)(Math.PI * 2.0f) / iNumVertices;

            for (int i = 0; i < iNumVertices; i++)
            {
                a += da;

                axVertices.Add(new JVector((float)Math.Cos(a) * radius, (float)Math.Sin(a) * radius));
            }
            return axVertices;
        }
    }
}
