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
using Jitter2D.LinearMath;
using Jitter2D.Dynamics;
using Jitter2D;
using System.Reflection;
using Jitter2D.Collision;
using Jitter2D.Collision.Shapes;
using Jitter2D.Dynamics.Constraints;
using Jitter2D.Dynamics.Springs;

namespace JitterDemo
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class JitterDemo : Microsoft.Xna.Framework.Game
    {
        public Camera Camera { private set; get; }
        public Display Display { private set; get; }
        public DebugDrawer DebugDrawer { private set; get; }
        public List<Scenes.Scene> PhysicScenes { private set; get; }
        public World World { private set; get; }

        private int currentScene = 0;

        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        #region update - global variables

        // Hold previous input states.
        KeyboardState keyboardPreviousState = new KeyboardState();
        GamePadState gamePadPreviousState = new GamePadState();
        MouseState mousePreviousState = new MouseState();

        private GamePadState padState;
        private KeyboardState keyState;
        private MouseState mouseState;

        #endregion

        private Color backgroundColor = new Color(63, 66, 73);
        private bool multithread = true;
        private int activeBodies = 0;
        private FixedLinearSpring grabSpring = null;
        private RigidBody grabBody = null;

        //private bool debugDraw = false;

        public JitterDemo()
        {
            this.IsMouseVisible = true;
            graphics = new GraphicsDeviceManager(this);

            graphics.PreferMultiSampling = true;

            Content.RootDirectory = "Content";

            graphics.PreferredBackBufferHeight =  720;
            graphics.PreferredBackBufferWidth =  1280;

            this.IsFixedTimeStep = false;
            this.TargetElapsedTime = TimeSpan.FromSeconds(1.0 / 100.0);
            this.graphics.SynchronizeWithVerticalRetrace = false;

            CollisionSystem collision = new CollisionSystemPersistentSAP();
            collision.EnableSpeculativeContacts = false;
            World = new World(collision); World.AllowDeactivation = false;
            
            this.Window.AllowUserResizing = true;

            this.Window.Title = "Jitter 2D Physics Demo - Jitter 2D "
                + Assembly.GetAssembly(typeof(Jitter2D.World)).GetName().Version.ToString();
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            Camera = new Camera(this);
            Camera.Position = new Vector2(0, 0);
            this.Components.Add(Camera);

            DebugDrawer = new DebugDrawer(this);
            this.Components.Add(DebugDrawer);

            Display = new Display(this);
            Display.DrawOrder = int.MaxValue;
            this.Components.Add(Display);
          
            this.PhysicScenes = new List<Scenes.Scene>();

            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes())
            {
                if (type.Namespace == "JitterDemo.Scenes" && !type.IsAbstract)
                {
                    if (type.Name == "EmptyScene") currentScene = PhysicScenes.Count;
                    Scenes.Scene scene = (Scenes.Scene)Activator.CreateInstance(type, this);
                    this.PhysicScenes.Add(scene);
                }
            }

            if (PhysicScenes.Count > 0)
                this.PhysicScenes[currentScene].Build();

            base.Initialize();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);
        }

        private void DestroyCurrentScene()
        {
            for (int i = this.Components.Count - 1; i >= 0; i--)
            {
                IGameComponent component = this.Components[i];

                if (component is Camera) continue;
                if (component is Display) continue;
                if (component is DebugDrawer) continue;

                this.Components.RemoveAt(i);
            }

            World.Clear();
        }

        private bool PressedOnce(Keys key, Buttons button)
        {
            bool keyboard = keyState.IsKeyDown(key) && !keyboardPreviousState.IsKeyDown(key);

            if (key == Keys.Add) key = Keys.OemPlus;
            keyboard |= keyState.IsKeyDown(key) && !keyboardPreviousState.IsKeyDown(key);

            if (key == Keys.Subtract) key = Keys.OemMinus;
            keyboard |= keyState.IsKeyDown(key) && !keyboardPreviousState.IsKeyDown(key);

            bool gamePad = padState.IsButtonDown(button) && !gamePadPreviousState.IsButtonDown(button);

            return keyboard || gamePad;
        }

        

        protected override void Update(GameTime gameTime)
        {
            padState = GamePad.GetState(PlayerIndex.One);
            keyState = Keyboard.GetState();
            mouseState = Mouse.GetState();

            // let the user escape the demo
            if (PressedOnce(Keys.Escape, Buttons.Back)) this.Exit();

            // change threading mode
            if (PressedOnce(Keys.M, Buttons.A)) multithread = !multithread;

            #region drag and drop physical objects with the mouse

            JVector mouseLocation = Camera.ScreenToWorldSpace(new JVector(mouseState.X, mouseState.Y));

            // MouseDown
            if (mouseState.LeftButton == ButtonState.Pressed &&
                mousePreviousState.LeftButton == ButtonState.Released)
            {
                JBBox mouseBox = new JBBox(mouseLocation - new JVector(0.01f), mouseLocation + new JVector(0.01f));
                
                //World.CollisionSystem.Query((foundItem) =>
                //{
                //    grabBody = foundItem as RigidBody;
                //    // don't continue
                //    return false;
                //}, ref mouseBox);

                if (grabBody != null)
                {
                    if (grabSpring != null) World.RemoveSpring(grabSpring);

                    // convert mouse coordinates to foundBody's local space
                    var localMouseLocation = JVector.Transform(mouseLocation - grabBody.Position, JMatrix.CreateRotationZ(-grabBody.Orientation));
                    grabBody.IsActive = true;
                    grabSpring = new FixedLinearSpring(grabBody, localMouseLocation, mouseLocation, 50 * grabBody.Mass, 15 * grabBody.Mass);
                    grabSpring.IsOnlyPull = true;
                    World.AddSpring(grabSpring);
                }
            }

            // MouseMove
            if (mouseState.LeftButton == ButtonState.Pressed)
            {
                if (grabBody != null)
                {
                    //grabSpring._worldAttachPoint = mouseLocation;
                }
            }
            // MouseUp
            else if (mouseState.LeftButton == ButtonState.Released && mousePreviousState.LeftButton == ButtonState.Pressed)
            {
                if (grabSpring != null) World.RemoveSpring(grabSpring);
                grabSpring = null;
                grabBody = null;
            }
            #endregion

            #region create random primitives
            Random r = new Random();

            if (PressedOnce(Keys.Space, Buttons.B))
            {
                Random rand = new Random();

                RigidBody body = new RigidBody(new BoxShape(5.5f, 5.5f))
                {
                    EnableDebugDraw = true,
                    //Position = new JVector((float)rand.NextDouble(), 0),
                    AngularVelocity = 0,
                    LinearVelocity = new JVector(0, -10),
                    Orientation = 0.0f,//0.001f + (float)rand.NextDouble(),
                    Material = new Material()
                    {
                        DynamicFriction = 1f,
                        StaticFriction = 1f,
                        Restitution = 0f,
                    },
                    Position = new JVector(0, 15),
                };
                World.AddBody(body);
            }
            #endregion

            #region switch through physic scenes
            if (PressedOnce(Keys.Add, Buttons.X))
            {
                DestroyCurrentScene();
                currentScene++;
                currentScene = currentScene % PhysicScenes.Count;
                PhysicScenes[currentScene].Build();
            }

            if (PressedOnce(Keys.Subtract, Buttons.Y))
            {
                DestroyCurrentScene();
                currentScene += PhysicScenes.Count - 1;
                currentScene = currentScene % PhysicScenes.Count;
                PhysicScenes[currentScene].Build();
            }
            #endregion

            UpdateDisplayText(gameTime);

            float step = (float)gameTime.ElapsedGameTime.TotalSeconds;

            if (step > 1.0f / 60.0f) step = 1.0f / 60.0f;
            World.Step(step, multithread);

            // TODO - add options for variable timestep vs fixed
            //World.Step((float)gameTime.ElapsedGameTime.TotalSeconds, multithread, 1f/ 100f, 1);

            gamePadPreviousState = padState;
            keyboardPreviousState = keyState;
            mousePreviousState = mouseState;

            base.Update(gameTime);
        }

        private void DrawJitterDebugInfo()
        {
            foreach (RigidBody body in World.RigidBodies)
            {
                DebugDrawer.Color = Color.Gray;//rndColors[cc % rndColors.Length];
                body.DebugDraw(DebugDrawer);

                //DebugDrawer.DrawAabb(body.BoundingBox.Min, body.BoundingBox.Max, Color.Pink);

                //if (body.Shape.GetType() == typeof(PolygonShape))
                //{
                //    var ch = body.Shape as PolygonShape;

                //    JMatrix o = JMatrix.CreateRotationZ(body.Orientation);
                //    foreach (var point in ch.vertices)
                //    {
                //        var t = JVector.Transform(point, o);
                //        DebugDrawer.DrawPoint(t + body.Position);
                //    }
                //}

                //DebugDrawer.Color = Color.Red;
                //foreach (Arbiter item in body.Arbiters)
                //{
                //    foreach (var contact in item.ContactList)
                //    {
                //        DebugDrawer.DrawLine(contact.Position1, contact.Position1 + contact.Axis * 0.15f);
                //        DebugDrawer.DrawLine(contact.Position2, contact.Position2 + contact.Axis * 0.15f);
                //        DebugDrawer.DrawPoint(contact.Position1);
                //        DebugDrawer.DrawPoint(contact.Position2);
                //    }
                //}
            }

            DebugDrawer.Color = Color.Blue;
            foreach (Spring spring in World.Springs)
            {
                spring.DebugDraw(DebugDrawer);
            }

            DebugDrawer.Color = Color.Red;
            foreach (Constraint c in World.Constraints)
            {
                c.DebugDraw(DebugDrawer);
            }
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(backgroundColor);

            DrawJitterDebugInfo();

            base.Draw(gameTime);
        }

        #region update the display text informations

        private float accUpdateTime = 0.0f;
        private void UpdateDisplayText(GameTime time)
        {
            accUpdateTime += (float)time.ElapsedGameTime.TotalSeconds;
            if (accUpdateTime < 0.1f) return;

            accUpdateTime = 0.0f;

            int contactCount = 0;
            foreach (Arbiter ar in World.ArbiterMap.Arbiters)
                contactCount += ar.ContactList.Count;

            Display.DisplayText[1] = World.CollisionSystem.ToString();

            Display.DisplayText[0] = "Current Scene: " + PhysicScenes[currentScene].ToString();
            //
            Display.DisplayText[2] = "Arbitercount: " + World.ArbiterMap.Arbiters.Count.ToString() + ";" + " Contactcount: " + contactCount.ToString();
            Display.DisplayText[3] = "Islandcount: " + World.Islands.Count.ToString();
            Display.DisplayText[4] = "Bodycount: " + World.RigidBodies.Count + " (" + activeBodies.ToString() + ")";
            Display.DisplayText[5] = (multithread) ? "Multithreaded" : "Single Threaded";

            int entries = (int)Jitter2D.World.DebugType.Num;
            double total = 0;

            for (int i = 0; i < entries; i++)
            {
                World.DebugType type = (World.DebugType)i;

                Display.DisplayText[8 + i] = type.ToString() + ": " +
                    ((double)World.DebugTimes[i]).ToString("0.00");

                total += World.DebugTimes[i];
            }

            Display.DisplayText[8 + entries] = "------------------------------";
            Display.DisplayText[9 + entries] = "Total Physics Time: " + total.ToString("0.00");
            Display.DisplayText[10 + entries] = "Physics Framerate: " + (1000.0d / total).ToString("0") + " fps";


            Display.DisplayText[6] = "gen0: " + GC.CollectionCount(0).ToString() +
                "  gen1: " + GC.CollectionCount(1).ToString() +
                "  gen2: " + GC.CollectionCount(2).ToString() + " total mem: " + (GC.GetTotalMemory(false) / 1024).ToString() + "KB";

        }
        #endregion
    }
}
