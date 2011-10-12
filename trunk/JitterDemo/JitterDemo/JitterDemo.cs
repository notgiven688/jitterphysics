#region Using Statements
using System;
using System.Collections.Generic;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

using Jitter;
using Jitter.Dynamics;
using Jitter.Collision;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using Jitter.Dynamics.Constraints;
using Jitter.Dynamics.Joints;
using System.Reflection;
using Jitter.Forces;
using System.Diagnostics;

using SingleBodyConstraints = Jitter.Dynamics.Constraints.SingleBody;
using System.IO;
using Jitter.DataStructures;
#endregion

namespace JitterDemo
{

    public enum BodyTag { DrawMe, DontDrawMe }

    public class JitterDemo : Microsoft.Xna.Framework.Game
    {
        private GraphicsDeviceManager graphics;

        private enum Primitives { box,sphere,cylinder,cone,capsule }

        private Primitives3D.GeometricPrimitive[] primitives =
            new Primitives3D.GeometricPrimitive[5];

        private Random random = new Random();

        private Color backgroundColor = new Color(63, 66, 73);
        private bool multithread = true;
        private int activeBodies = 0;

        private GamePadState padState;
        private KeyboardState keyState;
        private MouseState mouseState;

        public Camera Camera { private set; get; }
        public Display Display { private set; get; }
        public DebugDrawer DebugDrawer { private set; get; }
        public BasicEffect BasicEffect { private set; get; }
        public List<Scenes.Scene> PhysicScenes { private set; get;   }
        public World World { private set; get; }

        private int currentScene = 0;

        RasterizerState wireframe, cullMode,normal;

        Color[] rndColors;

        public JitterDemo()
        {
            this.IsMouseVisible = true;
            graphics = new GraphicsDeviceManager(this);

            graphics.GraphicsProfile = GraphicsProfile.HiDef;
            graphics.PreferMultiSampling = true;

            Content.RootDirectory = "Content";

            graphics.PreferredBackBufferHeight = 600;
            graphics.PreferredBackBufferWidth = 800;

            this.IsFixedTimeStep = false;
            this.graphics.SynchronizeWithVerticalRetrace = false;

            CollisionSystem collision = new CollisionSystemPersistentSAP();

            World = new World(collision); World.AllowDeactivation = true;

            this.Window.AllowUserResizing = true;

#if(WINDOWS)
            this.Window.Title = "Jitter Physics Demo - Jitter "
                + Assembly.GetAssembly(typeof(Jitter.World)).GetName().Version.ToString();
#else
            this.Window.Title = "Jitter Physics Demo - Jitter";
#endif


            Random rr = new Random();
            rndColors = new Color[20];

            for (int i = 0; i < 20; i++)
            {
                rndColors[i] = new Color((float)rr.NextDouble(), (float)rr.NextDouble(), (float)rr.NextDouble());
            }


            wireframe = new RasterizerState();
            wireframe.FillMode = FillMode.WireFrame;

            cullMode = new RasterizerState();
            cullMode.CullMode = CullMode.None;

 
            normal = new RasterizerState();
        }

        protected override void Initialize()
        {
            Camera = new Camera(this);
            Camera.Position = new Vector3(15, 15, 30);
            Camera.Target = Camera.Position + Vector3.Normalize(new Vector3(10, 5, 20));
            this.Components.Add(Camera);

            DebugDrawer = new DebugDrawer(this);
            this.Components.Add(DebugDrawer);

            Display = new Display(this);
            Display.DrawOrder = int.MaxValue;
            this.Components.Add(Display);

            primitives[(int)Primitives.box] = new Primitives3D.BoxPrimitive(GraphicsDevice);
            primitives[(int)Primitives.capsule] = new Primitives3D.CapsulePrimitive(GraphicsDevice);
            primitives[(int)Primitives.cone] = new Primitives3D.ConePrimitive(GraphicsDevice);
            primitives[(int)Primitives.cylinder] = new Primitives3D.CylinderPrimitive(GraphicsDevice);
            primitives[(int)Primitives.sphere] = new Primitives3D.SpherePrimitive(GraphicsDevice);

            BasicEffect = new BasicEffect(GraphicsDevice);
            BasicEffect.EnableDefaultLighting();
            BasicEffect.PreferPerPixelLighting = true;

            this.PhysicScenes = new List<Scenes.Scene>();


            foreach (Type type in Assembly.GetExecutingAssembly().GetTypes())
            {
                if (type.Namespace == "JitterDemo.Scenes" && !type.IsAbstract)
                {
                    if (type.Name == "SoftBodyJenga") currentScene = PhysicScenes.Count;
                    Scenes.Scene scene = (Scenes.Scene)Activator.CreateInstance(type, this);
                    this.PhysicScenes.Add(scene);
                }
            }

            if (PhysicScenes.Count > 0)
                this.PhysicScenes[currentScene].Build();

            base.Initialize();

        }


        private Vector3 RayTo(int x, int y)
        {
            Vector3 nearSource = new Vector3(x, y, 0);
            Vector3 farSource = new Vector3(x, y, 1);

            Matrix world = Matrix.Identity;

            Vector3 nearPoint = graphics.GraphicsDevice.Viewport.Unproject(nearSource, Camera.Projection, Camera.View, world);
            Vector3 farPoint = graphics.GraphicsDevice.Viewport.Unproject(farSource, Camera.Projection, Camera.View, world);

            Vector3 direction = farPoint - nearPoint;
            return direction;
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


        #region update - global variables
        // Hold previous input states.
        KeyboardState keyboardPreviousState = new KeyboardState();
        GamePadState gamePadPreviousState = new GamePadState();
        MouseState mousePreviousState = new MouseState();

        // Store information for drag and drop
        JVector hitPoint, hitNormal;
        SingleBodyConstraints.PointOnPoint grabConstraint;
        RigidBody grabBody;
        float hitDistance = 0.0f;
        int scrollWheel = 0;
        #endregion

        protected override void Update(GameTime gameTime)
        {
            padState = GamePad.GetState(PlayerIndex.One);
            keyState = Keyboard.GetState();
            mouseState = Mouse.GetState();

            // let the user escape the demo
            if (PressedOnce(Keys.Escape, Buttons.Back)) this.Exit();

            // change threading mode
            if (PressedOnce(Keys.M, Buttons.A)) multithread = !multithread;

            if (PressedOnce(Keys.P,Buttons.A))
            {
                var e = World.RigidBodies.GetEnumerator();
                e.MoveNext(); e.MoveNext();
                World.RemoveBody(e.Current as RigidBody);
            }

            #region drag and drop physical objects with the mouse
            if (mouseState.LeftButton == ButtonState.Pressed &&
                mousePreviousState.LeftButton == ButtonState.Released ||
                padState.IsButtonDown(Buttons.RightThumbstickDown) &&
                gamePadPreviousState.IsButtonUp(Buttons.RightThumbstickUp))
            {
                JVector ray = Conversion.ToJitterVector(RayTo(mouseState.X, mouseState.Y));
                JVector camp = Conversion.ToJitterVector(Camera.Position);

                ray = JVector.Normalize(ray) * 100;

                float fraction;

                bool result = World.CollisionSystem.Raycast(camp, ray, RaycastCallback, out grabBody, out hitNormal, out fraction);

                if (result)
                {
                    hitPoint = camp + fraction * ray;

                    if (grabConstraint != null) World.RemoveConstraint(grabConstraint);

                    JVector lanchor = hitPoint - grabBody.Position;
                    lanchor = JVector.Transform(lanchor, JMatrix.Transpose(grabBody.Orientation));

                    grabConstraint = new SingleBodyConstraints.PointOnPoint(grabBody, lanchor);
                    grabConstraint.Softness = 0.01f;
                    grabConstraint.BiasFactor = 0.1f;
                    
                    World.AddConstraint(grabConstraint);
                    hitDistance = (Conversion.ToXNAVector(hitPoint) - Camera.Position).Length();
                    scrollWheel = mouseState.ScrollWheelValue;
                    grabConstraint.Anchor = hitPoint;
                }
            }

            if (mouseState.LeftButton == ButtonState.Pressed || padState.IsButtonDown(Buttons.RightThumbstickDown))
            {
                hitDistance += (mouseState.ScrollWheelValue - scrollWheel) * 0.01f;
                scrollWheel = mouseState.ScrollWheelValue;

                if (grabBody != null)
                {
                    Vector3 ray = RayTo(mouseState.X, mouseState.Y); ray.Normalize();
                    grabConstraint.Anchor = Conversion.ToJitterVector(Camera.Position + ray * hitDistance);
                    grabBody.IsActive = true;
                    if (!grabBody.IsStatic)
                    {
                        grabBody.LinearVelocity *= 0.98f;
                        grabBody.AngularVelocity *= 0.98f;
                    }
                }
            }
            else
            {
                if (grabConstraint != null) World.RemoveConstraint(grabConstraint);
                grabBody = null;
                grabConstraint = null;
            }
            #endregion

            #region create random primitives

            if (PressedOnce(Keys.Space,Buttons.B))
            {
                SpawnRandomPrimitive(Conversion.ToJitterVector(Camera.Position),
                    Conversion.ToJitterVector((Camera.Target - Camera.Position) * 40.0f));

            }
            #endregion

            #region switch through physic scenes
            if (PressedOnce(Keys.Add,Buttons.X))
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

            if(step > 1.0f / 100.0f) step = 1.0f / 100.0f;
            World.Step(step,multithread);
   
            gamePadPreviousState = padState;
            keyboardPreviousState = keyState;
            mousePreviousState = mouseState;

            base.Update(gameTime);
        }



        private bool RaycastCallback(RigidBody body, JVector normal, float fraction)
        {
            if (body.IsStatic) return false;
            else return true;
        }

        RigidBody lastBody = null;

        #region Spawn Random Primitive
        private void SpawnRandomPrimitive(JVector position, JVector velocity)
        {
            RigidBody body = null;
            int rndn = rndn = random.Next(7);

            // less of the more advanced objects
            if (rndn == 5 || rndn == 6) rndn = random.Next(7);

            switch (rndn)
            {
                case 0:
                    body = new RigidBody(new ConeShape((float)random.Next(5, 50) / 20.0f, (float)random.Next(10, 20) / 20.0f));
                    break;
                case 1:
                    body = new RigidBody(new BoxShape((float)random.Next(10, 30) / 20.0f, (float)random.Next(10, 30) / 20.0f, (float)random.Next(10, 30) / 20.0f));
                    break;
                case 2:
                    body = new RigidBody(new SphereShape(0.4f));
                    break;
                case 3:
                    body = new RigidBody(new CylinderShape(1.0f, 0.5f));
                    break;
                case 4:
                    body = new RigidBody(new CapsuleShape(1.0f, 0.5f));
                    break;
                case 5:
                    Shape b1 = new BoxShape(new JVector(3, 1, 1));
                    Shape b2 = new BoxShape(new JVector(1, 1, 3));
                    Shape b3 = new CylinderShape(3.0f, 0.5f);

                    CompoundShape.TransformedShape t1 = new CompoundShape.TransformedShape(b1, JMatrix.Identity, JVector.Zero);
                    CompoundShape.TransformedShape t2 = new CompoundShape.TransformedShape(b2, JMatrix.Identity, JVector.Zero);
                    CompoundShape.TransformedShape t3 = new CompoundShape.TransformedShape(b3, JMatrix.Identity, new JVector(0, 0, 0));

                    CompoundShape ms = new CompoundShape(new CompoundShape.TransformedShape[3] { t1, t2, t3 });

                    body = new RigidBody(ms);
                    break;
                case 6:
                    ConvexHullObject obj2 = new ConvexHullObject(this);
                    Components.Add(obj2);
                    body = obj2.body;
                    body.Material.Restitution = 0.2f;
                    body.Material.StaticFriction = 0.8f;
                    break;
            }

            World.AddBody(body);
            //body.IsParticle = true;
           // body.EnableSpeculativeContacts = true;
            body.Position = position;
            body.LinearVelocity = velocity;
            lastBody = body;
        }
        #endregion

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

            int entries = (int)Jitter.World.DebugType.Num;
            double total = 0;

            for (int i = 0; i < entries; i++)
            {
                World.DebugType type = (World.DebugType)i;

                Display.DisplayText[8 + i] = type.ToString() + ": " +
                    ((double)World.DebugTimes[i]).ToString("0.00");

                total += World.DebugTimes[i];
            }

            Display.DisplayText[8+entries] = "------------------------------";
            Display.DisplayText[9 + entries] = "Total Physics Time: " + total.ToString("0.00");
            Display.DisplayText[10 + entries] = "Physics Framerate: " + (1000.0d / total).ToString("0") + " fps";

#if(WINDOWS)
            Display.DisplayText[6] = "gen0: " + GC.CollectionCount(0).ToString() +
                "  gen1: " + GC.CollectionCount(1).ToString() +
                "  gen2: " + GC.CollectionCount(2).ToString();
#endif

        }
        #endregion

        #region add draw matrices to the different primitives
        private void AddShapeToDrawList(Shape shape, JMatrix ori, JVector pos)
        {
            Primitives3D.GeometricPrimitive primitive = null;
            Matrix scaleMatrix = Matrix.Identity;

            if (shape is BoxShape)
            {
                primitive = primitives[(int)Primitives.box];
                scaleMatrix = Matrix.CreateScale(Conversion.ToXNAVector((shape as BoxShape).Size));
            }
            else if (shape is SphereShape)
            {
                primitive = primitives[(int)Primitives.sphere];
                scaleMatrix = Matrix.CreateScale((shape as SphereShape).Radius);
            }
            else if (shape is CylinderShape)
            {
                primitive = primitives[(int)Primitives.cylinder];
                CylinderShape cs = shape as CylinderShape;
                scaleMatrix = Matrix.CreateScale(cs.Radius, cs.Height, cs.Radius);
            }
            else if (shape is CapsuleShape)
            {
                primitive = primitives[(int)Primitives.capsule];
                CapsuleShape cs = shape as CapsuleShape;
                scaleMatrix = Matrix.CreateScale(cs.Radius * 2, cs.Length, cs.Radius * 2);

            }
            else if (shape is ConeShape)
            {
                ConeShape cs = shape as ConeShape;
                scaleMatrix = Matrix.CreateScale(cs.Radius, cs.Height, cs.Radius);
                primitive = primitives[(int)Primitives.cone];
            }

            if(primitive != null)
            primitive.AddWorldMatrix(scaleMatrix * Conversion.ToXNAMatrix(ori) *
                        Matrix.CreateTranslation(Conversion.ToXNAVector(pos)));
        }

        private void AddBodyToDrawList(RigidBody rb)
        {
            if (rb.Tag is BodyTag && ((BodyTag)rb.Tag) == BodyTag.DontDrawMe) return;

            bool isCompoundShape = (rb.Shape is CompoundShape);

            if (!isCompoundShape)
            {
                AddShapeToDrawList(rb.Shape, rb.Orientation, rb.Position);
            }
            else
            {
                CompoundShape cShape = rb.Shape as CompoundShape;
                JMatrix orientation = rb.Orientation;
                JVector position = rb.Position;

                foreach (var ts in cShape.Shapes)
                {
                    JVector pos = ts.Position;
                    JMatrix ori = ts.Orientation;

                    JVector.Transform(ref pos,ref orientation,out pos);
                    JVector.Add(ref pos, ref position, out pos);

                    JMatrix.Multiply(ref ori, ref orientation, out ori);

                    AddShapeToDrawList(ts.Shape, ori, pos);
                }

            }

        }
        #endregion

        #region draw jitter debug data


        private void DrawJitterDebugInfo()
        {
            int cc = 0;

            foreach (Constraint constr in World.Constraints)
                constr.DebugDraw(DebugDrawer);

            foreach (RigidBody body in World.RigidBodies)
            {
                DebugDrawer.Color = rndColors[cc % rndColors.Length];
                body.DebugDraw(DebugDrawer);
                cc++;
            }
        }

        private void Walk(DynamicTree<SoftBody.Triangle> tree, int index)
        {
            DynamicTreeNode<SoftBody.Triangle> tn = tree.Nodes[index];
            if (tn.IsLeaf()) return;
            else
            {
                Walk(tree,tn.Child1);
                Walk(tree,tn.Child2);

                DebugDrawer.DrawAabb(tn.AABB.Min,tn.AABB.Max, Color.Red);
            }
        }

        private void DrawDynamicTree(SoftBody cloth)
        {
            Walk(cloth.DynamicTree,cloth.DynamicTree.Root);
        }


        private void DrawIslands()
        {
            JBBox box;

            foreach (CollisionIsland island in World.Islands)
            {
                box = JBBox.SmallBox;

                foreach (RigidBody body in island.Bodies)
                {
                    box = JBBox.CreateMerged(box, body.BoundingBox);
                }

                DebugDrawer.DrawAabb(box.Min, box.Max, island.IsActive() ? Color.Green : Color.Yellow);

            }
        }
        #endregion

        #region Draw Cloth

        private void DrawCloth()
        {
     
            foreach (SoftBody body in World.SoftBodies)
            {
                if (body.Tag is BodyTag && ((BodyTag)body.Tag) == BodyTag.DontDrawMe) return;

                for (int i = 0; i < body.Triangles.Count; i++)
                {
                    DebugDrawer.DrawTriangle(body.Triangles[i].VertexBody1.Position,
                        body.Triangles[i].VertexBody2.Position,
                        body.Triangles[i].VertexBody3.Position,
                        new Color(0, 0.95f, 0, 0.5f));
                }
                //DrawDynamicTree(body);
            }
        }
        #endregion


        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(backgroundColor);
            GraphicsDevice.DepthStencilState = DepthStencilState.Default;
           
            BasicEffect.View = Camera.View;
            BasicEffect.Projection = Camera.Projection;

            activeBodies = 0;

            // Draw all shapes
            foreach (RigidBody body in World.RigidBodies)
            {
                if (body.IsActive) activeBodies++;
                if (body.Tag is int || body.IsParticle) continue;
                AddBodyToDrawList(body);
            }

            BasicEffect.DiffuseColor = Color.LightGray.ToVector3();

            DrawCloth();

            PhysicScenes[currentScene].Draw();

            // Draw the debug data provided by Jitter
            // DrawIslands();
            DrawJitterDebugInfo();

            #region Debug Draw All Contacts
            //foreach (Arbiter a in World.ArbiterMap)
            //{
            //    foreach (Contact c in a.ContactList)
            //    {
            //        DebugDrawer.DrawLine(c.Position1 + 0.5f * JVector.Left, c.Position1 + 0.5f * JVector.Right, Color.Green);
            //        DebugDrawer.DrawLine(c.Position1 + 0.5f * JVector.Up, c.Position1 + 0.5f * JVector.Down, Color.Green);
            //        DebugDrawer.DrawLine(c.Position1 + 0.5f * JVector.Forward, c.Position1 + 0.5f * JVector.Backward, Color.Green);


            //        DebugDrawer.DrawLine(c.Position2 + 0.5f * JVector.Left, c.Position2 + 0.5f * JVector.Right, Color.Red);
            //        DebugDrawer.DrawLine(c.Position2 + 0.5f * JVector.Up, c.Position2 + 0.5f * JVector.Down, Color.Red);
            //        DebugDrawer.DrawLine(c.Position2 + 0.5f * JVector.Forward, c.Position2 + 0.5f * JVector.Backward, Color.Red);
            //    }
            //}
            #endregion

            foreach (Primitives3D.GeometricPrimitive prim in primitives) prim.Draw(BasicEffect);

            GraphicsDevice.RasterizerState = cullMode;

            base.Draw(gameTime);

            GraphicsDevice.RasterizerState = normal;
        }


    }
}
