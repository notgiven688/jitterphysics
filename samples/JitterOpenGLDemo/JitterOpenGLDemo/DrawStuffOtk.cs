
#region LICENSE

/*  Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
    All rights reserved.  Email: russ@q12.org   Web: http://www.q12.org
    Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke
    All rights reserved.  Email: ode4j@gmx.de   Web: http://www.ode4j.org
    CsODE, Copyright (C) 2011 Miguel Angel Guirado López.
    All rights reserved.  Email: bitiopiasite@gmail.com
    Web: http://www.bitiopia.com  
 
    This library is free software; you can redistribute it and/or
    modify it under the terms of EITHER:                         
     (1) The GNU Lesser General Public License as published by the Free
         Software Foundation; either version 2.1 of the License, or (at
         your option) any later version. The text of the GNU Lesser   
         General Public License is included with this library in the  
         file LICENSE.TXT.                                            
     (2) The BSD-style license that is included with this library in  
         the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT and
         CsODE-LICENSE-BSD.TXT.      
                                                                     
    This library is distributed in the hope that it will be useful,    
    but WITHOUT ANY WARRANTY; without even the implied warranty of     
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files 
    LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT and
    CsODE-LICENSE-BSD.TXT for more details. */

#endregion LICENSE

using System;
using System.IO;
using System.Reflection;
using System.Drawing;
using System.Drawing.Imaging;

using OpenTK;
using OpenTK.Input;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using Jitter.LinearMath;

namespace JitterOpenGLDemo
{
    public sealed class Conversion
    {
        public static float[] ToFloat(JVector vector)
        {
            return new float[4] { vector.X, vector.Y, vector.Z, 0.0f };
        }

        public static float[] ToFloat(JMatrix matrix)
        {
            return new float[12] { matrix.M11, matrix.M21, matrix.M31, 0.0f,
                                   matrix.M12, matrix.M22, matrix.M32, 0.0f,
                                   matrix.M13, matrix.M23, matrix.M33, 1.0f };
        }
    }

    public class DrawStuffOtk : GameWindow
    {
        Color clearColor = Color.FromArgb(20, 168, 57);
        MouseState current, previous;
        private const double M_PI = Math.PI;

        private const double DEG_TO_RAD = Math.PI / 180.0;
        // light vector. LIGHTZ is implicitly 1
        private const float LIGHTX = 1.0f;
        private const float LIGHTY = 0.4f;

        // ground color for when there's no texture
        private const float GROUND_R = 0.5f;
        private const float GROUND_G = 0.5f;
        private const float GROUND_B = 0.3f;

        // ground and sky
        private const float SHADOW_INTENSITY = 0.65f;

        private const float ground_scale = 1.0f / 1.0f;	// ground texture scale (1/size)
        private const float ground_ofsx = 0.5f;		// offset of ground texture
        private const float ground_ofsy = 0.5f;
        private const float sky_scale = 1.0f / 4.0f;	// sky texture scale (1/size)
        private const float sky_height = 1.0f;		// sky height above viewpoint

        protected double fieldOfView = 45.0f;
        protected double nearClipDistance = 0.1f;
        protected double farClipDistance = 5000.0f;

        private float offset = 0.0f;

        // the current state:
        //    0 = uninitialized
        //    1 = dsSimulationLoop() called
        //    2 = dsDrawFrame() called
        private int current_state = 0;

        // textures and shadows
        private bool use_textures = true;		// 1 if textures to be drawn
        private bool use_shadows = true;		// 1 if shadows to be drawn
        private Texture sky_texture = null;
        protected Texture ground_texture = null;
        private Texture wood_texture = null;
        private Texture checkered_texture = null;
        protected Texture grass_texture = null;

        private Texture[] texture = new Texture[5 + 1]; // +1 since index 0 is not used

        private float[] color = { 0, 0, 0, 0 };	// current r,g,b,alpha color
        private DS_TEXTURE_NUMBER tnum = DS_TEXTURE_NUMBER.DS_NONE;

        private float[] s_params_SDM;
        private float[] t_params_SDM;

        private float[] light_ambient2;
        private float[] light_diffuse2;
        private float[] light_specular2;

        private float[] light_position = new float[] { LIGHTX, LIGHTY, 1.0f, 0.0f };
        private float[] light_ambient = new float[] { 0.5f, 0.5f, 0.5f, 1.0f };
        private float[] light_diffuse = new float[] { 1.0f, 1.0f, 1.0f, 1.0f };
        private float[] light_specular = new float[] { 1.0f, 1.0f, 1.0f, 1.0f };

        private readonly float[] s_params_SSDM;
        private readonly float[] t_params_SSDM;

        // current camera position and orientation
        private float[] view_xyz = new float[3];	// position x,y,z
        private float[] view_hpr = new float[3];	// heading, pitch, roll (degrees)

        #region Constructores

        #region public GameBase()

        /// <summary>Constructs a new GameWindow with sensible default attributes.</summary>
        public DrawStuffOtk()
            : this(640, 480, GraphicsMode.Default, "OpenTK Game Window", 0, DisplayDevice.Default) { }

        #endregion

        #region public GameBase(int width, int height)

        /// <summary>Constructs a new GameWindow with the specified attributes.</summary>
        /// <param Name="width">The width of the GameWindow in pixels.</param>
        /// <param Name="height">The height of the GameWindow in pixels.</param>
        public DrawStuffOtk(int width, int height)
            : this(width, height, GraphicsMode.Default, "OpenTK Game Window", 0, DisplayDevice.Default) { }

        #endregion

        #region public GameBase(int width, int height, GraphicsMode mode)

        /// <summary>Constructs a new GameWindow with the specified attributes.</summary>
        /// <param Name="width">The width of the GameWindow in pixels.</param>
        /// <param Name="height">The height of the GameWindow in pixels.</param>
        /// <param Name="mode">The OpenTK.Graphic.GraphicsMode of the GameWindow.</param>
        public DrawStuffOtk(int width, int height, GraphicsMode mode)
            : this(width, height, mode, "OpenTK Game Window", 0, DisplayDevice.Default) { }

        #endregion

        #region public GameBase(int width, int height, GraphicsMode mode, string title)

        /// <summary>Constructs a new GameWindow with the specified attributes.</summary>
        /// <param Name="width">The width of the GameWindow in pixels.</param>
        /// <param Name="height">The height of the GameWindow in pixels.</param>
        /// <param Name="mode">The OpenTK.Graphic.GraphicsMode of the GameWindow.</param>
        /// <param Name="title">The title of the GameWindow.</param>
        public DrawStuffOtk(int width, int height, GraphicsMode mode, string title)
            : this(width, height, mode, title, 0, DisplayDevice.Default) { }

        #endregion

        #region public GameBase(int width, int height, GraphicsMode mode, string title, GameWindowFlags options)

        /// <summary>Constructs a new GameWindow with the specified attributes.</summary>
        /// <param Name="width">The width of the GameWindow in pixels.</param>
        /// <param Name="height">The height of the GameWindow in pixels.</param>
        /// <param Name="mode">The OpenTK.Graphic.GraphicsMode of the GameWindow.</param>
        /// <param Name="title">The title of the GameWindow.</param>
        /// <param Name="options">GameWindow options regarding window appearance and behavior.</param>
        public DrawStuffOtk(int width, int height, GraphicsMode mode, string title, GameWindowFlags options)
            : this(width, height, mode, title, options, DisplayDevice.Default) { }

        #endregion

        #region public GameBase(int width, int height, GraphicsMode mode, string title, GameWindowFlags options, DisplayDevice device)

        /// <summary>Constructs a new GameWindow with the specified attributes.</summary>
        /// <param Name="width">The width of the GameWindow in pixels.</param>
        /// <param Name="height">The height of the GameWindow in pixels.</param>
        /// <param Name="mode">The OpenTK.Graphic.GraphicsMode of the GameWindow.</param>
        /// <param Name="title">The title of the GameWindow.</param>
        /// <param Name="options">GameWindow options regarding window appearance and behavior.</param>
        /// <param Name="device">The OpenTK.Graphic.DisplayDevice to construct the GameWindow in.</param>
        public DrawStuffOtk(int width, int height, GraphicsMode mode, string title, GameWindowFlags options, DisplayDevice device)
            : this(width, height, mode, title, options, device, 1, 0, GraphicsContextFlags.Default)
        { }

        #endregion

        #region public GameBase(int width, int height, GraphicsMode mode, string title, GameWindowFlags options, DisplayDevice device, int major, int minor, GraphicsContextFlags flags)

        /// <summary>Constructs a new GameWindow with the specified attributes.</summary>
        /// <param Name="width">The width of the GameWindow in pixels.</param>
        /// <param Name="height">The height of the GameWindow in pixels.</param>
        /// <param Name="mode">The OpenTK.Graphic.GraphicsMode of the GameWindow.</param>
        /// <param Name="title">The title of the GameWindow.</param>
        /// <param Name="options">GameWindow options regarding window appearance and behavior.</param>
        /// <param Name="device">The OpenTK.Graphic.DisplayDevice to construct the GameWindow in.</param>
        /// <param Name="major">The major version for the OpenGL GraphicsContext.</param>
        /// <param Name="minor">The minor version for the OpenGL GraphicsContext.</param>
        /// <param Name="flags">The GraphicsContextFlags version for the OpenGL GraphicsContext.</param>
        public DrawStuffOtk(int width, int height, GraphicsMode mode, string title, GameWindowFlags options, DisplayDevice device,
            int major, int minor, GraphicsContextFlags flags)
            : this(width, height, mode, title, options, device, major, minor, flags, null)
        { }

        #endregion

        public DrawStuffOtk(int width, int height, GraphicsMode mode, string title, GameWindowFlags options, DisplayDevice device,
                        int major, int minor, GraphicsContextFlags flags, IGraphicsContext sharedContext)
            : base(width, height, mode, title, options, device, major, minor, flags, sharedContext)
        {
            s_params_SDM = new float[] { 1.0f, 1.0f, 0.0f, 1 };
            t_params_SDM = new float[] { 0.817f, -0.817f, 0.817f, 1 };

            s_params_SSDM = new float[] { ground_scale, 0, 0, ground_ofsx };
            t_params_SSDM = new float[] { 0, ground_scale, 0, ground_ofsy };

            Mouse.ButtonDown += new EventHandler<OpenTK.Input.MouseButtonEventArgs>(Mouse_ButtonDown);
            Mouse.ButtonUp += new EventHandler<OpenTK.Input.MouseButtonEventArgs>(Mouse_ButtonUp);
        }
        #endregion Constructores

        bool buttonDownLeft = false;
        bool buttonDownMiddle = false;
        bool buttonDownRight = false;

        void Mouse_ButtonDown(object sender, OpenTK.Input.MouseButtonEventArgs e)
        {
            if (e.Button == OpenTK.Input.MouseButton.Left)
                buttonDownLeft = true;
            else if (e.Button == OpenTK.Input.MouseButton.Middle)
                buttonDownMiddle = true;
            else if (e.Button == OpenTK.Input.MouseButton.Right)
                buttonDownRight = true;
        }
        void Mouse_ButtonUp(object sender, OpenTK.Input.MouseButtonEventArgs e)
        {
            if (e.Button == OpenTK.Input.MouseButton.Left)
                buttonDownLeft = false;
            else if (e.Button == OpenTK.Input.MouseButton.Middle)
                buttonDownMiddle = false;
            else if (e.Button == OpenTK.Input.MouseButton.Right)
                buttonDownRight = false;
        }

        protected void gluPerspective(double fovy, double aspect, double zNear, double zFar)
        {
            double xmin, xmax, ymin, ymax;

            ymax = zNear * Math.Tan(fovy * Math.PI / 360.0);
            ymin = -ymax;

            xmin = ymin * aspect;
            xmax = ymax * aspect;

            GL.Frustum(xmin, xmax, ymin, ymax, zNear, zFar);
        }

        #region OnXXX of GameWindow

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            current_state = 1;
            initMotionModel();

            // StartGraphics
            Stream stmChecker;
            Stream stmGround;
            Stream stmSky;
            Stream stmWood;
            Stream stmGrass;

            try
            {
                Assembly a = Assembly.GetExecutingAssembly();
                // Para ver los nombres de los recursos incrustados
                String[] nombres = a.GetManifestResourceNames(); 
                stmChecker = a.GetManifestResourceStream("JitterOpenGLDemo.Resources.checkered.png");
                stmGround = a.GetManifestResourceStream("JitterOpenGLDemo.Resources.ground.png");
                stmSky = a.GetManifestResourceStream("JitterOpenGLDemo.Resources.sky.png");
                stmWood = a.GetManifestResourceStream("JitterOpenGLDemo.Resources.wood.png");
                stmGrass = a.GetManifestResourceStream("JitterOpenGLDemo.Resources.grass.jpeg");
            }
            catch
            {
                throw new Exception("Error accessing resources!");
            }
            sky_texture = new Texture(stmSky);
            texture[(int)DS_TEXTURE_NUMBER.DS_SKY] = sky_texture;

            //		strcpy (s,prefix);
            //		strcat (s,"/ground.ppm");
            ground_texture = new Texture(stmGround);
            texture[(int)DS_TEXTURE_NUMBER.DS_GROUND] = ground_texture;

            //		strcpy (s,prefix);
            //		strcat (s,"/wood.ppm");
            wood_texture = new Texture(stmWood);
            texture[(int)DS_TEXTURE_NUMBER.DS_WOOD] = wood_texture;

            //		strcpy (s,prefix);
            //		strcat (s,"/checkered.ppm");
            checkered_texture = new Texture(stmChecker);
            texture[(int)DS_TEXTURE_NUMBER.DS_CHECKERED] = checkered_texture;

            grass_texture = new Texture(stmGrass);
            texture[(int)DS_TEXTURE_NUMBER.DS_GRASS] = grass_texture;

            ground_texture = grass_texture;
        }
        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);

            if (Width != 0 && Height != 0)
            {
                GL.Viewport(0, 0, Width, Height);
                double aspectRatio = Width / (double)Height;
                // Set projection
                GL.MatrixMode(MatrixMode.Projection);
                GL.LoadIdentity();
                gluPerspective(fieldOfView, aspectRatio, nearClipDistance, farClipDistance);
            }
        }
        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);

            //handleKeyboard(fn);
            updateMouseState();

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            dsDrawFrame(Width, Height, false);

            OnBeginRender(e.Time);
            OnEndRender();

            SwapBuffers();
        }
        /// <summary>
        /// Virtual pura
        /// </summary>
        protected virtual void OnBeginRender(double elapsedTime) { }
        /// <summary>
        /// Virtual pura
        /// </summary>
        protected virtual void OnEndRender() { }

        #endregion OnXXX of GameWindow

        private void normalizeVector3(float[] v)//[3])
        {
            float len = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if (len <= 0.0f)
            {
                v[0] = 1;
                v[1] = 0;
                v[2] = 0;
            }
            else
            {
                len = 1.0f / (float)Math.Sqrt(len);
                v[0] *= len;
                v[1] *= len;
                v[2] *= len;
            }
        }
        protected override void OnDisposed(EventArgs e)
        {
            base.OnDisposed(e);

            sky_texture.finalize();
            grass_texture.finalize();
            ground_texture.finalize();
            wood_texture.finalize();
            checkered_texture.finalize();
        }
        private void updateMouseState()
        {
            int dx = 0;
            int dy = 0;
            int dw = 0;

            current = OpenTK.Input.Mouse.GetState();
            if (current != previous)
            {
                // Mouse state has changed
                dx = current.X - previous.X;
                dy = current.Y - previous.Y;
                dw = current.Wheel - previous.Wheel;
            }
            previous = current;


            // get out if no movement
            if (dx == dy && dx == 0 && dw == 0)
            {
                return;
            }

            //LWJGL: 0=left 1=right 2=middle
            //GL: 0=left 1=middle 2=right

            int mode = 0;
            if (buttonDownLeft)
                mode |= 1;
            else if (buttonDownMiddle)
                mode |= 2;
            else if (buttonDownRight)
                mode |= 4;
            if (mode != 0)
            {
                //LWJGL has inverted dy wrt C++/GL
                dsMotion(mode, dx, dy);
            }
        }
        protected void dsSetClearColor(byte red, byte green, byte blue)
        {
            clearColor = Color.FromArgb(red, green, blue);
        }
        void dsMotion(int mode, int deltax, int deltay)
        {
            float side = 0.01f * (float)deltax;
            float fwd = (mode == 4) ? (0.01f * (float)deltay) : 0.0f;
            float s = (float)Math.Sin(view_hpr[0] * DEG_TO_RAD);
            float c = (float)Math.Cos(view_hpr[0] * DEG_TO_RAD);

            if (mode == 1)
            {
                view_hpr[0] += (float)(deltax) * -0.5f;
                view_hpr[1] += (float)(deltay) * -0.5f;
            }
            else
            {
                view_xyz[0] += -s * side + c * fwd;
                view_xyz[1] += c * side + s * fwd;
                if (mode == 2 || mode == 5)
                    view_xyz[2] += 0.01f * (float)(deltay);
            }
            wrapCameraAngles();
        }
        private void wrapCameraAngles()
        {
            for (int i = 0; i < 3; i++)
            {
                while (view_hpr[i] > 180) view_hpr[i] -= 360;
                while (view_hpr[i] < -180) view_hpr[i] += 360;
            }
        }
        void dsDrawFrame(int width, int height, bool pause)
        {
            if (current_state < 1)
                throw new Exception("internal error");
            current_state = 2;

            // setup stuff
            GL.Enable(EnableCap.Lighting);
            GL.Enable(EnableCap.Light0);
            GL.Disable(EnableCap.Texture2D);
            GL.Disable(EnableCap.TextureGenS);
            GL.Disable(EnableCap.TextureGenT);
            GL.ShadeModel(ShadingModel.Flat);
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Less);
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.FrontFace(FrontFaceDirection.Ccw);

            // setup viewport
            GL.Viewport(0, 0, width, height);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            float vnear = 0.1f;
            float vfar = 1000.0f;
            float k = 0.8f;     // view scale, 1 = +/- 45 degrees
            if (width >= height)
            {
                float k2 = (float)height / (float)width;
                GL.Frustum(-vnear * k, vnear * k, -vnear * k * k2, vnear * k * k2, vnear, vfar);
            }
            else
            {
                float k2 = (float)width / (float)height;
                GL.Frustum(-vnear * k * k2, vnear * k * k2, -vnear * k, vnear * k, vnear, vfar);
            }

            // setup lights. it makes a difference whether this is done in the
            // GL_PROJECTION matrix mode (lights are scene relative) or the
            // GL_MODELVIEW matrix mode (lights are camera relative, bad!).
            //		static GLfloat light_ambient[] = { 0.5, 0.5, 0.5, 1.0 };
            //		static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
            //		static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
            GL.Light(LightName.Light0, LightParameter.Ambient, light_ambient);
            GL.Light(LightName.Light0, LightParameter.Diffuse, light_diffuse);
            GL.Light(LightName.Light0, LightParameter.Specular, light_specular);
            GL.Color3(1.0f, 1.0f, 1.0f);

            // clear the window
            //GL.ClearColor(0.5f, 0.5f, 0.5f, 0);
            GL.ClearColor(clearColor.R / 255f, clearColor.G / 255f, clearColor.B / 255f, 0);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            // snapshot camera position (in MS Windows it is changed by the GUI thread)
            float[] view2_xyz = (float[])view_xyz.Clone();
            float[] view2_hpr = (float[])view_hpr.Clone();
            //		memcpy (view2_xyz,view_xyz);//,sizeof(float)*3);
            //		memcpy (view2_hpr,view_hpr);//,sizeof(float)*3);

            // go to GL_MODELVIEW matrix mode and set the camera
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadIdentity();
            setCamera(view2_xyz[0], view2_xyz[1], view2_xyz[2],
                    view2_hpr[0], view2_hpr[1], view2_hpr[2]);

            // set the light position (for some reason we have to do this in model view.
            //		static GLfloat light_position[] = { LIGHTX, LIGHTY, 1.0, 0.0 };
            GL.Light(LightName.Light0, LightParameter.Position, light_position);

            // draw the background (ground, sky etc)
            drawSky(view2_xyz);
            drawGround();

            // draw the little markers on the ground
            drawPyramidGrid();

            // leave openGL in a known state - flat shaded white, no textures
            GL.Enable(EnableCap.Lighting);
            GL.Disable(EnableCap.Texture2D);
            GL.ShadeModel(ShadingModel.Flat);
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Less);
            GL.Color3(1.0f, 1.0f, 1.0f);
            setColor(1.0f, 1.0f, 1.0f, 1.0f);

            // draw the rest of the objects. set drawing state first.
            color[0] = 1.0f;
            color[1] = 1.0f;
            color[2] = 1.0f;
            color[3] = 1.0f;
            tnum = DS_TEXTURE_NUMBER.DS_NONE;
            //if (fn.step) 
            //fn.step(pause);
        }
        private void drawPyramidGrid()
        {
            // setup stuff
            GL.Enable(EnableCap.Lighting);
            GL.Disable(EnableCap.Texture2D);
            GL.ShadeModel(ShadingModel.Flat);
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Less);

            // draw the pyramid grid
            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    GL.PushMatrix();
                    GL.Translate((float)i, (float)j, (float)0);
                    if (i == 1 && j == 0)
                        setColor(1.0f, 0, 0, 1.0f);
                    else if (i == 0 && j == 1)
                        setColor(0, 0, 1.0f, 1.0f);
                    else
                        setColor(1.0f, 1.0f, 0, 1.0f);
                    float k = 0.03f;
                    GL.Begin(BeginMode.TriangleFan);
                    GL.Normal3(0, -1.0f, 1.0f); // GL.Normal3(0, -1, 1) esto no funciona
                    GL.Vertex3(0, 0, k);
                    GL.Vertex3(-k, -k, 0);
                    GL.Vertex3(k, -k, 0);
                    GL.Normal3(1.0f, 0, 1.0f); // GL.Normal3(1, 0, 1) esto no funciona
                    GL.Vertex3(k, k, 0);
                    GL.Normal3(0, 1.0f, 1.0f); // GL.Normal3(0, 1, 1) esto no funciona
                    GL.Vertex3(-k, k, 0);
                    GL.Normal3(-1.0f, 0, 1.0f); // GL.Normal3(-1, 0, 1) esto no funciona
                    GL.Vertex3(-k, -k, 0);
                    GL.End();
                    GL.PopMatrix();
                }
            }
        }
        private void drawGround()
        {
            GL.Disable(EnableCap.Lighting);
            GL.ShadeModel(ShadingModel.Flat);
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Less);
            //GL.DepthRange (1.0f,1.0f);

            if (use_textures)
            {
                GL.Enable(EnableCap.Texture2D);
                ground_texture.bind(false);
            }
            else
            {
                GL.Disable(EnableCap.Texture2D);
                GL.Color3(GROUND_R, GROUND_G, GROUND_B);
            }

            // ground fog seems to cause problems with TNT2 under windows
            /*
            GLfloat fogColor[4] = {0.5, 0.5, 0.5, 1};
            GL11.glEnable (GL_FOG);
            GL11.glFogi (GL_FOG_MODE, GL_EXP2);
            GL11.glFogfv (GL_FOG_COLOR, fogColor);
            GL11.glFogf (GL_FOG_DENSITY, 0.05f);
            GL11.glHint (GL_FOG_HINT, GL_NICEST); // GL_DONT_CARE);
            GL11.glFogf (GL_FOG_START, 1.0);
            GL11.glFogf (GL_FOG_END, 5.0);
             */

            float gsize = 100.0f;
            float offset = 0; // -0.001f; ... polygon offsetting doesn't work well

            GL.Begin(BeginMode.Quads);
            GL.Normal3(0, 0, 1.0f); // GL.Normal3(0, 0, 1) esto no funciona
            GL.TexCoord2(-gsize * ground_scale + ground_ofsx,
                         -gsize * ground_scale + ground_ofsy);
            GL.Vertex3(-gsize, -gsize, offset);
            GL.TexCoord2(gsize * ground_scale + ground_ofsx,
                        -gsize * ground_scale + ground_ofsy);
            GL.Vertex3(gsize, -gsize, offset);
            GL.TexCoord2(gsize * ground_scale + ground_ofsx,
                         gsize * ground_scale + ground_ofsy);
            GL.Vertex3(gsize, gsize, offset);
            GL.TexCoord2(-gsize * ground_scale + ground_ofsx,
                         gsize * ground_scale + ground_ofsy);
            GL.Vertex3(-gsize, gsize, offset);
            GL.End();

            GL.Disable(EnableCap.Fog);
        }
        private void drawSky(float[] view_xyz)
        {
            GL.Disable(EnableCap.Lighting);
            if (use_textures)
            {
                GL.Enable(EnableCap.Texture2D);
                sky_texture.bind(false);
            }
            else
            {
                GL.Disable(EnableCap.Texture2D);
                GL.Color3(0f, 0.5f, 1.0f);
            }

            // make sure sky depth is as far back as possible
            GL.ShadeModel(ShadingModel.Flat);
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Lequal);
            GL.DepthRange(1.0, 1.0);

            float ssize = 1000.0f;

            float x = ssize * sky_scale;
            float z = view_xyz[2] + sky_height;

            GL.Begin(BeginMode.Quads);
            GL.Normal3(0, 0, -1.0f); // GL.Normal3(0, 0, -1) esto no funciona
            GL.TexCoord2(-x + offset, -x + offset);
            GL.Vertex3(-ssize + view_xyz[0], -ssize + view_xyz[1], z);
            GL.TexCoord2(-x + offset, x + offset);
            GL.Vertex3(-ssize + view_xyz[0], ssize + view_xyz[1], z);
            GL.TexCoord2(x + offset, x + offset);
            GL.Vertex3(ssize + view_xyz[0], ssize + view_xyz[1], z);
            GL.TexCoord2(x + offset, -x + offset);
            GL.Vertex3(ssize + view_xyz[0], -ssize + view_xyz[1], z);
            GL.End();

            offset = offset + 0.001f * (float)(60.0d / RenderFrequency);
            if (offset > 1) offset -= 1;

            GL.DepthFunc(DepthFunction.Less);
            GL.DepthRange(0, 1.0);
        }
        private void setCamera(float x, float y, float z, float h, float p, float r)
        {
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadIdentity();
            GL.Rotate(90, 0, 0, 1);
            GL.Rotate(90, 0, 1, 0);
            GL.Rotate(r, 1, 0, 0);
            GL.Rotate(p, 0, 1, 0);
            GL.Rotate(-h, 0, 0, 1);
            GL.Translate(-x, -y, -z);
        }

        // initialize the above variables

        private void initMotionModel()
        {
            view_xyz[0] = 2;
            view_xyz[1] = 0;
            view_xyz[2] = 1;
            view_hpr[0] = 180;
            view_hpr[1] = 0;
            view_hpr[2] = 0;
        }
        protected void dsSetupTextures(string texturesPath)
        {
            string checkeredName = null;
            string groundName = null;
            string skyName = null;
            string woodName = null;
            string grassName = null;

            if(File.Exists(texturesPath + "checkered.png"))
                checkeredName = texturesPath + "checkered.png";
            if (File.Exists(texturesPath + "checkered.jpeg"))
                checkeredName = texturesPath + "checkered.jpeg";

            if (File.Exists(texturesPath + "ground.png"))
                groundName = texturesPath + "ground.png";
            if (File.Exists(texturesPath + "ground.jpeg"))
                groundName = texturesPath + "ground.jpeg";

            if (File.Exists(texturesPath + "sky.png"))
                skyName = texturesPath + "sky.png";
            if (File.Exists(texturesPath + "sky.jpeg"))
                skyName = texturesPath + "sky.jpeg";

            if (File.Exists(texturesPath + "wood.png"))
                woodName = texturesPath + "wood.png";
            if (File.Exists(texturesPath + "wood.jpeg"))
                woodName = texturesPath + "wood.jpeg";

            if (File.Exists(texturesPath + "grass.png"))
                grassName = texturesPath + "grass.png";
            if (File.Exists(texturesPath + "grass.jpeg"))
                grassName = texturesPath + "grass.jpeg";

            if (checkeredName != null)
            {
                checkered_texture.finalize();
                checkered_texture = new Texture(checkeredName);
                texture[(int)DS_TEXTURE_NUMBER.DS_CHECKERED] = checkered_texture;
            }
            if (groundName != null)
            {
                ground_texture.finalize();
                ground_texture = new Texture(groundName);
                texture[(int)DS_TEXTURE_NUMBER.DS_GROUND] = ground_texture;
            }
            if (skyName != null)
            {
                sky_texture.finalize();
                sky_texture = new Texture(skyName);
                texture[(int)DS_TEXTURE_NUMBER.DS_SKY] = sky_texture;
            }
            if (woodName != null)
            {
                wood_texture.finalize();
                wood_texture = new Texture(woodName);
                texture[(int)DS_TEXTURE_NUMBER.DS_WOOD] = wood_texture;
            }
            if (grassName != null)
            {
                grass_texture.finalize();
                grass_texture = new Texture(grassName);
                texture[(int)DS_TEXTURE_NUMBER.DS_GRASS] = grass_texture;
            }
        }
        protected void dsSetSphereQuality(int n)
        {
            sphere_quality = n;
        }
        protected void dsSetCapsuleQuality(int n)
        {
            capped_cylinder_quality = n;
        }
        protected void dsSetTexture(DS_TEXTURE_NUMBER texture_number)
        {
            if (current_state != 2)
                throw new Exception("drawing function called outside simulation loop");
            tnum = texture_number;
        }
        protected void dsSetColor(double red, double green, double blue)
        {
            dsSetColor((float)red, (float)green, (float)blue);
        }
        protected void dsSetColor(float red, float green, float blue)
        {
            if (current_state != 2)
                throw new Exception("drawing function called outside simulation loop");
            color[0] = red;
            color[1] = green;
            color[2] = blue;
            color[3] = 1;
        }
        public void dsSetColorAlpha(float red, float green, float blue, float alpha)
        {
            if (current_state != 2)
                throw new Exception("drawing function called outside simulation loop");
            color[0] = red;
            color[1] = green;
            color[2] = blue;
            color[3] = alpha;
        }

        protected void dsGetViewPoint(out JVector position, out JVector angles)
        {
            position.X = view_xyz[0];
            position.Y = view_xyz[1];
            position.Z = view_xyz[2];

            angles.X = view_hpr[0];
            angles.Y = view_hpr[1];
            angles.Z = view_hpr[2];
        }

        protected void dsSetViewpoint(float[] xyz, float[] hpr)
        {
            if (current_state < 1)
                Console.WriteLine("dsSetViewpoint() called before simulation started");
            if (xyz != null)
            {
                view_xyz[0] = xyz[0];
                view_xyz[1] = xyz[1];
                view_xyz[2] = xyz[2];
            }
            if (hpr != null)
            {
                view_hpr[0] = hpr[0];
                view_hpr[1] = hpr[1];
                view_hpr[2] = hpr[2];
                wrapCameraAngles();
            }
        }
        protected void dsDrawLine(JVector _pos1, JVector _pos2)
        {
            float[] pos1 = Conversion.ToFloat(_pos1);
            float[] pos2 = Conversion.ToFloat(_pos2);
            dsDrawLine(pos1, pos2);
        }
        public void dsDrawLine(float[] pos1, float[] pos2)
        {
            setupDrawingMode();
            GL.Color3(color[0], color[1], color[2]);
            GL.Disable(EnableCap.Lighting);
            GL.LineWidth(2);
            GL.ShadeModel(ShadingModel.Flat);
            GL.Begin(BeginMode.Lines);
            GL.Vertex3(pos1[0], pos1[1], pos1[2]);
            GL.Vertex3(pos2[0], pos2[1], pos2[2]);
            GL.End();
        }
        protected void dsDrawSphere(JVector pos, JMatrix R, float radius)
        {
            float[] pos2 = Conversion.ToFloat(pos);
            float[] R2 = Conversion.ToFloat(R);
            dsDrawSphere(pos2, R2, radius);
        }
        protected void dsDrawSphere(float[] pos, float[] R, float radius)
        {
            if (current_state != 2)
                Console.WriteLine("drawing function called outside simulation loop");
            setupDrawingMode();
            GL.Enable(EnableCap.Normalize);
            GL.ShadeModel(ShadingModel.Smooth);
            setTransform(pos, R);
            GL.Scale(radius, radius, radius);
            drawSphere();
            GL.PopMatrix();
            GL.Disable(EnableCap.Normalize);

            // draw shadows
            if (use_shadows)
            {
                GL.DepthRange(0.0, 1.0);
                GL.Disable(EnableCap.Lighting);
                if (use_textures)
                {
                    ground_texture.bind(true);
                    GL.Enable(EnableCap.Texture2D);
                    GL.Disable(EnableCap.TextureGenS);
                    GL.Disable(EnableCap.TextureGenT);
                    GL.Color3(SHADOW_INTENSITY, SHADOW_INTENSITY, SHADOW_INTENSITY);
                }
                else
                {
                    GL.Disable(EnableCap.Texture2D);
                    GL.Color3(GROUND_R * SHADOW_INTENSITY, GROUND_G * SHADOW_INTENSITY,
                            GROUND_B * SHADOW_INTENSITY);
                }
                GL.ShadeModel(ShadingModel.Flat);
                GL.DepthRange(0, 0.9999);
                drawSphereShadow(pos[0], pos[1], pos[2], radius);
            }
        }
        protected void dsDrawTriangle(JVector pos, JMatrix R, float[] vAll, int v0, int v1,
                                       int v2, bool solid)
        {
            if (current_state != 2)
                Console.WriteLine("drawing function called outside simulation loop");
            setupDrawingMode();
            GL.ShadeModel(ShadingModel.Flat);
            setTransform(pos, R);
            drawTriangle(vAll, v0, v1, v2, solid);
            GL.PopMatrix();
        }
        public void dsDrawTriangle(JVector pos, JMatrix R,
                                    JVector v0, JVector v1,
                                    JVector v2, bool solid)
        {
            setupDrawingMode();
            GL.ShadeModel(ShadingModel.Flat);
            setTransform(pos, R);
            drawTriangle(v0, v1, v2, solid);
            GL.PopMatrix();
        }
        private void drawTriangle(JVector v0, JVector v1,
                                   JVector v2, bool solid)
        {
            float[] u = new float[3], v = new float[3], normal = new float[3];
            u[0] = (float)(v1.X - v0.X);
            u[1] = (float)(v1.Y - v0.Y);
            u[2] = (float)(v1.Z - v0.Z);
            v[0] = (float)(v2.X - v0.X);
            v[1] = (float)(v2.Y - v0.Y);
            v[2] = (float)(v2.Z - v0.Z);

            //OdeMath.dCROSS(normal, CsODE.OP.EQ, u, v);
            normal[0] = u[1] * v[2] - u[2] - v[1];
            normal[1] = u[2] * v[0] - u[0] - v[2];
            normal[2] = u[0] * v[1] - u[1] - v[0];

            normalizeVector3(normal);

            GL.Begin(solid ? BeginMode.Triangles : BeginMode.LineStrip);
            GL.Normal3(normal[0], normal[1], normal[2]);
            GL.Vertex3(v0.X, v0.Y, v0.Z);
            GL.Vertex3(v1.X, v1.Y, v1.Z);
            GL.Vertex3(v2.X, v2.Y, v2.Z);
            GL.End();
        }
        private void drawTriangle(float[] vAll, int v0, int v1, int v2, bool solid)
        {
            float[] u = new float[3], v = new float[3], normal = new float[3];
            u[0] = vAll[v1] - vAll[v0];
            u[1] = vAll[v1 + 1] - vAll[v0 + 1];
            u[2] = vAll[v1 + 2] - vAll[v0 + 2];
            v[0] = vAll[v2] - vAll[v0];
            v[1] = vAll[v2 + 1] - vAll[v0 + 1];
            v[2] = vAll[v2 + 2] - vAll[v0 + 2];
            dCROSS(normal, OP.EQ, u, v);
            normalizeVector3(normal);

            GL.Begin(solid ? BeginMode.Triangles : BeginMode.LineStrip);
            GL.Normal3(normal[0], normal[1], normal[2]);
            GL.Vertex3(vAll[v0], vAll[v0 + 1], vAll[v0 + 2]);
            GL.Vertex3(vAll[v1], vAll[v1 + 1], vAll[v1 + 2]);
            GL.Vertex3(vAll[v2], vAll[v2 + 1], vAll[v2 + 2]);
            GL.End();
        }
        enum OP
        {
            ADD, SUB, MUL, DIV, EQ, /** += */ ADD_EQ, /** =- */ EQ_SUB,
            /** *= */
            MUL_EQ, /** -= */ SUB_EQ
        }
        static void dCROSS(float[] a, OP op, float[] b, float[] c)
        {
            if (op == OP.EQ)
            {
                a[0] = ((b)[1] * (c)[2] - (b)[2] * (c)[1]);
                a[1] = ((b)[2] * (c)[0] - (b)[0] * (c)[2]);
                a[2] = ((b)[0] * (c)[1] - (b)[1] * (c)[0]);
            }
            else if (op == OP.ADD_EQ)
            {
                a[0] += ((b)[1] * (c)[2] - (b)[2] * (c)[1]);
                a[1] += ((b)[2] * (c)[0] - (b)[0] * (c)[2]);
                a[2] += ((b)[0] * (c)[1] - (b)[1] * (c)[0]);
            }
            else if (op == OP.SUB_EQ)
            {
                a[0] -= ((b)[1] * (c)[2] - (b)[2] * (c)[1]);
                a[1] -= ((b)[2] * (c)[0] - (b)[0] * (c)[2]);
                a[2] -= ((b)[0] * (c)[1] - (b)[1] * (c)[0]);
            }
            else
            {
                throw new InvalidOperationException(op.ToString());
            }
        }
        private void setTransform(JVector pos, JMatrix R)
        {
            //GLdouble
            double[] matrix = new double[16];
            matrix[0] = R.M11;
            matrix[1] = R.M21;
            matrix[2] = R.M31;
            matrix[3] = 0;
            matrix[4] = R.M12;
            matrix[5] = R.M22;
            matrix[6] = R.M32;
            matrix[7] = 0;
            matrix[8] = R.M13;
            matrix[9] = R.M23;
            matrix[10] = R.M33;
            matrix[11] = 0;
            matrix[12] = pos.X;
            matrix[13] = pos.Y;
            matrix[14] = pos.Z;
            matrix[15] = 1;
            GL.PushMatrix();
            GL.MultMatrix(matrix);
        }
        private static bool init = false;
        private static float len2, len1, scale;
        private void drawSphereShadow(float px, float py, float pz, float radius)
        {
            // calculate shadow constants based on light vector
            if (!init)
            {
                len2 = LIGHTX * LIGHTX + LIGHTY * LIGHTY;
                len1 = 1.0f / (float)Math.Sqrt(len2);
                scale = (float)Math.Sqrt(len2 + 1);
                init = true;
            }

            // map sphere center to ground plane based on light vector
            px -= LIGHTX * pz;
            py -= LIGHTY * pz;

            float kx = 0.96592582628907f;
            float ky = 0.25881904510252f;
            float x = radius, y = 0;

            GL.Begin(BeginMode.TriangleFan);
            for (int i = 0; i < 24; i++)
            {
                // for all points on circle, scale to elongated rotated shadow and draw
                float x2 = (LIGHTX * x * scale - LIGHTY * y) * len1 + px;
                float y2 = (LIGHTY * x * scale + LIGHTX * y) * len1 + py;
                GL.TexCoord2(x2 * ground_scale + ground_ofsx, y2 * ground_scale + ground_ofsy);
                GL.Vertex3(x2, y2, 0);

                // rotate [x,y] vector
                float xtmp = kx * x - ky * y;
                y = ky * x + kx * y;
                x = xtmp;
            }
            GL.End();
        }
        private static int listnum = 0; //GLunint TZ
        private const float ICX = 0.525731112119133606f;
        private const float ICZ = 0.850650808352039932f;
        private readonly float[][] idata = new float[][]
        {
		    new float[]{-ICX, 0, ICZ},
		    new float[]{ICX, 0, ICZ},
		    new float[]{-ICX, 0, -ICZ},
		    new float[]{ICX, 0, -ICZ},
		    new float[]{0, ICZ, ICX},
		    new float[]{0, ICZ, -ICX},
		    new float[]{0, -ICZ, ICX},
		    new float[]{0, -ICZ, -ICX},
		    new float[]{ICZ, ICX, 0},
		    new float[]{-ICZ, ICX, 0},
		    new float[]{ICZ, -ICX, 0},
		    new float[]{-ICZ, -ICX, 0}
	    };
        private readonly int[][] index = new int[][]
        {
		    new int[]{0, 4, 1}, new int[]{0, 9, 4},
		    new int[]{9, 5, 4},	  new int[]{4, 5, 8},
		    new int[]{4, 8, 1},	  new int[]{8, 10, 1},
		    new int[]{8, 3, 10},   new int[]{5, 3, 8},
		    new int[]{5, 2, 3},	  new int[]{2, 7, 3},
		    new int[]{7, 10, 3},   new int[]{7, 6, 10},
		    new int[]{7, 11, 6},   new int[]{11, 0, 6},
		    new int[]{0, 1, 6},	  new int[]{6, 1, 10},
		    new int[]{9, 0, 11},   new int[]{9, 11, 2},
		    new int[]{9, 2, 5},	 new int[] {7, 2, 11},
	    };
        private int sphere_quality = 1;

        private void drawSphere()
        {
            // icosahedron data for an icosahedron of radius 1.0
            //		# define ICX 0.525731112119133606f
            //		# define ICZ 0.850650808352039932f
            if (listnum == 0)
            {
                listnum = GL.GenLists(1);
                GL.NewList(listnum, ListMode.Compile);
                GL.Begin(BeginMode.Triangles);
                for (int i = 0; i < 20; i++)
                {
                    //				drawPatch (&idata[index[i][2]][0],&idata[index[i][1]][0],
                    //						&idata[index[i][0]][0],sphere_quality);
                    drawPatch(idata[index[i][2]], idata[index[i][1]],
                            idata[index[i][0]], sphere_quality);
                }
                GL.End();
                GL.EndList();
            }
            GL.CallList(listnum);
        }
        // This is recursively subdivides a triangular area (vertices p1,p2,p3) into
        // smaller triangles, and then draws the triangles. All triangle vertices are
        // normalized to a distance of 1.0 from the origin (p1,p2,p3 are assumed
        // to be already normalized). Note this is not super-fast because it draws
        // triangles rather than triangle strips.

        //	static void drawPatch (float p1[3], float p2[3], float p3[3], int level)
        private void drawPatch(float[] p1, float[] p2, float[] p3, int level)
        {
            int i;
            if (level > 0)
            {
                float[] q1 = new float[3], q2 = new float[3], q3 = new float[3];		 // sub-vertices
                for (i = 0; i < 3; i++)
                {
                    q1[i] = 0.5f * (p1[i] + p2[i]);
                    q2[i] = 0.5f * (p2[i] + p3[i]);
                    q3[i] = 0.5f * (p3[i] + p1[i]);
                }
                float length1 = (float)(1.0 / Math.Sqrt(q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2]));
                float length2 = (float)(1.0 / Math.Sqrt(q2[0] * q2[0] + q2[1] * q2[1] + q2[2] * q2[2]));
                float length3 = (float)(1.0 / Math.Sqrt(q3[0] * q3[0] + q3[1] * q3[1] + q3[2] * q3[2]));
                for (i = 0; i < 3; i++)
                {
                    q1[i] *= length1;
                    q2[i] *= length2;
                    q3[i] *= length3;
                }
                drawPatch(p1, q1, q3, level - 1);
                drawPatch(q1, p2, q2, level - 1);
                drawPatch(q1, q2, q3, level - 1);
                drawPatch(q3, q2, p3, level - 1);
            }
            else
            {
                GL.Normal3(p1[0], p1[1], p1[2]);
                GL.Vertex3(p1[0], p1[1], p1[2]);
                GL.Normal3(p2[0], p2[1], p2[2]);
                GL.Vertex3(p2[0], p2[1], p2[2]);
                GL.Normal3(p3[0], p3[1], p3[2]);
                GL.Vertex3(p3[0], p3[1], p3[2]);
            }
        }
        public void dsDrawConvex(JVector pos, JMatrix R,
                                double[] _planes, int _planecount,
                                double[] _points,
                                int _pointcount,
                                int[] _polygons)
        {
            if (current_state != 2)
                Console.WriteLine("drawing function called outside simulation loop");
            setupDrawingMode();
            GL.ShadeModel(ShadingModel.Flat);
            setTransform(pos, R);
            drawConvexD(_planes, _planecount, _points, _pointcount, _polygons);
            GL.PopMatrix();
            if (use_shadows)
            {
                GL.DepthRange(0, 1);
                setShadowDrawingMode();
                setShadowTransform();
                setTransform(pos, R);
                drawConvexD(_planes, _planecount, _points, _pointcount, _polygons);
                GL.PopMatrix();
                GL.PopMatrix();
            }
        }
        private void drawConvexD(double[] _planes, int _planecount,
                                double[] _points,
                                int _pointcount,
                                int[] _polygons)
        {
            //unsigned 
            int polyindex = 0;
            for (int i = 0; i < _planecount; ++i)
            {
                //unsigned 
                int pointcount = _polygons[polyindex];
                polyindex++;
                GL.Begin(BeginMode.Polygon);
                GL.Normal3(_planes[(i * 4) + 0],
                        _planes[(i * 4) + 1],
                        _planes[(i * 4) + 2]);
                for (int j = 0; j < pointcount; ++j)
                {
                    GL.Vertex3(_points[_polygons[polyindex] * 3],
                            _points[(_polygons[polyindex] * 3) + 1],
                            _points[(_polygons[polyindex] * 3) + 2]);
                    polyindex++;
                }
                GL.End();
            }
        }
        protected void dsDrawCapsule(JVector pos, JMatrix R,
            float length, float radius)
        {
            float[] pos2 = Conversion.ToFloat(pos);
            float[] R2 = Conversion.ToFloat(R);
            dsDrawCapsule(pos2, R2, length, radius);
        }
        protected void dsDrawCapsule(float[] pos, float[] R,
            float length, float radius)
        {
            if (current_state != 2)
                throw new Exception("drawing function called outside simulation loop");
            setupDrawingMode();
            GL.ShadeModel(ShadingModel.Smooth);
            setTransform(pos, R);
            drawCapsule(length, radius);
            GL.PopMatrix();

            if (use_shadows)
            {
                GL.DepthRange(0, 1);
                setShadowDrawingMode();
                setShadowTransform();
                setTransform(pos, R);
                drawCapsule(length, radius);
                GL.PopMatrix();
                GL.PopMatrix();
            }
        }
        private int capped_cylinder_quality = 3;

        private void drawCapsule(float l, float r)
        {
            int i, j;
            float tmp, nx, ny, nz, start_nx, start_ny, a, ca, sa;
            // number of sides to the cylinder (divisible by 4):
            int n = capped_cylinder_quality * 4;

            l *= 0.5f;
            a = (float)((M_PI * 2.0) / (float)n);
            sa = (float)Math.Sin(a);
            ca = (float)Math.Cos(a);

            // draw cylinder body
            ny = 1; nz = 0;		  // normal vector = (0,ny,nz)
            GL.Begin(BeginMode.TriangleStrip);
            for (i = 0; i <= n; i++)
            {
                GL.Normal3(ny, nz, 0);
                GL.Vertex3(ny * r, nz * r, l);
                GL.Normal3(ny, nz, 0);
                GL.Vertex3(ny * r, nz * r, -l);
                // rotate ny,nz
                tmp = ca * ny - sa * nz;
                nz = sa * ny + ca * nz;
                ny = tmp;
            }
            GL.End();

            // draw first cylinder cap
            start_nx = 0;
            start_ny = 1;
            for (j = 0; j < (n / 4); j++)
            {
                // get start_n2 = rotated start_n
                float start_nx2 = ca * start_nx + sa * start_ny;
                float start_ny2 = -sa * start_nx + ca * start_ny;
                // get n=start_n and n2=start_n2
                nx = start_nx; ny = start_ny; nz = 0;
                float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
                GL.Begin(BeginMode.TriangleStrip);
                for (i = 0; i <= n; i++)
                {
                    GL.Normal3(ny2, nz2, nx2);
                    GL.Vertex3(ny2 * r, nz2 * r, l + nx2 * r);
                    GL.Normal3(ny, nz, nx);
                    GL.Vertex3(ny * r, nz * r, l + nx * r);
                    // rotate n,n2
                    tmp = ca * ny - sa * nz;
                    nz = sa * ny + ca * nz;
                    ny = tmp;
                    tmp = ca * ny2 - sa * nz2;
                    nz2 = sa * ny2 + ca * nz2;
                    ny2 = tmp;
                }
                GL.End();
                start_nx = start_nx2;
                start_ny = start_ny2;
            }

            // draw second cylinder cap
            start_nx = 0;
            start_ny = 1;
            for (j = 0; j < (n / 4); j++)
            {
                // get start_n2 = rotated start_n
                float start_nx2 = ca * start_nx - sa * start_ny;
                float start_ny2 = sa * start_nx + ca * start_ny;
                // get n=start_n and n2=start_n2
                nx = start_nx; ny = start_ny; nz = 0;
                float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
                GL.Begin(BeginMode.TriangleStrip);
                for (i = 0; i <= n; i++)
                {
                    GL.Normal3(ny, nz, nx);
                    GL.Vertex3(ny * r, nz * r, -l + nx * r);
                    GL.Normal3(ny2, nz2, nx2);
                    GL.Vertex3(ny2 * r, nz2 * r, -l + nx2 * r);
                    // rotate n,n2
                    tmp = ca * ny - sa * nz;
                    nz = sa * ny + ca * nz;
                    ny = tmp;
                    tmp = ca * ny2 - sa * nz2;
                    nz2 = sa * ny2 + ca * nz2;
                    ny2 = tmp;
                }
                GL.End();
                start_nx = start_nx2;
                start_ny = start_ny2;
            }
        }
        protected void dsDrawCylinder(JVector pos, JMatrix R, float length, float radius)
        {
            float[] pos2 = Conversion.ToFloat(pos);
            float[] R2 = Conversion.ToFloat(R);
            dsDrawCylinder(pos2, R2, length, radius);
        }

        public void dsDrawCylinder(float[] pos, float[] R, float length, float radius)
        {
            if (current_state != 2)
                Console.WriteLine("drawing function called outside simulation loop");
            setupDrawingMode();
            GL.ShadeModel(ShadingModel.Smooth);
            setTransform(pos, R);
            drawCylinder(length, radius, 0);
            GL.PopMatrix();

            if (use_shadows)
            {
                GL.DepthRange(0, 1.0);
                setShadowDrawingMode();
                setShadowTransform();
                setTransform(pos, R);
                drawCylinder(length, radius, 0);
                GL.PopMatrix();
                GL.PopMatrix();
            }
        }
        void drawCylinder(float l, float r, float zoffset)
        {
            int i;
            float tmp, ny, nz, a, ca, sa;
            int n = 24;	// number of sides to the cylinder (divisible by 4)

            l *= 0.5f;
            a = (float)(M_PI * 2.0 / (float)n);
            sa = (float)Math.Sin(a);
            ca = (float)Math.Cos(a);

            // draw cylinder body
            ny = 1; nz = 0;		  // normal vector = (0,ny,nz)
            GL.Begin(BeginMode.TriangleStrip);
            for (i = 0; i <= n; i++)
            {
                GL.Normal3(ny, nz, 0);
                GL.Vertex3(ny * r, nz * r, l + zoffset);
                GL.Normal3(ny, nz, 0);
                GL.Vertex3(ny * r, nz * r, -l + zoffset);
                // rotate ny,nz
                tmp = ca * ny - sa * nz;
                nz = sa * ny + ca * nz;
                ny = tmp;
            }
            GL.End();

            // draw top cap
            GL.ShadeModel(ShadingModel.Flat);
            ny = 1; nz = 0;		  // normal vector = (0,ny,nz)
            GL.Begin(BeginMode.TriangleFan);
            GL.Normal3(0, 0, 1.0f);
            GL.Vertex3(0, 0, l + zoffset);
            for (i = 0; i <= n; i++)
            {
                if (i == 1 || i == n / 2 + 1)
                    setColor(color[0] * 0.75f, color[1] * 0.75f, color[2] * 0.75f, color[3]);
                GL.Normal3(0, 0, 1.0f);
                GL.Vertex3(ny * r, nz * r, l + zoffset);
                if (i == 1 || i == n / 2 + 1)
                    setColor(color[0], color[1], color[2], color[3]);

                // rotate ny,nz
                tmp = ca * ny - sa * nz;
                nz = sa * ny + ca * nz;
                ny = tmp;
            }
            GL.End();

            // draw bottom cap
            ny = 1; nz = 0;		  // normal vector = (0,ny,nz)
            GL.Begin(BeginMode.TriangleFan);
            GL.Normal3(0, 0, -1.0f);
            GL.Vertex3(0, 0, -l + zoffset);
            for (i = 0; i <= n; i++)
            {
                if (i == 1 || i == n / 2 + 1)
                    setColor(color[0] * 0.75f, color[1] * 0.75f, color[2] * 0.75f, color[3]);
                GL.Normal3(0, 0, -1.0f);
                GL.Vertex3(ny * r, nz * r, -l + zoffset);
                if (i == 1 || i == n / 2 + 1)
                    setColor(color[0], color[1], color[2], color[3]);

                // rotate ny,nz
                tmp = ca * ny + sa * nz;
                nz = -sa * ny + ca * nz;
                ny = tmp;
            }
            GL.End();
        }
        private void drawBox(float[] sides)
        {
            float lx = sides[0] * 0.5f;
            float ly = sides[1] * 0.5f;
            float lz = sides[2] * 0.5f;

            // sides
            GL.Begin(BeginMode.TriangleStrip);
            GL.Normal3(-1.0f, 0, 0); // GL.Normal3(-1, 0, 0) no funciona
            GL.Vertex3(-lx, -ly, -lz);
            GL.Vertex3(-lx, -ly, lz);
            GL.Vertex3(-lx, ly, -lz);
            GL.Vertex3(-lx, ly, lz);
            GL.Normal3(0, 1.0f, 0); // GL.Normal3(0, 1, 0) no funciona
            GL.Vertex3(lx, ly, -lz);
            GL.Vertex3(lx, ly, lz);
            GL.Normal3(1.0f, 0, 0); // GL.Normal3(1, 0, 0) no funciona
            GL.Vertex3(lx, -ly, -lz);
            GL.Vertex3(lx, -ly, lz);
            GL.Normal3(0, -1.0f, 0); // GL.Normal3(0, -1, 0) no funciona
            GL.Vertex3(-lx, -ly, -lz);
            GL.Vertex3(-lx, -ly, lz);
            GL.End();

            // top face
            GL.Begin(BeginMode.TriangleFan);
            GL.Normal3(0, 0, 1.0f); // GL.Normal3(0, 0, 1) no funciona
            GL.Vertex3(-lx, -ly, lz);
            GL.Vertex3(lx, -ly, lz);
            GL.Vertex3(lx, ly, lz);
            GL.Vertex3(-lx, ly, lz);
            GL.End();

            // bottom face
            GL.Begin(BeginMode.TriangleFan);
            GL.Normal3(0, 0, -1.0f); // GL.Normal3(0, 0, -1) no funciona
            GL.Vertex3(-lx, -ly, -lz);
            GL.Vertex3(-lx, ly, -lz);
            GL.Vertex3(lx, ly, -lz);
            GL.Vertex3(lx, -ly, -lz);
            GL.End();
        }
        protected void dsDrawBox(JVector pos, JMatrix R, JVector sides)
        {
            float[] pos2 = Conversion.ToFloat(pos);
            float[] R2 = Conversion.ToFloat(R);
            float[] fsides = Conversion.ToFloat(sides);
            dsDrawBox(pos2, R2, fsides);
        }
        protected void dsDrawBox(float[] pos, float[] R, float[] sides)
        {
            if (current_state != 2)
                throw new Exception("drawing function called outside simulation loop");
            setupDrawingMode();
            GL.ShadeModel(ShadingModel.Flat);
            setTransform(pos, R);
            drawBox(sides);
            GL.PopMatrix();

            if (use_shadows)
            {
                GL.DepthRange(0, 1.0);
                setShadowDrawingMode();
                setShadowTransform();
                setTransform(pos, R);
                drawBox(sides);
                GL.PopMatrix();
                GL.PopMatrix();
            }
        }
        private void setShadowDrawingMode()
        {
            GL.Disable(EnableCap.Lighting);
            if (use_textures)
            {
                GL.Enable(EnableCap.Texture2D);
                ground_texture.bind(true);
                GL.Color3(SHADOW_INTENSITY, SHADOW_INTENSITY, SHADOW_INTENSITY);
                GL.Enable(EnableCap.Texture2D);
                GL.Enable(EnableCap.TextureGenS);
                GL.Enable(EnableCap.TextureGenT);
                GL.TexGen(TextureCoordName.S, TextureGenParameter.TextureGenMode, (int)All.EyeLinear);
                GL.TexGen(TextureCoordName.T, TextureGenParameter.TextureGenMode, (int)All.EyeLinear);
                //			static GLfloat s_params[4] = {ground_scale,0,0,ground_ofsx};
                //			static GLfloat t_params[4] = {0,ground_scale,0,ground_ofsy};
                GL.TexGen(TextureCoordName.S, TextureGenParameter.EyePlane, s_params_SSDM);
                GL.TexGen(TextureCoordName.T, TextureGenParameter.EyePlane, t_params_SSDM);
            }
            else
            {
                GL.Disable(EnableCap.Texture2D);
                GL.Color3(GROUND_R * SHADOW_INTENSITY, GROUND_G * SHADOW_INTENSITY, GROUND_B * SHADOW_INTENSITY);
            }
            GL.DepthRange(0, 0.9999);
        }
        private void setShadowTransform()
        {
            //GLfloat
            float[] matrix = new float[16];
            for (int i = 0; i < 16; i++) matrix[i] = 0;
            matrix[0] = 1.0f;
            matrix[5] = 1.0f;
            matrix[8] = -LIGHTX;
            matrix[9] = -LIGHTY;
            matrix[15] = 1.0f;

            GL.PushMatrix();
            GL.MultMatrix(matrix);
        }
        private void setupDrawingMode()
        {
            GL.Enable(EnableCap.Lighting);
            if (tnum != DS_TEXTURE_NUMBER.DS_NONE)
            {
                if (use_textures)
                {
                    GL.Enable(EnableCap.Texture2D);
                    texture[(int)tnum].bind(true);
                    GL.Enable(EnableCap.TextureGenS);
                    GL.Enable(EnableCap.TextureGenT);
                    GL.TexGen(TextureCoordName.S, TextureGenParameter.TextureGenMode, (int)All.ObjectLinear);
                    GL.TexGen(TextureCoordName.T, TextureGenParameter.TextureGenMode, (int)All.ObjectLinear);

                    GL.TexGen(TextureCoordName.S, TextureGenParameter.ObjectPlane, s_params_SDM);
                    GL.TexGen(TextureCoordName.T, TextureGenParameter.ObjectPlane, t_params_SDM);
                }
                else
                {
                    GL.Disable(EnableCap.Texture2D);
                }
            }
            else
            {
                GL.Disable(EnableCap.Texture2D);
            }
            setColor(color[0], color[1], color[2], color[3]);

            if (color[3] < 1.0f)
            {
                GL.Enable(EnableCap.Blend);
                GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);
            }
            else
            {
                GL.Disable(EnableCap.Blend);
            }
        }

        private void setColor(float r, float g, float b, float alpha)
        {
            light_ambient2 = new float[] { r * 0.3f, g * 0.3f, b * 0.3f, alpha };
            light_diffuse2 = new float[] { r * 0.7f, g * 0.7f, b * 0.7f, alpha };
            light_specular2 = new float[] { r * 0.2f, g * 0.2f, b * 0.2f, alpha };

            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Ambient, light_ambient2);
            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Diffuse, light_diffuse2);
            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Specular, light_specular2);
            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Shininess, 5.0f);
        }
        private void setTransform(float[] pos, float[] R)
        {
            //GLfloat
            float[] matrix = new float[16];
            matrix[0] = R[0];
            matrix[1] = R[4];
            matrix[2] = R[8];
            matrix[3] = 0;
            matrix[4] = R[1];
            matrix[5] = R[5];
            matrix[6] = R[9];
            matrix[7] = 0;
            matrix[8] = R[2];
            matrix[9] = R[6];
            matrix[10] = R[10];
            matrix[11] = 0;
            matrix[12] = pos[0];
            matrix[13] = pos[1];
            matrix[14] = pos[2];
            matrix[15] = 1;
            GL.PushMatrix();
            GL.MultMatrix(matrix);
        }
    }

    #region Clases e interfaces auxiliares

    public class Texture
    {
        private int name; // GLuint TZ
        //public:
        //Texture (String filename);
        //  ~Texture();
        //void bind (int modulate);

        public Texture(Stream stm)
        {
            CreateFromBitmap(new Bitmap(stm));
        }
        public Texture(String filename)
        {
            CreateFromBitmap(new Bitmap(filename));
        }
        public Texture(Bitmap bmp)
        {
            CreateFromBitmap(bmp);
        }
        void CreateFromBitmap(Bitmap image)
        {
            image.RotateFlip(RotateFlipType.RotateNoneFlipY);

            GL.GenTextures(1, out name);
            GL.BindTexture(TextureTarget.Texture2D, name);

            // set pixel unpacking mode
            GL.PixelStore(PixelStoreParameter.UnpackSwapBytes, 0);
            GL.PixelStore(PixelStoreParameter.PackRowLength, 0);
            GL.PixelStore(PixelStoreParameter.UnpackAlignment, 1);
            GL.PixelStore(PixelStoreParameter.UnpackSkipRows, 0);
            GL.PixelStore(PixelStoreParameter.UnpackSkipPixels, 0);

            BitmapData data = image.LockBits(new System.Drawing.Rectangle(0, 0, image.Width, image.Height),
                ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

            // Requieres OpenGL >= 1.4
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.GenerateMipmap, 1); // 1 = True
            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, data.Width, data.Height, 0,
                          OpenTK.Graphics.OpenGL.PixelFormat.Bgra, PixelType.UnsignedByte, data.Scan0);

            image.UnlockBits(data);

            // set texture parameters - will these also be bound to the texture???
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)All.Repeat);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)All.Repeat);

            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)All.Linear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)All.LinearMipmapLinear);

            GL.TexEnv(TextureEnvTarget.TextureEnv, TextureEnvParameter.TextureEnvMode, (int)All.Decal);

            image = null;
            // Unbind
            GL.BindTexture(TextureTarget.Texture2D, 0);
        }


        //Texture::~Texture()
        //{
        //  delete image;
        //  glDeleteTextures (1,&name);
        //}
        internal void finalize()
        {
            //			GL11.glDeleteTextures (1, name));
            GL.DeleteTextures(0, ref name);
            //super.finalize();
        }

        public void bind(bool modulate)
        {
            GL.BindTexture(TextureTarget.Texture2D, name);
            GL.TexEnv(TextureEnvTarget.TextureEnv, TextureEnvParameter.TextureEnvMode,
                      modulate ? (int)All.Modulate : (int)All.Decal);
        }
    }

    #endregion Clases auxiliares

    /* texture numbers */
    public enum DS_TEXTURE_NUMBER
    {
        DS_NONE, // = 0,       /* uses the current color instead of a texture */
        DS_WOOD,
        DS_CHECKERED,
        DS_GROUND,
        DS_SKY,
        DS_GRASS
    }
}
