using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace GJKCollisionDemo
{
    /// <summary>
    /// 2D camera.
    /// </summary>
    public class Camera : GameComponent
    {
        private Matrix view;
        private Matrix projection;

        private Vector2 position = new Vector2(0, 0);
        private Vector2 moveVector;
        private float angle = 0.0f;
        private float zoom = 0.5f;
        private float zoomForce = 0.0f;

        private int widthOver2;
        private int heightOver2;

        private float aspectRatio;
        
        private MouseState prevMouseState = new MouseState();

        /// <summary>
        /// Initializes new camera component.
        /// </summary>
        /// <param name="game">Game to which attach this camera.</param>
        public Camera(Game game)
            : base(game)
        {
            widthOver2 = game.Window.ClientBounds.Width / 2;
            heightOver2 = game.Window.ClientBounds.Height / 2;
            aspectRatio = (float)game.Window.ClientBounds.Width / (float)game.Window.ClientBounds.Height;
            UpdateProjection();
            Mouse.SetPosition(widthOver2, heightOver2);
        }

        /// <summary>
        /// Gets camera view matrix.
        /// </summary>
        public Matrix View { get { return view; } }
        /// <summary>
        /// Gets or sets camera projection matrix.
        /// </summary>
        public Matrix Projection { get { return projection; } set { projection = value; } }
        /// <summary>
        /// Gets camera view matrix multiplied by projection matrix.
        /// </summary>
        public Matrix ViewProjection { get { return view * projection; } }

        /// <summary>
        /// Gets or sets camera position.
        /// </summary>
        public Vector2 Position { get { return position; } set { position = value; } }

        /// <summary>
        /// Gets or sets camera aspect ratio.
        /// </summary>
        public float AspectRatio { get { return aspectRatio; } set { aspectRatio = value; UpdateProjection(); } }
        
        /// <summary>
        /// Updates camera with input and updates view matrix.
        /// </summary>
        /// <param name="gameTime"></param>
        public override void Update(GameTime gameTime)
        {
            if (Enabled)
            {
                double elapsedTime = (double)gameTime.ElapsedGameTime.Ticks / (double)TimeSpan.TicksPerSecond;
                ProcessInput((float)elapsedTime);
                UpdateView();
                UpdateProjection();

                base.Update(gameTime);
            }
        }

        private void ProcessInput(float amountOfMovement)
        {
             KeyboardState keys = Keyboard.GetState();
            GamePadState buttons = GamePad.GetState(PlayerIndex.One);

            if (keys.IsKeyDown(Keys.D))
                moveVector.X += amountOfMovement;
            if (keys.IsKeyDown(Keys.A))
                moveVector.X -= amountOfMovement;
            if (keys.IsKeyDown(Keys.S))
                moveVector.Y -= amountOfMovement;
            if (keys.IsKeyDown(Keys.W))
                moveVector.Y += amountOfMovement;

            
            MouseState currentMouseState = Mouse.GetState();
            // allow smooth zooming
            if (currentMouseState.ScrollWheelValue != prevMouseState.ScrollWheelValue)
            {
                zoomForce += (prevMouseState.ScrollWheelValue - currentMouseState.ScrollWheelValue) * 0.00005f;
            }
            // allow smooth panning - needs to feel more like actually grabbing the world
            if (currentMouseState.RightButton == ButtonState.Pressed && prevMouseState.RightButton == ButtonState.Pressed)
            {
                moveVector += (new Vector2(prevMouseState.X, -prevMouseState.Y) - new Vector2(currentMouseState.X, -currentMouseState.Y)) * amountOfMovement * 0.5f * zoom;
            }

            zoom += zoomForce;
            zoomForce *= 0.98f;
            zoom = MathHelper.Clamp(zoom, 0.25f, 10f);

            position += moveVector;
            moveVector *= 0.98f;

            prevMouseState = currentMouseState;
        }

        private void UpdateProjection()
        {
            projection = Matrix.CreateOrthographic(40 * zoom * aspectRatio, 40 * zoom, 0, 1);
        }

        private void UpdateView()
        {
            view = Matrix.CreateTranslation(new Vector3(-position, 0));
        }
    }
}
