using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace JitterDemo
{
    public class Display : DrawableGameComponent
    {
        private ContentManager content;
        private SpriteBatch spriteBatch;
        private SpriteFont font1, font2;
        private Texture2D texture;

        private int frameRate = 0;
        private int frameCounter = 0;
        private TimeSpan elapsedTime = TimeSpan.Zero;

        private int bbWidth, bbHeight;

        public Display(Game game)
            : base(game)
        {
            content = new ContentManager(game.Services);
            DisplayText = new List<string>();
            for(int i = 0;i<25;i++)DisplayText.Add(string.Empty);
        }

        private void GraphicsDevice_DeviceReset(object sender, EventArgs e)
        {
            bbWidth = GraphicsDevice.PresentationParameters.BackBufferWidth;
            bbHeight = GraphicsDevice.PresentationParameters.BackBufferHeight;
        }

        protected override void LoadContent()
        {
            this.GraphicsDevice.DeviceReset +=new EventHandler<EventArgs>(GraphicsDevice_DeviceReset);
            GraphicsDevice_DeviceReset(null, null);

            spriteBatch = new SpriteBatch(GraphicsDevice);
            font1 = content.Load<SpriteFont>("Content/font1");
            font2 = content.Load<SpriteFont>("Content/font2");

            texture = content.Load<Texture2D>("Content/logo2");
        }

        protected override void UnloadContent()
        {
            content.Unload();
        }

        public override void Update(GameTime gameTime)
        {
            elapsedTime += gameTime.ElapsedGameTime;

            if (elapsedTime > TimeSpan.FromSeconds(1))
            {
                elapsedTime -= TimeSpan.FromSeconds(1);
                frameRate = frameCounter;
                frameCounter = 0;
            }
        }

        public List<string> DisplayText { set; get; }

        public override void Draw(GameTime gameTime)
        {
            frameCounter++;

            string fps = frameRate.ToString();

            spriteBatch.Begin();

            spriteBatch.Draw(texture, new Rectangle(bbWidth - 105,5, 100, 91), Color.White);
            spriteBatch.DrawString(font1, fps, new Vector2(11, 6), Color.Black);
            spriteBatch.DrawString(font1, fps, new Vector2(12, 7), Color.Yellow);

            for (int i = 0; i < DisplayText.Count; i++)
            {
                if(!string.IsNullOrEmpty(DisplayText[i]))
                spriteBatch.DrawString(font2, DisplayText[i], new Vector2(11, 40 + i*20), Color.White);
            }

            spriteBatch.End();
        }
    }
}
