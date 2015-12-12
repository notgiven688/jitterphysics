#region Using Statements
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Jitter.Collision.Shapes;
using Jitter;
using Jitter.LinearMath;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Graphics;
#endregion

namespace JitterDemo.Vehicle
{
    public class CarObject : DrawableGameComponent
    {
        private Model chassisModel = null;
        private Model tireModel = null;

        public DefaultCar carBody = null;

        public CarObject(Game game)
            : base(game)
        {
            BuildCar();
        }

        private void BuildCar()
        {
            JitterDemo demo = this.Game as JitterDemo;
            World world = demo.World;

            CompoundShape.TransformedShape lower = new CompoundShape.TransformedShape(
                new BoxShape(2.5f, 1f, 6.0f), JMatrix.Identity, JVector.Zero);

            CompoundShape.TransformedShape upper = new CompoundShape.TransformedShape(
                new BoxShape(2.0f, 0.5f, 3.0f), JMatrix.Identity, JVector.Up * 0.75f + JVector.Backward * 1.0f);

            CompoundShape.TransformedShape[] subShapes = { lower, upper };

            Shape chassis = new CompoundShape(subShapes);

            //chassis = new BoxShape(2.5f, 1f, 6.0f);

            carBody = new DefaultCar(world, chassis);

            // use the inertia of the lower box.

            // adjust some driving values
            carBody.SteerAngle = 30; carBody.DriveTorque = 155;
            carBody.AccelerationRate = 10;
            carBody.SteerRate = 2f;
            carBody.AdjustWheelValues();

            carBody.Tag = BodyTag.DontDrawMe;
            carBody.AllowDeactivation = false;

            // place the car two units above the ground.
            carBody.Position = new JVector(0, 5, 0);

            world.AddBody(carBody);
        }

        public override void Update(GameTime gameTime)
        {
            KeyboardState keyState = Keyboard.GetState();

            float steer, accelerate;
            if (keyState.IsKeyDown(Keys.Up)) accelerate = 1.0f;
            else if (keyState.IsKeyDown(Keys.Down)) accelerate = -1.0f;
            else accelerate = 0.0f;

            if (keyState.IsKeyDown(Keys.Left)) steer = 1;
            else if (keyState.IsKeyDown(Keys.Right)) steer = -1;
            else steer = 0.0f;

            carBody.SetInput(accelerate, steer);

            base.Update(gameTime);
        }

        #region Draw Wheels
        private void DrawWheels()
        {
            JitterDemo demo = this.Game as JitterDemo;

            for(int i = 0;i<carBody.Wheels.Length;i++)
            {
                Wheel wheel = carBody.Wheels[i];

                Vector3 position = Conversion.ToXNAVector(wheel.GetWorldPosition());

                foreach (ModelMesh mesh in tireModel.Meshes)
                {
                    foreach (BasicEffect effect in mesh.Effects)
                    {
                        Matrix addOrienation;

                        if (i % 2 != 0) addOrienation = Matrix.CreateRotationX(MathHelper.Pi);
                        else addOrienation = Matrix.Identity;

                        effect.World =
                            addOrienation *
                            Matrix.CreateRotationZ(MathHelper.PiOver2) *
                            Matrix.CreateRotationX(MathHelper.ToRadians(-wheel.WheelRotation)) *
                            Matrix.CreateRotationY(MathHelper.ToRadians(wheel.SteerAngle)) *
                            Conversion.ToXNAMatrix(carBody.Orientation) *
                            Matrix.CreateTranslation(position);

                        effect.EnableDefaultLighting();
                        effect.View = demo.Camera.View;
                        effect.Projection = demo.Camera.Projection;
                    }
                    mesh.Draw();
                }

            }

        }
        #endregion

        private void DrawChassis()
        {
            JitterDemo demo = this.Game as JitterDemo;

            foreach (ModelMesh mesh in chassisModel.Meshes)
            {
                foreach (BasicEffect effect in mesh.Effects)
                {
                    Matrix matrix = Conversion.ToXNAMatrix(carBody.Orientation);
                    matrix.Translation = Conversion.ToXNAVector(carBody.Position) -
                        Vector3.Transform(new Vector3(0,1.0f,0),matrix);

                    effect.EnableDefaultLighting();
                    effect.World = matrix;
                    effect.View = demo.Camera.View;
                    effect.Projection = demo.Camera.Projection;
                }
                mesh.Draw();
            }
        }

        protected override void LoadContent()
        {
            chassisModel = this.Game.Content.Load<Model>("car");
            tireModel = this.Game.Content.Load<Model>("wheel");

            base.LoadContent();
        }

        public override void Draw(GameTime gameTime)
        {
            DrawWheels();
            DrawChassis();
            base.Draw(gameTime);
        }


    }
}
