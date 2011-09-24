//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using Jitter;
//using Microsoft.Xna.Framework;
//using Jitter.Collision.Shapes;
//using Jitter.Dynamics;
//using Jitter.LinearMath;
//using System.Diagnostics;

//namespace JitterDemo.Scenes
//{
//    class CardHouse : Scene
//    {
//        public CardHouse(JitterDemo demo) : base(demo) { }

//        static JVector cardHouseStartingPosition = new JVector(0, 0, 0);
//        const int cardHouseLayers = 10; // starting from 1



//        const double cardThickness = 0.05;
//        const double cardHeight = 3;
//        const double cardWidth = 2;
//        const float degree = 75;

//        const float angle = degree * (float)Math.PI / 180f;
//        const float oppositeAngle = (float)Math.PI - angle;
//        static double cardThicknessVerticalMargin = cardThickness / 2 * Math.Sin(MathHelper.PiOver2 - angle);
//        static double cardThicknessHorizontalMargin = cardThickness / 2 * Math.Cos(MathHelper.PiOver2 - angle);
//        static float layerHeight = (float)(cardHeight * Math.Sin(angle) + 2 * cardThicknessVerticalMargin);
//        static float cardSpacing = (float)(cardHeight * Math.Cos(angle) + 2 * cardThicknessHorizontalMargin);

//        public override void Build()
//        {
//            Demo.World.ContactSettings.AllowedPenetration = 0.001f;
//            Demo.World.ContactSettings.BiasFactor = 0.05f;

//            // Demo.World.SetIterations(60, 5);
//            AddGround();

//            for (int layer = 0; layer < cardHouseLayers; layer++)
//            {
//                int layerCards = (cardHouseLayers - layer) * 2;

//                AddCardLayer(
//                    cardHouseStartingPosition +
//                        new JVector(cardSpacing * layer, (layerHeight + (float)(2 * cardThickness)) * layer, 0),
//                    layerCards);
//            }
//        }

//        private void AddCardLayer(JVector startPosition, int angledCards)
//        {
//            Debug.Assert(angledCards % 2 == 0);

//            foreach (int i in Enumerable.Range(0, angledCards))
//            {
//                AddCard(
//                    startPosition + new JVector(cardSpacing * i, layerHeight / 2f, 0),
//                    (i % 2 == 0) ? angle : oppositeAngle);
//            }

//            for (float distance = 1.5f; distance < angledCards - 0.5; distance += 4)
//            {
//                AddCard(startPosition + new JVector(cardSpacing * distance, layerHeight, 0), 0);
//            }

//            for (float distance = 3.5f; distance < angledCards - 0.5; distance += 4)
//            {
//                AddCard(startPosition + new JVector(cardSpacing * distance, layerHeight + (float)cardThickness, 0), 0);
//            }
//        }

//        private void AddCard(JVector position, float rollOrientation)
//        {
//            RigidBody body = new RigidBody(new BoxShape((float)cardHeight, (float)cardThickness, (float)cardWidth));
//            body.Mass = 0.5f;
//            body.Material.Restitution = 0;
//            body.Position = position;
//            if (rollOrientation != 0)
//            {
//                body.Orientation = JMatrix.CreateFromYawPitchRoll(0, 0, rollOrientation);
//            }

//            Demo.World.AddBody(body);
//        }

//        public override void Destroy()
//        {
//            RemoveGround();
//            Demo.World.Clear();
//        }

//    }

//}