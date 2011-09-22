using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter.Collision.Shapes;
using Jitter.LinearMath;
using Jitter.Dynamics;
using Microsoft.Xna.Framework;
using Jitter;
using Jitter.Dynamics.Constraints;
using JitterDemo.Classes_from_Matt;

namespace JitterDemo.Scenes
{

    public class EmptyScene : Scene
    {
        Tile[,] Map = new Tile[500, 500];

        public List<RigidBody> BodyList = new List<RigidBody>();
        Dictionary<Point, RigidBody> affectedTiles = new Dictionary<Point, RigidBody>();
        Dictionary<Point, RigidBody> previousAffectedTiles = new Dictionary<Point, RigidBody>();
        BodyPool BodyPool = new BodyPool();

        public EmptyScene(JitterDemo demo)
            : base(demo)
        {
        }

        Random rnd = new Random();

        public override void Build()
        {
            AddGround();

            // create a sine wave of tiles
            Random r = new Random();

            for (int y = 0; y < 500; y++)
            {
                for (int x = 0; x < 500; x++)
                {
                    Map[x, y] = new Tile()
                    {
                        HasPhysicsBody = false,
                        IsSolid = false,
                    };

                    if (y < (Math.Sin(x * 0.05f) * 15))
                    {
                        Map[x, y].IsSolid = true;
                    }
                }
            }

            // here we add a simple small pyramid
            for (int i = 0; i < 5; i++)
            {
                for (int e = i; e < 5; e++)
                {
                    RigidBody body = new RigidBody(new BoxShape(new JVector(1.0f, 1.0f, 1.0f)));
                    body.Position = new JVector((e - i * 0.5f) * 1.01f + 7, 0.5f + i * 1.0f + 15f, 0.0f);
                    body.Material.Restitution = 0.0f;
                    // add the dynamic body to the engine
                    Demo.World.AddBody(body);
                    // add 2D constraints
                    body.Make2D(Demo.World);
                    // add the body to our list
                    BodyList.Add(body);
                }
            }

        }

        public override void Draw()
        {
            // clear all non affected tiles from the screen area
            // normally we only do the tiles that are on screen + a decent border
            for (int y = 0; y < 500; y++)
            {
                for (int x = 0; x < 500; x++)
                {
                    if (x < 0 || y < 0 || x > 500 || y > 500)
                        continue;

                    var tile = Map[x, y];

                    if (!affectedTiles.ContainsKey(new Point(x, y)) && tile.HasPhysicsBody)
                    {
                        RigidBody body;
                        var yes = previousAffectedTiles.TryGetValue(new Point(x, y), out body);
                        if (yes)
                        {
                            Map[x, y].HasPhysicsBody = false;
                            this.Demo.World.RemoveBody(body);
                            previousAffectedTiles.Remove(new Point(x, y));
                            BodyPool.GiveBack(body);
                        }
                    }
                }
            }

            affectedTiles.Clear();

            foreach (var item in BodyList)
            {
                if (item.IsActive)
                {
                    JBBox bbox;
                    var position = item.Position;
                    var futurePosition = item.Position + item.LinearVelocity;

                    bbox = item.BoundingBox;

                    int tileX = ((int)(bbox.Min.X / 1f)) - 1;
                    int maxTilesX = ((int)(bbox.Max.X / 1f)) + 2;
                    int tileY = ((int)(bbox.Min.Y / 1f)) - 1;
                    int maxTilesY = ((int)(bbox.Max.Y / 1f)) + 2;

                    // find tiles at the objects future position

                    int tileFutureX = ((int)(bbox.Min.X / 1)) - 1;
                    int maxFutureTilesX = ((int)(bbox.Max.X / 1)) + 2;
                    int tileFutureY = ((int)(bbox.Min.Y / 1)) - 1;
                    int maxFutureTilesY = ((int)(bbox.Max.Y / 1)) + 2;

                    // adjust the tile field to match all tiles from current to future positions
                    // NOTE: for fast moving objects this can potentially lead to a lot of tiles being tested

                    if (tileFutureX < tileX)
                        tileX = tileFutureX;
                    if (maxFutureTilesX > maxTilesX)
                        maxTilesX = maxFutureTilesX;
                    if (tileFutureY < tileY)
                        tileY = tileFutureY;
                    if (maxFutureTilesY > maxTilesY)
                        maxTilesY = maxFutureTilesY;

                    // for each tile perform a collision detection
                    for (int u = tileX; u < maxTilesX; u++)
                    {
                        for (int v = tileY; v < maxTilesY; v++)
                        {
                            if (u < 0 || v < 0 || u > 500 || v > 500)
                                continue;
                            // cache the tile
                            var tile = Map[u, v];

                            // make sure the tile is active
                            if (tile.IsSolid && !tile.HasPhysicsBody)
                            {
                                Vector2 tilePosition;
                                tilePosition.X = u * 1f;
                                tilePosition.Y = v * 1f;

                                // here we should just get a pre-allocated body from the pool and set it's position
                                var body = BodyPool.GetNew();
                                body.IsStatic = true;
                                body.Position = Conversion.ToJitterVector(new Vector3(tilePosition.X, tilePosition.Y, 0));
                                this.Demo.World.AddBody(body);

                                tile.HasPhysicsBody = true;

                                // mark affected tiles
                                if (!affectedTiles.ContainsKey(new Point(u, v)))
                                    affectedTiles.Add(new Point(u, v), body);
                            }

                            Map[u, v] = tile;
                        }
                    }

                    foreach (var item2 in affectedTiles)
                    {
                        if (!previousAffectedTiles.ContainsKey(item2.Key))
                            previousAffectedTiles.Add(item2.Key, item2.Value);
                    }
                }
            }

            base.Draw();
        }

        public override void Destroy()
        {
            RemoveGround();
        }
    }


}
