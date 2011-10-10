#region Using Statements
using System;
using System.Collections.Generic;

using Jitter2D.Dynamics;
using Jitter2D.LinearMath;
using Jitter2D.Collision.Shapes;
#endregion

namespace Jitter2D.Collision
{
    public class CollisionSystemDynamicTree : CollisionSystem
    {
        private Action<object> detectCallback;
        private List<int> proxyBuffer;
        private DynamicTree<IBroadphaseEntity> dt = new DynamicTree<IBroadphaseEntity>();
        private int proxyCount;
        private Func<int, bool> _queryCallback;

        /// <summary>
        /// Creates a new instance of the CollisionSystemDynamicTree class.
        /// </summary>
        public CollisionSystemDynamicTree()
        {
            detectCallback = new Action<object>(DetectCallback);
            _queryCallback = new Func<int, bool>(QueryCallback);
            proxyBuffer = new List<int>(100);
        }

        public override bool RemoveEntity(IBroadphaseEntity body)
        {
            proxyBuffer.Remove(body.BroadphaseTag);
            --proxyCount;
            dt.RemoveProxy(body.BroadphaseTag);
            return true;
        }

        public override void AddEntity(IBroadphaseEntity body)
        {
            JBBox bb = body.BoundingBox;
            body.BroadphaseTag = dt.AddProxy(ref bb, body);
            proxyBuffer.Add(body.BroadphaseTag);
            ++proxyCount;
        }

        public override bool Raycast(JVector rayOrigin, JVector rayDirection, RaycastCallback raycast, out RigidBody body, out JVector normal, out float fraction)
        {
            throw new NotImplementedException();
        }

        public override bool Raycast(RigidBody body, JVector rayOrigin, JVector rayDirection, out JVector normal, out float fraction)
        {
            throw new NotImplementedException();
        }

        public override void Detect(bool multiThreaded)
        {
            if (multiThreaded)
            {
                // create a threaded task for each moving proxy
            }
            else
            {
                // just loop over all the moving proxies
                foreach (var proxy in proxyBuffer)
                {
                    // We have to query the tree with the fat AABB so that
                    // we don't fail to create a pair that may touch later.
                    JBBox fatAABB;
                    dt.GetFatAABB(proxy, out fatAABB);

                    // Query tree, create pairs and add them pair buffer.
                    //dt.Query(
                }
            }
        }

        private bool QueryCallback(int proxyId)
        {
            // A proxy cannot form a pair with itself.
            //if (proxyId == _queryProxyId)
            {
                return true;
            }
        }

        private void DetectCallback(object obj)
        {
            Pair pair = obj as Pair;
            base.Detect(pair.entity1, pair.entity2);
            Pair.Pool.GiveBack(pair);
        }
    }
}
