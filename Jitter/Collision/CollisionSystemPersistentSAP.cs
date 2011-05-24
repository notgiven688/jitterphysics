/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

#region Using Statements
using System;
using System.Collections.Generic;

using Jitter.Dynamics;
using Jitter.LinearMath;
using Jitter.Collision.Shapes;
using System.Collections;
#endregion

namespace Jitter.Collision
{
    /// <summary>
    /// EXPERIMENTAL: FULL SAP WITH COHERENT UPDATES
    /// </summary>
    public class CollisionSystemPersistentSAP : CollisionSystem
    {
        private const int MatrixGrowFactor = 500;
        private const int AddedObjectsBruteForceIsUsed = 250;

        #region private class Triangular2BitMatrix
        private class Triangular2BitMatrix
        {
            private BitArray bitMatrix0 = new BitArray(0);
            private BitArray bitMatrix1 = new BitArray(0);

            public Triangular2BitMatrix()
            {
                Size = 0;
            }

            public void ZeroAll()
            {
                bitMatrix0.SetAll(false);
                bitMatrix1.SetAll(false);
            }

            public int Size
            {
                set
                {
                    int length = (int)((double)(value - 1.0d) * ((double)value / 2.0d)) + value;
                    bitMatrix0.Length = length;
                    bitMatrix1.Length = length;
                }
                get
                {
                    int maxIndex = bitMatrix0.Length - 1;
                    return (int)(Math.Sqrt(2.25d + 2.0d * (double)maxIndex) - 0.5d);
                }
            }

            public int GetValue(int row, int column)
            {
                if (column > row) { int temp = column; column = row; row = temp; }
                int index = (int)((float)(row - 1) * ((float)row / 2.0f)) + column;

                return (bitMatrix1[index] ? 1 : 0) * 2 + (bitMatrix0[index] ? 1 : 0);
            }

            public void SetValue(int row, int column, int value)
            {
                if (column > row) { int temp = column; column = row; row = temp; }
                int index = (int)((float)(row - 1) * ((float)row / 2.0f)) + column;

                bitMatrix0[index] = !(value % 2 == 0);
                bitMatrix1[index] = !(value / 2 == 0);
            }

            public int IncrementCounter(int row, int column)
            {
                if (column > row) { int temp = column; column = row; row = temp; }
                int index = (int)((float)(row - 1) * ((float)row / 2.0f)) + column;

                bool b0 = bitMatrix0[index];
                bool b1 = bitMatrix1[index];

                if (b0 && b1) return 3;

                b1 = b0 ^ b1;
                b0 = !b0;

                bitMatrix0[index] = b0;
                bitMatrix1[index] = b1;

                return (b1 ? 1 : 0) * 2 + (b0 ? 1 : 0);
            }

            public int DecrementCounter(int row, int column)
            {
                if (column > row) { int temp = column; column = row; row = temp; }
                int index = (int)((float)(row - 1) * ((float)row / 2.0f)) + column;

                bool b0 = bitMatrix0[index];
                bool b1 = bitMatrix1[index];

                if (!b0 && !b1) return 0;

                b1 = !(b0 ^ b1);
                b0 = !b0;

                bitMatrix0[index] = b0;
                bitMatrix1[index] = b1;

                return (b1 ? 1 : 0) * 2 + (b0 ? 1 : 0);
            }
        }
        #endregion

        #region private class SweepPoint
        private class SweepPoint
        {
            public IBroadphaseEntity Body;
            public bool Begin;
            public int Axis;

            public SweepPoint(IBroadphaseEntity body, bool begin, int axis)
            {
                this.Body = body;
                this.Begin = begin;
                this.Axis = axis;
            }

            public float GetValue()
            {
                 if (Begin)
                {
                    if (Axis == 0) return Body.BoundingBox.Min.X;
                    else if (Axis == 1) return Body.BoundingBox.Min.Y;
                    else return Body.BoundingBox.Min.Z;
                }
                else
                {
                    if (Axis == 0) return Body.BoundingBox.Max.X;
                    else if (Axis == 1) return Body.BoundingBox.Max.Y;
                    else return Body.BoundingBox.Max.Z;
                }
            }
        }
        #endregion

        #region private struct BodyPair
        private struct BodyPair
        {
            // internal values for faster access within the engine
            internal IBroadphaseEntity body1, body2;

            /// <summary>
            /// Initializes a new instance of the BodyPair class.
            /// </summary>
            /// <param name="body1"></param>
            /// <param name="body2"></param>
            public BodyPair(IBroadphaseEntity body1, IBroadphaseEntity body2)
            {
                this.body1 = body1;
                this.body2 = body2;
            }

            /// <summary>
            /// Don't call this, while the key is used in the arbitermap.
            /// It changes the hashcode of this object.
            /// </summary>
            /// <param name="body1">The first body.</param>
            /// <param name="body2">The second body.</param>
            internal void SetBodies(RigidBody body1, RigidBody body2)
            {
                this.body1 = body1;
                this.body2 = body2;
            }

            /// <summary>
            /// Checks if two objects are equal.
            /// </summary>
            /// <param name="obj">The object to check against.</param>
            /// <returns>Returns true if they are equal, otherwise false.</returns>
            public override bool Equals(object obj)
            {
                BodyPair other = (BodyPair)obj;
                return (other.body1.Equals(body1) && other.body2.Equals(body2) ||
                    other.body1.Equals(body2) && other.body2.Equals(body1));
            }

            /// <summary>
            /// Returns the hashcode of the BodyPair.
            /// The hashcode is the same if an BodyPair contains the same bodies.
            /// </summary>
            /// <returns></returns>
            public override int GetHashCode()
            {
                return body1.GetHashCode() + body2.GetHashCode();
            }
        }
        #endregion

        private List<IBroadphaseEntity> bodyList = new List<IBroadphaseEntity>();
        private List<SweepPoint> axis1 = new List<SweepPoint>();
        private List<SweepPoint> axis2 = new List<SweepPoint>();
        private List<SweepPoint> axis3 = new List<SweepPoint>();

        private Triangular2BitMatrix t2bM;


        private HashSet<BodyPair> fullOverlaps = new HashSet<BodyPair>();

        Action<object> detectCallback, sortCallback;

        public CollisionSystemPersistentSAP()
        {
            detectCallback = new Action<object>(DetectCallback);
            sortCallback = new Action<object>(SortCallback);

            t2bM = new Triangular2BitMatrix();
        }

        #region Incoherent Update - Quicksort

        private int QuickSort(SweepPoint sweepPoint1, SweepPoint sweepPoint2)
        {
            float val1 = sweepPoint1.GetValue();
            float val2 = sweepPoint2.GetValue();

            if (val1 > val2) return 1;
            else if (val2 > val1) return -1;
            else return 0;
        }

        List<IBroadphaseEntity> activeList = new List<IBroadphaseEntity>();

        private void DirtySortAxis(List<SweepPoint> axis)
        {
            axis.Sort(QuickSort);
            activeList.Clear();

            for (int i = 0; i < axis.Count; i++)
            {
                SweepPoint keyelement = axis[i];

                if (keyelement.Begin)
                {
                    foreach (IBroadphaseEntity body in activeList)
                    {
                        int count = t2bM.IncrementCounter(body.BroadphaseTag, keyelement.Body.BroadphaseTag);
                        if (count == 3) fullOverlaps.Add(new BodyPair(body, keyelement.Body));
                    }

                    activeList.Add(keyelement.Body);
                }
                else
                {
                    activeList.Remove(keyelement.Body);
                }
            }
        }
        #endregion

        #region Coherent Update - Insertionsort

        private void SortAxis(List<SweepPoint> axis)
        {
            for (int j = 1; j < axis.Count; j++)
            {
                SweepPoint keyelement = axis[j];
                float key = keyelement.GetValue();

                int i = j - 1;

                while (i >= 0 && axis[i].GetValue() > key)
                {
                    SweepPoint swapper = axis[i];

                    if (keyelement.Begin && !swapper.Begin)
                    {
                        lock (t2bM)
                        {
                            int count = t2bM.IncrementCounter(keyelement.Body.BroadphaseTag,
                                swapper.Body.BroadphaseTag);

                            if (count == 3)
                            {
                                BodyPair pair = new BodyPair(keyelement.Body, swapper.Body);
                                fullOverlaps.Add(pair);
                            }
                        }
                    }

                    if (!keyelement.Begin && swapper.Begin)
                    {
                        lock (t2bM)
                        {
                            int count = t2bM.DecrementCounter(keyelement.Body.BroadphaseTag,
                                swapper.Body.BroadphaseTag);

                            if (count == 2)
                            {
                                BodyPair pair = new BodyPair(keyelement.Body, swapper.Body);
                                fullOverlaps.Remove(pair);
                            }
                        }
                    }

                    axis[i + 1] = swapper;
                    i = i - 1;
                }
                axis[i + 1] = keyelement;
            }
        }
        #endregion

        private void ResizeMatrix(int growValue)
        {
            if (bodyList.Count > t2bM.Size)
            {
                t2bM.Size += growValue;
            }
            else if (t2bM.Size - bodyList.Count > growValue)
            {
                t2bM.Size -= growValue;
            }
        }

        int addCounter = 0;
        public override void AddEntity(IBroadphaseEntity body)
        {
            if (body.BroadphaseTag < bodyList.Count && bodyList[body.BroadphaseTag] == body) return;

            body.BroadphaseTag = bodyList.Count;

            bodyList.Add(body);

            axis1.Add(new SweepPoint(body, true, 0)); axis1.Add(new SweepPoint(body, false, 0));
            axis2.Add(new SweepPoint(body, true, 1)); axis2.Add(new SweepPoint(body, false, 1));
            axis3.Add(new SweepPoint(body, true, 2)); axis3.Add(new SweepPoint(body, false, 2));

            ResizeMatrix(MatrixGrowFactor);

            addCounter++;
        }

        Stack<BodyPair> depricated = new Stack<BodyPair>();
        public override bool RemoveEntity(IBroadphaseEntity body)
        {
            if (body.BroadphaseTag > bodyList.Count || bodyList[body.BroadphaseTag] != body) return false;

            int count;

            count = 0;
            for (int i = 0; i < axis1.Count; i++)
            { if (axis1[i].Body == body) { count++; axis1.RemoveAt(i); if (count == 2) break; i--; } }

            count = 0;
            for (int i = 0; i < axis2.Count; i++)
            { if (axis2[i].Body == body) { count++; axis2.RemoveAt(i); if (count == 2) break; i--; } }

            count = 0;
            for (int i = 0; i < axis3.Count; i++)
            { if (axis3[i].Body == body) { count++; axis3.RemoveAt(i); if (count == 2) break; i--; } }

            foreach (var pair in fullOverlaps) if (pair.body1 == body || pair.body2 == body) depricated.Push(pair);
            while (depricated.Count > 0) fullOverlaps.Remove(depricated.Pop());

            IBroadphaseEntity lastBody = bodyList[bodyList.Count - 1];

            if (body == lastBody)
            {
                for (int i = 0; i < bodyList.Count; i++)
                {
                    t2bM.SetValue(body.BroadphaseTag, i, 0);
                }

                bodyList.RemoveAt(body.BroadphaseTag);
            }
            else
            {
                for (int i = 0; i < bodyList.Count; i++)
                {
                    int value = t2bM.GetValue(lastBody.BroadphaseTag, i);
                    t2bM.SetValue(body.BroadphaseTag, i, value);
                }

                bodyList.RemoveAt(lastBody.BroadphaseTag);
                bodyList[body.BroadphaseTag] = lastBody;

                lastBody.BroadphaseTag = body.BroadphaseTag;
            }

            ResizeMatrix(MatrixGrowFactor);

            return true;
        }

        bool swapOrder = false;

        /// <summary>
        /// Tells the collisionsystem to check all bodies for collisions. Hook into the
        /// <see cref="CollisionSystem.PassedBroadphase"/>
        /// and <see cref="CollisionSystem.CollisionDetected"/> events to get the results.
        /// </summary>
        /// <param name="multiThreaded">If true internal multithreading is used.</param>
        public override void Detect(bool multiThreaded)
        {
            if (addCounter > AddedObjectsBruteForceIsUsed)
            {
                t2bM.ZeroAll();
                fullOverlaps.Clear();

                DirtySortAxis(axis1);
                DirtySortAxis(axis2);
                DirtySortAxis(axis3);
            }
            else
            {
                if (multiThreaded)
                {
                    ThreadManager.internalInstance.AddTask(sortCallback, axis1);
                    ThreadManager.internalInstance.AddTask(sortCallback, axis2);
                    ThreadManager.internalInstance.AddTask(sortCallback, axis3);

                    ThreadManager.internalInstance.Execute();
                }
                else
                {
                    SortAxis(axis1);
                    SortAxis(axis2);
                    SortAxis(axis3);
                }
            }

            addCounter = 0;

            if (multiThreaded)
            {
                foreach (BodyPair key in fullOverlaps)
                {
                    if (this.CheckBothStaticOrInactive(key.body1, key.body2)) continue;

                    if (base.RaisePassedBroadphase(key.body1, key.body2))
                    {
                        Pair pair = Pair.Pool.GetNew();

                        if (swapOrder) { pair.entity1 = key.body1; pair.entity2 = key.body2; }
                        else { pair.entity2 = key.body2; pair.entity1 = key.body1; }
                        swapOrder = !swapOrder;

                        ThreadManager.internalInstance.AddTask(detectCallback, pair);
                    }
                }

                ThreadManager.internalInstance.Execute();
            }
            else
            {
                foreach (BodyPair key in fullOverlaps)
                {
                    if (this.CheckBothStaticOrInactive(key.body1, key.body2)) continue;

                    if (base.RaisePassedBroadphase(key.body1, key.body2))
                    {

                        if (swapOrder) { Detect(key.body1, key.body2); }
                        else Detect(key.body2, key.body1);
                        swapOrder = !swapOrder;

                    }
                }
            }
        }

        private void SortCallback(object obj)
        {
            SortAxis(obj as List<SweepPoint>);
        }

        private void DetectCallback(object obj)
        {
            Pair pair = obj as Pair;
            base.Detect(pair.entity1, pair.entity2);
            Pair.Pool.GiveBack(pair);
        }

        /// <summary>
        /// Sends a ray (definied by start and direction) through the scene (all bodies added).
        /// NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        #region public override bool Raycast(JVector rayOrigin, JVector rayDirection, out JVector normal,out float fraction)
        public override bool Raycast(JVector rayOrigin, JVector rayDirection, RaycastCallback raycast, out RigidBody body, out JVector normal, out float fraction)
        {
            body = null; normal = JVector.Zero; fraction = float.MaxValue;

            JVector tempNormal;float tempFraction;
            bool result = false;

            // TODO: This can be done better in CollisionSystemPersistenSAP
            foreach (IBroadphaseEntity e in bodyList)
            {
                if (e is SoftBody)
                {
                    SoftBody softBody = e as SoftBody;
                    foreach (RigidBody b in softBody.points)
                    {
                        if (this.Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction))
                        {
                            if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                            {
                                body = b;
                                normal = tempNormal;
                                fraction = tempFraction;
                                result = true;
                            }
                        }
                    }
                }
                else
                {
                    RigidBody b = e as RigidBody;

                    if (this.Raycast(b, rayOrigin, rayDirection, out tempNormal, out tempFraction))
                    {
                        if (tempFraction < fraction && (raycast == null || raycast(b, tempNormal, tempFraction)))
                        {
                            body = b;
                            normal = tempNormal;
                            fraction = tempFraction;
                            result = true;
                        }
                    }
                }
            }

            return result;
        }
        #endregion


        /// <summary>
        /// Raycasts a single body. NOTE: For performance reasons terrain and trianglemeshshape aren't checked
        /// against rays (rays are of infinite length). They are checked against segments
        /// which start at rayOrigin and end in rayOrigin + rayDirection.
        /// </summary>
        #region public override bool Raycast(RigidBody body, JVector rayOrigin, JVector rayDirection, out JVector normal, out float fraction)
        public override bool Raycast(RigidBody body, JVector rayOrigin, JVector rayDirection, out JVector normal, out float fraction)
        {
            fraction = float.MaxValue; normal = JVector.Zero;

            if (!body.BoundingBox.RayIntersect(ref rayOrigin, ref rayDirection)) return false;

            if (body.Shape is Multishape)
            {
                Multishape ms = (body.Shape as Multishape).RequestWorkingClone();
                
                JVector tempNormal;float tempFraction;
                bool multiShapeCollides = false;

                JVector transformedOrigin; JVector.Subtract(ref rayOrigin, ref body.position, out transformedOrigin);
                JVector.Transform(ref transformedOrigin, ref body.invOrientation, out transformedOrigin);
                JVector transformedDirection; JVector.Transform(ref rayDirection, ref body.invOrientation, out transformedDirection);

                int msLength = ms.Prepare(ref transformedOrigin, ref transformedDirection);

                for (int i = 0; i < msLength; i++)
                {
                    ms.SetCurrentShape(i);

                    if (GJKCollide.Raycast(ms, ref body.orientation, ref body.invOrientation, ref body.position,
                        ref rayOrigin, ref rayDirection, out tempFraction, out tempNormal))
                    {
                        if (tempFraction < fraction)
                        {
                            if (useTerrainNormal && ms is TerrainShape)
                            {
                                (ms as TerrainShape).CollisionNormal(out tempNormal);
                                JVector.Transform(ref tempNormal, ref body.orientation, out tempNormal);
                                tempNormal.Negate();
                            }
                            else if (useTriangleMeshNormal && ms is TriangleMeshShape)
                            {
                                (ms as TriangleMeshShape).CollisionNormal(out tempNormal);
                                JVector.Transform(ref tempNormal, ref body.orientation, out tempNormal);
                                tempNormal.Negate();
                            }

                            normal = tempNormal;
                            fraction = tempFraction;
                            multiShapeCollides = true;
                        }
                    }
                }

                ms.ReturnWorkingClone();
                return multiShapeCollides;
            }
            else
            {
                return (GJKCollide.Raycast(body.Shape, ref body.orientation, ref body.invOrientation, ref body.position,
                    ref rayOrigin, ref rayDirection, out fraction, out normal));
            }


        }
        #endregion


    }
}
