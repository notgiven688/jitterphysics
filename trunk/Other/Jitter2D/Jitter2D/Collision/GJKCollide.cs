using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Jitter2D.LinearMath;

namespace Jitter2D.Collision
{
    public sealed class GJKCollide
    {
        public static int MaxIterations = 15;

        public static int IterationsTaken;

        private static ResourcePool<VoronoiSimplexSolver> simplexSolverPool = new ResourcePool<VoronoiSimplexSolver>();

        #region private static void SupportMapTransformed(ISupportMappable support, ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        public static void SupportMapTransformed(ISupportMappable support,
            ref JMatrix orientation, ref JVector position, ref JVector direction, out JVector result)
        {
            // THIS IS *THE* HIGH FREQUENCY CODE OF THE COLLLISION PART OF THE ENGINE
            result.X = ((direction.X * orientation.M11) + (direction.Y * orientation.M12));
            result.Y = ((direction.X * orientation.M21) + (direction.Y * orientation.M22));

            support.SupportMapping(ref result, out result);

            float x = ((result.X * orientation.M11) + (result.Y * orientation.M21));
            float y = ((result.X * orientation.M12) + (result.Y * orientation.M22));

            result.X = position.X + x;
            result.Y = position.Y + y;
        }
        #endregion

        public static bool ClosestPoints(ISupportMappable support1, ISupportMappable support2, ref JMatrix orientation1,
            ref JMatrix orientation2, ref JVector position1, ref JVector position2,
            out JVector p1, out JVector p2, out JVector normal)
        {

            VoronoiSimplexSolver simplexSolver = simplexSolverPool.GetNew();
            simplexSolver.Reset();

            p1 = p2 = JVector.Zero;

            JVector r = position1 - position2;
            JVector w, v;

            JVector supVertexA;
            JVector rn, vn;

            rn = JVector.Negate(r);

            SupportMapTransformed(support1, ref orientation1, ref position1, ref rn, out supVertexA);

            JVector supVertexB;
            SupportMapTransformed(support2, ref orientation2, ref position2, ref r, out supVertexB);

            v = supVertexA - supVertexB;

            normal = JVector.Zero;

            int iter = 0;

            float distSq = v.LengthSquared();
            float epsilon = 0.00001f;

            while ((distSq > epsilon) && (iter++ < MaxIterations))
            {
                IterationsTaken = iter;
                vn = JVector.Negate(v);
                SupportMapTransformed(support1, ref orientation1, ref position1, ref vn, out supVertexA);
                SupportMapTransformed(support2, ref orientation2, ref position2, ref v, out supVertexB);
                w = supVertexA - supVertexB;

                if (!simplexSolver.InSimplex(w)) simplexSolver.AddVertex(w, supVertexA, supVertexB);
                if (simplexSolver.Closest(out v))
                {
                    distSq = v.LengthSquared();
                    normal = v;
                }
                else distSq = 0.0f;
            }


            simplexSolver.ComputePoints(out p1, out p2);

            if (normal.LengthSquared() > JMath.Epsilon * JMath.Epsilon)
                normal.Normalize();

            simplexSolverPool.GiveBack(simplexSolver);

            return true;
        }

        // see: btVoronoiSimplexSolver.cpp
        #region private class VoronoiSimplexSolver - Bullet

        // Bullet has problems with raycasting large objects - so does jitter
        // hope to fix that in the next versions.

        /*
          Bullet for XNA Copyright (c) 2003-2007 Vsevolod Klementjev http://www.codeplex.com/xnadevru
          Bullet original C++ version Copyright (c) 2003-2007 Erwin Coumans http://bulletphysics.com

          This software is provided 'as-is', without any express or implied
          warranty.  In no event will the authors be held liable for any damages
          arising from the use of this software.

          Permission is granted to anyone to use this software for any purpose,
          including commercial applications, and to alter it and redistribute it
          freely, subject to the following restrictions:

          1. The origin of this software must not be misrepresented; you must not
             claim that you wrote the original software. If you use this software
             in a product, an acknowledgment in the product documentation would be
             appreciated but is not required.
          2. Altered source versions must be plainly marked as such, and must not be
             misrepresented as being the original software.
          3. This notice may not be removed or altered from any source distribution.
        */

        private class UsageBitfield
        {
            private bool _usedVertexA, _usedVertexB, _usedVertexC;

            public bool UsedVertexA { get { return _usedVertexA; } set { _usedVertexA = value; } }
            public bool UsedVertexB { get { return _usedVertexB; } set { _usedVertexB = value; } }
            public bool UsedVertexC { get { return _usedVertexC; } set { _usedVertexC = value; } }

            public void Reset()
            {
                _usedVertexA = _usedVertexB = _usedVertexC = false;
            }
        }

        private class SubSimplexClosestResult
        {
            private JVector _closestPointOnSimplex;

            //MASK for m_usedVertices
            //stores the simplex vertex-usage, using the MASK, 
            // if m_usedVertices & MASK then the related vertex is used
            private UsageBitfield _usedVertices = new UsageBitfield();
            private float[] _barycentricCoords = new float[4];
            private bool _degenerate;

            public JVector ClosestPointOnSimplex { get { return _closestPointOnSimplex; } set { _closestPointOnSimplex = value; } }
            public UsageBitfield UsedVertices { get { return _usedVertices; } set { _usedVertices = value; } }
            public float[] BarycentricCoords { get { return _barycentricCoords; } set { _barycentricCoords = value; } }
            public bool Degenerate { get { return _degenerate; } set { _degenerate = value; } }

            public void Reset()
            {
                _degenerate = false;
                SetBarycentricCoordinates();
                _usedVertices.Reset();
            }

            public bool IsValid
            {
                get
                {
                    return (_barycentricCoords[0] >= 0f) &&
                            (_barycentricCoords[1] >= 0f) &&
                            (_barycentricCoords[2] >= 0f);
                }
            }

            public void SetBarycentricCoordinates()
            {
                SetBarycentricCoordinates(0f, 0f, 0f);
            }

            public void SetBarycentricCoordinates(float a, float b, float c)
            {
                _barycentricCoords[0] = a;
                _barycentricCoords[1] = b;
                _barycentricCoords[2] = c;
            }
        }

        /// VoronoiSimplexSolver is an implementation of the closest point distance
        /// algorithm from a 1-3 points simplex to the origin.
        /// Can be used with GJK, as an alternative to Johnson distance algorithm. 
        private class VoronoiSimplexSolver
        {
            private const int VertexA = 0, VertexB = 1, VertexC = 2;

            private const int VoronoiSimplexMaxVerts = 4;
            private const bool CatchDegenerateTetrahedron = true;

            private int _numVertices;

            private JVector[] _simplexVectorW = new JVector[VoronoiSimplexMaxVerts];
            private JVector[] _simplexPointsP = new JVector[VoronoiSimplexMaxVerts];
            private JVector[] _simplexPointsQ = new JVector[VoronoiSimplexMaxVerts];

            private JVector _cachedPA;
            private JVector _cachedPB;
            private JVector _cachedV;
            private JVector _lastW;
            private bool _cachedValidClosest;

            private SubSimplexClosestResult _cachedBC = new SubSimplexClosestResult();

            // Note that this assumes ray-casts and point-casts will always be called from the
            // same thread which I assume is true from the _cachedBC member
            // If this needs to made multi-threaded a resource pool will be needed
            private SubSimplexClosestResult tempResult = new SubSimplexClosestResult();

            private bool _needsUpdate;

            #region ISimplexSolver Members

            public bool FullSimplex
            {
                get
                {
                    return _numVertices == 3;
                }
            }

            public int NumVertices
            {
                get
                {
                    return _numVertices;
                }
            }

            public void Reset()
            {
                _cachedValidClosest = false;
                _numVertices = 0;
                _needsUpdate = true;
                _lastW = new JVector(1e30f, 1e30f);
                _cachedBC.Reset();
            }

            public void AddVertex(JVector w, JVector p, JVector q)
            {
                _lastW = w;
                _needsUpdate = true;

                _simplexVectorW[_numVertices] = w;
                _simplexPointsP[_numVertices] = p;
                _simplexPointsQ[_numVertices] = q;

                _numVertices++;
            }

            //return/calculate the closest vertex
            public bool Closest(out JVector v)
            {
                bool succes = UpdateClosestVectorAndPoints();
                v = _cachedV;
                return succes;
            }

            public float MaxVertex
            {
                get
                {
                    int numverts = NumVertices;
                    float maxV = 0f, curLen2;
                    for (int i = 0; i < numverts; i++)
                    {
                        curLen2 = _simplexVectorW[i].LengthSquared();
                        if (maxV < curLen2) maxV = curLen2;
                    }
                    return maxV;
                }
            }

            //return the current simplex
            public int GetSimplex(out JVector[] pBuf, out JVector[] qBuf, out JVector[] yBuf)
            {
                int numverts = NumVertices;
                pBuf = new JVector[numverts];
                qBuf = new JVector[numverts];
                yBuf = new JVector[numverts];
                for (int i = 0; i < numverts; i++)
                {
                    yBuf[i] = _simplexVectorW[i];
                    pBuf[i] = _simplexPointsP[i];
                    qBuf[i] = _simplexPointsQ[i];
                }
                return numverts;
            }

            public bool InSimplex(JVector w)
            {
                //check in case lastW is already removed
                if (w == _lastW) return true;

                //w is in the current (reduced) simplex
                int numverts = NumVertices;
                for (int i = 0; i < numverts; i++)
                    if (_simplexVectorW[i] == w) return true;

                return false;
            }

            public void BackupClosest(out JVector v)
            {
                v = _cachedV;
            }

            public bool EmptySimplex
            {
                get
                {
                    return NumVertices == 0;
                }
            }

            public void ComputePoints(out JVector p1, out JVector p2)
            {
                UpdateClosestVectorAndPoints();
                p1 = _cachedPA;
                p2 = _cachedPB;
            }

            #endregion

            public void RemoveVertex(int index)
            {
                _numVertices--;
                _simplexVectorW[index] = _simplexVectorW[_numVertices];
                _simplexPointsP[index] = _simplexPointsP[_numVertices];
                _simplexPointsQ[index] = _simplexPointsQ[_numVertices];
            }

            public void ReduceVertices(UsageBitfield usedVerts)
            {
                if ((NumVertices >= 3) && (!usedVerts.UsedVertexC)) RemoveVertex(2);
                if ((NumVertices >= 2) && (!usedVerts.UsedVertexB)) RemoveVertex(1);
                if ((NumVertices >= 1) && (!usedVerts.UsedVertexA)) RemoveVertex(0);
            }

            public bool UpdateClosestVectorAndPoints()
            {
                if (_needsUpdate)
                {
                    _cachedBC.Reset();
                    _needsUpdate = false;

                    JVector p, a, b, c, d;
                    switch (NumVertices)
                    {
                        case 0:
                            _cachedValidClosest = false;
                            break;
                        case 1:
                            _cachedPA = _simplexPointsP[0];
                            _cachedPB = _simplexPointsQ[0];
                            _cachedV = _cachedPA - _cachedPB;
                            _cachedBC.Reset();
                            _cachedBC.SetBarycentricCoordinates(1f, 0f, 0f);
                            _cachedValidClosest = _cachedBC.IsValid;
                            break;
                        case 2:
                            //closest point origin from line segment
                            JVector from = _simplexVectorW[0];
                            JVector to = _simplexVectorW[1];
                            JVector nearest;

                            JVector diff = from * (-1);
                            JVector v = to - from;
                            float t = JVector.Dot(v, diff);

                            if (t > 0)
                            {
                                float dotVV = v.LengthSquared();
                                if (t < dotVV)
                                {
                                    t /= dotVV;
                                    diff -= t * v;
                                    _cachedBC.UsedVertices.UsedVertexA = true;
                                    _cachedBC.UsedVertices.UsedVertexB = true;
                                }
                                else
                                {
                                    t = 1;
                                    diff -= v;
                                    //reduce to 1 point
                                    _cachedBC.UsedVertices.UsedVertexB = true;
                                }
                            }
                            else
                            {
                                t = 0;
                                //reduce to 1 point
                                _cachedBC.UsedVertices.UsedVertexA = true;
                            }

                            _cachedBC.SetBarycentricCoordinates(1 - t, t, 0);
                            nearest = from + t * v;

                            _cachedPA = _simplexPointsP[0] + t * (_simplexPointsP[1] - _simplexPointsP[0]);
                            _cachedPB = _simplexPointsQ[0] + t * (_simplexPointsQ[1] - _simplexPointsQ[0]);
                            _cachedV = _cachedPA - _cachedPB;

                            ReduceVertices(_cachedBC.UsedVertices);

                            _cachedValidClosest = _cachedBC.IsValid;
                            break;
                        case 3:
                            //closest point origin from triangle
                            p = new JVector();
                            a = _simplexVectorW[0];
                            b = _simplexVectorW[1];
                            c = _simplexVectorW[2];

                            ClosestPtPointTriangle(p, a, b, c, ref _cachedBC);
                            _cachedPA = _simplexPointsP[0] * _cachedBC.BarycentricCoords[0] +
                                            _simplexPointsP[1] * _cachedBC.BarycentricCoords[1] +
                                            _simplexPointsP[2] * _cachedBC.BarycentricCoords[2] +
                                            _simplexPointsP[3] * _cachedBC.BarycentricCoords[3];

                            _cachedPB = _simplexPointsQ[0] * _cachedBC.BarycentricCoords[0] +
                                            _simplexPointsQ[1] * _cachedBC.BarycentricCoords[1] +
                                            _simplexPointsQ[2] * _cachedBC.BarycentricCoords[2] +
                                            _simplexPointsQ[3] * _cachedBC.BarycentricCoords[3];

                            _cachedV = _cachedPA - _cachedPB;

                            ReduceVertices(_cachedBC.UsedVertices);
                            _cachedValidClosest = _cachedBC.IsValid;
                            break;

                        default:
                            _cachedValidClosest = false;
                            break;
                    }
                }

                return _cachedValidClosest;
            }

            public bool ClosestPtPointTriangle(JVector p, JVector a, JVector b, JVector c,
                ref SubSimplexClosestResult result)
            {
                result.UsedVertices.Reset();

                float v, w;

                // Check if P in vertex region outside A
                JVector ab = b - a;
                JVector ac = c - a;
                JVector ap = p - a;
                float d1 = JVector.Dot(ab, ap);
                float d2 = JVector.Dot(ac, ap);
                if (d1 <= 0f && d2 <= 0f)
                {
                    result.ClosestPointOnSimplex = a;
                    result.UsedVertices.UsedVertexA = true;
                    result.SetBarycentricCoordinates(1, 0, 0);
                    return true; // a; // barycentric coordinates (1,0,0)
                }

                // Check if P in vertex region outside B
                JVector bp = p - b;
                float d3 = JVector.Dot(ab, bp);
                float d4 = JVector.Dot(ac, bp);
                if (d3 >= 0f && d4 <= d3)
                {
                    result.ClosestPointOnSimplex = b;
                    result.UsedVertices.UsedVertexB = true;
                    result.SetBarycentricCoordinates(0, 1, 0);

                    return true; // b; // barycentric coordinates (0,1,0)
                }
                // Check if P in edge region of AB, if so return projection of P onto AB
                float vc = d1 * d4 - d3 * d2;
                if (vc <= 0f && d1 >= 0f && d3 <= 0f)
                {
                    v = d1 / (d1 - d3);
                    result.ClosestPointOnSimplex = a + v * ab;
                    result.UsedVertices.UsedVertexA = true;
                    result.UsedVertices.UsedVertexB = true;
                    result.SetBarycentricCoordinates(1 - v, v, 0);
                    return true;
                    //return a + v * ab; // barycentric coordinates (1-v,v,0)
                }

                // Check if P in vertex region outside C
                JVector cp = p - c;
                float d5 = JVector.Dot(ab, cp);
                float d6 = JVector.Dot(ac, cp);
                if (d6 >= 0f && d5 <= d6)
                {
                    result.ClosestPointOnSimplex = c;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(0, 0, 1);
                    return true;//c; // barycentric coordinates (0,0,1)
                }

                // Check if P in edge region of AC, if so return projection of P onto AC
                float vb = d5 * d2 - d1 * d6;
                if (vb <= 0f && d2 >= 0f && d6 <= 0f)
                {
                    w = d2 / (d2 - d6);
                    result.ClosestPointOnSimplex = a + w * ac;
                    result.UsedVertices.UsedVertexA = true;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(1 - w, 0, w);
                    return true;
                    //return a + w * ac; // barycentric coordinates (1-w,0,w)
                }

                // Check if P in edge region of BC, if so return projection of P onto BC
                float va = d3 * d6 - d5 * d4;
                if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
                {
                    w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

                    result.ClosestPointOnSimplex = b + w * (c - b);
                    result.UsedVertices.UsedVertexB = true;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(0, 1 - w, w);
                    return true;
                    // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
                }

                // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
                float denom = 1.0f / (va + vb + vc);
                v = vb * denom;
                w = vc * denom;

                result.ClosestPointOnSimplex = a + ab * v + ac * w;
                result.UsedVertices.UsedVertexA = true;
                result.UsedVertices.UsedVertexB = true;
                result.UsedVertices.UsedVertexC = true;
                result.SetBarycentricCoordinates(1 - v - w, v, w);

                return true;
            }

            /// Test if point p and d lie on opposite sides of plane through abc
            /*
            public int PointOutsideOfPlane(JVector p, JVector a, JVector b, JVector c, JVector d)
            {
                JVector normal = JVector.Cross(b - a, c - a);

                float signp = JVector.Dot(p - a, normal); // [AP AB AC]
                float signd = JVector.Dot(d - a, normal); // [AD AB AC]

                //if (CatchDegenerateTetrahedron)
                if (signd * signd < (1e-4f * 1e-4f)) return -1;

                // Points on opposite sides if expression signs are opposite
                return signp * signd < 0f ? 1 : 0;
            }
             * */
        }
        #endregion
    }
}
