using DelaunatorSharp;
using HKX2;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace HKX2Builders
{
    public class hkaiDirectedGraphExplicitCostBuilder
    {
        public static hkaiDirectedGraphExplicitCost Build(hkaiNavMesh navMesh, NavmeshBuilderConfig config)
        {
            var graph = new hkaiDirectedGraphExplicitCost()
            {
                m_positions = new List<Vector4>(navMesh.m_faces.Count),
                m_nodes = new List<hkaiDirectedGraphExplicitCostNode>(navMesh.m_faces.Count),
                m_edges = new List<hkaiDirectedGraphExplicitCostEdge>(navMesh.m_edges.Count),
                m_edgeData = new List<uint>(),
                m_edgeDataStriding = 0,
                m_nodeData = new List<uint>(),
                m_nodeDataStriding = 0,
                m_streamingSets = Enumerable.Repeat(
                    new hkaiStreamingSet()
                    {
                        m_thisUid = 0,
                        m_oppositeUid = 0,
                        m_meshConnections = new List<hkaiStreamingSetNavMeshConnection>(0),
                        m_graphConnections = new List<hkaiStreamingSetGraphConnection>(0),
                        m_volumeConnections = new List<hkaiStreamingSetVolumeConnection>(0)
                    }, 8).ToList(),
            };


            Vector4[] positions = new Vector4[navMesh.m_faces.Count];

            // Set positions to center of all polygons
            for (int i = 0; i < navMesh.m_faces.Count; i++)
            {
                var face = navMesh.m_faces[i];
                for (int j = face.m_startEdgeIndex; j < face.m_numEdges + face.m_startEdgeIndex; j++)
                {
                    var edge = navMesh.m_edges[j];
                    positions[i] += navMesh.m_vertices[edge.m_a];
                }
                positions[i] /= face.m_numEdges;
            }

            if (positions.Length < 3)
                return graph; // Maybe eventually we can just manually link them

            Tuple<int, int[]>[] clusters = KMeansCluster(positions, config.KMeansClusteringK < positions.Length ? config.KMeansClusteringK : positions.Length - 1);

            Vector4[] clusteredPositions = new Vector4[clusters.Length];
            for (int i = 0; i < clusters.Length; i++)
                clusteredPositions[i] = positions[clusters[i].Item1];

            // Here we would usually project our 3D points to a 2D plane for delaunay triangulation.
            // However, for navigation stuff we can assume the normal of that plane is the Y-axis.
            // We can therefore just omit the Y value of our points.
            IndexedPoint[] indexedPoints = new IndexedPoint[clusteredPositions.Length];
            for (int i = 0; i < clusteredPositions.Length; i++)
            {
                indexedPoints[i].X = clusteredPositions[i].X;
                indexedPoints[i].Y = clusteredPositions[i].Z;
                indexedPoints[i].Index = i;
            }

            Delaunator d;
            try
            {
                d = new Delaunator(indexedPoints.Select(x => (IPoint)x).ToArray());
            }
            catch
            {
                return graph;
            }

            Dictionary<int, List<int>> directedEdges = new Dictionary<int, List<int>>();

            d.ForEachTriangleEdge((IEdge e) =>
            {
                IndexedPoint p = (IndexedPoint)e.P;
                IndexedPoint q = (IndexedPoint)e.Q;

                if (Vector4.Abs(Vector4.Normalize((clusteredPositions[p.Index] - clusteredPositions[q.Index]))).Y > config.WalkableSlopeAngle / 90f)
                    return;

                if (directedEdges.ContainsKey(p.Index))
                    directedEdges[p.Index].Add(q.Index);
                else
                    directedEdges.Add(p.Index, new List<int>() { q.Index });

                if (directedEdges.ContainsKey(q.Index))
                    directedEdges[q.Index].Add(p.Index);
                else
                    directedEdges.Add(q.Index, new List<int>() { p.Index });
            });

            int posStartIdx = graph.m_positions.Count;

            for (int i = 0; i < clusteredPositions.Length; i++)
            {
                graph.m_positions.Add(clusteredPositions[i]);

                int edgeStartIdx = graph.m_edges.Count;

                List<int> targets;
                directedEdges.TryGetValue(i, out targets);
                if (targets != null)
                {
                    foreach (int target in targets)
                    {
                        if (CalcEdgeCost(clusteredPositions[i], clusteredPositions[target]) > (System.Half)5.0f)
                            continue;
                        graph.m_edges.Add(new hkaiDirectedGraphExplicitCostEdge()
                        {
                            m_cost = CalcEdgeCost(clusteredPositions[i], clusteredPositions[target]),
                            m_flags = EdgeBits.EDGE_IS_USER,
                            m_target = (uint)(posStartIdx + target)
                        });
                    }
                }

                graph.m_nodes.Add(new hkaiDirectedGraphExplicitCostNode()
                {
                    m_startEdgeIndex = edgeStartIdx,
                    m_numEdges = graph.m_edges.Count - edgeStartIdx
                });
            }

            return graph;
        }

        private static System.Half CalcEdgeCost(Vector4 self, Vector4 target)
        {
            Vector4 vec = target - self;
            return (System.Half)(vec.Length() / 20f /*+ (vec.Y * config.CostYScale)*/);
        }

        struct IndexedPoint : IPoint
        {
            public double X { get; set; }

            public double Y { get; set; }

            public int Index { get; set; }

            public IndexedPoint(double x, double y, int index)
            {
                X = x;
                Y = y;
                Index = index;
            }

            public override string ToString()
            {
                return $"{X},{Y}";
            }
        }

        // Returns centroids as indices into points array, as well as what points the clusters contain.
        // Because of this, these centroids are not perfectly accurate,
        // but they *do* land on already defined points.
        private static int _randSeed = 0;
        private static Tuple<int, int[]>[] KMeansCluster(Vector4[] points, int k)
        {
            Random rand = new Random(_randSeed);
            _randSeed = rand.Next(); // Might as well make it super random

            int[] clusterCentroids = new int[k]; // We want to make sure that centroids fall on points to match vanilla behaviour. We use indexes instead of Vector3s to represent.
            for (int i = 0; i < k; i++)
            {
                int randIdx = -1;
                while (randIdx == -1 || clusterCentroids.Contains(randIdx))
                    randIdx = rand.Next(points.Length);
                clusterCentroids[i] = randIdx;
            }

            List<int>[] lastClusters = null;

            List<int>[] currentClusters = Enumerable.Range(0, k).Select(x => new List<int>()).ToArray();
            while (lastClusters == null || !currentClusters.SequenceEqual(lastClusters, new ListEqualityComparer<int>()))
            {
                lastClusters = currentClusters;
                currentClusters = Enumerable.Range(0, k).Select(x => new List<int>()).ToArray();

                for (int i = 0; i < points.Length; i++)
                {
                    // Find what cluster centroid this point is closest to.
                    int clusterIdx = -1;
                    float minLength = float.MaxValue;
                    for (int j = 0; j < clusterCentroids.Length; j++)
                    {
                        float length = (points[i] - points[clusterCentroids[j]]).Length();
                        if (length < minLength)
                        {
                            minLength = length;
                            clusterIdx = j;
                        }
                    }

                    currentClusters[clusterIdx].Add(i);
                }

                // Update centroids
                for (int i = 0; i < currentClusters.Length; i++)
                {
                    Vector4 centroid = new Vector4(0);

                    foreach (int pointIdx in currentClusters[i])
                        centroid += points[pointIdx];
                    centroid /= currentClusters[i].Count;

                    // Find closest point, again, this is done to emulate vanilla behaviour.
                    int centroidPointIdx = -1;
                    float minLength = float.MaxValue;
                    foreach (int pointIdx in currentClusters[i])
                    {
                        float length = (points[pointIdx] - centroid).Length();
                        if (length < minLength)
                        {
                            minLength = length;
                            centroidPointIdx = pointIdx;
                        }
                    }

                    clusterCentroids[i] = centroidPointIdx;
                }
            }

            Tuple<int, int[]>[] res = new Tuple<int, int[]>[currentClusters.Length];
            for (int i = 0; i < currentClusters.Length; i++)
            {
                res[i] = new Tuple<int, int[]>(clusterCentroids[i], currentClusters[i].ToArray());
            }

            return res;
        }

        private class ListEqualityComparer<T> : IEqualityComparer<List<T>>
        {
            public bool Equals(List<T> a, List<T> b)
            {
                return a.SequenceEqual(b);
            }

            public int GetHashCode(List<T> a)
            {
                return a.GetHashCode();
            }
        }
    }
}
