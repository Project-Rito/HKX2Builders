using System;
using System.Linq;
using System.Collections.Generic;
using System.Numerics;
using HKX2;

namespace HKX2Builders
{
    public static class hkaiNavMeshBuilder
    {
        public static hkaiNavMesh Build(Config config, List<Vector3> verts, List<int> indices)
        {
            return (hkaiNavMesh)BuildRoot(config, verts, indices).m_namedVariants[0].m_variant;
        }

        public static hkRootLevelContainer BuildRoot(Config config, List<Vector3> verts, List<int> indices)
        {
            var root = new hkRootLevelContainer();

            NavMeshNative.SetNavmeshBuildParams(
                config.CellSize, config.CellHeight,
                config.WalkableSlopeAngle, config.WalkableHeight,
                config.WalkableClimb, config.WalkableRadius,
                config.MinRegionArea);

            if (!NavMeshNative.BuildNavmeshForMesh(
                verts.ToArray(), verts.Count, indices.ToArray(), indices.Count))
                throw new Exception("Couldn't build Navmesh!");

            var vcount = NavMeshNative.GetMeshVertCount();
            var icount = NavMeshNative.GetMeshTriCount();
            if (vcount == 0 || icount == 0) throw new Exception("Resulting Navmesh is empty!");

            var bverts = new ushort[vcount * 3];
            var bindices = new ushort[icount * 3 * 2];
            var vbverts = new Vector3[vcount];
            NavMeshNative.GetMeshVerts(bverts);
            NavMeshNative.GetMeshTris(bindices);

            var bounds = new Vector3[2];
            NavMeshNative.GetBoundingBox(bounds);

            var navMesh = new hkaiNavMesh
            {
                m_faces = new List<hkaiNavMeshFace>(),
                m_edges = new List<hkaiNavMeshEdge>(),
                m_vertices = new List<Vector4>(),
                m_streamingSets = new List<hkaiStreamingSet>(),
                m_faceData = new List<int>(),
                m_edgeData = new List<int>(),
                m_faceDataStriding = 1,
                m_edgeDataStriding = 1,
                m_flags = NavMeshFlagBits.MESH_NONE,
                m_aabb = new hkAabb
                {
                    m_min = new Vector4(bounds[0].X, bounds[0].Y, bounds[0].Z, 1.0f),
                    m_max = new Vector4(bounds[1].X, bounds[1].Y, bounds[1].Z, 1.0f)
                },
                m_erosionRadius = 0.0f,
                m_userData = 0
            };

            for (var i = 0; i < bverts.Length / 3; i++)
            {
                var vx = bverts[i * 3];
                var vy = bverts[i * 3 + 1];
                var vz = bverts[i * 3 + 2];

                var vert = new Vector3(bounds[0].X + (float)vx * config.CellSize,
                    bounds[0].Y + (float)vy * config.CellHeight,
                    bounds[0].Z + (float)vz * config.CellSize);
                navMesh.m_vertices.Add(new Vector4(vert.X, vert.Y, vert.Z, 1.0f));
                vbverts[i] = vert;
            }

            for (var t = 0; t < bindices.Length / 2; t += 3)
            {
                navMesh.m_faces.Add(
                    new hkaiNavMeshFace
                    {
                        m_clusterIndex = 0,
                        m_numEdges = 3,
                        m_startEdgeIndex = navMesh.m_edges.Count,
                        m_startUserEdgeIndex = -1,
                        m_padding = 0xCDCD
                    });
                navMesh.m_faceData.Add(0);

                for (var i = 0; i < 3; i++)
                {
                    var e = new hkaiNavMeshEdge
                    {
                        m_a = bindices[t * 2 + i],
                        m_b = bindices[t * 2 + ((i + 1) % 3)],
                        m_flags = EdgeFlagBits.EDGE_ORIGINAL
                    };
                    // Record adjacency
                    if (bindices[t * 2 + 3 + i] == 0xFFFF)
                    {
                        // No adjacency
                        e.m_oppositeEdge = 0xFFFFFFFF;
                        e.m_oppositeFace = 0xFFFFFFFF;
                    }
                    else
                    {
                        e.m_oppositeFace = bindices[t * 2 + 3 + i];
                        // Find the edge that has this face as an adjacency
                        for (int j = 0; j < 3; j++)
                        {
                            var edge = bindices[t * 2 + 3 + i] * 6 + 3 + j;
                            if (bindices[edge] == t / 3)
                                e.m_oppositeEdge = (uint)bindices[t * 2 + 3 + i] * 3 + (uint)j;
                        }
                    }

                    navMesh.m_edges.Add(e);
                    navMesh.m_edgeData.Add(0);
                }
            }

            root.m_namedVariants = new List<hkRootLevelContainerNamedVariant>();
            var nmvariant = new hkRootLevelContainerNamedVariant();
            nmvariant.m_className = "hkaiNavMesh";
            nmvariant.m_name = "hkaiNavMesh";
            nmvariant.m_variant = navMesh;

            // Next step: build a bvh
            var shortIndices = new uint[bindices.Length / 2];
            for (int i = 0; i < bindices.Length / 2; i += 3)
            {
                shortIndices[i] = bindices[i * 2];
                shortIndices[i + 1] = bindices[i * 2 + 1];
                shortIndices[i + 2] = bindices[i * 2 + 2];
            }
            bool didbuild = BVHNative.BuildBVHForMesh(vbverts, shortIndices, shortIndices.Length);
            if (!didbuild)
            {
                return null;
            }

            var nodecount = BVHNative.GetBVHSize();
            var nsize = BVHNative.GetNodeSize();
            var nodes = new NativeBVHNode[nodecount];
            BVHNative.GetBVHNodes(nodes);

            // Rebuild in friendlier tree form
            List<BVNode> bnodes = new List<BVNode>((int)nodecount);
            foreach (var n in nodes)
            {
                var bnode = new BVNode();
                bnode.Min = new Vector3(n.minX, n.minY, n.minZ);
                bnode.Max = new Vector3(n.maxX, n.maxY, n.maxZ);
                bnode.IsLeaf = n.isLeaf;
                bnode.PrimitiveCount = n.primitiveCount;
                bnode.Primitive = n.firstChildOrPrimitive;
                bnodes.Add(bnode);
            }
            for (int i = 0; i < nodes.Length; i++)
            {
                if (!nodes[i].isLeaf)
                {
                    bnodes[i].Left = bnodes[(int)nodes[i].firstChildOrPrimitive];
                    bnodes[i].Right = bnodes[(int)nodes[i].firstChildOrPrimitive + 1];
                }
            }

            
            var querymediatorvariant = new hkRootLevelContainerNamedVariant();
            querymediatorvariant.m_className = "hkaiStaticTreeNavMeshQueryMediator";
            querymediatorvariant.m_name = "";
            var querymediator = new hkaiStaticTreeNavMeshQueryMediator();
            querymediatorvariant.m_variant = querymediator;

            var tree = new hkcdStaticAabbTree();

            querymediator.m_tree = tree;
            querymediator.m_navMesh = navMesh;

            tree.m_treePtr = new hkcdStaticTreeDefaultTreeStorage6();
            tree.m_treePtr.m_nodes = bnodes[0].BuildAxis6Tree();
            var min = bnodes[0].Min;
            var max = bnodes[0].Max;
            tree.m_treePtr.m_domain = new hkAabb();
            tree.m_treePtr.m_domain.m_min = new Vector4(min.X, min.Y, min.Z, 1.0f);
            tree.m_treePtr.m_domain.m_max = new Vector4(max.X, max.Y, max.Z, 1.0f);

            // Build a dummy directed graph
            var gvariant = new hkRootLevelContainerNamedVariant();
            gvariant.m_className = "hkaiDirectedGraphExplicitCost";
            gvariant.m_name = "hkaiDirectedGraphExplicitCost";
            var graph = BuildGraph(navMesh, config);
            gvariant.m_variant = graph;

            root.m_namedVariants.Add(nmvariant);
            root.m_namedVariants.Add(gvariant);
            root.m_namedVariants.Add(querymediatorvariant);

            return root;
        }

        private static hkaiDirectedGraphExplicitCost BuildGraph(hkaiNavMesh navMesh, Config config)
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
                m_streamingSets = new List<hkaiStreamingSet>()
            };

            // Init positions with center of all ngons
            foreach (var face in navMesh.m_faces)
            {
                Vector4 center = Vector4.Zero;
                for (int i = face.m_startEdgeIndex; i < face.m_numEdges + face.m_startEdgeIndex; i++)
                {
                    var edge = navMesh.m_edges[i];
                    center += navMesh.m_vertices[edge.m_a];
                }
                center /= face.m_numEdges;

                graph.m_positions.Add(center);

                // Set up edges to all adjacent (non-diagonal) faces
                int numEdges = 0;
                int startEdgeIndex = graph.m_edges.Count;
                for (int i = face.m_startEdgeIndex; i < face.m_numEdges + face.m_startEdgeIndex; i++)
                {
                    var edge = navMesh.m_edges[i];
                    if (edge.m_oppositeFace != 0xFFFFFFFF)
                    {
                        graph.m_edges.Add(new hkaiDirectedGraphExplicitCostEdge()
                        {
                            m_cost = -1, // Calc later
                            m_flags = EdgeBits.EDGE_IS_USER,
                            m_target = edge.m_oppositeFace
                        });
                        numEdges++;
                    }
                }
                // TODO: Add non-adjacent faces that are within config.WalkableHeight radius

                // Add the node
                graph.m_nodes.Add(new hkaiDirectedGraphExplicitCostNode()
                {
                    m_numEdges = numEdges,
                    m_startEdgeIndex = startEdgeIndex
                });
            }

            // Calculate costs for all edges
            CalcEdgeCosts();

            // Remove unnessisary nodes
            SortedSet<uint> nodesToRemove = new SortedSet<uint>();
            for (int i = 0; i < graph.m_nodes.Count; i++)
            {
                var node = graph.m_nodes[i];

                short edgeAvgCost = 0;
                short edgeCount = 0;
                short adjacentEdgeAvgCost = 0;
                short adjacentEdgeCount = 0;
                for (int e = node.m_startEdgeIndex; e < node.m_startEdgeIndex + node.m_numEdges; e++)
                {
                    if (nodesToRemove.Contains(graph.m_edges[e].m_target))
                        continue;

                    edgeAvgCost += graph.m_edges[e].m_cost;
                    edgeCount++;

                    var oppositeNode = graph.m_nodes[(int)graph.m_edges[e].m_target];
                    for (int oe = oppositeNode.m_startEdgeIndex; oe < oppositeNode.m_startEdgeIndex + oppositeNode.m_numEdges; oe++)
                    {
                        if (graph.m_edges[oe].m_target == i)
                            continue;

                        adjacentEdgeAvgCost += graph.m_edges[oe].m_cost;
                        adjacentEdgeCount++;
                    }
                }
                if (adjacentEdgeCount == 0)
                {
                    nodesToRemove.Add((uint)i);
                    continue;
                }

                edgeAvgCost /= edgeCount;
                adjacentEdgeAvgCost /= adjacentEdgeCount;

                if (Math.Abs((float)edgeAvgCost - (float)adjacentEdgeAvgCost) < config.MinGraphGrouping)
                    nodesToRemove.Add((uint)i);
            }

            uint[] removedNodeIndexMapping = Enumerable.Repeat(0xFFFFFFFF, graph.m_nodes.Count).ToArray();
            List<Vector4> simplifiedPositions = new List<Vector4>(graph.m_positions.Count - nodesToRemove.Count);
            List<hkaiDirectedGraphExplicitCostNode> simplifiedNodes = new List<hkaiDirectedGraphExplicitCostNode>(graph.m_nodes.Count - nodesToRemove.Count);
            List<hkaiDirectedGraphExplicitCostEdge> simplifiedEdges = new List<hkaiDirectedGraphExplicitCostEdge>(graph.m_edges.Count);
            for (int i = 0; i < graph.m_nodes.Count; i++)
            {
                if (nodesToRemove.Contains((uint)i))
                    continue;

                var node = graph.m_nodes[i];

                removedNodeIndexMapping[i] = (uint)simplifiedNodes.Count;

                List<uint> newOppositeNodes = new List<uint>();
                List<uint> nodesToDissolve = new List<uint>() { (uint)i };
                for (int ndi = 0; ndi < nodesToDissolve.Count; ndi++)
                {
                    uint nd = nodesToDissolve[ndi];

                    var nodeToDissolve = graph.m_nodes[(int)nd];

                    // Find anything connected that needs to be dissolved...
                    for (int ed = nodeToDissolve.m_startEdgeIndex; ed < nodeToDissolve.m_startEdgeIndex + nodeToDissolve.m_numEdges; ed++)
                    {
                        var nodeToDissolveEdge = graph.m_edges[ed];

                        if (nodesToDissolve.Contains(nodeToDissolveEdge.m_target)) // Slow, should look into using BinarySearch
                            continue;

                        if (nodesToRemove.Contains(nodeToDissolveEdge.m_target))
                            nodesToDissolve.Add(nodeToDissolveEdge.m_target);
                        else
                            newOppositeNodes.Add(nodeToDissolveEdge.m_target);
                    }
                }

                node.m_startEdgeIndex = simplifiedEdges.Count;
                node.m_numEdges = newOppositeNodes.Count;
                foreach (uint newOppositeNode in newOppositeNodes)
                {
                    simplifiedEdges.Add(new hkaiDirectedGraphExplicitCostEdge()
                    {
                        m_flags = EdgeBits.EDGE_IS_USER,
                        m_cost = -1, // calc later
                        m_target = newOppositeNode,
                    });
                }

                simplifiedPositions.Add(graph.m_positions[i]);
                simplifiedNodes.Add(node);
            }

            // Apply mapping
            foreach (var simplifiedEdge in simplifiedEdges)
                simplifiedEdge.m_target = removedNodeIndexMapping[simplifiedEdge.m_target];

            graph.m_positions = simplifiedPositions;
            graph.m_nodes = simplifiedNodes;
            graph.m_edges = simplifiedEdges;

            // Calculate costs for all edges... again, now that we've cleaned up stuff again
            CalcEdgeCosts();

            return graph;

            void CalcEdgeCosts()
            {
                for (int i = 0; i < graph.m_nodes.Count; i++)
                {
                    var node = graph.m_nodes[i];
                    var position = graph.m_positions[i];

                    for (int e = node.m_startEdgeIndex; e < node.m_startEdgeIndex + node.m_numEdges; e++)
                    {
                        Vector4 oppositePosition = graph.m_positions[(int)graph.m_edges[e].m_target];

                        Vector4 vec = oppositePosition - position;

                        graph.m_edges[e].m_cost = (short)(vec.Length() + (vec.Y * config.CostYScale));
                    }
                }
            }
        }

        public struct Config
        {
            public float CellSize;
            public float CellHeight;
            public float WalkableSlopeAngle;
            public float WalkableHeight;
            public float WalkableClimb;
            public float WalkableRadius;
            public int MinRegionArea;

            public float CostYScale;
            public float MinGraphGrouping;

            public static Config Default()
            {
                return new Config
                {
                    CellSize = 0.05f,
                    CellHeight = 0.05f,
                    WalkableSlopeAngle = 30.0f,
                    WalkableHeight = 1.0f,
                    WalkableClimb = 0.5f,
                    WalkableRadius = 0.001f,
                    MinRegionArea = 3,

                    CostYScale = 2f,
                    MinGraphGrouping = 8f,
                };
            }
        }
    }
}