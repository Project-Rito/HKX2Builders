using System;
using System.Linq;
using System.Collections.Generic;
using System.Numerics;
using HKX2;
using DelaunatorSharp;

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

            var bregs = new ushort[icount];
            NavMeshNative.GetMeshRegions(bregs);

            var bounds = new Vector3[2];
            NavMeshNative.GetBoundingBox(bounds);

            var navMesh = new hkaiNavMesh
            {
                m_faces = new List<hkaiNavMeshFace>(),
                m_edges = new List<hkaiNavMeshEdge>(),
                m_vertices = new List<Vector4>(),
                m_streamingSets = Enumerable.Repeat(
                    new hkaiStreamingSet()
                    {
                        m_thisUid = 0,
                        m_oppositeUid = 0,
                        m_meshConnections = new List<hkaiStreamingSetNavMeshConnection>(0),
                        m_graphConnections = new List<hkaiStreamingSetGraphConnection>(0),
                        m_volumeConnections = new List<hkaiStreamingSetVolumeConnection>(0)
                    }, 8).ToList(),
                m_faceData = new List<int>(),
                m_edgeData = new List<int>(),
                m_faceDataStriding = 1,
                m_edgeDataStriding = 1,
                m_flags = NavMeshFlagBits.MESH_NONE,
                m_aabb = new hkAabb
                {
                    m_min = new Vector4(bounds[0].X, bounds[0].Y, bounds[0].Z, 1.0f), // There seems to be some overlap between aabs.
                    m_max = new Vector4(bounds[1].X, bounds[1].Y, bounds[1].Z, 1.0f)  // Maybe it's because it includes streaming sets though?
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
                        m_clusterIndex = (short)bregs[t / 3],
                        m_numEdges = 3,
                        m_startEdgeIndex = navMesh.m_edges.Count,
                        m_startUserEdgeIndex = -1,
                        m_padding = 0xCDCD
                    });
                navMesh.m_faceData.Add(unchecked((int)0b01111111111111110000000000000000));

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
                        navMesh.m_faceData[navMesh.m_faceData.Count - 1] |= unchecked((int)0b10000000000000000000000000000000);
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
            var navmeshVariant = new hkRootLevelContainerNamedVariant();
            navmeshVariant.m_className = "hkaiNavMesh";
            navmeshVariant.m_name = "";
            navmeshVariant.m_variant = navMesh;

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

            
            var queryMediatorVariant = new hkRootLevelContainerNamedVariant();
            queryMediatorVariant.m_className = "hkaiStaticTreeNavMeshQueryMediator";
            queryMediatorVariant.m_name = "";
            var querymediator = new hkaiStaticTreeNavMeshQueryMediator();
            queryMediatorVariant.m_variant = querymediator;

            var tree = new hkcdStaticAabbTree();

            querymediator.m_tree = tree;
            querymediator.m_navMesh = navMesh;

            tree.m_treePtr = new hkcdStaticTreeDefaultTreeStorage6();
            tree.m_treePtr.m_nodes = bnodes[0].BuildAxis6Tree();
            var min = bnodes[0].Min;
            var max = bnodes[0].Max;
            tree.m_treePtr.m_domain = new hkAabb();
            tree.m_treePtr.m_domain.m_min = new Vector4(min.X, min.Y,  min.Z, 1.0f);
            tree.m_treePtr.m_domain.m_max = new Vector4(max.X, max.Y, max.Z, 1.0f);

            // Build a directed graph
            var graphVariant = new hkRootLevelContainerNamedVariant();
            graphVariant.m_className = "hkaiDirectedGraphExplicitCost";
            graphVariant.m_name = "";
            var graph = BuildGraph(navMesh, config, bregs);
            graphVariant.m_variant = graph;

            root.m_namedVariants.Add(navmeshVariant);
            root.m_namedVariants.Add(graphVariant);
            root.m_namedVariants.Add(queryMediatorVariant);

            return root;
        }

        public static hkRootLevelContainer UpdateStreamingSets(hkRootLevelContainer root, Vector3 rootOrigin, hkRootLevelContainer[] streamables, Vector3[] streamableOrigins, Config config)
        {
            for (int i = 0; i < streamables.Length; i++) {
                root = UpdateStreamingSet(root, rootOrigin, streamables[i], streamableOrigins[i], i, config);
            }

            return root;
        }

        public static hkRootLevelContainer UpdateStreamingSet(hkRootLevelContainer root, Vector3 rootOrigin, hkRootLevelContainer streamable, Vector3 streamableOrigin, int index, Config config)
        {
            root = BuildNavmeshStreamingSet(root, rootOrigin, streamable, streamableOrigin, index, config);

            root = BuildGraphStreamingSet(root, rootOrigin, streamable, streamableOrigin, index, config);

            return root;
        }

        private static hkRootLevelContainer BuildNavmeshStreamingSet(hkRootLevelContainer root, Vector3 rootOrigin, hkRootLevelContainer streamable, Vector3 streamableOrigin, int index, Config config)
        {
            // Useful vector stuff
            Vector4 rootOriginVec4 = new Vector4(rootOrigin.X, rootOrigin.Y, rootOrigin.Z, 1f);
            Vector4 streamableOriginVec4 = new Vector4(streamableOrigin.X, streamableOrigin.Y, streamableOrigin.Z, 1f);

            hkaiNavMesh rootNavmesh = (hkaiNavMesh)root.m_namedVariants[0].m_variant;
            hkaiNavMesh streamableNavmesh = (hkaiNavMesh)streamable.m_namedVariants[0].m_variant;

            hkaiStreamingSet nmStreamingSet = new hkaiStreamingSet()
            {
                m_thisUid = Convert.ToUInt32(root.m_namedVariants[0].m_name.Split("/")[1].Split(",")[0], 16),
                m_oppositeUid = Convert.ToUInt32(streamable.m_namedVariants[0].m_name.Split("/")[1].Split(",")[0], 16),
                m_meshConnections = new List<hkaiStreamingSetNavMeshConnection>(0),
                m_graphConnections = new List<hkaiStreamingSetGraphConnection>(0),
                m_volumeConnections = new List<hkaiStreamingSetVolumeConnection>(0)
            };

            bool xMask = true;
            bool yMask = true;
            bool zMask = true;
            switch (index)
            {
                case 0:
                    break;
                case 1:
                    xMask = false;
                    break;
                case 2:
                    break;
                case 3:
                    zMask = false;
                    break;
                case 4:
                    zMask = false;
                    break;
                case 5:
                    break;
                case 6:
                    xMask = false;
                    break;
                case 7:
                    break;
            }

            HashSet<FaceEdgePair> streamableBorderFaceEdgePairs = new HashSet<FaceEdgePair>();
            for (int f = 0; f < streamableNavmesh.m_faces.Count; f++)
            {
                hkaiNavMeshFace face = streamableNavmesh.m_faces[f];

                for (int e = face.m_startEdgeIndex; e < face.m_startEdgeIndex + face.m_numEdges; e++)
                {
                    hkaiNavMeshEdge edge = streamableNavmesh.m_edges[e];
                    Vector4 edgeA = streamableNavmesh.m_vertices[edge.m_a];
                    Vector4 edgeB = streamableNavmesh.m_vertices[edge.m_b];
                    Vector4 edgeMid = (edgeA + edgeB) / 2f;
                    Vector4 edgeARootSpace = edgeA + streamableOriginVec4 - rootOriginVec4;
                    Vector4 edgeBRootSpace = edgeB + streamableOriginVec4 - rootOriginVec4;
                    Vector4 edgeMidRootSpace = edgeMid + streamableOriginVec4 - rootOriginVec4;

                    if (Math.Abs((edgeMidRootSpace - Vector4.Zero).Length()) > config.StreamingSetSearchRadius)
                        continue;

                    if (edge.m_oppositeEdge == 0xFFFFFFFF)
                    {
                        streamableBorderFaceEdgePairs.Add(new FaceEdgePair { FaceIdx = f, EdgeIdx = e });
                    }
                }
            }

            HashSet<int> rootBorderFaceIndices = new HashSet<int>();
            for (int f = 0; f < rootNavmesh.m_faces.Count; f++)
            {
                hkaiNavMeshFace face = rootNavmesh.m_faces[f];

                for (int e = face.m_startEdgeIndex; e < face.m_startEdgeIndex + face.m_numEdges; e++)
                {

                    hkaiNavMeshEdge edge = rootNavmesh.m_edges[e];

                    if (edge.m_oppositeEdge == 0xFFFFFFFF)
                    {
                        rootBorderFaceIndices.Add(f);
                    }
                }
            }

            if (streamableBorderFaceEdgePairs.Count == 0 || rootBorderFaceIndices.Count == 0)
                return root;

            // Remove the edge vertices from root
            // Make sure that the only vertices removed are the ones that matter
            // Or maybe when merging into polys we can do this, looking for 0xFFFFFFFF or whatever.

            // As adding points create streaming sets. look for if point already there, if so use it in a streaming set, otherwise add one.

            Dictionary<Vector4, int> addedVerticesToIndices = new Dictionary<Vector4, int>();
            foreach (FaceEdgePair streamablePair in streamableBorderFaceEdgePairs)
            {
                Vector4 streamableEdgeA = streamableNavmesh.m_vertices[streamableNavmesh.m_edges[streamablePair.EdgeIdx].m_a];
                Vector4 streamableEdgeB = streamableNavmesh.m_vertices[streamableNavmesh.m_edges[streamablePair.EdgeIdx].m_b];
                Vector4 streamableEdgeMid = (streamableEdgeA + streamableEdgeB) / 2f;
                Vector4 streamableEdgeARootSpace = streamableEdgeA + streamableOriginVec4 - rootOriginVec4;
                Vector4 streamableEdgeBRootSpace = streamableEdgeB + streamableOriginVec4 - rootOriginVec4;
                Vector4 streamableEdgeMidRootSpace = streamableEdgeMid + streamableOriginVec4 - rootOriginVec4;

                int aIdx;
                int bIdx;
                if (addedVerticesToIndices.ContainsKey(streamableEdgeARootSpace))
                    aIdx = addedVerticesToIndices[streamableEdgeARootSpace];
                else
                {
                    aIdx = rootNavmesh.m_vertices.Count;
                    rootNavmesh.m_vertices.Add(streamableEdgeARootSpace);
                    addedVerticesToIndices.Add(streamableEdgeARootSpace, aIdx);
                }
                if (addedVerticesToIndices.ContainsKey(streamableEdgeBRootSpace))
                    bIdx = addedVerticesToIndices[streamableEdgeBRootSpace];
                else
                {
                    bIdx = rootNavmesh.m_vertices.Count;
                    rootNavmesh.m_vertices.Add(streamableEdgeBRootSpace);
                    addedVerticesToIndices.Add(streamableEdgeBRootSpace, bIdx);
                }

                //float aRootFaceMinDist = float.MaxValue;
                //float bRootFaceMinDist = float.MaxValue;
                //int aRootFaceIdx = -1;
                //int bRootFaceIdx = -1;
                float rootFaceMinDist = float.MaxValue;
                int rootFaceMinDistIdx = -1;
                foreach (int rootFaceIdx in rootBorderFaceIndices)
                {
                    hkaiNavMeshFace rootNavmeshEdgeFace = rootNavmesh.m_faces[rootFaceIdx];
                    if (rootNavmeshEdgeFace.m_numEdges > 10000)
                        Console.WriteLine();

                    Vector4 rootFaceMid = new Vector4();
                    for (int rootNavmeshEdgeIdx = rootNavmeshEdgeFace.m_startEdgeIndex; rootNavmeshEdgeIdx < rootNavmeshEdgeFace.m_startEdgeIndex + rootNavmeshEdgeFace.m_numEdges; rootNavmeshEdgeIdx++)
                    {
                        rootFaceMid += rootNavmesh.m_vertices[rootNavmesh.m_edges[rootNavmeshEdgeIdx].m_b];
                    }
                    rootFaceMid /= rootNavmeshEdgeFace.m_numEdges;

                    Vector4 rootFaceMidMasked = new Vector4();
                    rootFaceMidMasked.X = xMask ? rootFaceMid.X : 0f;
                    rootFaceMidMasked.Y = yMask ? rootFaceMid.Y : 0f;
                    rootFaceMidMasked.Z = zMask ? rootFaceMid.Z : 0f;

                    float dist = (streamableEdgeMidRootSpace - rootFaceMidMasked).Length();

                    if (dist < rootFaceMinDist)
                    {
                        rootFaceMinDist = dist;
                        rootFaceMinDistIdx = rootFaceIdx;
                    }

                    /*
                    float aDist = (streamableEdgeARootSpace - rootFaceMid).Length();
                    float bDist = (streamableEdgeBRootSpace - rootFaceMid).Length();

                    if (aDist < aRootFaceMinDist)
                    {
                        aRootFaceMinDist = aDist;
                        aRootFaceIdx = rootFaceIdx;
                    }
                    if (bDist < bRootFaceMinDist)
                    {
                        bRootFaceMinDist = bDist;
                        bRootFaceIdx = rootFaceIdx;
                    }
                    */
                }

                Console.WriteLine(rootFaceMinDistIdx);

                if (rootFaceMinDistIdx == -1)
                    return root;

                hkaiNavMeshFace rootFace = rootNavmesh.m_faces[rootFaceMinDistIdx];

                float rootFaceEdgeMinDist = float.MaxValue;
                int rootFaceEdgeMinDistIdx = -1;
                for (int rootNavmeshEdgeIdx = rootFace.m_startEdgeIndex; rootNavmeshEdgeIdx < rootFace.m_startEdgeIndex + rootFace.m_numEdges; rootNavmeshEdgeIdx++)
                {
                    hkaiNavMeshEdge rootNavmeshEdge = rootNavmesh.m_edges[rootNavmeshEdgeIdx];

                    // Find a root edge that has a point on it closest to the midpoint of the streamable edge

                    Vector4 rootNavmeshEdgeA = rootNavmesh.m_vertices[rootNavmeshEdge.m_a];
                    Vector4 rootNavmeshEdgeB = rootNavmesh.m_vertices[rootNavmeshEdge.m_b];
                    Vector4 rootNavmeshEdgeMid = (rootNavmeshEdgeA + rootNavmeshEdgeB) / 2f;

                    float dist = (rootNavmeshEdgeMid - streamableEdgeMidRootSpace).Length();

                    if (dist < rootFaceEdgeMinDist)
                    {
                        rootFaceEdgeMinDistIdx = rootNavmeshEdgeIdx;
                        rootFaceEdgeMinDist = dist;
                    }
                }

                if (rootFaceEdgeMinDistIdx == -1)
                    return root;

                hkaiNavMeshEdge replaceEdge = rootNavmesh.m_edges[rootFaceEdgeMinDistIdx];
                rootNavmesh.m_edges.RemoveAt(rootFaceEdgeMinDistIdx);
                rootNavmesh.m_edges.InsertRange(rootFaceEdgeMinDistIdx, new List<hkaiNavMeshEdge>() 
                {
                    new hkaiNavMeshEdge()
                    {
                        m_a = replaceEdge.m_a,
                        m_b = aIdx,
                        m_oppositeEdge = 0xFFFFFFFF,
                        m_oppositeFace = 0xFFFFFFFF,
                        m_flags = 0
                    },
                    new hkaiNavMeshEdge()
                    {
                        m_a = aIdx,
                        m_b = bIdx,
                        m_oppositeEdge = 0xFFFFFFFF,
                        m_oppositeFace = 0xFFFFFFFF,
                        m_flags = 0
                    },
                    new hkaiNavMeshEdge()
                    {
                        m_a = bIdx,
                        m_b = replaceEdge.m_b,
                        m_oppositeEdge = 0xFFFFFFFF,
                        m_oppositeFace = 0xFFFFFFFF,
                        m_flags = 0
                    },
                });
                rootNavmesh.m_faces[rootFaceMinDistIdx].m_numEdges += 2;

                // We need to change the starts of everything after.
                for (int rootFaceIdx = rootFaceMinDistIdx + 1; rootFaceIdx < rootNavmesh.m_faces.Count; rootFaceIdx++)
                {
                    rootNavmesh.m_faces[rootFaceIdx].m_startEdgeIndex += 2;
                }
            }

            




            rootNavmesh.m_streamingSets[index] = nmStreamingSet;
            root.m_namedVariants[0].m_variant = rootNavmesh;

            return root;
        }

        private static hkRootLevelContainer BuildNavmeshStreamingSetOld2(hkRootLevelContainer root, Vector3 rootOrigin, hkRootLevelContainer streamable, Vector3 streamableOrigin, int index, Config config)
        {
            // Useful vector stuff
            Vector4 rootOriginVec4 = new Vector4(rootOrigin.X, rootOrigin.Y, rootOrigin.Z, 1f);
            Vector4 streamableOriginVec4 = new Vector4(streamableOrigin.X, streamableOrigin.Y, streamableOrigin.Z, 1f);

            hkaiNavMesh rootNavmesh = (hkaiNavMesh)root.m_namedVariants[0].m_variant;
            hkaiNavMesh streamableNavmesh = (hkaiNavMesh)streamable.m_namedVariants[0].m_variant;

            hkaiStreamingSet nmStreamingSet = new hkaiStreamingSet()
            {
                m_thisUid = Convert.ToUInt32(root.m_namedVariants[0].m_name.Split("/")[1].Split(",")[0], 16),
                m_oppositeUid = Convert.ToUInt32(streamable.m_namedVariants[0].m_name.Split("/")[1].Split(",")[0], 16),
                m_meshConnections = new List<hkaiStreamingSetNavMeshConnection>(0),
                m_graphConnections = new List<hkaiStreamingSetGraphConnection>(0),
                m_volumeConnections = new List<hkaiStreamingSetVolumeConnection>(0)
            };

            bool xMask = true;
            bool yMask = true;
            bool zMask = true;
            switch (index)
            {
                case 0:
                    break;
                case 1:
                    xMask = false;
                    break;
                case 2:
                    break;
                case 3:
                    zMask = false;
                    break;
                case 4:
                    zMask = false;
                    break;
                case 5:
                    break;
                case 6:
                    xMask = false;
                    break;
                case 7:
                    break;
            }

            HashSet<FaceEdgePair> streamableBorderFaceEdgePairs = new HashSet<FaceEdgePair>();
            for (int f = 0; f < streamableNavmesh.m_faces.Count; f++)
            {
                hkaiNavMeshFace face = streamableNavmesh.m_faces[f];

                for (int e = face.m_startEdgeIndex; e < face.m_startEdgeIndex + face.m_numEdges; e++)
                {

                    hkaiNavMeshEdge edge = streamableNavmesh.m_edges[e];

                    if (edge.m_oppositeEdge == 0xFFFFFFFF)
                    {
                        streamableBorderFaceEdgePairs.Add(new FaceEdgePair { FaceIdx = f, EdgeIdx = e });
                    }
                }
            }

            for (int f = 0; f < rootNavmesh.m_faces.Count; f++)
            {
                hkaiNavMeshFace face = rootNavmesh.m_faces[f];

                for (int e = face.m_startEdgeIndex; e < face.m_startEdgeIndex + face.m_numEdges; e++)
                {
                    hkaiNavMeshEdge edge = rootNavmesh.m_edges[e];
                    Vector4 edgePosA = rootNavmesh.m_vertices[edge.m_a];
                    Vector4 edgePosB = rootNavmesh.m_vertices[edge.m_b];
                    Vector4 edgeCenter = (edgePosA + edgePosB) / 2f;
                    Vector4 edgeCenterMasked = edgeCenter;
                    if (!xMask)
                        edgeCenterMasked.X = 0f;
                    if (!yMask)
                        edgeCenterMasked.Y = 0f;
                    if (!zMask)
                        edgeCenterMasked.Z = 0f;

                    if (edge.m_oppositeFace == 0xFFFFFFFF)
                    {
                        float minDist = float.MaxValue;
                        float minDistMasked = float.MaxValue;
                        FaceEdgePair streamableBorderFaceEdgePair = null;
                        Vector4 streamableBorderEdgePairPosA = new Vector4();
                        Vector4 streamableBorderEdgePairPosB = new Vector4();
                        Vector4 streamableBorderEdgePairCenter = new Vector4();
                        foreach (FaceEdgePair faceEdgePair in streamableBorderFaceEdgePairs)
                        {
                            hkaiNavMeshEdge streamableBorderEdge = streamableNavmesh.m_edges[faceEdgePair.EdgeIdx];

                            Vector4 streamableBorderEdgePosA = streamableNavmesh.m_vertices[streamableBorderEdge.m_a];
                            Vector4 streamableBorderEdgePosB = streamableNavmesh.m_vertices[streamableBorderEdge.m_b];
                            Vector4 streamableBorderEdgeCenter = (streamableBorderEdgePosA + streamableBorderEdgePosB) / 2f;

                            float dist = Math.Abs((edgeCenter - (streamableBorderEdgeCenter + streamableOriginVec4 - rootOriginVec4)).Length());
                            Vector4 streamableBorderEdgeCenterMasked = streamableBorderEdgeCenter;
                            if (!xMask)
                                streamableBorderEdgeCenterMasked.X = 0f;
                            if (!yMask)
                                streamableBorderEdgeCenterMasked.Y = 0f;
                            if (!zMask)
                                streamableBorderEdgeCenterMasked.Z = 0f;
                            float distMasked = Math.Abs((edgeCenterMasked - (streamableBorderEdgeCenterMasked + streamableOriginVec4 - rootOriginVec4)).Length());

                            if (dist < minDist)
                            {
                                streamableBorderFaceEdgePair = faceEdgePair;
                                streamableBorderEdgePairPosA = streamableBorderEdgePosA;
                                streamableBorderEdgePairPosB = streamableBorderEdgePosB;
                                streamableBorderEdgePairCenter = streamableBorderEdgeCenter;
                                minDist = dist;
                                minDistMasked = distMasked;
                            }
                        }
                        if (streamableBorderFaceEdgePair == null)
                            continue;
                        if (minDistMasked > config.StreamingSetSearchRadius)
                            continue;
                        nmStreamingSet.m_meshConnections.Add(new hkaiStreamingSetNavMeshConnection()
                        {
                            m_edgeIndex = e,
                            m_faceIndex = f,
                            m_oppositeFaceIndex = streamableBorderFaceEdgePair.FaceIdx,
                            m_oppositeEdgeIndex = streamableBorderFaceEdgePair.EdgeIdx
                        });

                        edgePosA = streamableBorderEdgePairPosA + streamableOriginVec4 - rootOriginVec4;
                        edgePosB = streamableBorderEdgePairPosB + streamableOriginVec4 - rootOriginVec4;
                        rootNavmesh.m_vertices[edge.m_a] = edgePosA;
                        rootNavmesh.m_vertices[edge.m_b] = edgePosB;

                        // Sometimes it'll need to split into two edges...
                    }
                }
            }

            rootNavmesh.m_streamingSets[index] = nmStreamingSet;
            root.m_namedVariants[0].m_variant = rootNavmesh;

            return root;
        }

        private static hkRootLevelContainer BuildGraphStreamingSet(hkRootLevelContainer root, Vector3 rootOrigin, hkRootLevelContainer streamable, Vector3 streamableOrigin, int index, Config config)
        {
            // Useful vector stuff
            Vector4 rootOriginVec4 = new Vector4(rootOrigin.X, rootOrigin.Y, rootOrigin.Z, 1f);
            Vector4 streamableOriginVec4 = new Vector4(streamableOrigin.X, streamableOrigin.Y, streamableOrigin.Z, 1f);

            hkaiDirectedGraphExplicitCost rootGraph = (hkaiDirectedGraphExplicitCost)root.m_namedVariants[1].m_variant;
            hkaiDirectedGraphExplicitCost streamableGraph = (hkaiDirectedGraphExplicitCost)streamable.m_namedVariants[1].m_variant;

            hkaiStreamingSet graphStreamingSet = new hkaiStreamingSet()
            {
                m_thisUid = Convert.ToUInt32(root.m_namedVariants[1].m_name.Split("/")[1].Split(",")[0], 16),
                m_oppositeUid = Convert.ToUInt32(streamable.m_namedVariants[1].m_name.Split("/")[1].Split(",")[0], 16),
                m_meshConnections = new List<hkaiStreamingSetNavMeshConnection>(0),
                m_graphConnections = new List<hkaiStreamingSetGraphConnection>(0),
                m_volumeConnections = new List<hkaiStreamingSetVolumeConnection>(0)
            };

            bool xMask = true;
            bool yMask = true;
            bool zMask = true;
            switch (index)
            {
                case 0:
                    break;
                case 1:
                    xMask = false;
                    break;
                case 2:
                    break;
                case 3:
                    zMask = false;
                    break;
                case 4:
                    zMask = false;
                    break;
                case 5:
                    break;
                case 6:
                    xMask = false;
                    break;
                case 7:
                    break;
            }

            for (int n = 0; n < rootGraph.m_nodes.Count; n++)
            {
                Vector4 nodePos = rootGraph.m_positions[n];
                if (!xMask)
                    nodePos.X = 0f;
                if (!yMask)
                    nodePos.Y = 0f;
                if (!zMask)
                    nodePos.Z = 0f;

                float minDist = float.MaxValue;
                int streamableNPair = -1;
                Vector4 streamableNodePairPos = new Vector4();
                for (int streamableN = 0; streamableN < streamableGraph.m_nodes.Count; streamableN++)
                {
                    Vector4 streamableNodePos = streamableGraph.m_positions[streamableN];
                    if (!xMask)
                        streamableNodePos.X = 0f;
                    if (!yMask)
                        streamableNodePos.Y = 0f;
                    if (!zMask)
                        streamableNodePos.Z = 0f;

                    float dist = (nodePos - (streamableNodePos + streamableOriginVec4 - rootOriginVec4)).Length();

                    if (dist < minDist)
                    {
                        streamableNPair = streamableN;
                        streamableNodePairPos = streamableNodePos;
                        minDist = dist;
                    }
                }

                if (streamableNPair != -1)
                {
                    graphStreamingSet.m_graphConnections.Add(new hkaiStreamingSetGraphConnection()
                    {
                        m_nodeIndex = n,
                        m_oppositeNodeIndex = streamableNPair,
                        m_edgeCost = CalcEdgeCost(nodePos, streamableNodePairPos),
                        m_edgeData = 0u
                    });
                }
            }

            rootGraph.m_streamingSets[index] = graphStreamingSet;
            root.m_namedVariants[1].m_variant = rootGraph;

            return root;
        }

        private class FaceEdgePair
        {
            public int FaceIdx;
            public int EdgeIdx;
        }

        public static hkRootLevelContainer UpdateStreamingSetOld(hkRootLevelContainer root, Vector3 rootOrigin, hkRootLevelContainer streamable, Vector3 streamableOrigin, int index, Config config)
        {
            // A little nicer to work with.
            Vector4 rootOriginV4 = new Vector4(rootOrigin.X, rootOrigin.Y, rootOrigin.Z, 1);
            Vector4 streamableOriginV4 = new Vector4(streamableOrigin.X, streamableOrigin.Y, streamableOrigin.Z, 1);

            hkaiStreamingSet nmStreamingSet = new hkaiStreamingSet()
            {
                m_thisUid = Convert.ToUInt32(root.m_namedVariants[0].m_name.Split("/")[1].Split(",")[0], 16),
                m_oppositeUid = Convert.ToUInt32(streamable.m_namedVariants[0].m_name.Split("/")[1].Split(",")[0], 16),
                m_meshConnections = new List<hkaiStreamingSetNavMeshConnection>(0),
                m_graphConnections = new List<hkaiStreamingSetGraphConnection>(0),
                m_volumeConnections = new List<hkaiStreamingSetVolumeConnection>(0)
            };
            // K: relevant face index, V: relevant edge indices, in a voxel grid.
            // Each voxel in the grid has the len, width, and height of the streaming set search radius.
            Dictionary<Tuple<int, int, int>, Dictionary<int, List<int>>> rootEdgeVoxelGrid = new Dictionary<Tuple<int, int, int>, Dictionary<int, List<int>>>();
            for (int faceIdx = 0; faceIdx < ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_faces.Count; faceIdx++)
            {
                var face = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_faces[faceIdx];

                for (int edgeIdx = face.m_startEdgeIndex; edgeIdx < face.m_startEdgeIndex + face.m_numEdges; edgeIdx++)
                {
                    var edge = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_edges[edgeIdx];
                    Vector4 edgeVtxA = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[edge.m_a];
                    Vector4 edgeVtxB = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[edge.m_b];
                    Vector4 edgeCenter = (edgeVtxA + edgeVtxB) / 2;

                    int voxelIdxX = (int)(edgeCenter.X / config.StreamingSetSearchRadius);
                    int voxelIdxY = (int)(edgeCenter.Y / config.StreamingSetSearchRadius);
                    int voxelIdxZ = (int)(edgeCenter.Z / config.StreamingSetSearchRadius);

                    if (!rootEdgeVoxelGrid.ContainsKey(new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)))
                        rootEdgeVoxelGrid[new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)] = new Dictionary<int, List<int>>();
                    if (!rootEdgeVoxelGrid[new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)].ContainsKey(faceIdx))
                        rootEdgeVoxelGrid[new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)].Add(faceIdx, new List<int> { edgeIdx } );
                    else
                        rootEdgeVoxelGrid[new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)][faceIdx].Add(edgeIdx);
                }
            } 
            // K: relevant face index, V: relevant edge indices, in a voxel grid.
            // Each voxel in the grid has the len, width, and height of the streaming set search radius.
            Dictionary<Tuple<int, int, int>, Dictionary<int, List<int>>> streamableEdgeVoxelGrid = new Dictionary<Tuple<int, int, int>, Dictionary<int, List<int>>>();
            for (int faceIdx = 0; faceIdx < ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_faces.Count; faceIdx++)
            {
                var face = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_faces[faceIdx];

                for (int edgeIdx = face.m_startEdgeIndex; edgeIdx < face.m_startEdgeIndex + face.m_numEdges; edgeIdx++)
                {
                    var edge = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_edges[edgeIdx];
                    Vector4 edgeVtxA = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[edge.m_a];
                    Vector4 edgeVtxB = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[edge.m_b];
                    Vector4 edgeCenter = (edgeVtxA + edgeVtxB) / 2;

                    int voxelIdxX = (int)(edgeCenter.X / config.StreamingSetSearchRadius);
                    int voxelIdxY = (int)(edgeCenter.Y / config.StreamingSetSearchRadius);
                    int voxelIdxZ = (int)(edgeCenter.Z / config.StreamingSetSearchRadius);

                    if (!streamableEdgeVoxelGrid.ContainsKey(new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)))
                        streamableEdgeVoxelGrid[new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)] = new Dictionary<int, List<int>>();
                    if (!streamableEdgeVoxelGrid[new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)].ContainsKey(faceIdx))
                        streamableEdgeVoxelGrid[new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)].Add(faceIdx, new List<int> { edgeIdx });
                    else
                        streamableEdgeVoxelGrid[new Tuple<int, int, int>(voxelIdxX, voxelIdxY, voxelIdxZ)][faceIdx].Add(edgeIdx);
                }
            }

            Vector4 rootMin = new Vector4(float.MaxValue, float.MaxValue, float.MaxValue, 1);
            Vector4 rootMax = new Vector4(float.MinValue, float.MinValue, float.MinValue, 1); ;
            {
                foreach (Tuple<int, int, int> vox in rootEdgeVoxelGrid.Keys)
                {
                    rootMin.X = vox.Item1 < rootMin.X ? vox.Item1 : rootMin.X;
                    rootMin.Y = vox.Item2 < rootMin.Y ? vox.Item2 : rootMin.Y;
                    rootMin.Z = vox.Item3 < rootMin.Z ? vox.Item3 : rootMin.Z;

                    rootMax.X = vox.Item1 >= rootMax.X ? vox.Item1 : rootMax.X;
                    rootMax.Y = vox.Item2 >= rootMax.Y ? vox.Item2 : rootMax.Y;
                    rootMax.Z = vox.Item3 >= rootMax.Z ? vox.Item3 : rootMax.Z;
                }
            }

            Vector4 streamableMin = new Vector4(float.MaxValue, float.MaxValue, float.MaxValue, 1);
            Vector4 streamableMax = new Vector4(float.MinValue, float.MinValue, float.MinValue, 1); ;
            {
                foreach (Tuple<int, int, int> vox in rootEdgeVoxelGrid.Keys)
                {
                    streamableMin.X = vox.Item1 < streamableMin.X ? vox.Item1 : streamableMin.X;
                    streamableMin.Y = vox.Item2 < streamableMin.Y ? vox.Item2 : streamableMin.Y;
                    streamableMin.Z = vox.Item3 < streamableMin.Z ? vox.Item3 : streamableMin.Z;

                    streamableMax.X = vox.Item1 >= streamableMax.X ? vox.Item1 : streamableMax.X;
                    streamableMax.Y = vox.Item2 >= streamableMax.Y ? vox.Item2 : streamableMax.Y;
                    streamableMax.Z = vox.Item3 >= streamableMax.Z ? vox.Item3 : streamableMax.Z;
                }
            }

            Vector4 sharedMin = new Vector4(
                rootMin.X > streamableMin.X ? rootMin.X : streamableMin.X,
                rootMin.Y > streamableMin.Y ? rootMin.Y : streamableMin.Y,
                rootMin.Z > streamableMin.Z ? rootMin.Z : streamableMin.Z,
                rootMin.W > streamableMin.W ? rootMin.W : streamableMin.W
                );
            Vector4 sharedMax = new Vector4(
                rootMax.X < streamableMax.X ? rootMax.X : streamableMax.X,
                rootMax.Y < streamableMax.Y ? rootMax.Y : streamableMax.Y,
                rootMax.Z < streamableMax.Z ? rootMax.Z : streamableMax.Z,
                rootMax.W < streamableMax.W ? rootMax.W : streamableMax.W
                );

            // Root stich voxels and streamable voxels' indices should match up as neighbors.
            List<Tuple<int, int, int>> rootStitchVoxels;
            {
                List<int> xValues;
                List<int> yValues;
                List<int> zValues;
                switch (index)
                {
                    case 0:
                        {
                            xValues = new List<int>() { (int)(rootMin.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(rootMin.Z) };
                            break;
                        }
                    case 1:
                        {
                            xValues = Enumerable.Range((int)(sharedMin.X), (int)(sharedMax.X)).ToList();
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(rootMin.Z) };
                            break;
                        }
                    case 2:
                        {
                            xValues = new List<int>() { (int)(rootMax.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(rootMin.Z) };
                            break;
                        }
                    case 3:
                        {
                            xValues = new List<int>() { (int)(rootMin.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = Enumerable.Range((int)(sharedMin.Z), (int)(sharedMax.Z)).ToList();
                            break;
                        }
                    case 4:
                        {
                            xValues = new List<int>() { (int)(rootMax.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = Enumerable.Range((int)(sharedMin.Z), (int)(sharedMax.Z)).ToList();
                            break;
                        }
                    case 5:
                        {
                            xValues = new List<int>() { (int)(rootMin.Z) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(rootMax.Z) };
                            break;
                        }
                    case 6:
                        {
                            xValues = Enumerable.Range((int)(sharedMin.X), (int)(sharedMax.X)).ToList();
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(rootMax.Z) };
                            break;
                        }
                    case 7:
                        {
                            xValues = new List<int>() { (int)(rootMax.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(rootMax.Z) };
                            break;
                        }
                    default:
                        throw new Exception("Not a valid streaming adjacency index!");
                }
                List<Tuple<int, int, int>> voxels = new List<Tuple<int, int, int>>(xValues.Count * yValues.Count * zValues.Count);
                foreach (int xValue in xValues)
                {
                    foreach (int yValue in yValues)
                    {
                        foreach (int zValue in zValues)
                            voxels.Add(new Tuple<int, int, int>(xValue, yValue, zValue));
                    }
                }

                rootStitchVoxels = voxels;
            }
            List<Tuple<int, int, int>> streamableStitchVoxels;
            {
                List<int> xValues;
                List<int> yValues;
                List<int> zValues;
                switch (7 - index)
                {
                    case 0:
                        {
                            xValues = new List<int>() { (int)(streamableMin.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(streamableMin.Z) };
                            break;
                        }
                    case 1:
                        {
                            xValues = Enumerable.Range((int)(sharedMin.X), (int)(sharedMax.X)).ToList();
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(streamableMin.Z) };
                            break;
                        }
                    case 2:
                        {
                            xValues = new List<int>() { (int)(streamableMax.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(streamableMin.Z) };
                            break;
                        }
                    case 3:
                        {
                            xValues = new List<int>() { (int)(streamableMin.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = Enumerable.Range((int)(sharedMin.Z), (int)(sharedMax.Z)).ToList();
                            break;
                        }
                    case 4:
                        {
                            xValues = new List<int>() { (int)(streamableMax.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = Enumerable.Range((int)(sharedMin.Z), (int)(sharedMax.Z)).ToList();
                            break;
                        }
                    case 5:
                        {
                            xValues = new List<int>() { (int)(streamableMin.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(streamableMax.Z) };
                            break;
                        }
                    case 6:
                        {
                            xValues = Enumerable.Range((int)(sharedMin.X), (int)(sharedMax.X)).ToList();
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(streamableMax.Z) };
                            break;
                        }
                    case 7:
                        {
                            xValues = new List<int>() { (int)(streamableMax.X) };
                            yValues = Enumerable.Range((int)(sharedMin.Y), (int)(sharedMax.Y)).ToList();
                            zValues = new List<int>() { (int)(streamableMax.Z) };
                            break;
                        }
                    default:
                        throw new Exception("Not a valid streaming adjacency index!");
                }
                List<Tuple<int, int, int>> voxels = new List<Tuple<int, int, int>>(xValues.Count * yValues.Count * zValues.Count);
                foreach (int xValue in xValues)
                {
                    foreach (int yValue in yValues)
                    {
                        foreach (int zValue in zValues)
                            voxels.Add(new Tuple<int, int, int>(xValue, yValue, zValue));
                    }
                }

                streamableStitchVoxels = voxels;
            }

            for (int stitchVoxelIdx = 0; stitchVoxelIdx < rootStitchVoxels.Count; stitchVoxelIdx++)
            {
                Tuple<int, int, int> rootStitchVoxel = rootStitchVoxels[stitchVoxelIdx];
                Tuple<int, int, int> streamableStitchVoxel = streamableStitchVoxels[stitchVoxelIdx];

                // Make sure both this voxel and its streamable neighbor have data.
                if (!rootEdgeVoxelGrid.ContainsKey(new Tuple<int, int, int> (rootStitchVoxel.Item1, rootStitchVoxel.Item2, rootStitchVoxel.Item3)) ||
                    rootEdgeVoxelGrid[new Tuple<int, int, int>(rootStitchVoxel.Item1, rootStitchVoxel.Item2, rootStitchVoxel.Item3)] == null)
                    continue;
                if (!streamableEdgeVoxelGrid.ContainsKey(new Tuple<int, int, int>(streamableStitchVoxel.Item1, streamableStitchVoxel.Item2, streamableStitchVoxel.Item3)) ||
                    streamableEdgeVoxelGrid[new Tuple<int, int, int>(streamableStitchVoxel.Item1, streamableStitchVoxel.Item2, streamableStitchVoxel.Item3)] == null)
                    continue;

                foreach (int rootFaceIdx in rootEdgeVoxelGrid[new Tuple<int, int, int>(rootStitchVoxel.Item1, rootStitchVoxel.Item2, rootStitchVoxel.Item3)].Keys)
                {
                    foreach (int rootEdgeIdx in rootEdgeVoxelGrid[new Tuple<int, int, int>(rootStitchVoxel.Item1, rootStitchVoxel.Item2, rootStitchVoxel.Item3)][rootFaceIdx])
                    {
                        var rootEdge = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_edges[rootEdgeIdx];
                        if (rootEdge.m_oppositeEdge != 0xFFFFFFFF)
                            continue;
                        Vector4 rootEdgeVtxA = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[rootEdge.m_a];
                        Vector4 rootEdgeVtxB = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[rootEdge.m_b];
                        Vector4 rootEdgeCenter = (rootEdgeVtxA + rootEdgeVtxB) / 2;

                        int pairStreamableFaceIdx = -1;
                        int pairStreamableEdgeIdx = -1;
                        float minEdgeCenterDist = float.MaxValue;
                        foreach (int streamableFaceIdx in streamableEdgeVoxelGrid[new Tuple<int, int, int>(streamableStitchVoxel.Item1, streamableStitchVoxel.Item2, streamableStitchVoxel.Item3)].Keys)
                        {
                            foreach (int streamableEdgeIdx in streamableEdgeVoxelGrid[new Tuple<int, int, int>(streamableStitchVoxel.Item1, streamableStitchVoxel.Item2, streamableStitchVoxel.Item3)][streamableFaceIdx])
                            {
                                var streamableEdge = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_edges[streamableEdgeIdx];
                                if (streamableEdge.m_oppositeEdge != 0xFFFFFFFF)
                                    continue;
                                Vector4 streamableEdgeVtxA = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[streamableEdge.m_a];
                                Vector4 streamableEdgeVtxB = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[streamableEdge.m_b];
                                Vector4 streamableEdgeCenter = (streamableEdgeVtxA + streamableEdgeVtxB) / 2;

                                float dist = (streamableEdgeCenter + streamableOriginV4 - rootEdgeCenter + rootOriginV4).Length();
                                if (dist < minEdgeCenterDist)
                                {
                                    minEdgeCenterDist = dist;
                                    pairStreamableFaceIdx = streamableFaceIdx;
                                    pairStreamableEdgeIdx = streamableEdgeIdx;
                                }
                            }
                        }
                        if (pairStreamableEdgeIdx == -1)
                            continue;
                        // Move vertices to match up with vertices in streamable.
                        // Not ideal, but the most ideal method would be bake the entire field's navmesh and chop it up.
                        // Which we're obviously not doing.
                        var pairStreamableEdge = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_edges[pairStreamableEdgeIdx];
                        Vector4 pairStreamableEdgeVtxA = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[pairStreamableEdge.m_a];
                        Vector4 pairStreamableEdgeVtxB = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[pairStreamableEdge.m_b];
                        rootEdgeVtxA = pairStreamableEdgeVtxA + streamableOriginV4 - rootOriginV4; // We have to convert from space local to streamable
                        rootEdgeVtxB = pairStreamableEdgeVtxB + streamableOriginV4 - rootOriginV4; // to space local to root.
                        ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[rootEdge.m_a] = rootEdgeVtxA;
                        ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[rootEdge.m_b] = rootEdgeVtxB;

                        // Set up the streaming set.
                        nmStreamingSet.m_meshConnections.Add(new hkaiStreamingSetNavMeshConnection()
                        {
                            m_faceIndex = rootFaceIdx,
                            m_edgeIndex = rootEdgeIdx,
                            m_oppositeFaceIndex = pairStreamableFaceIdx,
                            m_oppositeEdgeIndex = pairStreamableEdgeIdx
                        });
                    }
                }
            }

            /*
            // K: relevant face index, V: relevant edge indices
            Dictionary<int, List<int>> rootBoundaryFaces = new Dictionary<int, List<int>>();
            for (int faceIdx = 0; faceIdx < ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_faces.Count; faceIdx++)
            {
                var face = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_faces[faceIdx];
                
                for (int edgeIdx = face.m_startEdgeIndex; edgeIdx < face.m_startEdgeIndex + face.m_numEdges; edgeIdx++)
                {
                    var edge = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_edges[edgeIdx];
                    if (edge.m_oppositeFace == 0xffffffff)
                    {
                        if (!rootBoundaryFaces.ContainsKey(faceIdx))
                            rootBoundaryFaces.Add(faceIdx, new List<int> { edgeIdx });
                        else
                            rootBoundaryFaces[faceIdx].Add(edgeIdx);
                    }
                }
            }
            // K: relevant face index, V: relevant edge indices
            Dictionary<int, List<int>> streamableBoundaryFaces = new Dictionary<int, List<int>>();
            for (int faceIdx = 0; faceIdx < ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_faces.Count; faceIdx++)
            {
                var face = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_faces[faceIdx];

                for (int edgeIdx = face.m_startEdgeIndex; edgeIdx < face.m_startEdgeIndex + face.m_numEdges; edgeIdx++)
                {
                    var edge = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_edges[edgeIdx];
                    if (edge.m_oppositeFace == 0xffffffff)
                    {
                        if (!streamableBoundaryFaces.ContainsKey(faceIdx))
                            streamableBoundaryFaces.Add(faceIdx, new List<int> { edgeIdx });
                        else
                            streamableBoundaryFaces[faceIdx].Add(edgeIdx);
                    }
                }
            }

            foreach (int rootFaceIdx in rootBoundaryFaces.Keys)
            {
                var rootFace = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_faces[rootFaceIdx];
                foreach (int rootEdgeIdx in rootBoundaryFaces[rootFaceIdx])
                {
                    var rootEdge = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_edges[rootEdgeIdx];
                    Vector4 rootEdgeVtxA = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[rootEdge.m_a];
                    Vector4 rootEdgeVtxB = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[rootEdge.m_b];
                    Vector4 rootEdgeCenter = (rootEdgeVtxA + rootEdgeVtxB) / 2;
  
                    int pairStreamableFaceIdx = -1;
                    int pairStreamableEdgeIdx = -1;
                    float minEdgeCenterDist = float.MaxValue;
                    foreach (int streamableFaceIdx in streamableBoundaryFaces.Keys)
                    {
                        var streamableFace = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_faces[streamableFaceIdx];
                        foreach (int streamableEdgeIdx in streamableBoundaryFaces[streamableFaceIdx])
                        {
                            var streamableEdge = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_edges[streamableEdgeIdx];
                            Vector4 streamableEdgeVtxA = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[streamableEdge.m_a];
                            Vector4 streamableEdgeVtxB = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[streamableEdge.m_b];
                            Vector4 streamableEdgeCenter = (streamableEdgeVtxA + streamableEdgeVtxB) / 2;

                            float dist = (streamableEdgeCenter - rootEdgeCenter).Length();
                            if (dist < minEdgeCenterDist)
                            {
                                minEdgeCenterDist = dist;
                                pairStreamableFaceIdx = streamableFaceIdx;
                                pairStreamableEdgeIdx = streamableEdgeIdx;
                            }
                        }
                    }

                    nmStreamingSet.m_meshConnections.Add(new hkaiStreamingSetNavMeshConnection()
                    {
                        m_faceIndex = rootFaceIdx,
                        m_edgeIndex = rootEdgeIdx,
                        m_oppositeFaceIndex = pairStreamableFaceIdx,
                        m_oppositeEdgeIndex = pairStreamableEdgeIdx
                    });
                }
            }
            */


            /*
            List<int> rootBoundaryVertices = new List<int>();
            for (int i = 0; i < ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_edges.Count; i++)
            {
                var edge = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_edges[i];
                if (edge.m_oppositeFace == 0xffffffff)
                {
                    rootBoundaryVertices.Add(edge.m_a);
                    rootBoundaryVertices.Add(edge.m_b);
                } 
            }
            List<int> streamableBoundaryVertices = new List<int>();
            for (int i = 0; i < ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_edges.Count; i++)
            {
                var edge = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_edges[i];
                if (edge.m_oppositeFace == 0xffffffff)
                {
                    streamableBoundaryVertices.Add(edge.m_a);
                    streamableBoundaryVertices.Add(edge.m_b);
                }
            }
            foreach (var rootBoundaryVtxIdx in rootBoundaryVertices)
            {
                Vector4 rootBoundaryVtx = ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_vertices[rootBoundaryVtxIdx];

                int pairStreamableVtxIdx = -1;
                float minDist = float.MaxValue;
                foreach (var streamableBoundaryVtxIdx in streamableBoundaryVertices)
                {
                    Vector4 streamableBoundaryVtx = ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_vertices[streamableBoundaryVtxIdx];
                    float dist = (streamableBoundaryVtx - rootBoundaryVtx).Length();
                    if (dist < minDist)
                    {
                        minDist = dist;
                        pairStreamableVtxIdx = streamableBoundaryVtxIdx;
                    }
                }

                //nmStreamingSet.m_meshConnections.Add(new hkaiStreamingSetNavMeshConnection()
                //{
                    
                //});
            }
            */

            ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_streamingSets[index] = nmStreamingSet;
            hkaiStreamingSet graphStreamingSet = new hkaiStreamingSet()
            {
                m_thisUid = Convert.ToUInt32(root.m_namedVariants[1].m_name.Split("/")[1].Split(",")[0], 16),
                m_oppositeUid = Convert.ToUInt32(streamable.m_namedVariants[1].m_name.Split("/")[1].Split(",")[0], 16),
                m_meshConnections = new List<hkaiStreamingSetNavMeshConnection>(0),
                m_graphConnections = new List<hkaiStreamingSetGraphConnection>(0),
                m_volumeConnections = new List<hkaiStreamingSetVolumeConnection>(0)
            };
            ((hkaiDirectedGraphExplicitCost)root.m_namedVariants[1].m_variant).m_streamingSets[index] = graphStreamingSet;

            return root;
        }

        public static hkRootLevelContainer LinkStreamingSet(hkRootLevelContainer root, Vector3 rootOrigin, hkRootLevelContainer streamable, Vector3 streamableOrigin, int index)
        {
            ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_streamingSets[index].m_meshConnections.Clear();
            foreach (var meshConnection in ((hkaiNavMesh)streamable.m_namedVariants[0].m_variant).m_streamingSets[7 - index].m_meshConnections)
            {
                ((hkaiNavMesh)root.m_namedVariants[0].m_variant).m_streamingSets[index].m_meshConnections.Add(new hkaiStreamingSetNavMeshConnection()
                {
                    m_faceIndex = meshConnection.m_oppositeFaceIndex,
                    m_edgeIndex = meshConnection.m_oppositeEdgeIndex,
                    m_oppositeFaceIndex = meshConnection.m_faceIndex,
                    m_oppositeEdgeIndex = meshConnection.m_edgeIndex
                });
            }

            ((hkaiDirectedGraphExplicitCost)root.m_namedVariants[1].m_variant).m_streamingSets[index].m_graphConnections.Clear();
            foreach (var graphConnection in ((hkaiDirectedGraphExplicitCost)streamable.m_namedVariants[1].m_variant).m_streamingSets[7 - index].m_graphConnections)
            {
                ((hkaiDirectedGraphExplicitCost)root.m_namedVariants[1].m_variant).m_streamingSets[index].m_graphConnections.Add(new hkaiStreamingSetGraphConnection()
                {
                    m_nodeIndex = graphConnection.m_oppositeNodeIndex,
                    m_oppositeNodeIndex = graphConnection.m_nodeIndex,
                    m_edgeCost = graphConnection.m_edgeCost,
                    m_edgeData = graphConnection.m_edgeData
                });
            }

            return root;
        }


        private static hkaiDirectedGraphExplicitCost BuildGraph(hkaiNavMesh navMesh, Config config, ushort[] regionsMapping)
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
            


            /*
            #region Generate triangulation per-region
            // Map from region to face
            Dictionary<int, List<hkaiNavMeshFace>> regions = new Dictionary<int, List<hkaiNavMeshFace>>();
            for (int i = 0; i < navMesh.m_faces.Count; i++)
            {
                if (regions.ContainsKey(regionsMapping[i]))
                    regions[regionsMapping[i]].Add(navMesh.m_faces[i]);
                else
                    regions.Add(regionsMapping[i], new List<hkaiNavMeshFace>() { navMesh.m_faces[i] });
            }

            foreach (var region in regions.Values)
            {
                Vector4[] positions = new Vector4[region.Count];

                // Set positions to center of all polygons
                for (int i = 0; i < region.Count; i++)
                {
                    var face = region[i];
                    for (int j = face.m_startEdgeIndex; j < face.m_numEdges + face.m_startEdgeIndex; j++)
                    {
                        var edge = navMesh.m_edges[j];
                        positions[i] += navMesh.m_vertices[edge.m_a];
                    }
                    positions[i] /= face.m_numEdges;
                }

                if (positions.Length < 3)
                    continue; // Maybe eventually we can just manually link them

                Tuple<int, int[]>[] clusters = KMeansCluster(positions, config.KMeansClusteringK < positions.Length ? config.KMeansClusteringK : positions.Length - 1);

                Vector4[] clusteredPositions = new Vector4[clusters.Length];
                for (int i = 0; i < clusters.Length; i++)
                    clusteredPositions[i] = positions[clusters[i].Item1];

                // Here we would usually project our 3D points to a 2D plane for delaunay triangulation.
                // However, for navigation stuff we can assume the normal of that plane is the Y-axis.
                // We can therefore just omit the Y value of our points.
                IndexedPoint[] indexedPoints = new IndexedPoint[positions.Length];
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
                    continue;
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
            }
            #endregion
            */

            /*
            // Remove unnessisary nodes
            SortedSet<uint> nodesToRemove = new SortedSet<uint>();
            for (int i = 0; i < graph.m_nodes.Count; i++)
            {
                var node = graph.m_nodes[i];

                System.Half edgeAvgCost = (System.Half)0;
                short edgeCount = 0;
                System.Half adjacentEdgeAvgCost = (System.Half)0;
                short adjacentEdgeCount = 0;
                for (int e = node.m_startEdgeIndex; e < node.m_startEdgeIndex + node.m_numEdges; e++)
                {
                    if (nodesToRemove.Contains(graph.m_edges[e].m_target))
                        continue;

                    edgeAvgCost = (System.Half)((float)edgeAvgCost + (float)graph.m_edges[e].m_cost);
                    edgeCount++;

                    var oppositeNode = graph.m_nodes[(int)graph.m_edges[e].m_target];
                    for (int oe = oppositeNode.m_startEdgeIndex; oe < oppositeNode.m_startEdgeIndex + oppositeNode.m_numEdges; oe++)
                    {
                        if (graph.m_edges[oe].m_target == i)
                            continue;

                        adjacentEdgeAvgCost = (System.Half)((float)adjacentEdgeAvgCost + (float)graph.m_edges[oe].m_cost);
                        adjacentEdgeCount++;
                    }
                }
                if (adjacentEdgeCount == 0)
                {
                    nodesToRemove.Add((uint)i);
                    continue;
                }

                edgeAvgCost = (System.Half)((float)edgeAvgCost / (float)edgeCount);
                adjacentEdgeAvgCost = (System.Half)((float)adjacentEdgeAvgCost / (float)adjacentEdgeCount);

                if (Math.Abs((float)edgeAvgCost - (float)adjacentEdgeAvgCost) < config.GraphMinGrouping)
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
                        m_cost = (System.Half)0, // calc later
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
            graph.m_nodes = simplifiedNodes; // These will be re-assigned after the next step.
            graph.m_edges = simplifiedEdges;

            CalcEdgeCosts(); // Update our costs
            */

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

                        graph.m_edges[e].m_cost = CalcEdgeCost(oppositePosition, position);
                    }
                }
            }
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
            public int KMeansClusteringK;
            //public float GraphMinGrouping;
            //public float GraphEdgeRadius;
            public float StreamingSetSearchRadius;

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
                    KMeansClusteringK = 30,
                    //GraphMinGrouping = 8f,
                    //GraphEdgeRadius = 0.5f,s
                    StreamingSetSearchRadius = 2f,
                };
            }
        }
    }
}