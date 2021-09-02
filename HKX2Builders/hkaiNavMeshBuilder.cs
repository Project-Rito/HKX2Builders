using System;
using System.Collections.Generic;
using System.Numerics;
using HKX2;

namespace HKX2Builders
{
    public static class hkaiNavMeshBuilder
    {
        public static hkaiNavMesh Build(Config c, List<Vector3> verts, List<int> indices)
        {
            NavMeshNative.SetNavmeshBuildParams(
                c.CellSize, c.CellHeight,
                c.WalkableSlopeAngle, c.WalkableHeight,
                c.WalkableClimb, c.WalkableRadius,
                c.MinRegionArea);

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

                var vert = new Vector3(bounds[0].X + vx * c.CellSize,
                    bounds[0].Y + vy * c.CellHeight,
                    bounds[0].Z + vz * c.CellSize);
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
                        m_b = bindices[t * 2 + (i + 1) % 3],
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
                        for (var j = 0; j < 3; j++)
                        {
                            var edge = bindices[t * 2 + 3 + i] * 6 + 3 + j;
                            if (bindices[edge] == t / 3)
                                e.m_oppositeEdge = (uint) bindices[t * 2 + 3 + i] * 3 + (uint) j;
                        }
                    }

                    navMesh.m_edges.Add(e);
                    navMesh.m_edgeData.Add(0);
                }
            }

            return navMesh;
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

            public static Config Default()
            {
                return new Config
                {
                    CellSize = 0.3f,
                    CellHeight = 0.3f,
                    WalkableSlopeAngle = 30.0f,
                    WalkableHeight = 2.0f,
                    WalkableClimb = 1.0f,
                    WalkableRadius = 0.5f,
                    MinRegionArea = 3
                };
            }
        }
    }
}