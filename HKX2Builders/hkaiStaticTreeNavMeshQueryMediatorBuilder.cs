using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using HKX2;

namespace HKX2Builders
{
    public static class hkaiStaticTreeNavMeshQueryMediatorBuilder
    {
        public static hkaiStaticTreeNavMeshQueryMediator Build(hkaiNavMesh navmesh)
        {
            // TODO: This stuff relies on navmesh being triangles.
            // In future, consider porting BVH builder from Detour to C#.
            Vector3[] verticesVec3 = new Vector3[navmesh.m_vertices.Count];
            for (int i = 0; i < navmesh.m_vertices.Count; i++)
                verticesVec3[i] = new Vector3(navmesh.m_vertices[i].X, navmesh.m_vertices[i].Y, navmesh.m_vertices[i].Z);

            List<uint> indices = new List<uint>(navmesh.m_edges.Count * 2);
            foreach (var face in navmesh.m_faces)
            {
                for (int e = face.m_startEdgeIndex; e < face.m_startEdgeIndex + face.m_numEdges; e++)
                {
                    indices.Add((uint)navmesh.m_edges[e].m_a);
                }
            }
            bool didbuild = BVHNative.BuildBVHForMesh(verticesVec3, indices.ToArray(), indices.Count);
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


            var querymediator = new hkaiStaticTreeNavMeshQueryMediator()
            {
                m_tree = new hkcdStaticAabbTree()
                {
                    m_treePtr = new hkcdStaticTreeDefaultTreeStorage6()
                    {
                        m_nodes = bnodes[0].BuildAxis6Tree(),
                        m_domain = new hkAabb()
                        {
                            m_min = new Vector4(bnodes[0].Min.X, bnodes[0].Min.Y, bnodes[0].Min.Z, 1.0f),
                            m_max = new Vector4(bnodes[0].Max.X, bnodes[0].Max.Y, bnodes[0].Max.Z, 1.0f),
                        }
                    }
                },
                m_navMesh = navmesh
            };

            return querymediator;
        }
    }
}
