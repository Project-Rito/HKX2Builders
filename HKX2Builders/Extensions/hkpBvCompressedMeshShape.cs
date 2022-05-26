using System;
using System.Numerics;
using System.Collections.Generic;
using System.Linq;
using HKX2;

namespace HKX2Builders.Extensions
{
    // Represents a tree node for a mesh's BVH tree when it's expanded from its packed format
    [Serializable]
    public class BVHNode
    {
        // If a terminal, this is the index of the chunk/triangle for this terminal
        public uint Index;

        // Terminal leaf in the node which means it points directly to a chunk or a triangle
        public bool IsTerminal;

        // Left and right children nodes
        public BVHNode Left;

        public Vector3 Max;

        // Bounding box AABB that contains all the children as well
        public Vector3 Min;
        public BVHNode Right;
    }

    public static partial class Extensions
    {
        private static BVNode BuildBvhTree(this hkpBvCompressedMeshShape _this, Vector3 parentBBMin,
            Vector3 parentBBMax, uint nodeIndex)
        {
            var cnode = _this.m_tree.m_nodes[(int) nodeIndex];
            var node = new BVNode
            {
                Min = cnode.DecompressMin(parentBBMin, parentBBMax),
                Max = cnode.DecompressMax(parentBBMin, parentBBMax)
            };

            if ((cnode.m_hiData & 0x80) > 0)
            {
                node.Left = BuildBvhTree(_this, node.Min, node.Max, nodeIndex + 1);
                node.Right = BuildBvhTree(_this, node.Min, node.Max,
                    nodeIndex + ((((uint) cnode.m_hiData & 0x7F) << 8) | cnode.m_loData) * 2);
            }
            else
            {
                node.IsLeaf = true;
                node.Primitive = (((uint) cnode.m_hiData & 0x7F) << 8) | cnode.m_loData;

                node.PrimitiveCount = 1;
            }

            return node;
        }

        // Extracts an easily processable BVH tree from the packed version in the mesh data
        public static BVNode GetMeshBvh(this hkpBvCompressedMeshShape _this)
        {
            if (_this.m_tree.m_nodes == null || _this.m_tree.m_nodes.Count == 0) return null;

            var root = new BVNode
            {
                Min = new Vector3(_this.m_tree.m_domain.m_min.X, _this.m_tree.m_domain.m_min.Y,
                    _this.m_tree.m_domain.m_min.Z),
                Max = new Vector3(_this.m_tree.m_domain.m_max.X, _this.m_tree.m_domain.m_max.Y,
                    _this.m_tree.m_domain.m_max.Z)
            };

            var cnode = _this.m_tree.m_nodes[0];
            if ((cnode.m_hiData & 0x80) > 0)
            {
                root.Left = BuildBvhTree(_this, root.Min, root.Max, 1);
                root.Right = BuildBvhTree(_this, root.Min, root.Max,
                    ((((uint) cnode.m_hiData & 0x7F) << 8) | cnode.m_loData) * 2);

                root.PrimitiveCount = root.ComputePrimitiveCounts();
            }
            else
            {
                root.IsLeaf = true;
                root.Primitive = (((uint) cnode.m_hiData & 0x7F) << 8) | cnode.m_loData;

                root.PrimitiveCount = 1;
            }

            return root;
        }

        public static hkAabb GetDomain(this hkpBvCompressedMeshShape _this)
        {
            return _this.m_tree.m_domain;
        }

        public static MeshContainer ToMesh(this hkpBvCompressedMeshShape _this)
        {
            var sections = new List<MeshSection>();
            var domain = new Domain(_this.m_tree.m_domain.m_min, _this.m_tree.m_domain.m_max);
            var sharedVerts = _this.m_tree.m_sharedVertices.Select(vtx2 => new SharedVertex(vtx2).Decompress(domain))
                .ToList();

            foreach (var sec in _this.m_tree.m_sections)
            {
                var s = new MeshSection
                {
                    Domain = new Domain(sec.m_domain.m_min, sec.m_domain.m_max),
                    Vertices = new List<Vector4>(),
                    Primitives = new List<List<byte>>()
                };
                sections.Add(s);
                var offset = new Vector3(sec.m_codecParms_0, sec.m_codecParms_1, sec.m_codecParms_2);
                var scale = new Vector3(sec.m_codecParms_3, sec.m_codecParms_4, sec.m_codecParms_5);
                var sharedVtxIndexStart = sec.m_sharedVertices.m_data >> 8;
                var primitiveStart = sec.m_primitives.m_data >> 8;
                var primitiveCount = sec.m_primitives.m_data & 0xFFu;
                foreach (var vtx in new ArraySegment<uint>(_this.m_tree.m_packedVertices.ToArray(),
                    (int)sec.m_firstPackedVertex, sec.m_numPackedVertices))
                    s.Vertices.Add(new PackedVertex(vtx).Decompress(offset, scale));

                foreach (var idx in new ArraySegment<ushort>(_this.m_tree.m_sharedVerticesIndex.ToArray(),
                    (int)sharedVtxIndexStart, sec.m_numSharedIndices))
                    s.Vertices.Add(sharedVerts[idx]);

                foreach (var primitive2 in new
                    ArraySegment<hkcdStaticMeshTreeBasePrimitive>(_this.m_tree.m_primitives.ToArray(),
                        (int)primitiveStart,
                        (int)primitiveCount))
                    s.Primitives.Add(new List<byte>
                    {
                        primitive2.m_indices_0, primitive2.m_indices_1, primitive2.m_indices_2, primitive2.m_indices_3
                    });
            }

            var cont = new MeshContainer(_this.GetType().Name);
            var vtxCount = 0;
            foreach (var sec in sections)
            {
                cont.Vertices.AddRange(sec.Vertices);
                var primitives = sec.Primitives.Select(primitive => new List<int>
                {
                    primitive[0] + vtxCount, primitive[1] + vtxCount, primitive[2] + vtxCount, primitive[3] + vtxCount
                }).ToList();

                cont.Primitives.AddRange(primitives);
                vtxCount += sec.Vertices.Count;
            }

            return cont;
        }

    }
}