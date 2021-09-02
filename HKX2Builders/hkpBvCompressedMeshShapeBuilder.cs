using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using HKX2;

namespace HKX2Builders
{
    public static class hkpBvCompressedMeshShapeBuilder
    {
        private static Vector3 DecompressSharedVertex(ulong vertex, Vector4 bbMin, Vector4 bbMax)
        {
            var scaleX = (bbMax.X - bbMin.X) / ((1 << 21) - 1);
            var scaleY = (bbMax.Y - bbMin.Y) / ((1 << 21) - 1);
            var scaleZ = (bbMax.Z - bbMin.Z) / ((1 << 22) - 1);
            var x = (vertex & 0x1FFFFF) * scaleX + bbMin.X;
            var y = ((vertex >> 21) & 0x1FFFFF) * scaleY + bbMin.Y;
            var z = ((vertex >> 42) & 0x3FFFFF) * scaleZ + bbMin.Z;
            return new Vector3(x, y, z);
        }

        private static Vector3 DecompressPackedVertex(uint vertex, Vector3 scale, Vector3 offset)
        {
            var x = (vertex & 0x7FF) * scale.X + offset.X;
            var y = ((vertex >> 11) & 0x7FF) * scale.Y + offset.Y;
            var z = ((vertex >> 22) & 0x3FF) * scale.Z + offset.Z;
            return new Vector3(x, y, z);
        }

        private static ulong CompressSharedVertex(Vector3 vert, Vector3 min, Vector3 max)
        {
            var scaleX = (max.X - min.X) / ((1 << 21) - 1);
            var scaleY = (max.Y - min.Y) / ((1 << 21) - 1);
            var scaleZ = (max.Z - min.Z) / ((1 << 22) - 1);
            var x = (ulong) ((vert.X - min.X) / scaleX);
            var y = (ulong) ((vert.Y - min.Y) / scaleY);
            var z = (ulong) ((vert.Z - min.Z) / scaleZ);
            return (x & 0x1FFFFF) | ((y & 0x1FFFFF) << 21) | ((z & 0x3FFFFF) << 42);
        }

        private static uint CompressPackedVertex(Vector3 vert, Vector3 scale, Vector3 offset)
        {
            var x = (uint) MathF.Min(MathF.Max(vert.X - offset.X, 0) / scale.X, 0x7FF);
            var y = (uint) MathF.Min(MathF.Max(vert.Y - offset.Y, 0) / scale.Y, 0x7FF);
            var z = (uint) MathF.Min(MathF.Max(vert.Z - offset.Z, 0) / scale.Z, 0x3FF);
            return (x & 0x7FF) | ((y & 0x7FF) << 11) | ((z & 0x3FF) << 22);
        }

        private static int GetBitLength(int n)
        {
            return n == 0 ? 0 : Convert.ToString(n, 2).Length;
        }

        public static hkpBvCompressedMeshShape Build(List<Vector3> verts, List<uint> indices,
            IEnumerable<Tuple<uint, uint>> primitiveInfos)
        {
            var shape = new hkpBvCompressedMeshShape
            {
                m_dispatchType = ShapeDispatchTypeEnum.USER,
                m_bitsPerKey = 0,
                m_shapeInfoCodecType = 0,
                m_userData = 0,
                m_bvTreeType = BvTreeType.BVTREE_COMPRESSED_MESH,
                m_convexRadius = .0f,
                m_weldingType = WeldingType.WELDING_TYPE_ANTICLOCKWISE,
                m_hasPerPrimitiveUserData = true,
                m_hasPerPrimitiveCollisionFilterInfo = true,
                m_collisionFilterInfoPalette = new List<uint>(),
                m_userDataPalette = new List<uint>(),
                m_userStringPalette = new List<string> {"A lonely compressed mesh"},
                m_tree = new hkpBvCompressedMeshShapeTree
                {
                    m_nodes = new List<hkcdStaticTreeCodec3Axis5>(),
                    m_domain = new hkAabb(),
                    m_sections = new List<hkcdStaticMeshTreeBaseSection>(),
                    m_primitives = new List<hkcdStaticMeshTreeBasePrimitive>(),
                    m_sharedVerticesIndex = new List<ushort>(),
                    m_packedVertices = new List<uint>(),
                    m_sharedVertices = new List<ulong>(),
                    m_primitiveDataRuns = new List<hkpBvCompressedMeshShapeTreeDataRun>()
                }
            };

            // Try and build the BVH for the mesh
            var bv = verts.ToArray();
            var bi = indices.ToArray();
            var bnodes = BVNode.BuildBVHForMesh(bv, bi, bi.Length);

            // Split the mesh into sections using the BVH and primitive counts as
            // guidence
            bnodes[0].ComputePrimitiveCounts();
            bnodes[0].ComputeUniqueIndicesCounts(indices);
            bnodes[0].IsSectionHead = true;
            bnodes[0].AttemptSectionSplit();

            // Take out the section heads and replace them with new leafs that reference
            // the new section
            var sectionBVHs = new List<BVNode>();
            foreach (var node in bnodes.Where(node => node.IsSectionHead))
            {
                sectionBVHs.Add(new BVNode
                {
                    Min = node.Min,
                    Max = node.Max,
                    PrimitiveCount = node.PrimitiveCount,
                    Left = node.Left,
                    Right = node.Right,
                    IsLeaf = node.IsLeaf
                });
                node.Left = null;
                node.Right = null;
                node.IsLeaf = true;
                node.Primitive = (uint) (sectionBVHs.Count - 1);
            }

            var primitiveInfoIndices = new List<Tuple<ushort, ushort>>();
            foreach (var (cfi, ud) in primitiveInfos)
            {
                var cfiIndex = shape.m_collisionFilterInfoPalette.IndexOf(cfi);
                var udIndex = shape.m_userDataPalette.IndexOf(ud);

                if (cfiIndex == -1)
                {
                    cfiIndex = shape.m_collisionFilterInfoPalette.Count;
                    shape.m_collisionFilterInfoPalette.Add(cfi);
                }

                if (udIndex == -1)
                {
                    udIndex = shape.m_userDataPalette.Count;
                    shape.m_userDataPalette.Add(ud);
                }

                primitiveInfoIndices.Add(new Tuple<ushort, ushort>((ushort) cfiIndex, (ushort) udIndex));
            }

            var sections = new List<CollisionSection>();
            foreach (var s in sectionBVHs.Select(b => new CollisionSection {SectionHead = b}))
            {
                s.GatherSectionIndices(indices, primitiveInfoIndices);
                sections.Add(s);
            }

            // Count all the indices across sections to figure out what vertices need to be shared
            var indicescount = new byte[indices.Count];
            foreach (var v in sections.SelectMany(s => s.UsedIndices)) indicescount[v]++;

            var shared = new HashSet<uint>();
            for (uint i = 0; i < indices.Count; i++)
                if (indicescount[i] > 1)
                    shared.Add(i);

            // Build shared indices mapping table and compress the shared verts
            var sharedVerts = new List<ulong>();
            var sharedIndexRemapTable = new Dictionary<uint, uint>();
            foreach (var i in shared.OrderBy(x => x))
            {
                sharedIndexRemapTable.Add(i, (uint) sharedVerts.Count);
                sharedVerts.Add(CompressSharedVertex(verts[(int) i], bnodes[0].Min, bnodes[0].Max));
            }


            var tree = shape.m_tree;

            tree.m_domain.m_min = new Vector4(bnodes[0].Min.X, bnodes[0].Min.Y, bnodes[0].Min.Z, 1f);
            tree.m_domain.m_max = new Vector4(bnodes[0].Max.X, bnodes[0].Max.Y, bnodes[0].Max.Z, 1f);
            tree.m_nodes = bnodes[0].BuildAxis5Tree();
            tree.m_sharedVertices = sharedVerts;

            foreach (var s in sections)
            {
                var sharedindexbase = tree.m_sharedVerticesIndex.Count;
                var packedvertbase = tree.m_packedVertices.Count;
                var primitivesbase = tree.m_primitives.Count;
                var datarunsbase = tree.m_primitiveDataRuns.Count;

                var sec = new hkcdStaticMeshTreeBaseSection();
                var offset = s.SectionHead.Min;
                var scale = (s.SectionHead.Max - s.SectionHead.Min) / new Vector3(0x7FF, 0x7FF, 0x3FF);
                sec.m_codecParms_0 = offset.X;
                sec.m_codecParms_1 = offset.Y;
                sec.m_codecParms_2 = offset.Z;
                sec.m_codecParms_3 = scale.X;
                sec.m_codecParms_4 = scale.Y;
                sec.m_codecParms_5 = scale.Z;
                sec.m_domain = new hkAabb
                {
                    m_min = new Vector4(s.SectionHead.Min.X, s.SectionHead.Min.Y, s.SectionHead.Min.Z, 1f),
                    m_max = new Vector4(s.SectionHead.Max.X, s.SectionHead.Max.Y, s.SectionHead.Max.Z, 1f)
                };

                // Map the indices to either shared/packed verts and pack verts that need packing
                var packedIndicesRemap = new Dictionary<uint, byte>();
                byte idxcounter = 0;
                foreach (var idx in s.UsedIndices.OrderBy(x => x))
                {
                    if (shared.Contains(idx)) continue;
                    packedIndicesRemap.Add(idx, idxcounter);
                    var vert = verts[(int) idx];
                    tree.m_packedVertices.Add(CompressPackedVertex(vert, scale, offset));
                    idxcounter++;
                }

                var sharedstart = idxcounter;
                idxcounter = 0;
                foreach (var idx in s.UsedIndices.OrderBy(x => x))
                {
                    if (!shared.Contains(idx)) continue;
                    packedIndicesRemap.Add(idx, (byte) (idxcounter + sharedstart));
                    tree.m_sharedVerticesIndex.Add((ushort) sharedIndexRemapTable[idx]);
                    idxcounter++;
                }

                sec.m_firstPackedVertex = (uint) packedvertbase;
                sec.m_numSharedIndices = idxcounter;
                sec.m_numPackedVertices = sharedstart;
                sec.m_sharedVertices = new hkcdStaticMeshTreeBaseSectionSharedVertices
                {
                    m_data = ((uint) sharedstart & 0xFF) | ((uint) sharedindexbase << 8)
                };

                uint dataRunCount = 0;
                for (byte i = 0; i < s.Indices.Count / 3; i++)
                {
                    // Pack primitive
                    tree.m_primitives.Add(new hkcdStaticMeshTreeBasePrimitive
                    {
                        m_indices_0 = packedIndicesRemap[s.Indices[i * 3]],
                        m_indices_1 = packedIndicesRemap[s.Indices[i * 3 + 1]],
                        m_indices_2 = packedIndicesRemap[s.Indices[i * 3 + 2]],
                        m_indices_3 = packedIndicesRemap[s.Indices[i * 3 + 2]]
                    });

                    // Create datarun
                    var (cfiIndex, udIndex) = s.PrimitiveInfos[i];
                    var run = new hkpBvCompressedMeshShapeTreeDataRun
                    {
                        m_count = 1,
                        m_index = i,
                        // Stores collisionFilterInfoPalette index, userDataPalette index, override flags
                        m_value = (uint) (cfiIndex | (udIndex << 8) | 0)
                    };

                    if (i > 0)
                    {
                        // Check if last dataRun contains the same value
                        // If yes, use it instead of creating a new one
                        var lastRun = tree.m_primitiveDataRuns.Last();
                        if (lastRun.m_value == run.m_value)
                        {
                            lastRun.m_count += 1;
                            continue;
                        }
                    }

                    tree.m_primitiveDataRuns.Add(run);
                    dataRunCount += 1;
                }

                sec.m_primitives = new hkcdStaticMeshTreeBaseSectionPrimitives
                {
                    m_data = (((uint) s.Indices.Count / 3) & 0xFF) | ((uint) primitivesbase << 8)
                };

                sec.m_dataRuns = new hkcdStaticMeshTreeBaseSectionDataRuns
                {
                    m_data = (dataRunCount & 0xFF) | ((uint) datarunsbase << 8)
                };

                sec.m_nodes = s.SectionHead.BuildAxis4Tree();

                tree.m_sections.Add(sec);
            }

            tree.m_numPrimitiveKeys = tree.m_primitives.Count;

            // The maxKeyValue is probably not correct,
            // but the calculated bitsPerKey value seems legit.
            tree.m_maxKeyValue = (uint) tree.m_sections.Select(section => section.m_nodes.Count).Max();
            tree.m_bitsPerKey = GetBitLength((int) tree.m_maxKeyValue) +
                                GetBitLength(tree.m_sections.Count - 1);

            return shape;
        }

        private class CollisionSection
        {
            public List<uint> Indices;

            public List<Tuple<ushort, ushort>> PrimitiveInfos;
            public BVNode SectionHead;
            public HashSet<uint> UsedIndices;

            public void GatherSectionIndices(IReadOnlyList<uint> indices,
                IReadOnlyList<Tuple<ushort, ushort>> primitiveInfos)
            {
                Indices = new List<uint>();
                UsedIndices = new HashSet<uint>();

                PrimitiveInfos = new List<Tuple<ushort, ushort>>();

                var tstack = new Stack<BVNode>();
                tstack.Push(SectionHead);
                while (tstack.TryPop(out var n))
                    if (n.IsLeaf)
                    {
                        var p = n.Primitive;
                        n.Primitive = (uint) (Indices.Count / 3);
                        Indices.Add(indices[(int) p * 3]);
                        Indices.Add(indices[(int) p * 3 + 1]);
                        Indices.Add(indices[(int) p * 3 + 2]);
                        UsedIndices.Add(indices[(int) p * 3]);
                        UsedIndices.Add(indices[(int) p * 3 + 1]);
                        UsedIndices.Add(indices[(int) p * 3 + 2]);
                        PrimitiveInfos.Add(primitiveInfos[(int) p]);
                    }
                    else
                    {
                        tstack.Push(n.Left);
                        tstack.Push(n.Right);
                    }
            }
        }
    }
}