using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using HKX2;
using HKX2Builders.Extensions;

namespace HKX2Builders
{
    public class BVNode
    {
        public bool IsLeaf;

        public bool IsSectionHead;
        public BVNode Left;
        public Vector3 Max;
        public Vector3 Min;
        public uint Primitive;
        public uint PrimitiveCount;
        public BVNode Right;
        private uint UniqueIndicesCount;

        public static List<BVNode> BuildBVHForMesh(Vector3[] vertices, uint[] indices, int indicesCount)
        {
            if (!BVHNative.BuildBVHForMesh(vertices, indices, indicesCount)) throw new Exception("Couldn't build BVH!");

            return BuildFriendlyBVH();
        }

        public static List<BVNode> BuildBVHForDomains(Vector3[] domains, int domainCount)
        {
            if (!BVHNative.BuildBVHForDomains(domains, domainCount)) throw new Exception("Couldn't build BVH!");

            return BuildFriendlyBVH();
        }

        private static List<BVNode> BuildFriendlyBVH()
        {
            var nodes = new NativeBVHNode[BVHNative.GetBVHSize()];
            BVHNative.GetBVHNodes(nodes);

            // Rebuild in friendlier tree form
            var bnodes = nodes.Select(n => new BVNode
            {
                Min = new Vector3(n.minX, n.minY, n.minZ),
                Max = new Vector3(n.maxX, n.maxY, n.maxZ),
                IsLeaf = n.isLeaf,
                PrimitiveCount = n.primitiveCount,
                Primitive = n.firstChildOrPrimitive
            }).ToList();

            for (var i = 0; i < nodes.Length; i++)
            {
                if (nodes[i].isLeaf) continue;
                bnodes[i].Left = bnodes[(int) nodes[i].firstChildOrPrimitive];
                bnodes[i].Right = bnodes[(int) nodes[i].firstChildOrPrimitive + 1];
            }

            return bnodes;
        }

        public uint ComputePrimitiveCounts()
        {
            if (IsLeaf) return PrimitiveCount;

            PrimitiveCount = Left.ComputePrimitiveCounts() + Right.ComputePrimitiveCounts();
            return PrimitiveCount;
        }

        public HashSet<uint> ComputeUniqueIndicesCounts(List<uint> indices)
        {
            if (IsLeaf)
            {
                var s = new HashSet<uint>
                {
                    indices[(int) Primitive * 3],
                    indices[(int) Primitive * 3 + 1],
                    indices[(int) Primitive * 3 + 2]
                };
                UniqueIndicesCount = 3;
                return s;
            }

            var left = Left.ComputeUniqueIndicesCounts(indices);
            var right = Right.ComputeUniqueIndicesCounts(indices);
            left.UnionWith(right);
            UniqueIndicesCount = (uint) left.Count;
            return left;
        }

        /// <summary>
        ///     Marks nodes that are the head of sections - independently compressed mesh
        ///     chunks with their own BVH
        /// </summary>
        public void AttemptSectionSplit()
        {
            // Very simple primitive count based splitting heuristic for now
            if (IsLeaf || PrimitiveCount <= 127 && UniqueIndicesCount <= 255) return;
            IsSectionHead = false;
            Left.IsSectionHead = true;
            Right.IsSectionHead = true;
            Left.AttemptSectionSplit();
            Right.AttemptSectionSplit();
        }

        private static byte CompressDim(float min, float max, float pmin, float pmax)
        {
            var snorm = 226.0f / (pmax - pmin);
            var rmin = MathF.Sqrt(MathF.Max((min - pmin) * snorm, 0));
            var rmax = MathF.Sqrt(MathF.Max((max - pmax) * -snorm, 0));
            var a = (byte) Math.Min(0xF, (int) MathF.Floor(rmin));
            var b = (byte) Math.Min(0xF, (int) MathF.Floor(rmax));
            return (byte) ((a << 4) | b);
        }

        public List<hkcdStaticTreeCodec3Axis4> BuildAxis4Tree()
        {
            var ret = new List<hkcdStaticTreeCodec3Axis4>();

            void CompressNode(BVNode node, Vector3 parentMin, Vector3 parentMax)
            {
                var currindex = ret.Count();
                var compressed = new hkcdStaticTreeCodec3Axis4();
                ret.Add(compressed);

                // Compress the bounding box
                compressed.m_xyz_0 = CompressDim(node.Min.X, node.Max.X, parentMin.X, parentMax.X);
                compressed.m_xyz_1 = CompressDim(node.Min.Y, node.Max.Y, parentMin.Y, parentMax.Y);
                compressed.m_xyz_2 = CompressDim(node.Min.Z, node.Max.Z, parentMin.Z, parentMax.Z);

                // Read back the decompressed bounding box to use as reference for next compression
                var min = compressed.DecompressMin(parentMin, parentMax);
                var max = compressed.DecompressMax(parentMin, parentMax);

                if (node.IsLeaf)
                {
                    compressed.m_data = (byte) (node.Primitive * 2);
                }
                else
                {
                    // Add the left as the very next node
                    CompressNode(node.Left, min, max);

                    // Encode the index of the right then add it. The index should
                    // always be even
                    compressed.m_data = (byte) ((ret.Count - currindex) | 0x1);

                    // Now encode the right
                    CompressNode(node.Right, min, max);
                }
            }

            CompressNode(this, Min, Max);
            return ret;
        }

        public List<hkcdStaticTreeCodec3Axis5> BuildAxis5Tree()
        {
            var ret = new List<hkcdStaticTreeCodec3Axis5>();

            void CompressNode(BVNode node, Vector3 pbbmin, Vector3 pbbmax, bool root = false)
            {
                var currindex = ret.Count();
                var compressed = new hkcdStaticTreeCodec3Axis5();
                ret.Add(compressed);

                // Compress the bounding box
                compressed.m_xyz_0 = CompressDim(node.Min.X, node.Max.X, pbbmin.X, pbbmax.X);
                compressed.m_xyz_1 = CompressDim(node.Min.Y, node.Max.Y, pbbmin.Y, pbbmax.Y);
                compressed.m_xyz_2 = CompressDim(node.Min.Z, node.Max.Z, pbbmin.Z, pbbmax.Z);

                // Read back the decompressed bounding box to use as reference for next compression
                var min = compressed.DecompressMin(pbbmin, pbbmax);
                var max = compressed.DecompressMax(pbbmin, pbbmax);

                if (node.IsLeaf)
                {
                    var data = (ushort) node.Primitive;
                    compressed.m_loData = (byte) (data & 0xFF);
                    compressed.m_hiData = (byte) ((data >> 8) & 0x7F);
                }
                else
                {
                    // Add the left as the very next node
                    CompressNode(node.Left, min, max);

                    // Encode the index of the right then add it. The index should
                    // always be even
                    var data = (ushort) ((ret.Count() - currindex) / 2);
                    compressed.m_loData = (byte) (data & 0xFF);
                    compressed.m_hiData = (byte) (((data >> 8) & 0x7F) | 0x80);

                    // Now encode the right
                    CompressNode(node.Right, min, max);
                }

                if (root)
                {
                    compressed.m_xyz_0 = 0;
                    compressed.m_xyz_1 = 0;
                    compressed.m_xyz_2 = 0;
                }
            }

            CompressNode(this, Min, Max, true);
            return ret;
        }

        public List<hkcdStaticTreeCodec3Axis6> BuildAxis6Tree()
        {
            var ret = new List<hkcdStaticTreeCodec3Axis6>();

            void CompressNode(BVNode node, Vector3 parentMin, Vector3 parentMax, bool root = false)
            {
                var currindex = ret.Count();
                var compressed = new hkcdStaticTreeCodec3Axis6();
                ret.Add(compressed);

                // Compress the bounding box
                compressed.m_xyz_0 = CompressDim(node.Min.X, node.Max.X, parentMin.X, parentMax.X);
                compressed.m_xyz_1 = CompressDim(node.Min.Y, node.Max.Y, parentMin.Y, parentMax.Y);
                compressed.m_xyz_2 = CompressDim(node.Min.Z, node.Max.Z, parentMin.Z, parentMax.Z);

                // Read back the decompressed bounding box to use as reference for next compression
                var min = compressed.DecompressMin(parentMin, parentMax);
                var max = compressed.DecompressMax(parentMin, parentMax);

                if (node.IsLeaf)
                {
                    var data = node.Primitive;
                    compressed.m_loData = (ushort) (data & 0xFFFF);
                    compressed.m_hiData = (byte) ((data >> 16) & 0x7F);
                }
                else
                {
                    // Add the left as the very next node
                    CompressNode(node.Left, min, max);

                    // Encode the index of the right then add it. The index should
                    // always be even
                    var data = (ushort) ((ret.Count - currindex) / 2);
                    compressed.m_loData = (ushort) (data & 0xFFFF);
                    compressed.m_hiData = (byte) (((data >> 16) & 0x7F) | 0x80);

                    // Now encode the right
                    CompressNode(node.Right, min, max);
                }

                if (root)
                {
                    compressed.m_xyz_0 = 0;
                    compressed.m_xyz_1 = 0;
                    compressed.m_xyz_2 = 0;
                }
            }

            CompressNode(this, Min, Max, true);
            return ret;
        }
    }
}