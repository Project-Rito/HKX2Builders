using System.Numerics;
using HKX2;

namespace HKX2Builders.Extensions
{
    public static partial class Extensions
    {
        // Recursively builds the BVH tree from the compressed packed array
        private static BVNode buildBVHTree(this hkcdStaticMeshTreeBaseSection _this, Vector3 parentBBMin,
            Vector3 parentBBMax, uint nodeIndex)
        {
            var cnode = _this.m_nodes[(int) nodeIndex];
            var node = new BVNode
            {
                Min = cnode.DecompressMin(parentBBMin, parentBBMax),
                Max = cnode.DecompressMax(parentBBMin, parentBBMax)
            };

            if ((cnode.m_data & 0x01) > 0)
            {
                node.Left = buildBVHTree(_this, node.Min, node.Max, nodeIndex + 1);
                node.Right = buildBVHTree(_this, node.Min, node.Max, nodeIndex + ((uint) cnode.m_data & 0xFE));
            }
            else
            {
                node.IsLeaf = true;
                node.Primitive = (uint) cnode.m_data / 2;

                node.PrimitiveCount = 1;
            }

            return node;
        }

        // Extracts an easily processable BVH tree from the packed version in the mesh data
        public static BVNode getSectionBVH(this hkcdStaticMeshTreeBaseSection _this)
        {
            if (_this.m_nodes == null || _this.m_nodes.Count == 0) return null;

            var root = new BVNode
            {
                Min = new Vector3(_this.m_domain.m_min.X, _this.m_domain.m_min.Y, _this.m_domain.m_min.Z),
                Max = new Vector3(_this.m_domain.m_max.X, _this.m_domain.m_max.Y, _this.m_domain.m_max.Z)
            };

            var cnode = _this.m_nodes[0];
            if ((cnode.m_data & 0x01) > 0)
            {
                root.Left = buildBVHTree(_this, root.Min, root.Max, 1);
                root.Right = buildBVHTree(_this, root.Min, root.Max, (uint) cnode.m_data & 0xFE);

                root.PrimitiveCount = root.ComputePrimitiveCounts();
                root.IsSectionHead = true; // This is a guess.
            }
            else
            {
                root.IsLeaf = true;
                root.Primitive = (uint) cnode.m_data / 2;

                root.PrimitiveCount = 1;
            }

            return root;
        }
    }
}