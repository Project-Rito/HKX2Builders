using System.Numerics;
using HKX2;

namespace HKX2Builders.Extensions
{
    public static partial class Extensions
    {
        private static BVHNode BuildBvhTree(this hkcdStaticTreeDefaultTreeStorage6 _this, Vector3 parentBBMin,
            Vector3 parentBBMax, uint nodeIndex)
        {
            var cnode = _this.m_nodes[(int) nodeIndex];
            var node = new BVHNode
            {
                Min = cnode.DecompressMin(parentBBMin, parentBBMax),
                Max = cnode.DecompressMax(parentBBMin, parentBBMax)
            };

            if ((cnode.m_hiData & 0x80) > 0)
            {
                node.Left = BuildBvhTree(_this, node.Min, node.Max, nodeIndex + 1);
                node.Right = BuildBvhTree(_this, node.Min, node.Max,
                    nodeIndex + ((((uint) cnode.m_hiData & 0x7F) << 16) | cnode.m_loData) * 2);
            }
            else
            {
                node.IsTerminal = true;
                node.Index = (((uint) cnode.m_hiData & 0x7F) << 16) | cnode.m_loData;
            }

            return node;
        }

        // Extracts an easily processable BVH tree from the packed version in the mesh data
        public static BVHNode GetBVH(this hkcdStaticTreeDefaultTreeStorage6 _this)
        {
            if (_this.m_nodes == null || _this.m_nodes.Count == 0) return null;

            var root = new BVHNode
            {
                Min = new Vector3(_this.m_domain.m_min.X, _this.m_domain.m_min.Y,
                    _this.m_domain.m_min.Z),
                Max = new Vector3(_this.m_domain.m_max.X, _this.m_domain.m_max.Y,
                    _this.m_domain.m_max.Z)
            };

            var cnode = _this.m_nodes[0];
            if ((cnode.m_hiData & 0x80) > 0)
            {
                root.Left = BuildBvhTree(_this, root.Min, root.Max, 1);
                root.Right = BuildBvhTree(_this, root.Min, root.Max,
                    ((((uint) cnode.m_hiData & 0x7F) << 16) | cnode.m_loData) * 2);
            }
            else
            {
                root.IsTerminal = true;
                root.Index = (((uint) cnode.m_hiData & 0x7F) << 16) | cnode.m_loData;
            }

            return root;
        }
    }
}