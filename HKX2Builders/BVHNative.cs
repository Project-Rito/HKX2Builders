using System.Numerics;
using System.Runtime.InteropServices;

namespace HKX2Builders
{
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct NativeBVHNode
    {
        public readonly float minX;
        public readonly float maxX;
        public readonly float minY;
        public readonly float maxY;
        public readonly float minZ;
        public readonly float maxZ;
        public readonly uint primitiveCount;
        public readonly uint firstChildOrPrimitive;

        public bool isLeaf => primitiveCount > 0;
    }

    public static class BVHNative
    {
        [DllImport("libNavGen")]
        public static extern bool BuildBVHForDomains([In] Vector3[] domains, int domainCount);

        [DllImport("libNavGen")]
        public static extern bool BuildBVHForMesh([In] Vector3[] vertices, [In] uint[] indices, int icount);

        [DllImport("libNavGen")]
        public static extern ulong GetNodeSize();

        [DllImport("libNavGen")]
        public static extern ulong GetBVHSize();

        [DllImport("libNavGen")]
        public static extern void GetBVHNodes([In] [Out] NativeBVHNode[] buffer);
    }
}