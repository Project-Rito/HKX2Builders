using System.Numerics;
using System.Runtime.InteropServices;

namespace HKX2Builders
{
    public static class NavMeshNative
    {
        [DllImport("libNavGen")]
        public static extern bool SetNavmeshBuildParams(
            float cs, float ch, float walkableSlopeAngle,
            float walkableHeight, float walkableClimb,
            float walkableRadius, int minRegionArea);

        [DllImport("libNavGen")]
        public static extern bool BuildNavmeshForMesh([In] Vector3[] verts, int vcount, [In] int[] indices, int icount);

        [DllImport("libNavGen")]
        public static extern int GetMeshVertCount();

        [DllImport("libNavGen")]
        public static extern int GetMeshTriCount();

        [DllImport("libNavGen")]
        public static extern void GetMeshVerts([In] [Out] ushort[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetMeshTris([In] [Out] ushort[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetMeshAreas([In][Out] byte[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetMeshRegions([In][Out] ushort[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetBoundingBox([In] [Out] Vector3[] buffer);
    }
}