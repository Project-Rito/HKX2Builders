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
        public static extern bool AddTileForMesh([In] Vector3[] verts, int vcount, [In] int[] indices, int icount);

        [DllImport("libNavGen")]
        public static extern void ClearTiles();

        [DllImport("libNavGen")]
        public static extern bool MergeTiles();

        [DllImport("libNavGen")]
        public static extern int GetMergedMeshMaxVertsPerPoly();

        [DllImport("libNavGen")]
        public static extern int GetMergedMeshVertCount();

        [DllImport("libNavGen")]
        public static extern int GetMergedMeshPolyCount();

        [DllImport("libNavGen")]
        public static extern void GetMergedMeshVerts([In] [Out] ushort[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetMergedMeshPolys([In] [Out] ushort[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetMergedMeshAreas([In][Out] byte[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetMergedMeshRegions([In][Out] ushort[] buffer);

        //[DllImport("libNavGen")]
        //public static extern void GetBoundingBox([In] [Out] Vector3[] buffer);
    }
}