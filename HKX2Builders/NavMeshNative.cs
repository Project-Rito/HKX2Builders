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
            float walkableRadius, int minRegionArea, float detailSampleDist, float detailSampleMaxError);

        [DllImport("libNavGen")]
        public static extern bool BuildTileForMesh([In] Vector3[] verts, int vcount, [In] int[] indices, int icount, int tx, int ty, [In] Vector3 orig, bool field);

        [DllImport("libNavGen")]
        public static extern bool AddPrebuiltTile([In] Vector3[] verts, uint vcount, [In] uint[] indices, [In] uint[] neighbors, uint icount, uint nvp, int tx, int ty, [In] Vector3 orig);

        [DllImport("libNavGen")]
        public static extern void ClearTiles();

        [DllImport("libNavGen")]
        public static extern int GetMaxVertsPerPoly(int tx, int ty);

        [DllImport("libNavGen")]
        public static extern int GetVertCount(int tx, int ty);

        [DllImport("libNavGen")]
        public static extern int GetPolyCount(int tx, int ty);

        [DllImport("libNavGen")]
        public static extern void GetBoundingBox(int tx, int ty, [In] [Out] Vector3[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetVerts(int tx, int ty, [In] [Out] Vector3[] buffer);

        [DllImport("libNavGen")]
        public static extern void GetPolyVerts(int tx, int ty, [In] [Out] ushort[] buffer, uint polyIndex);

        [DllImport("libNavGen")]
        public static extern byte GetPolyVertCount(int tx, int ty, uint polyIndex);

        [DllImport("libNavGen")]
        public static extern void GetPolyNeighborInfo(int tx, int ty, [In][Out] ushort[] buffer, uint polyIndex);

        [StructLayout(LayoutKind.Sequential)]
        public struct PolyTileLink
        {
            public uint SrcPolyIndex;
            public uint SrcEdgeIndex;
            public uint DestPolyIndex;
            public uint DestEdgeIndex;
            public int SrcTileX;
            public int SrcTileY;
            public int DestTileX;
            public int DestTileY;
            public byte Bmin;
            public byte Bmax;
        };

        [DllImport("libNavGen")]
        public static extern void GetPolyLinks(int tx, int ty, [In][Out] PolyTileLink[] buffer, uint polyIndex);

        [DllImport("libNavGen")]
        public static extern uint GetPolyLinkCount(int tx, int ty, uint polyIndex);

        //[DllImport("libNavGen")]
        //public static extern void GetMergedMeshRegions([In][Out] ushort[] buffer);

        //[DllImport("libNavGen")]
        //public static extern void GetBoundingBox([In] [Out] Vector3[] buffer);
    }
}