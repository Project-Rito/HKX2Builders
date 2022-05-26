using System.Numerics;

namespace HKX2Builders
{
    internal class PackedVertex
    {
        private readonly uint _vtx;

        public PackedVertex(uint vtx)
        {
            _vtx = vtx;
        }

        public Vector4 Decompress(Vector3 offset, Vector3 scale)
        {
            return new Vector4((_vtx & 0x7FFu) * scale.X + offset.X,
                ((_vtx >> 11) & 0x7FFu) * scale.Y + offset.Y,
                ((_vtx >> 22) & 0x3FFu) * scale.Z + offset.Z, 1f);
        }
    }
}
