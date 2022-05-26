using System.Numerics;

namespace HKX2Builders
{
    internal class SharedVertex
    {
        private readonly ulong _vtx;

        public SharedVertex(ulong vtx)
        {
            _vtx = vtx;
        }

        public Vector4 Decompress(Domain domain)
        {
            var offset = domain.Min;
            var scale = Vector4.Subtract(domain.Max, domain.Min);
            return new Vector4((_vtx & 0x1FFFFF) / 2097151f * scale.X + offset.X,
                ((_vtx >> 21) & 0x1FFFFF) / 2097151f * scale.Y + offset.Y,
                ((_vtx >> 42) & 0x3FFFFF) / 4194303f * scale.Z + offset.Z, 1f);
        }
    }

}
