using System.Collections.Generic;
using System.Numerics;

namespace HKX2Builders
{
    internal class MeshSection
    {
        public Domain Domain;

        public List<List<byte>> Primitives;

        public List<Vector4> Vertices;
    }

}
