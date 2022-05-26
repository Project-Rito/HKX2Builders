using System.Collections.Generic;
using System.Numerics;

namespace HKX2Builders
{
    public class MeshContainer
    {
        public string Name;

        public List<List<int>> Primitives;

        public List<Vector4> Vertices;

        public MeshContainer(string name)
        {
            Name = name;
            Vertices = new List<Vector4>();
            Primitives = new List<List<int>>();
        }

        public MeshContainer Offset(float offsetX, float offsetY, float offsetZ)
        {
            for (var i = 0; i < Vertices.Count; i++)
                Vertices[i] = new Vector4(
                    Vertices[i].X + offsetX,
                    Vertices[i].Y + offsetY,
                    Vertices[i].Z + offsetZ,
                    Vertices[i].W);

            return this;
        }
    }

}
