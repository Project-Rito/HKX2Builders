using System.Numerics;

namespace HKX2Builders
{
    internal class Domain
    {
        public Vector4 Max;
        public Vector4 Min;

        public Domain(Vector4 min, Vector4 max)
        {
            Min = min;
            Max = max;
        }
    }

}
