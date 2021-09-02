using System.Numerics;
using HKX2;

namespace HKX2Builders.Extensions
{
    public static partial class Extensions
    {
        public static Vector3 DecompressMin(this hkcdStaticTreeCodec3Axis _this, Vector3 parentMin, Vector3 parentMax)
        {
            var x =
                (_this.m_xyz_0 >> 4) * (float) (_this.m_xyz_0 >> 4) * (1.0f / 226.0f) * (parentMax.X - parentMin.X) +
                parentMin.X;
            var y =
                (_this.m_xyz_1 >> 4) * (float) (_this.m_xyz_1 >> 4) * (1.0f / 226.0f) * (parentMax.Y - parentMin.Y) +
                parentMin.Y;
            var z =
                (_this.m_xyz_2 >> 4) * (float) (_this.m_xyz_2 >> 4) * (1.0f / 226.0f) * (parentMax.Z - parentMin.Z) +
                parentMin.Z;
            return new Vector3(x, y, z);
        }

        public static Vector3 DecompressMax(this hkcdStaticTreeCodec3Axis _this, Vector3 parentMin, Vector3 parentMax)
        {
            var x = -((_this.m_xyz_0 & 0x0F) * (float) (_this.m_xyz_0 & 0x0F)) * (1.0f / 226.0f) *
                (parentMax.X - parentMin.X) + parentMax.X;
            var y = -((_this.m_xyz_1 & 0x0F) * (float) (_this.m_xyz_1 & 0x0F)) * (1.0f / 226.0f) *
                (parentMax.Y - parentMin.Y) + parentMax.Y;
            var z = -((_this.m_xyz_2 & 0x0F) * (float) (_this.m_xyz_2 & 0x0F)) * (1.0f / 226.0f) *
                (parentMax.Z - parentMin.Z) + parentMax.Z;
            return new Vector3(x, y, z);
        }
    }
}