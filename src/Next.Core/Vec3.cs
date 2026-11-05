using System;
using System.Runtime.CompilerServices;

namespace BetterPenetrationNext.Core
{
    [Serializable] public struct Vec3
    {
        public double X, Y, Z;
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public Vec3(double x, double y, double z) { X = x; Y = y; Z = z; }
        public double LengthSquared { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return X*X+Y*Y+Z*Z; } }
        public double Length { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Math.Sqrt(LengthSquared); } }
        public bool IsFinite { get { return Finite(X) && Finite(Y) && Finite(Z); } }
        public static bool Finite(double n) { return !double.IsNaN(n) && !double.IsInfinity(n); }
        public Vec3 Normalized { get { double l = Length; return l > 1e-30 ? this / l : new Vec3(); } }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Vec3 operator +(Vec3 a, Vec3 b) { return new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Vec3 operator -(Vec3 a, Vec3 b) { return new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Vec3 operator -(Vec3 a) { return a * -1; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Vec3 operator *(Vec3 a, double b) { return new Vec3(a.X * b, a.Y * b, a.Z * b); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Vec3 operator /(Vec3 a, double b) { return a * (1 / b); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static double Dot(Vec3 a, Vec3 b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Vec3 Cross(Vec3 a, Vec3 b) { return new Vec3(a.Y*b.Z-a.Z*b.Y, a.Z*b.X-a.X*b.Z, a.X*b.Y-a.Y*b.X); }
        public static Vec3 ClampLength(Vec3 v, double limit) { double l = v.Length; return l > limit && l > 0 ? v * (limit / l) : v; }
        public static Vec3 Min(Vec3 a, Vec3 b) { return new Vec3(Math.Min(a.X,b.X), Math.Min(a.Y,b.Y), Math.Min(a.Z,b.Z)); }
        public static Vec3 Max(Vec3 a, Vec3 b) { return new Vec3(Math.Max(a.X,b.X), Math.Max(a.Y,b.Y), Math.Max(a.Z,b.Z)); }
        public double Axis(int axis) { return axis == 0 ? X : axis == 1 ? Y : Z; }
        public override string ToString() { return string.Format("({0:G5}, {1:G5}, {2:G5})", X,Y,Z); }
    }

    // Linear skinning Jacobian: columns of the weighted bone matrix (translation excluded).
    public struct Linear3
    {
        public Vec3 X, Y, Z;
        public Linear3(Vec3 x, Vec3 y, Vec3 z) { X=x; Y=y; Z=z; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public Vec3 Multiply(Vec3 v) { return X*v.X + Y*v.Y + Z*v.Z; }
        public bool TryInverseMultiply(Vec3 v, out Vec3 result)
        {
            Vec3 yz = Vec3.Cross(Y,Z);
            double determinant = Vec3.Dot(X,yz);
            double scale = X.Length * Y.Length * Z.Length;
            if (scale < 1e-30 || Math.Abs(determinant) < scale * 1e-7)
            { result = new Vec3(); return false; }
            result = new Vec3(Vec3.Dot(v,yz), Vec3.Dot(v,Vec3.Cross(Z,X)), Vec3.Dot(v,Vec3.Cross(X,Y))) / determinant;
            return result.IsFinite;
        }
        public bool TryInverse(out Linear3 result)
        {
            Vec3 r0=Vec3.Cross(Y,Z),r1=Vec3.Cross(Z,X),r2=Vec3.Cross(X,Y); double determinant=Vec3.Dot(X,r0),scale=X.Length*Y.Length*Z.Length;
            if(scale<1e-30||Math.Abs(determinant)<scale*1e-7) { result=new Linear3(); return false; }
            r0/=determinant; r1/=determinant; r2/=determinant;
            result=new Linear3(new Vec3(r0.X,r1.X,r2.X),new Vec3(r0.Y,r1.Y,r2.Y),new Vec3(r0.Z,r1.Z,r2.Z)); return true;
        }
    }
}
