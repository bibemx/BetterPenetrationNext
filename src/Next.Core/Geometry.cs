using System;

namespace BetterPenetrationNext.Core
{
    public struct Bounds3
    {
        public Vec3 Min, Max;
        public Bounds3(Vec3 min, Vec3 max) { Min=min; Max=max; }
        public static Bounds3 Empty { get { return new Bounds3(new Vec3(double.PositiveInfinity,double.PositiveInfinity,double.PositiveInfinity), new Vec3(double.NegativeInfinity,double.NegativeInfinity,double.NegativeInfinity)); } }
        public static Bounds3 Unbounded { get { return new Bounds3(new Vec3(double.NegativeInfinity,double.NegativeInfinity,double.NegativeInfinity),new Vec3(double.PositiveInfinity,double.PositiveInfinity,double.PositiveInfinity)); } }
        public void Include(Vec3 p) { Min=Vec3.Min(Min,p); Max=Vec3.Max(Max,p); }
        public void Include(Bounds3 b) { Include(b.Min); Include(b.Max); }
        public double DistanceSquared(Vec3 p)
        {
            double x=p.X<Min.X?Min.X-p.X:p.X>Max.X?p.X-Max.X:0;
            double y=p.Y<Min.Y?Min.Y-p.Y:p.Y>Max.Y?p.Y-Max.Y:0;
            double z=p.Z<Min.Z?Min.Z-p.Z:p.Z>Max.Z?p.Z-Max.Z:0;
            return x*x+y*y+z*z;
        }
        public bool IntersectsRay(Vec3 origin, Vec3 direction)
        {
            double lo=0, hi=double.PositiveInfinity;
            for(int axis=0;axis<3;axis++)
            {
                double d=direction.Axis(axis), p=origin.Axis(axis);
                if(Math.Abs(d)<1e-30) { if(p<Min.Axis(axis)||p>Max.Axis(axis)) return false; continue; }
                double a=(Min.Axis(axis)-p)/d, b=(Max.Axis(axis)-p)/d;
                lo=Math.Max(lo,Math.Min(a,b)); hi=Math.Min(hi,Math.Max(a,b));
                if(hi<lo) return false;
            }
            return true;
        }
    }

    public static class Geometry
    {
        static Vec3 Segment(Vec3 p,Vec3 a,Vec3 b)
        { Vec3 ab=b-a; double n=ab.LengthSquared; return n>1e-30 ? a+ab*Math.Max(0,Math.Min(1,Vec3.Dot(p-a,ab)/n)) : a; }

        public static Vec3 ClosestPoint(Vec3 p,Vec3 a,Vec3 b,Vec3 c)
        {
            Vec3 ab=b-a, ac=c-a;
            if(Vec3.Cross(ab,ac).LengthSquared < ab.LengthSquared*ac.LengthSquared*1e-24)
            {
                Vec3 q=Segment(p,a,b), r=Segment(p,b,c), s=Segment(p,c,a);
                if((p-r).LengthSquared<(p-q).LengthSquared) q=r;
                return (p-s).LengthSquared<(p-q).LengthSquared ? s:q;
            }
            Vec3 ap=p-a; double d1=Vec3.Dot(ab,ap), d2=Vec3.Dot(ac,ap);
            if(d1<=0&&d2<=0) return a;
            Vec3 bp=p-b; double d3=Vec3.Dot(ab,bp), d4=Vec3.Dot(ac,bp);
            if(d3>=0&&d4<=d3) return b;
            double vc=d1*d4-d3*d2;
            if(vc<=0&&d1>=0&&d3<=0) return a+ab*(d1/(d1-d3));
            Vec3 cp=p-c; double d5=Vec3.Dot(ab,cp), d6=Vec3.Dot(ac,cp);
            if(d6>=0&&d5<=d6) return c;
            double vb=d5*d2-d1*d6;
            if(vb<=0&&d2>=0&&d6<=0) return a+ac*(d2/(d2-d6));
            double va=d3*d6-d5*d4;
            if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0) return b+(c-b)*((d4-d3)/((d4-d3)+(d5-d6)));
            double inv=1/(va+vb+vc); return a+ab*(vb*inv)+ac*(vc*inv);
        }

        public static bool RayTriangle(Vec3 origin, Vec3 direction, Vec3 a, Vec3 b, Vec3 c, out double distance)
        {
            Vec3 e1=b-a,e2=c-a,h=Vec3.Cross(direction,e2);
            double det=Vec3.Dot(e1,h), scale=e1.Length*e2.Length;
            distance=0;
            if(Math.Abs(det)<scale*1e-12 || scale<1e-30) return false;
            double inv=1/det; Vec3 s=origin-a;
            double u=inv*Vec3.Dot(s,h); if(u<0||u>1) return false;
            Vec3 q=Vec3.Cross(s,e1); double v=inv*Vec3.Dot(direction,q);
            if(v<0||u+v>1) return false;
            distance=inv*Vec3.Dot(e2,q); return distance>Math.Sqrt(scale)*1e-12;
        }
    }

    // Immutable for one source pose; shared by all nearest-point queries on a triangle.
    internal struct TriangleQuery
    {
        public Vec3 A,B,C,AB,AC,Normal;
        public bool Degenerate;
        public TriangleQuery(Vec3 a,Vec3 b,Vec3 c)
        {
            A=a; B=b; C=c; AB=b-a; AC=c-a; var cross=Vec3.Cross(AB,AC); double area=cross.Length;
            Degenerate=area*area<AB.LengthSquared*AC.LengthSquared*1e-24||area<1e-30;
            Normal=Degenerate?new Vec3():cross/area;
        }
        public Vec3 Closest(Vec3 p)
        {
            if(Degenerate) return Geometry.ClosestPoint(p,A,B,C);
            Vec3 ap=p-A; double d1=Vec3.Dot(AB,ap),d2=Vec3.Dot(AC,ap); if(d1<=0&&d2<=0) return A;
            Vec3 bp=p-B; double d3=Vec3.Dot(AB,bp),d4=Vec3.Dot(AC,bp); if(d3>=0&&d4<=d3) return B;
            double vc=d1*d4-d3*d2; if(vc<=0&&d1>=0&&d3<=0) return A+AB*(d1/(d1-d3));
            Vec3 cp=p-C; double d5=Vec3.Dot(AB,cp),d6=Vec3.Dot(AC,cp); if(d6>=0&&d5<=d6) return C;
            double vb=d5*d2-d1*d6; if(vb<=0&&d2>=0&&d6<=0) return A+AC*(d2/(d2-d6));
            double va=d3*d6-d5*d4; if(va<=0&&d4-d3>=0&&d5-d6>=0) return B+(C-B)*((d4-d3)/((d4-d3)+(d5-d6)));
            double inverse=1/(va+vb+vc); return A+AB*(vb*inverse)+AC*(vc*inverse);
        }
    }
}
