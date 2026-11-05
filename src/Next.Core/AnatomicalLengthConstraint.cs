using System;
namespace BetterPenetrationNext.Core
{
    public static class AnatomicalLengthConstraint
    {
        public static double Profile(double u){u=Math.Max(0,Math.Min(1,u));return u*u*(3-2*u);}
        public static double Arc(Vec3[] points,double[] station,Vec3 axis,double shift)
        {
            double arc=0;for(int i=1;i<points.Length;i++)arc+=(points[i]-points[i-1]+axis*(shift*(Profile(station[i])-Profile(station[i-1])))).Length;return arc;
        }
        public static double Solve(Vec3[] points,double[] station,Vec3 axis,double extra,double maximumShift)
        {
            if(points.Length<2||station.Length!=points.Length||!axis.IsFinite||!Vec3.Finite(extra)||!Vec3.Finite(maximumShift)||maximumShift<0)throw new ArgumentException("Invalid anatomical length constraint");
            axis=axis.Normalized;if(extra<=0||axis.LengthSquared<.5)return 0;
            double target=Arc(points,station,axis,0)+extra,lo=0,hi=maximumShift;
            if(Arc(points,station,axis,hi)<=target)return hi;
            for(int i=0;i<22;i++){double mid=(lo+hi)*.5;if(Arc(points,station,axis,mid)<target)lo=mid;else hi=mid;}
            return (lo+hi)*.5;
        }
    }
}
