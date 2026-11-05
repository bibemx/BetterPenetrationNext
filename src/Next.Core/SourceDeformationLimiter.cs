using System;

namespace BetterPenetrationNext.Core
{
    // Finds the connected interval of admissible corrections starting at zero.
    // A triangle's projected signed area is a quadratic in the correction scale.
    public static class SourceDeformationLimiter
    {
        public static double Limit(Vec3[] animated,Vec3[] offsets,int[] triangles,double minimumAreaRatio=.01)
        {
            if(animated==null||offsets==null||triangles==null||animated.Length!=offsets.Length||!Vec3.Finite(minimumAreaRatio)||minimumAreaRatio<=0||minimumAreaRatio>=1)
                throw new ArgumentException("Invalid source area constraint");
            double scale=1;
            for(int t=0;t<triangles.Length;t+=3)
            {
                int a=triangles[t],b=triangles[t+1],c=triangles[t+2];
                Vec3 ab=animated[b]-animated[a],ac=animated[c]-animated[a],normal=Vec3.Cross(ab,ac);
                double area=normal.LengthSquared;if(area<1e-24)continue;
                Vec3 db=offsets[b]-offsets[a],dc=offsets[c]-offsets[a];
                double quadratic=Vec3.Dot(Vec3.Cross(db,dc),normal)/area;
                double linear=Vec3.Dot(Vec3.Cross(db,ac)+Vec3.Cross(ab,dc),normal)/area;
                double constant=1-minimumAreaRatio,root=double.PositiveInfinity;
                if(!Vec3.Finite(quadratic)||!Vec3.Finite(linear))return 0;
                // Evaluate the exact polynomial; no repeated mesh walks or allocations.
                if(Math.Abs(quadratic)<1e-14)
                {
                    if(linear<0)root=-constant/linear;
                }
                else
                {
                    double discriminant=linear*linear-4*quadratic*constant;
                    if(discriminant>0)
                    {
                        double q=-.5*(linear+(linear>=0?1:-1)*Math.Sqrt(discriminant));
                        double r1=q/quadratic,r2=q==0?double.PositiveInfinity:constant/q;
                        if(r1>0)root=r1;if(r2>0)root=Math.Min(root,r2);
                    }
                }
                if(root<=scale)scale=Math.Max(0,root*(1-1e-10));
            }
            return scale;
        }
    }
}
