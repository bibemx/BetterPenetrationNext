using System;
namespace BetterPenetrationNext.Core
{
    // Bounded convex section proxy from cached REAL surface landmarks. Open
    // ends stay open: a point must lie between two evaluated material sections.
    public static class SourceHeadEnvelope
    {
        public static double Penetration(MeshLandmarkSection[] sections,Vec3 point,double margin,out Vec3 normal)
        {int span;double blend;return Penetration(sections,point,margin,out normal,out span,out blend);}
        public static double Penetration(MeshLandmarkSection[] sections,Vec3 point,double margin,out Vec3 normal,out int materialSpan,out double materialBlend,double minimumCoordinate=.75)
        {
            materialSpan=-1;materialBlend=0;
            int first=0;while(first<sections.Length-1&&sections[first].Coordinate<minimumCoordinate)first++;
            int span=-1;double blend=0,best=double.PositiveInfinity;normal=new Vec3();
            for(int i=first;i<sections.Length-1;i++)
            {
                Vec3 edge=sections[i+1].Centre-sections[i].Centre;double d=edge.LengthSquared;if(d<1e-20)continue;
                double t=Vec3.Dot(point-sections[i].Centre,edge)/d;if(t<0||t>1)continue;
                double distance=(point-sections[i].Centre-edge*t).LengthSquared;if(distance<best){best=distance;span=i;blend=t;}
            }
            if(span<0)return 0;
            var a=sections[span];var b=sections[span+1];Vec3 axis=(b.Centre-a.Centre).Normalized,centre=a.Centre*(1-blend)+b.Centre*blend;
            double sign=0,nearest=double.PositiveInfinity;Vec3 boundary=new Vec3(),facet=new Vec3();
            for(int k=0;k<a.Radial.Length;k++)
            {
                int j=(k+1)%a.Radial.Length;
                Vec3 p=a.Radial[k]*(1-blend)+b.Radial[k]*blend,q=a.Radial[j]*(1-blend)+b.Radial[j]*blend;
                p-=axis*Vec3.Dot(p-centre,axis);q-=axis*Vec3.Dot(q-centre,axis);
                Vec3 e=q-p;double side=Vec3.Dot(Vec3.Cross(e,point-p),axis);
                if(Math.Abs(side)>1e-16){if(sign!=0&&side*sign<0)return 0;sign=side;}
                double t=e.LengthSquared<1e-24?0:Math.Max(0,Math.Min(1,Vec3.Dot(point-p,e)/e.LengthSquared));Vec3 closest=p+e*t;double distance=(closest-point).LengthSquared;
                if(distance<nearest&&e.LengthSquared>1e-24)
                {
                    nearest=distance;boundary=closest;Vec3 longitudinal=(b.Radial[k]-a.Radial[k])*(1-t)+(b.Radial[j]-a.Radial[j])*t;
                    facet=Vec3.Cross(e,longitudinal).Normalized;if(Vec3.Dot(facet,boundary-centre)<0)facet=-facet;
                }
            }
            if(sign==0||facet.LengthSquared<.5||!Vec3.Finite(nearest))return 0;
            materialSpan=span;materialBlend=blend;
            normal=facet;return Math.Max(0,Vec3.Dot(boundary-point,facet)+Math.Max(0,margin));
        }
    }
}
