using System;
namespace BetterPenetrationNext.Core
{
    // One feasible amplitude for a continuous surface field. Constraints never
    // move adjacent material vertices independently or alter skin identities.
    public static class CoherentSurfaceLimiter
    {
        public static double PositivePrefix(double a,double b,double c)
        {
            if(c<0)return 0;
            if(Math.Abs(a)<1e-14*Math.Max(1e-300,Math.Abs(b)+Math.Abs(c)))return b<0?Math.Max(0,Math.Min(1,-c/b)):1;
            double disc=b*b-4*a*c;if(disc<0)return 1;
            double q=-.5*(b+(b>=0?1:-1)*Math.Sqrt(disc));double x=q/a,y=Math.Abs(q)>1e-300?c/q:x;
            if(c==0&&b==0&&a<0)return 0;
            double result=1;if(x>=0&&x<result&&2*a*x+b<0)result=x;if(y>=0&&y<result&&2*a*y+b<0)result=y;return Math.Max(0,result);
        }
        public static double Project(Vec3[] baseline,Vec3[] target,EdgeConstraint[] edges,double[] reference,int[] triangles,double maximumStrain,double[] coordinates=null,bool preserveOrientation=false)
        {
            double blend=1;
            for(int e=0;e<edges.Length;e++)
            {
                int a=edges[e].A,b=edges[e].B;Vec3 initial=baseline[b]-baseline[a],delta=target[b]-target[a]-initial;
                double limit=Math.Max(reference[e],initial.Length)*(1+Math.Max(0,maximumStrain)*.999);
                blend=Math.Min(blend,PositivePrefix(-delta.LengthSquared,-2*Vec3.Dot(initial,delta),limit*limit-initial.LengthSquared));
            }
            for(int t=0;t<triangles.Length;t+=3)
            {
                int a=triangles[t],b=triangles[t+1],c=triangles[t+2];Vec3 ab=baseline[b]-baseline[a],ac=baseline[c]-baseline[a],n=Vec3.Cross(ab,ac);
                bool interior=preserveOrientation||coordinates!=null&&coordinates[a]>=.125&&coordinates[b]>=.125&&coordinates[c]>=.125&&coordinates[a]<=.875&&coordinates[b]<=.875&&coordinates[c]<=.875;
                if(n.LengthSquared<1e-28||!interior&&Vec3.Cross(target[b]-target[a],target[c]-target[a]).LengthSquared>=n.LengthSquared*.0001)continue;Vec3 db=target[b]-target[a]-ab,dc=target[c]-target[a]-ac;
                blend=Math.Min(blend,PositivePrefix(Vec3.Dot(Vec3.Cross(db,dc),n),Vec3.Dot(Vec3.Cross(db,ac)+Vec3.Cross(ab,dc),n),n.LengthSquared*.99));
            }
            if(blend<1){blend*=.999999;for(int i=0;i<target.Length;i++)target[i]=baseline[i]+(target[i]-baseline[i])*blend;}
            return blend;
        }
    }
}
