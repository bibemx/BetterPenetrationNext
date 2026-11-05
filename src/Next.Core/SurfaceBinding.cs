using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    [Serializable] public struct VisualBinding
    {
        public int Vertex,A,B,C;
        public double U,V,W,Influence;
        public Vec3 Transfer(SoftBodySolver solver) { return (solver.Displacement(A)*U+solver.Displacement(B)*V+solver.Displacement(C)*W)*Influence; }
    }
    public static class SurfaceBinding
    {
        // Built offline, not every frame. The same accelerated nearest-triangle code
        // is shared with runtime contact detection; no all-triangle vertex scan.
        public static VisualBinding[] Build(Vec3[] visual,double[] influence,Vec3[] simulation,int[] triangles,double maximumDistance)
        {
            if(visual==null||influence==null||visual.Length!=influence.Length||!Vec3.Finite(maximumDistance)||maximumDistance<=0) throw new ArgumentException("Invalid binding input.");
            var surface=new TriangleSurface(simulation,triangles,SurfaceKind.OrientedShell); var result=new List<VisualBinding>();
            for(int i=0;i<visual.Length;i++)
            {
                if(!Vec3.Finite(influence[i])||influence[i]<0||influence[i]>1) throw new ArgumentException("Invalid visual influence.");
                if(influence[i]==0) continue;
                SurfaceSample sample; if(!surface.TrySample(visual[i],maximumDistance,out sample)) throw new ArgumentException("Visual vertex "+i+" has no simulation triangle within binding distance.");
                int t=sample.Triangle*3,a=triangles[t],b=triangles[t+1],c=triangles[t+2];
                Vec3 ab=simulation[b]-simulation[a],ac=simulation[c]-simulation[a],aq=sample.Point-simulation[a];
                double aa=Vec3.Dot(ab,ab),bb=Vec3.Dot(ab,ac),cc=Vec3.Dot(ac,ac),denominator=aa*cc-bb*bb;
                if(Math.Abs(denominator)<1e-30) throw new ArgumentException("Degenerate binding triangle.");
                double v=(cc*Vec3.Dot(aq,ab)-bb*Vec3.Dot(aq,ac))/denominator,w=(aa*Vec3.Dot(aq,ac)-bb*Vec3.Dot(aq,ab))/denominator;
                result.Add(new VisualBinding { Vertex=i,A=a,B=b,C=c,U=1-v-w,V=v,W=w,Influence=influence[i] });
            }
            if(result.Count==0) throw new ArgumentException("Visual influence mask contains no active vertices.");
            return result.ToArray();
        }
    }
}
