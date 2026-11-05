using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    public struct EdgeConstraint { public int A,B; public EdgeConstraint(int a,int b) { A=a; B=b; } }
    public sealed class SimulationTopology
    {
        public readonly int VertexCount;
        public readonly int[] Triangles;
        public readonly EdgeConstraint[] Stretch,Bend;
        public readonly int BoundaryEdges;
        public readonly bool Closed;
        public readonly double ReferenceSize;
        public SimulationTopology(Vec3[] rest,int[] indices,bool allowJunctions=false)
        {
            if(rest==null||rest.Length<3||indices==null||indices.Length<3||indices.Length%3!=0) throw new ArgumentException("Invalid simulation topology.");
            VertexCount=rest.Length; Triangles=(int[])indices.Clone(); var bounds=Bounds3.Empty;
            foreach(var p in rest) { if(!p.IsFinite) throw new ArgumentException("Non-finite rest vertex."); bounds.Include(p); }
            ReferenceSize=(bounds.Max-bounds.Min).Length; if(ReferenceSize<1e-8) throw new ArgumentException("Negligible simulation size.");
            var edges=new Dictionary<long,int>(); var count=new Dictionary<long,int>(); var directions=new Dictionary<long,int>();
            var stretch=new List<EdgeConstraint>(); var bend=new List<EdgeConstraint>();
            for(int i=0;i<Triangles.Length;i+=3)
            {
                int a=Triangles[i],b=Triangles[i+1],c=Triangles[i+2];
                if(a<0||b<0||c<0||a>=rest.Length||b>=rest.Length||c>=rest.Length||a==b||b==c||a==c) throw new ArgumentException("Invalid simulation triangle.");
                if(Vec3.Cross(rest[b]-rest[a],rest[c]-rest[a]).LengthSquared<Math.Pow(ReferenceSize,4)*1e-20) throw new ArgumentException("Degenerate simulation triangle.");
                for(int j=0;j<3;j++)
                {
                    int u=Triangles[i+j],v=Triangles[i+(j+1)%3],opposite=Triangles[i+(j+2)%3]; long key=((long)Math.Min(u,v)<<32)|(uint)Math.Max(u,v);
                    int n; count.TryGetValue(key,out n); count[key]=n+1;
                    int sign; directions.TryGetValue(key,out sign); directions[key]=sign+(u<v?1:-1);
                    if(n==0) { edges[key]=opposite; stretch.Add(new EdgeConstraint(u,v)); }
                    else if(n==1) bend.Add(new EdgeConstraint(edges[key],opposite));
                    else if(!allowJunctions) throw new ArgumentException("Nonmanifold simulation edge.");
                }
            }
            bool oriented=true; foreach(var item in count) { if(item.Value==1) BoundaryEdges++; else if(item.Value==2&&directions[item.Key]!=0) oriented=false; }
            if(!oriented&&!allowJunctions) throw new ArgumentException("Inconsistent triangle winding.");
            Closed=BoundaryEdges==0&&oriented;foreach(var e in count)if(e.Value>2)Closed=false; Stretch=stretch.ToArray(); Bend=bend.ToArray();
        }
        public double SignedVolume(Vec3[] points)
        {
            Vec3 origin=points[0]; double volume=0;
            for(int i=0;i<Triangles.Length;i+=3) volume+=Vec3.Dot(points[Triangles[i]]-origin,Vec3.Cross(points[Triangles[i+1]]-origin,points[Triangles[i+2]]-origin))/6;
            return volume;
        }
    }
    public enum PhysicsQuality { Performance,Balanced,Quality,Custom }
    public struct PhysicsPolicy
    {
        public int Iterations,MaximumSubsteps,NormalsInterval;
        public double FixedStep;
        public static PhysicsPolicy Resolve(PhysicsQuality quality,int iterations=6,int maximumSubsteps=3,double hz=90,int normals=3)
        {
            switch(quality)
            {
                case PhysicsQuality.Performance: return new PhysicsPolicy { Iterations=3,MaximumSubsteps=2,FixedStep=1.0/60,NormalsInterval=6 };
                case PhysicsQuality.Balanced: return new PhysicsPolicy { Iterations=6,MaximumSubsteps=3,FixedStep=1.0/90,NormalsInterval=3 };
                case PhysicsQuality.Quality: return new PhysicsPolicy { Iterations=10,MaximumSubsteps=4,FixedStep=1.0/120,NormalsInterval=1 };
                default:
                    if(iterations<1||iterations>32||maximumSubsteps<1||maximumSubsteps>8||!Vec3.Finite(hz)||hz<30||hz>240||normals<1) throw new ArgumentException("Invalid custom quality.");
                    return new PhysicsPolicy { Iterations=iterations,MaximumSubsteps=maximumSubsteps,FixedStep=1/hz,NormalsInterval=normals };
            }
        }
    }
}
