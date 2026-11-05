using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    // A contact-only representation. Original vertices and skinning remain
    // distinct; only matched rest-space cracks receive virtual joining faces.
    // A genuine open rim without a uniquely matched partner is never capped.
    public sealed class AnimatedSeamContactProxy
    {
        struct Edge { public int A,B,Count; }
        readonly int originalCount;
        readonly EdgeConstraint[] gaps;
        readonly int[][] centers;
        readonly double referenceSize;
        public readonly Vec3[] Positions;
        public readonly int[] Triangles;
        public int AddedTriangles { get; private set; }
        public double MaximumGap { get; private set; }
        static long Key(int a,int b) { return ((long)Math.Min(a,b)<<32)|(uint)Math.Max(a,b); }
        static Dictionary<long,Edge> Edges(IList<int> indices)
        {
            var edges=new Dictionary<long,Edge>();
            for(int t=0;t<indices.Count;t+=3) for(int k=0;k<3;k++) {
                int a=indices[t+k],b=indices[t+(k+1)%3]; long key=Key(a,b); Edge edge;
                if(!edges.TryGetValue(key,out edge)) edge=new Edge { A=a,B=b };
                edge.Count++; if(edge.Count>2) throw new ArgumentException("Nonmanifold contact input"); edges[key]=edge;
            }
            return edges;
        }
        static void Add(List<int> indices,int a,int b,int c)
        { if(a!=b&&b!=c&&a!=c) { indices.Add(a); indices.Add(b); indices.Add(c); } }
        public AnimatedSeamContactProxy(Vec3[] rest,int[] indices)
        {
            if(rest==null||indices==null) throw new ArgumentNullException();
            originalCount=rest.Length; var bounds=Bounds3.Empty; foreach(var p in rest) bounds.Include(p); referenceSize=(bounds.Max-bounds.Min).Length;
            var geometric=new WeldedContactMesh(rest,indices,1e-6,true);
            var paired=new Dictionary<long,List<Edge>>(); var all=Edges(indices); int boundaryCount=0;
            foreach(var item in all) if(item.Value.Count==1) {
                var e=item.Value; long key=Key(geometric.MappedVertex(e.A),geometric.MappedVertex(e.B)); List<Edge> bucket;
                if(!paired.TryGetValue(key,out bucket)) { bucket=new List<Edge>(); paired.Add(key,bucket); } bucket.Add(e); boundaryCount++;
            }
            if(boundaryCount==0) throw new ArgumentException("Already closed; proxy unnecessary");
            var result=new List<int>(indices); var gapSet=new HashSet<long>(); var gapList=new List<EdgeConstraint>();
            foreach(var item in paired) {
                var list=item.Value; if(list.Count!=2) throw new ArgumentException("Unmatched or ambiguous open rim; retain shell semantics");
                Edge a=list[0],b=list[1];
                if(geometric.MappedVertex(a.A)!=geometric.MappedVertex(b.B)||geometric.MappedVertex(a.B)!=geometric.MappedVertex(b.A)) throw new ArgumentException("Matched edges have incompatible orientation");
                Add(result,a.B,a.A,b.B); Add(result,a.B,b.B,b.A);
                if(a.A!=b.B&&gapSet.Add(Key(a.A,b.B))) gapList.Add(new EdgeConstraint(a.A,b.B));
                if(a.B!=b.A&&gapSet.Add(Key(a.B,b.A))) gapList.Add(new EdgeConstraint(a.B,b.A));
            }
            // Junction cracks may leave a small vertex fan. Only close a loop if
            // all its vertices were the SAME rest-space point before skinning.
            var remaining=Edges(result); var next=new Dictionary<int,int>();
            foreach(var item in remaining) if(item.Value.Count==1) { var e=item.Value; if(next.ContainsKey(e.A)) throw new ArgumentException("Ambiguous seam junction"); next.Add(e.A,e.B); }
            var caps=new List<int[]>();
            while(next.Count>0) {
                int start=-1; foreach(var item in next) { start=item.Key; break; }
                var loop=new List<int>(); int current=start;
                do { int end; if(!next.TryGetValue(current,out end)||loop.Count>boundaryCount*2) throw new ArgumentException("Noncyclic seam junction"); loop.Add(current); next.Remove(current); current=end; } while(current!=start);
                if(loop.Count<3) throw new ArgumentException("Invalid seam fan"); int group=geometric.MappedVertex(loop[0]);
                foreach(int i in loop) if(geometric.MappedVertex(i)!=group) throw new ArgumentException("Refusing to cap a real opening");
                int center=originalCount+caps.Count; caps.Add(loop.ToArray());
                for(int i=0;i<loop.Count;i++) Add(result,loop[(i+1)%loop.Count],loop[i],center);
            }
            gaps=gapList.ToArray(); centers=caps.ToArray(); Positions=new Vec3[originalCount+centers.Length]; Triangles=result.ToArray(); AddedTriangles=(Triangles.Length-indices.Length)/3;
            Update(rest,1); // Topological validation, including winding, uses the
            // actual triangle-surface validator; zero-area rest crack faces are valid.
            new TriangleSurface(Positions,Triangles,SurfaceKind.ClosedVolume);
        }
        public bool Update(Vec3[] animated,double maximumGapRatio=.05)
        {
            if(animated==null||animated.Length!=originalCount||!Vec3.Finite(maximumGapRatio)||maximumGapRatio<=0) throw new ArgumentException("Invalid proxy pose");
            Array.Copy(animated,Positions,originalCount); MaximumGap=0;
            foreach(var edge in gaps) MaximumGap=Math.Max(MaximumGap,(animated[edge.A]-animated[edge.B]).Length);
            for(int i=0;i<centers.Length;i++) { Vec3 p=new Vec3(); foreach(int v in centers[i]) p+=animated[v]; Positions[originalCount+i]=p/centers[i].Length; }
            return MaximumGap<=referenceSize*maximumGapRatio;
        }
    }
}
