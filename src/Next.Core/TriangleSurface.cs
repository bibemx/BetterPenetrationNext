using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    public enum SurfaceKind { ClosedVolume, OrientedShell }
    public struct SurfaceSample
    {
        public Vec3 Point, Normal;
        public double SignedDistance;
        public int Triangle;
    }

    // A persistent median-split BVH. Refit after each animated pose, no per-query allocations.
    public sealed partial class TriangleSurface : IHintedContactSurface,IContactBounds
    {
        struct Node { public Bounds3 Bounds; public int Start, Count, Left, Right; }
        readonly int[] triangles, order;
        readonly Node[] nodes;
        readonly Vec3[] vertices;
        readonly TriangleQuery[] queries;
        int used;
        readonly SurfaceKind kind;
        bool robustSign=true;
        Vec3[] vertexPseudo,edgePseudo;
        int[] triangleEdges;
        double orientation=1;
        public bool RobustSign
        {
            get { return robustSign; }
            set { if(robustSign==value) return; robustSign=value; if(!robustSign&&kind==SurfaceKind.ClosedVolume) { InitializeEdges(); UpdatePseudoNormals(); } }
        }
        public Bounds3 Bounds { get { return nodes[0].Bounds; } }
        public bool HasVolumeBounds { get { return kind==SurfaceKind.ClosedVolume; } }
        public Bounds3 ContactBounds { get { return Bounds; } }
        public int TriangleCount { get { return order.Length; } }
        public long NearestQueries { get; private set; }
        public long BvhVisits { get; private set; }
        public long TriangleTests { get; private set; }

        public TriangleSurface(Vec3[] positions,int[] indices,SurfaceKind surfaceKind)
        {
            if(positions==null||indices==null||positions.Length<3||indices.Length<3||indices.Length%3!=0)
                throw new ArgumentException("A non-empty triangle mesh is required.");
            vertices=(Vec3[])positions.Clone(); triangles=(int[])indices.Clone(); kind=surfaceKind;
            foreach(int index in triangles) if(index<0||index>=vertices.Length) throw new ArgumentException("Triangle index out of range.");
            ValidatePositions(positions);
            if(kind==SurfaceKind.ClosedVolume) ValidateClosed();
            order=new int[triangles.Length/3]; for(int i=0;i<order.Length;i++) order[i]=i;
            queries=new TriangleQuery[order.Length]; UpdateQueries();
            nodes=new Node[order.Length*2]; Build(0,order.Length);
        }

        void ValidateClosed()
        {
            var edges=new Dictionary<long,int>(); var directions=new Dictionary<long,int>();
            for(int i=0;i<triangles.Length;i+=3)
                for(int j=0;j<3;j++)
                {
                    int a=triangles[i+j],b=triangles[i+(j+1)%3];
                    if(a==b) throw new ArgumentException("Degenerate edge in closed volume.");
                    long key=((long)Math.Min(a,b)<<32)|(uint)Math.Max(a,b);
                    int n; edges.TryGetValue(key,out n); edges[key]=n+1;
                    directions.TryGetValue(key,out n); directions[key]=n+(a<b?1:-1);
                }
            foreach(var edge in edges)
                if(edge.Value!=2||directions[edge.Key]!=0)
                    throw new ArgumentException("ClosedVolume requires welded, consistently wound, manifold topology. Use a closed proxy or OrientedShell.");
        }

        static void ValidatePositions(Vec3[] p)
        { foreach(Vec3 v in p) if(!v.IsFinite) throw new ArgumentException("Non-finite mesh vertex."); }
        Vec3 Center(int triangle) { int t=triangle*3; return (vertices[triangles[t]]+vertices[triangles[t+1]]+vertices[triangles[t+2]])/3; }
        Bounds3 TriangleBounds(int triangle)
        { int t=triangle*3; var b=Bounds3.Empty; b.Include(vertices[triangles[t]]); b.Include(vertices[triangles[t+1]]); b.Include(vertices[triangles[t+2]]); return b; }
        int Build(int start,int count)
        {
            int index=used++; var bounds=Bounds3.Empty; var centers=Bounds3.Empty;
            for(int i=start;i<start+count;i++) { bounds.Include(TriangleBounds(order[i])); centers.Include(Center(order[i])); }
            Node n=new Node { Bounds=bounds,Start=start,Count=count,Left=-1,Right=-1 };
            if(count>8)
            {
                Vec3 size=centers.Max-centers.Min;
                int axis=size.X>=size.Y&&size.X>=size.Z ? 0 : size.Y>=size.Z ? 1:2;
                Array.Sort(order,start,count,Comparer<int>.Create((a,b)=>Center(a).Axis(axis).CompareTo(Center(b).Axis(axis))));
                int half=count/2; n.Left=Build(start,half); n.Right=Build(start+half,count-half); n.Count=0;
            }
            nodes[index]=n; return index;
        }
        public void Refit(Vec3[] positions)
        {
            if(positions.Length!=vertices.Length) throw new ArgumentException("Topology changed; rebuild the surface.");
            ValidatePositions(positions); Array.Copy(positions,vertices,vertices.Length);
            UpdateQueries();
            for(int i=used-1;i>=0;i--)
            {
                Node n=nodes[i]; var b=Bounds3.Empty;
                if(n.Count>0) for(int j=n.Start;j<n.Start+n.Count;j++) b.Include(TriangleBounds(order[j]));
                else { b.Include(nodes[n.Left].Bounds); b.Include(nodes[n.Right].Bounds); }
                n.Bounds=b; nodes[i]=n;
            }
            if(!robustSign&&kind==SurfaceKind.ClosedVolume) UpdatePseudoNormals();
        }
        void UpdateQueries()
        { for(int i=0;i<queries.Length;i++) { int t=i*3; queries[i]=new TriangleQuery(vertices[triangles[t]],vertices[triangles[t+1]],vertices[triangles[t+2]]); } }
        void InitializeEdges()
        {
            if(triangleEdges!=null) return;
            var edges=new Dictionary<long,int>(); triangleEdges=new int[triangles.Length]; vertexPseudo=new Vec3[vertices.Length];
            for(int t=0;t<triangles.Length;t+=3) for(int j=0;j<3;j++)
            {
                int a=triangles[t+j],b=triangles[t+(j+1)%3]; long key=((long)Math.Min(a,b)<<32)|(uint)Math.Max(a,b); int edge;
                if(!edges.TryGetValue(key,out edge)) { edge=edges.Count; edges.Add(key,edge); } triangleEdges[t+j]=edge;
            }
            edgePseudo=new Vec3[edges.Count];
        }
        void UpdatePseudoNormals()
        {
            Array.Clear(vertexPseudo,0,vertexPseudo.Length); Array.Clear(edgePseudo,0,edgePseudo.Length);
            double volume=0; Vec3 origin=(Bounds.Min+Bounds.Max)/2;
            for(int t=0;t<triangles.Length;t+=3)
            {
                int ia=triangles[t],ib=triangles[t+1],ic=triangles[t+2]; Vec3 a=vertices[ia],b=vertices[ib],c=vertices[ic];
                Vec3 cross=Vec3.Cross(b-a,c-a); double area=cross.Length; if(area<1e-24) continue;
                Vec3 normal=cross/area;
                double angleA=Math.Atan2(area,Vec3.Dot(b-a,c-a)),angleB=Math.Atan2(area,Vec3.Dot(a-b,c-b));
                vertexPseudo[ia]+=normal*angleA; vertexPseudo[ib]+=normal*angleB; vertexPseudo[ic]+=normal*Math.Max(0,Math.PI-angleA-angleB);
                edgePseudo[triangleEdges[t]]+=normal; edgePseudo[triangleEdges[t+1]]+=normal; edgePseudo[triangleEdges[t+2]]+=normal;
                volume+=Vec3.Dot(a-origin,Vec3.Cross(b-origin,c-origin));
            }
            orientation=volume<0?-1:1;
        }
        Vec3 PseudoNormal(int t,Vec3 q,Vec3 face)
        {
            Vec3 a=vertices[triangles[t]],b=vertices[triangles[t+1]],c=vertices[triangles[t+2]];
            double eps=Math.Max((b-a).LengthSquared,(c-a).LengthSquared)*1e-12;
            if((q-a).LengthSquared<=eps) return vertexPseudo[triangles[t]]*orientation;
            if((q-b).LengthSquared<=eps) return vertexPseudo[triangles[t+1]]*orientation;
            if((q-c).LengthSquared<=eps) return vertexPseudo[triangles[t+2]]*orientation;
            Vec3 ab=b-a,ac=c-a,aq=q-a; double aa=Vec3.Dot(ab,ab),bb=Vec3.Dot(ab,ac),cc=Vec3.Dot(ac,ac),d=aa*cc-bb*bb;
            if(Math.Abs(d)>1e-30)
            {
                double v=(cc*Vec3.Dot(aq,ab)-bb*Vec3.Dot(aq,ac))/d,w=(aa*Vec3.Dot(aq,ac)-bb*Vec3.Dot(aq,ab))/d;
                if(w<=1e-6) return edgePseudo[triangleEdges[t]]*orientation;
                if(v<=1e-6) return edgePseudo[triangleEdges[t+2]]*orientation;
                if(1-v-w<=1e-6) return edgePseudo[triangleEdges[t+1]]*orientation;
            }
            return face*orientation;
        }
        void Nearest(int index,Vec3 p,ref double best,ref int triangle,ref Vec3 point)
        {
            BvhVisits++;
            Node n=nodes[index]; if(n.Bounds.DistanceSquared(p)>best) return;
            if(n.Count>0)
            {
                for(int j=n.Start;j<n.Start+n.Count;j++)
                {
                    TriangleTests++;
                    ref TriangleQuery query=ref queries[order[j]];
                    if(query.Degenerate)continue;
                    double plane=Vec3.Dot(p-query.A,query.Normal); if(!query.Degenerate&&plane*plane>best) continue;
                    Vec3 q=query.Closest(p);
                    double d=(q-p).LengthSquared;
                    if(d<best||(d==best&&(triangle<0||order[j]<triangle))) { best=d; triangle=order[j]; point=q; }
                }
            }
            else
            {
                int first=n.Left,second=n.Right;
                if(nodes[second].Bounds.DistanceSquared(p)<nodes[first].Bounds.DistanceSquared(p)) { first=n.Right; second=n.Left; }
                Nearest(first,p,ref best,ref triangle,ref point); Nearest(second,p,ref best,ref triangle,ref point);
            }
        }
        int RayCount(int index,Vec3 p,Vec3 direction)
        {
            Node n=nodes[index]; if(!n.Bounds.IntersectsRay(p,direction)) return 0;
            if(n.Count==0) return RayCount(n.Left,p,direction)+RayCount(n.Right,p,direction);
            int count=0;
            for(int j=n.Start;j<n.Start+n.Count;j++)
            {
                int t=order[j]*3; double distance;
                if(Geometry.RayTriangle(p,direction,vertices[triangles[t]],vertices[triangles[t+1]],vertices[triangles[t+2]],out distance)) count++;
            }
            return count;
        }
        public bool TrySample(Vec3 p,double searchDistance,out SurfaceSample sample)
        { int hint=-1; return TrySampleHinted(p,searchDistance,ref hint,out sample); }
        void RayNearest(int index,Vec3 origin,Vec3 direction,ref double distance,ref int triangle)
        {
            BvhVisits++;var node=nodes[index];
            if(node.Bounds.DistanceSquared(origin)>distance*distance||!node.Bounds.IntersectsRay(origin,direction))return;
            if(node.Count==0)
            {
                int first=node.Left,second=node.Right;
                if(nodes[second].Bounds.DistanceSquared(origin)<nodes[first].Bounds.DistanceSquared(origin)){first=node.Right;second=node.Left;}
                RayNearest(first,origin,direction,ref distance,ref triangle);RayNearest(second,origin,direction,ref distance,ref triangle);return;
            }
            for(int i=node.Start;i<node.Start+node.Count;i++)
            {
                int face=order[i],t=face*3;double hit;TriangleTests++;
                if(Geometry.RayTriangle(origin,direction,vertices[triangles[t]],vertices[triangles[t+1]],vertices[triangles[t+2]],out hit)&&hit<=distance){distance=hit;triangle=face;}
            }
        }
        public bool TryRaycast(Vec3 origin,Vec3 end,out SurfaceSample sample)
        {
            sample=new SurfaceSample();if(!origin.IsFinite||!end.IsFinite)return false;
            Vec3 edge=end-origin;double distance=edge.Length;if(distance<1e-12)return false;
            Vec3 direction=edge/distance;int triangle=-1;RayNearest(0,origin,direction,ref distance,ref triangle);
            if(triangle<0)return false;
            sample=new SurfaceSample{Point=origin+direction*distance,Normal=queries[triangle].Normal,Triangle=triangle};return true;
        }
        public bool TrySampleHinted(Vec3 p,double searchDistance,ref int hint,out SurfaceSample sample)
        {
            NearestQueries++;
            sample=new SurfaceSample();
            if(!p.IsFinite||!Vec3.Finite(searchDistance)||searchDistance<=0) return false;
            double best=searchDistance*searchDistance; int triangle=-1; Vec3 q=new Vec3();
            if(hint>=0&&hint<queries.Length&&!queries[hint].Degenerate) { TriangleTests++;var point=queries[hint].Closest(p); double hintDistance=(point-p).LengthSquared; if(hintDistance<best) {best=hintDistance;q=point;triangle=hint;} }
            Nearest(0,p,ref best,ref triangle,ref q); if(triangle<0) return false;
            hint=triangle;
            int t=triangle*3;
            Vec3 normal=Vec3.Cross(vertices[triangles[t+1]]-vertices[triangles[t]],vertices[triangles[t+2]]-vertices[triangles[t]]).Normalized;
            if(normal.LengthSquared<0.5) return false;
            double distance=Math.Sqrt(best), sign=Vec3.Dot(p-q,normal)>=0 ? 1:-1;
            if(kind==SurfaceKind.ClosedVolume && distance>1e-12 && Bounds.DistanceSquared(p)==0)
            {
                // Three oblique rays reduce shared-edge / vertex parity ambiguity.
                if(!robustSign) sign=Vec3.Dot(p-q,PseudoNormal(t,q,normal))<0?-1:1;
                else
                {
                int votes=(RayCount(0,p,new Vec3(1,0.3713907,0.6947466))&1)
                    +(RayCount(0,p,new Vec3(-0.529171,1,0.213719))&1)
                    +(RayCount(0,p,new Vec3(0.173913,-0.618033,1))&1);
                sign=votes>=2 ? -1:1;
                }
                normal=sign<0 ? (q-p)/distance : (p-q)/distance;
            }
            else if(kind==SurfaceKind.ClosedVolume && distance>1e-12)
            { sign=1; normal=(p-q)/distance; }
            sample=new SurfaceSample { Point=q,Normal=normal,SignedDistance=distance*sign,Triangle=triangle }; return true;
        }
    }
}
