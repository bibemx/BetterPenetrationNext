using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    // Hints seed the search radius; complete BVH refinement still proves the
    // exact nearest points. Partitions and query workspace are allocated once.
    public sealed class PointMotionBvh
    {
        struct Node { public Bounds3 Bounds; public int Start,Count,Left,Right; }
        readonly int[] order,nearest,visited;
        readonly double[] distance;
        readonly Node[] nodes;
        Vec3[] points;
        int used,count,stamp;
        bool seeded;
        public long Visits,PointTests,Queries,HintPointTests;
        public int NeighbourCount { get { return nearest.Length; } }

        public PointMotionBvh(Vec3[] reference,int neighbours=16)
        {
            if(reference==null||reference.Length==0||neighbours<1) throw new ArgumentException("A nonempty point set and positive neighbour count are required");
            points=reference; order=new int[reference.Length]; visited=new int[reference.Length];
            for(int i=0;i<order.Length;i++) order[i]=i;
            nodes=new Node[reference.Length*2]; nearest=new int[Math.Min(reference.Length,neighbours)]; distance=new double[nearest.Length];
            Build(0,order.Length);
        }
        int Build(int start,int length)
        {
            int id=used++; var b=Bounds3.Empty;
            for(int i=start;i<start+length;i++) b.Include(points[order[i]]);
            var n=new Node{Bounds=b,Start=start,Count=length};
            if(length>8)
            {
                Vec3 size=b.Max-b.Min; int axis=size.X>=size.Y&&size.X>=size.Z?0:size.Y>=size.Z?1:2;
                Array.Sort(order,start,length,Comparer<int>.Create((a,c)=>{int q=points[a].Axis(axis).CompareTo(points[c].Axis(axis));return q==0?a.CompareTo(c):q;}));
                int half=length/2; n.Left=Build(start,half); n.Right=Build(start+half,length-half); n.Count=0;
            }
            nodes[id]=n; return id;
        }
        public void Refit(Vec3[] updated)
        {
            if(updated==null||updated.Length!=order.Length) throw new ArgumentException("Point topology changed");
            points=updated;
            for(int i=used-1;i>=0;i--)
            {
                var n=nodes[i]; var b=Bounds3.Empty;
                if(n.Count>0) for(int j=n.Start;j<n.Start+n.Count;j++) b.Include(points[order[j]]);
                else { b.Include(nodes[n.Left].Bounds); b.Include(nodes[n.Right].Bounds); }
                n.Bounds=b; nodes[i]=n;
            }
        }
        public void ResetStatistics() { Visits=PointTests=Queries=HintPointTests=0; }
        void Candidate(int vertex,Vec3 point,bool hint)
        {
            if(seeded)
            {
                if(visited[vertex]==stamp) return;
                visited[vertex]=stamp;
            }
            double d=(points[vertex]-point).LengthSquared; PointTests++; if(hint) HintPointTests++;
            if(count==nearest.Length&&(d>distance[count-1]||d==distance[count-1]&&vertex>=nearest[count-1])) return;
            int k=Math.Min(count,nearest.Length-1);
            while(k>0&&(d<distance[k-1]||d==distance[k-1]&&vertex<nearest[k-1]))
            { distance[k]=distance[k-1]; nearest[k]=nearest[k-1]; k--; }
            distance[k]=d; nearest[k]=vertex; if(count<nearest.Length) count++;
        }
        void Search(int index,Vec3 point)
        {
            Visits++; var n=nodes[index];
            if(count==nearest.Length&&n.Bounds.DistanceSquared(point)>distance[count-1]) return;
            if(n.Count==0)
            {
                int first=n.Left,second=n.Right;
                if(nodes[second].Bounds.DistanceSquared(point)<nodes[first].Bounds.DistanceSquared(point)) { first=n.Right;second=n.Left; }
                Search(first,point); Search(second,point); return;
            }
            for(int j=n.Start;j<n.Start+n.Count;j++) Candidate(order[j],point,false);
        }
        void Query(Vec3 point,int[] hints,int offset)
        {
            Queries++; count=0; seeded=hints!=null;
            if(seeded)
            {
                if(offset<0||offset>hints.Length-nearest.Length) throw new ArgumentException("Hint buffer is too short");
                unchecked { stamp++; }
                if(stamp==0) { Array.Clear(visited,0,visited.Length); stamp=1; }
                for(int i=0;i<nearest.Length;i++) { int index=hints[offset+i]; if(index>=0&&index<points.Length) Candidate(index,point,true); }
            }
            Search(0,point);
            if(seeded) for(int i=0;i<nearest.Length;i++) hints[offset+i]=i<count?nearest[i]:-1;
        }
        public Vec3 Velocity(Vec3 point,Vec3[] velocity) { return Velocity(point,velocity,null,0); }
        public Vec3 Velocity(Vec3 point,Vec3[] velocity,int[] hints,int offset)
        {
            if(velocity==null||velocity.Length!=points.Length) throw new ArgumentException("Velocity topology changed");
            Query(point,hints,offset);
            if(distance[0]<1e-22) return velocity[nearest[0]];
            double radius=Math.Max(distance[count-1]*1.000001,1e-20),total=0; Vec3 sum=new Vec3();
            for(int i=0;i<count;i++)
            {
                double t=Math.Max(0,1-Math.Sqrt(distance[i]/radius)),w=t*t/Math.Max(distance[i],1e-20);
                sum+=velocity[nearest[i]]*w; total+=w;
            }
            return total>1e-30?sum/total:velocity[nearest[0]];
        }
        public double NearestDistanceSquared(Vec3 point) { Query(point,null,0); return distance[0]; }
    }
}
