using System;
using System.Collections.Generic;
namespace BetterPenetrationNext.Core
{
    // Finite cuts of the evaluated open shell. The immutable triangle partition
    // is built once; one refit serves every section. No caps or skin welds.
    public sealed class SourceSectionContacts
    {
        struct Node { public Bounds3 Bounds;public int Start,Count,Left,Right; }
        readonly int[] faces,order;readonly Node[] nodes;int used;Vec3[] vertices;
        public int BoundsTests {get;private set;}public int TriangleTests {get;private set;}
        public SourceSectionContacts(Vec3[] reference,int[] triangles)
        {
            if(reference==null||triangles==null||triangles.Length==0||triangles.Length%3!=0)throw new ArgumentException("Source triangle topology required");
            vertices=reference;faces=(int[])triangles.Clone();order=new int[faces.Length/3];nodes=new Node[order.Length*2];
            for(int i=0;i<faces.Length;i++)if(faces[i]<0||faces[i]>=reference.Length)throw new ArgumentException("Invalid source index");
            for(int i=0;i<order.Length;i++)order[i]=i;Build(0,order.Length);
        }
        Vec3 Centre(int f){int i=f*3;return (vertices[faces[i]]+vertices[faces[i+1]]+vertices[faces[i+2]])/3;}
        Bounds3 Bounds(int start,int count)
        {var b=Bounds3.Empty;for(int j=start;j<start+count;j++){int f=order[j]*3;for(int k=0;k<3;k++)b.Include(vertices[faces[f+k]]);}return b;}
        int Build(int start,int count)
        {
            int id=used++;var n=new Node{Start=start,Count=count,Bounds=Bounds(start,count)};
            if(count>8){Vec3 extent=n.Bounds.Max-n.Bounds.Min;int axis=extent.X>=extent.Y&&extent.X>=extent.Z?0:extent.Y>=extent.Z?1:2;
                Array.Sort(order,start,count,Comparer<int>.Create((a,b)=>{int c=Centre(a).Axis(axis).CompareTo(Centre(b).Axis(axis));return c!=0?c:a.CompareTo(b);}));
                int half=count/2;n.Left=Build(start,half);n.Right=Build(start+half,count-half);n.Count=0;}
            nodes[id]=n;return id;
        }
        public void Refit(Vec3[] posed)
        {
            if(posed==null||posed.Length!=vertices.Length)throw new ArgumentException("Source topology changed");vertices=posed;BoundsTests=TriangleTests=0;
            for(int i=used-1;i>=0;i--){var n=nodes[i];if(n.Count>0)n.Bounds=Bounds(n.Start,n.Count);else{n.Bounds=nodes[n.Left].Bounds;n.Bounds.Include(nodes[n.Right].Bounds.Min);n.Bounds.Include(nodes[n.Right].Bounds.Max);}nodes[i]=n;}
        }
        public void Include(ReceiverRadialField field,int section,Vec3 centre,Vec3 normal,Vec3 x,Vec3 y,double clearance)
        {Search(0,field,section,centre,normal,x,y,clearance);}
        void Search(int id,ReceiverRadialField field,int section,Vec3 c,Vec3 n,Vec3 x,Vec3 y,double clearance)
        {
            BoundsTests++;var node=nodes[id];Vec3 midpoint=(node.Bounds.Min+node.Bounds.Max)*.5,extent=(node.Bounds.Max-node.Bounds.Min)*.5;
            double distance=Vec3.Dot(midpoint-c,n),radius=Math.Abs(n.X)*extent.X+Math.Abs(n.Y)*extent.Y+Math.Abs(n.Z)*extent.Z;
            if(Math.Abs(distance)>radius+1e-12)return;
            if(node.Count==0){Search(node.Left,field,section,c,n,x,y,clearance);Search(node.Right,field,section,c,n,x,y,clearance);return;}
            for(int j=node.Start;j<node.Start+node.Count;j++)
            {
                TriangleTests++;int f=order[j]*3;Vec3 a=vertices[faces[f]],b=vertices[faces[f+1]],d=vertices[faces[f+2]];
                double da=Vec3.Dot(a-c,n),db=Vec3.Dot(b-c,n),dd=Vec3.Dot(d-c,n);
                if((da>0&&db>0&&dd>0)||(da<0&&db<0&&dd<0))continue;
                Edge(field,section,a,b,da,db,c,x,y,clearance);Edge(field,section,b,d,db,dd,c,x,y,clearance);Edge(field,section,d,a,dd,da,c,x,y,clearance);
            }
        }
        static void Edge(ReceiverRadialField field,int section,Vec3 a,Vec3 b,double da,double db,Vec3 c,Vec3 x,Vec3 y,double clearance)
        {
            if(Math.Abs(da)<1e-12)field.Include(section,a-c,x,y,clearance);
            if((da<0&&db>0)||(db<0&&da>0))field.Include(section,a+(b-a)*(da/(da-db))-c,x,y,clearance);
        }
    }
}
