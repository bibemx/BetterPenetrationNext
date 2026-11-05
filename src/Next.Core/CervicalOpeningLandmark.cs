using System;
using System.Collections.Generic;
namespace BetterPenetrationNext.Core
{
    // Rn is bound to a verified ring of the actual cervical surface. A smallest
    // boundary alone can be the INTERNAL os at the far end of a cervical tube.
    // With an anatomical approach, prefer its canal-facing narrow throat.
    // All topology work is done once. Animation evaluates only cached vertices.
    public sealed class CervicalOpeningLandmark
    {
        readonly int[] loop;
        double normalOrientation;
        public readonly bool IsCanalFacingThroat;
        public int SurfacePointCount {get{return loop==null?0:loop.Length;}}
        public bool Available {get{return loop!=null;}}
        public Vec3 Centre,Normal;
        public double Radius;
        public readonly Vec3[] Points=new Vec3[5];
        public readonly int[] PointIndices=new int[]{-1,-1,-1,-1,-1};
        public CervicalOpeningLandmark(Vec3[] rest,int[] triangles,int[] regions,Vec3 approach=new Vec3())
        {
            if(regions==null||regions.Length!=triangles.Length/3)return;
            var edges=new Dictionary<long,int>();
            for(int f=0;f<regions.Length;f++)if(regions[f]==3)for(int j=0;j<3;j++)
            {int a=triangles[f*3+j],b=triangles[f*3+(j+1)%3];long key=((long)Math.Min(a,b)<<32)|(uint)Math.Max(a,b);int count;edges.TryGetValue(key,out count);edges[key]=count+1;}
            var adjacent=new Dictionary<int,List<int>>();
            foreach(var e in edges)if(e.Value==1)
            {int a=(int)(e.Key>>32),b=(int)(e.Key&uint.MaxValue);List<int> row;if(!adjacent.TryGetValue(a,out row)){row=new List<int>(2);adjacent.Add(a,row);}row.Add(b);if(!adjacent.TryGetValue(b,out row)){row=new List<int>(2);adjacent.Add(b,row);}row.Add(a);}
            var visited=new HashSet<int>();int[] selected=null;double smallest=double.PositiveInfinity,largest=0;
            foreach(int seed in adjacent.Keys)
            {
                if(visited.Contains(seed)||adjacent[seed].Count!=2)continue;
                var points=new List<int>();int previous=-1,current=seed;bool closed=false;
                while(!visited.Contains(current))
                {visited.Add(current);points.Add(current);var row=adjacent[current];if(row.Count!=2)break;int next=row[0]==previous?row[1]:row[0];previous=current;current=next;if(current==seed){closed=true;break;}}
                if(!closed||points.Count<3)continue;
                Vec3 c=new Vec3();foreach(int i in points)c+=rest[i]/points.Count;double radius=0;foreach(int i in points)radius+=(rest[i]-c).Length/points.Count;
                largest=Math.Max(largest,radius);if(radius<smallest){smallest=radius;selected=points.ToArray();}
            }
            // A single cylinder rim or an unverified open patch is not an os.
            if(selected!=null&&smallest>1e-8&&smallest<largest*.65)
            {
                loop=selected;
                if(approach.LengthSquared>.5)
                {
                    Vec3 centre=new Vec3();foreach(int i in selected)centre+=rest[i]/selected.Length;Vec3 normal=new Vec3();for(int i=0;i<selected.Length;i++)normal+=Vec3.Cross(rest[selected[i]]-centre,rest[selected[(i+1)%selected.Length]]-centre);
                    normal=normal.Normalized;if(Vec3.Dot(normal,approach)<0)normal=-normal;
                    var throat=CanalFacingRing(rest,triangles,regions,centre,normal,largest);
                    if(throat!=null){loop=throat;IsCanalFacingThroat=true;}
                }
                if(approach.LengthSquared>.5){Vec3 winding=new Vec3(),c=new Vec3();foreach(int i in loop)c+=rest[i]/loop.Length;for(int i=0;i<loop.Length;i++)winding+=Vec3.Cross(rest[loop[i]]-c,rest[loop[(i+1)%loop.Length]]-c);normalOrientation=Vec3.Dot(winding,approach)>=0?1:-1;}
            }
        }
        static int[] CanalFacingRing(Vec3[] rest,int[] triangles,int[] regions,Vec3 centre,Vec3 axis,double radius)
        {
            var used=new HashSet<int>();for(int f=0;f<regions.Length;f++)if(regions[f]==3)for(int k=0;k<3;k++)used.Add(triangles[f*3+k]);
            double min=double.PositiveInfinity,max=double.NegativeInfinity;foreach(int i in used){double d=Vec3.Dot(rest[i]-centre,axis);min=Math.Min(min,d);max=Math.Max(max,d);}double extent=max-min;
            if(extent<radius*.5)return null;
            double tolerance=extent*2e-5;var rows=new Dictionary<int,List<int>>();
            // Only existing constant-depth mesh edges enter this graph. No
            // coincident vertices, skin weights or seam identities are merged.
            var seenEdges=new HashSet<long>();
            for(int f=0;f<regions.Length;f++)if(regions[f]==3)for(int k=0;k<3;k++)
            {
                int a=triangles[f*3+k],b=triangles[f*3+(k+1)%3];double da=Vec3.Dot(rest[a]-centre,axis),db=Vec3.Dot(rest[b]-centre,axis);
                if(Math.Abs(da-db)>tolerance||Math.Max(da,db)>min+extent*.25)continue;
                long key=((long)Math.Min(a,b)<<32)|(uint)Math.Max(a,b);if(!seenEdges.Add(key))continue;
                List<int> row;if(!rows.TryGetValue(a,out row)){row=new List<int>(2);rows.Add(a,row);}row.Add(b);if(!rows.TryGetValue(b,out row)){row=new List<int>(2);rows.Add(b,row);}row.Add(a);
            }
            var visited=new HashSet<int>();int[] chosen=null;double first=double.PositiveInfinity;
            foreach(int seed in rows.Keys)
            {
                if(visited.Contains(seed)||rows[seed].Count!=2)continue;var ring=new List<int>();int previous=-1,current=seed;bool closed=false;
                while(!visited.Contains(current))
                {visited.Add(current);ring.Add(current);var row=rows[current];if(row.Count!=2)break;int next=row[0]==previous?row[1]:row[0];previous=current;current=next;if(current==seed){closed=true;break;}}
                if(!closed||ring.Count<8)continue;
                Vec3 c=new Vec3();foreach(int i in ring)c+=rest[i]/ring.Count;double depth=Vec3.Dot(c-centre,axis),r=0;foreach(int i in ring)r+=(rest[i]-c).Length/ring.Count;
                Vec3 lateral=c-centre-axis*depth;
                if(r>1e-8&&r<radius*.45&&lateral.Length<radius*.25&&depth<first){first=depth;chosen=ring.ToArray();}
            }
            return chosen;
        }
        public void Update(Vec3[] world,Vec3 approach,Vec3 orientation)
        {
            if(!Available)return;Vec3 origin=world[loop[0]],centre=new Vec3();double total=0;
            for(int j=0;j<loop.Length;j++){Vec3 a=world[loop[j]]-origin,b=world[loop[(j+1)%loop.Length]]-origin;double length=(b-a).Length;centre+=(a+b)*(length*.5);total+=length;}
            Centre=origin+centre/Math.Max(1e-12,total);Vec3 normal=new Vec3();Radius=0;
            for(int j=0;j<loop.Length;j++){Vec3 a=world[loop[j]]-Centre,b=world[loop[(j+1)%loop.Length]]-Centre;normal+=Vec3.Cross(a,b);Radius+=(a.Length+b.Length)*.5/loop.Length;}
            Normal=normal.Normalized;if(normalOrientation==0&&Normal.LengthSquared>.5)normalOrientation=Vec3.Dot(Normal,approach)>=0?1:-1;Normal*=normalOrientation;if(Normal.LengthSquared<.5)Normal=approach.Normalized;
            Vec3 x=orientation-Normal*Vec3.Dot(orientation,Normal);if(x.LengthSquared<1e-16)x=Vec3.Cross(Normal,Math.Abs(Normal.X)<.8?new Vec3(1,0,0):new Vec3(0,1,0));x=x.Normalized;Vec3 y=Vec3.Cross(Normal,x);Points[0]=Centre;
            for(int k=0;k<4;k++){Vec3 direction=k==0?x:k==1?y:k==2?-x:-y;double best=double.NegativeInfinity;int index=loop[0];foreach(int i in loop){double score=Vec3.Dot(world[i]-Centre,direction);if(score>best){best=score;index=i;}}Points[k+1]=world[index];PointIndices[k+1]=index;}
        }
    }
}
