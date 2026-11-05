using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    // Project invalid faces onto the small section field, not independently
    // onto seam vertices. Equal material coordinates retain equal corrections.
    public sealed class SourceSectionAreaConstraints
    {
        readonly int[] lower,triangles,ids=new int[6];
        readonly double[] fraction;
        readonly Vec3[] gradients=new Vec3[6];
        readonly int[][] sectionFaces;
        readonly int[] queue,visits;
        readonly bool[] queued;
        int head,tail,pending,visitLimit;
        public int LastChecks {get;private set;}
        int count;
        public SourceSectionAreaConstraints(double[] coordinates,int[] faces,int sections)
        {
            lower=new int[coordinates.Length];fraction=new double[coordinates.Length];triangles=faces;
            for(int i=0;i<coordinates.Length;i++){double u=coordinates[i]*(sections-1);lower[i]=Math.Min(sections-2,(int)u);fraction[i]=u-lower[i];}
            var adjacency=new List<int>[sections];for(int s=0;s<sections;s++)adjacency[s]=new List<int>();
            for(int f=0;f<triangles.Length/3;f++){count=0;for(int v=0;v<3;v++){int index=triangles[f*3+v];Add(lower[index],new Vec3());Add(lower[index]+1,new Vec3());}for(int s=0;s<count;s++)adjacency[ids[s]].Add(f);}
            sectionFaces=new int[sections][];for(int s=0;s<sections;s++)sectionFaces[s]=adjacency[s].ToArray();queue=new int[triangles.Length/3];visits=new int[queue.Length];queued=new bool[queue.Length];
        }
        Vec3 Offset(Vec3[] field,int vertex){return field[lower[vertex]]*(1-fraction[vertex])+field[lower[vertex]+1]*fraction[vertex];}
        void Add(int section,Vec3 gradient)
        {
            for(int i=0;i<count;i++)if(ids[i]==section){gradients[i]+=gradient;return;}
            ids[count]=section;gradients[count++]=gradient;
        }
        void VertexGradient(int vertex,Vec3 gradient){Add(lower[vertex],gradient*(1-fraction[vertex]));Add(lower[vertex]+1,gradient*fraction[vertex]);}
        void Enqueue(int face){if(queued[face]||visits[face]>=visitLimit)return;queue[tail]=face;tail=(tail+1)%queue.Length;pending++;queued[face]=true;}
        public void Project(Vec3[] animated,Vec3[] field,double maximum,int iterations=8)
        {
            Array.Clear(visits,0,visits.Length);Array.Clear(queued,0,queued.Length);head=tail=pending=0;visitLimit=iterations;LastChecks=0;
            for(int f=0;f<queue.Length;f++)Enqueue(f);
            while(pending>0)
            {
                int f=queue[head];head=(head+1)%queue.Length;pending--;queued[f]=false;visits[f]++;LastChecks++;int face=f*3;
                {
                    int a=triangles[face],b=triangles[face+1],c=triangles[face+2];
                    Vec3 ab=animated[b]-animated[a],ac=animated[c]-animated[a],normal=Vec3.Cross(ab,ac);double area=normal.Length;
                    if(area<1e-12)continue;normal/=area;
                    Vec3 correctedAB=ab+Offset(field,b)-Offset(field,a),correctedAC=ac+Offset(field,c)-Offset(field,a);
                    double error=Vec3.Dot(Vec3.Cross(correctedAB,correctedAC),normal)-area*.02;
                    if(error>=0)continue;
                    Vec3 gb=Vec3.Cross(correctedAC,normal),gc=Vec3.Cross(normal,correctedAB),ga=-gb-gc;
                    count=0;VertexGradient(a,ga);VertexGradient(b,gb);VertexGradient(c,gc);
                    double denominator=0;for(int i=0;i<count;i++)if(ids[i]>0&&ids[i]<field.Length-1)denominator+=gradients[i].LengthSquared;
                    if(denominator<1e-24)continue;double multiplier=-error/denominator;
                    for(int i=0;i<count;i++)if(ids[i]>0&&ids[i]<field.Length-1){int section=ids[i];Vec3 previous=field[section];field[section]=Vec3.ClampLength(previous+gradients[i]*multiplier,maximum);if((field[section]-previous).LengthSquared>1e-30)foreach(int affected in sectionFaces[section])Enqueue(affected);}
                }
            }
        }
    }
}
