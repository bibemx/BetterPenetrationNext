using System;
using System.Diagnostics;
namespace BetterPenetrationNext.Core
{
 // A local geometric backtrack affects only the two intersecting triangles.
 // It never scales the full receiver or merges material/seam vertices.
 public sealed class TriangleSelfCollisionGuard
 {
  readonly int[] faces,overlaps;readonly TriangleSurface surface;
  public int RemainingCrossings,Corrections;public double Milliseconds;
  public long Calls,PairTests,TotalCorrections;
  public TriangleSelfCollisionGuard(Vec3[] positions,int[] triangles){faces=(int[])triangles.Clone();overlaps=new int[triangles.Length/3];surface=new TriangleSurface(positions,triangles,SurfaceKind.OrientedShell);surface.InitializeIncrementalRefit();}
  static bool Segment(Vec3 p,Vec3 q,Vec3 a,Vec3 b,Vec3 c){Vec3 e1=b-a,e2=c-a,d=q-p,h=Vec3.Cross(d,e2);double det=Vec3.Dot(e1,h);if(Math.Abs(det)<1e-18)return false;Vec3 s=p-a;double u=Vec3.Dot(s,h)/det;if(u<1e-8||u>1-1e-8)return false;Vec3 x=Vec3.Cross(s,e1);double v=Vec3.Dot(d,x)/det;if(v<1e-8||u+v>1-1e-8)return false;double t=Vec3.Dot(e2,x)/det;return t>1e-8&&t<1-1e-8;}
  static bool Cross(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 e,Vec3 f){return Segment(a,b,d,e,f)||Segment(b,c,d,e,f)||Segment(c,a,d,e,f)||Segment(d,e,a,b,c)||Segment(e,f,a,b,c)||Segment(f,d,a,b,c);}
  bool Adjacent(int i,int j){for(int a=0;a<3;a++)for(int b=0;b<3;b++)if(faces[i*3+a]==faces[j*3+b])return true;return false;}
  bool MixedCross(Vec3[] previous,Vec3[] positions,int i,int j,double t){int a=faces[i*3],b=faces[i*3+1],c=faces[i*3+2],d=faces[j*3],e=faces[j*3+1],f=faces[j*3+2];return Cross(previous[a]+(positions[a]-previous[a])*t,previous[b]+(positions[b]-previous[b])*t,previous[c]+(positions[c]-previous[c])*t,previous[d]+(positions[d]-previous[d])*t,previous[e]+(positions[e]-previous[e])*t,previous[f]+(positions[f]-previous[f])*t);}
  public int Resolve(Vec3[] positions,Vec3[] previous,Vec3[] anchors=null)
  {
   long started=Stopwatch.GetTimestamp();Calls++;Corrections=RemainingCrossings=0;surface.Refit(positions);
   for(int sweep=0;sweep<8;sweep++){int corrected=Corrections;
   for(int i=0;i<faces.Length/3;i++){int a=faces[i*3],b=faces[i*3+1],c=faces[i*3+2];var bounds=Bounds3.Empty;bounds.Include(positions[a]);bounds.Include(positions[b]);bounds.Include(positions[c]);int count=surface.CollectTriangles(bounds,overlaps,i+1);
    for(int k=0;k<count;k++){int j=overlaps[k];if(Adjacent(i,j))continue;PairTests++;if(!MixedCross(previous,positions,i,j,1))continue;Vec3[] origin=previous;if(MixedCross(origin,positions,i,j,0)){if(anchors==null||MixedCross(anchors,positions,i,j,0)){RemainingCrossings++;continue;}origin=anchors;}double lo=0,hi=1;for(int iteration=0;iteration<18;iteration++){double mid=(lo+hi)*.5;if(MixedCross(origin,positions,i,j,mid))hi=mid;else lo=mid;}double scale=lo*.999;for(int v=0;v<3;v++){int p=faces[i*3+v],q=faces[j*3+v];positions[p]=origin[p]+(positions[p]-origin[p])*scale;positions[q]=origin[q]+(positions[q]-origin[q])*scale;}surface.RefitVertices(positions,faces[i*3],faces[i*3+1],faces[i*3+2]);surface.RefitVertices(positions,faces[j*3],faces[j*3+1],faces[j*3+2]);Corrections++;TotalCorrections++;}
   }
   if(corrected==Corrections)break;}
   Milliseconds+=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;return Corrections;
  }
 }
}
