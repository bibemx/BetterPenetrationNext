using System;
namespace BetterPenetrationNext.Core
{
 public sealed partial class TriangleSurface
 {
  public double OutwardOrientation {get{return orientation;}}
  public double CalculateSignedVolume(){Vec3 origin=(Bounds.Min+Bounds.Max)/2;double volume=0;for(int i=0;i<triangles.Length;i+=3)volume+=Vec3.Dot(vertices[triangles[i]]-origin,Vec3.Cross(vertices[triangles[i+1]]-origin,vertices[triangles[i+2]]-origin));return volume/6;}
  static bool Overlaps(Bounds3 a,Bounds3 b){return a.Min.X<=b.Max.X&&b.Min.X<=a.Max.X&&a.Min.Y<=b.Max.Y&&b.Min.Y<=a.Max.Y&&a.Min.Z<=b.Max.Z&&b.Min.Z<=a.Max.Z;}
  public int CollectTriangles(Bounds3 bounds,int[] output,int minimumTriangle=0){int count=0;Collect(0,bounds,output,minimumTriangle,ref count);return count;}
  void Collect(int node,Bounds3 bounds,int[] output,int minimum,ref int count)
  {
   var n=nodes[node];if(!Overlaps(n.Bounds,bounds))return;if(n.Count==0){Collect(n.Left,bounds,output,minimum,ref count);Collect(n.Right,bounds,output,minimum,ref count);return;}
   for(int j=n.Start;j<n.Start+n.Count;j++){int face=order[j];if(face<minimum||!Overlaps(TriangleBounds(face),bounds))continue;if(count>=output.Length)throw new ArgumentException("Triangle overlap buffer is too small");output[count++]=face;}
  }
  public void GetTriangle(int face,out Vec3 a,out Vec3 b,out Vec3 c){int i=face*3;a=vertices[triangles[i]];b=vertices[triangles[i+1]];c=vertices[triangles[i+2]];}
  public bool TrySampleWithRays(Vec3 point,double reach,ref int hint,out SurfaceSample sample)
  {
   bool previousSign=robustSign;try{robustSign=true;return TrySampleHinted(point,reach,ref hint,out sample);}finally{robustSign=previousSign;}
  }
 }
}
