using System;
namespace BetterPenetrationNext.Core
{
 // Cache material attachments, then transport each layer offset in the actual
 // animated/deformed face frame. A displacement-only field loses layer thickness
 // when the channel changes direction. No renderer bone weights are merged.
 public sealed class ReceiverLayerTransfer
 {
  readonly int[] a,b,c,physicalIndex;readonly Vec3[] bary;readonly double[] influence;
  public ReceiverLayerTransfer(Vec3[] rest,Vec3[] physicalRest,int[] physicalFaces,ReceiverSupportTransfer support,double[] falloff)
  {
   int n=rest.Length;a=new int[n];b=new int[n];c=new int[n];bary=new Vec3[n];physicalIndex=new int[n];influence=(double[])falloff.Clone();for(int i=0;i<n;i++)physicalIndex[i]=-1;for(int i=0;i<support.PhysicalIndices.Length;i++)physicalIndex[support.PhysicalIndices[i]]=i;
   var surface=new TriangleSurface(physicalRest,physicalFaces,SurfaceKind.OrientedShell);for(int i=0;i<n;i++){if(physicalIndex[i]>=0)continue;SurfaceSample hit;if(!surface.TrySample(rest[i],1,out hit))throw new ArgumentException("Missing original material attachment");int t=hit.Triangle*3;a[i]=physicalFaces[t];b[i]=physicalFaces[t+1];c[i]=physicalFaces[t+2];Vec3 ab=physicalRest[b[i]]-physicalRest[a[i]],ac=physicalRest[c[i]]-physicalRest[a[i]],aq=hit.Point-physicalRest[a[i]];double aa=Vec3.Dot(ab,ab),bb=Vec3.Dot(ab,ac),cc=Vec3.Dot(ac,ac),den=aa*cc-bb*bb,v=(cc*Vec3.Dot(aq,ab)-bb*Vec3.Dot(aq,ac))/den,w=(aa*Vec3.Dot(aq,ac)-bb*Vec3.Dot(aq,ab))/den;bary[i]=new Vec3(1-v-w,v,w);}
  }
  public void Apply(SoftBodySolver solver,Vec3[] physicalBase,Vec3[] fullBase,Vec3[] output)
  {
   for(int i=0;i<output.Length;i++){
    int p=physicalIndex[i];if(p>=0){output[i]=solver.Displacement(p);continue;}if(influence[i]<=0){output[i]=new Vec3();continue;}int ia=a[i],ib=b[i],ic=c[i];Vec3 pa=physicalBase[ia],pb=physicalBase[ib],pc=physicalBase[ic],qa=pa+solver.Displacement(ia),qb=pb+solver.Displacement(ib),qc=pc+solver.Displacement(ic),w=bary[i],basePoint=pa*w.X+pb*w.Y+pc*w.Z,newPoint=qa*w.X+qb*w.Y+qc*w.Z,delta=fullBase[i]-basePoint;
    Vec3 oldX=(pb-pa).Normalized,oldN=Vec3.Cross(pb-pa,pc-pa).Normalized,oldY=Vec3.Cross(oldN,oldX),newX=(qb-qa).Normalized,newN=Vec3.Cross(qb-qa,qc-qa).Normalized,newY=Vec3.Cross(newN,newX);
    if(oldN.LengthSquared<.5||newN.LengthSquared<.5){output[i]=(newPoint-basePoint)*influence[i];continue;}Vec3 rotated=newX*Vec3.Dot(delta,oldX)+newY*Vec3.Dot(delta,oldY)+newN*Vec3.Dot(delta,oldN);output[i]=(newPoint+rotated-fullBase[i])*influence[i];
   }
  }
 }
}
