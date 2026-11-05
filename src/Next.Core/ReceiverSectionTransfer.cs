using System;
namespace BetterPenetrationNext.Core
{
 // Smooth material-section frames carry neighbouring layers. Fine physical
 // displacement remains exact at every mucosal control through a residual map.
 public sealed class ReceiverSectionTransfer
 {
  readonly ReceiverGuideDefinition guide;readonly ReceiverSupportTransfer support;
  readonly double[] coordinates,influence,angles,station;
  readonly Vec3[] oldCentre,newCentre,oldX,newX,oldN,newN,residual;
  readonly int[] section;readonly double[] blend;readonly int[] physical;
  public ReceiverSectionTransfer(ReceiverGuideDefinition guide,ReceiverSupportTransfer support,double[] materialCoordinates,double[] falloff,double[] materialAngles)
  {
   this.guide=guide;this.support=support;coordinates=(double[])materialCoordinates.Clone();influence=(double[])falloff.Clone();angles=(double[])materialAngles.Clone();int n=guide.Contours.Length;station=new double[n];oldCentre=new Vec3[n];newCentre=new Vec3[n];oldX=new Vec3[n];newX=new Vec3[n];oldN=new Vec3[n];newN=new Vec3[n];residual=new Vec3[support.PhysicalIndices.Length];section=new int[coordinates.Length];blend=new double[coordinates.Length];physical=new int[coordinates.Length];for(int i=0;i<physical.Length;i++)physical[i]=-1;for(int i=0;i<support.PhysicalIndices.Length;i++)physical[support.PhysicalIndices[i]]=i;
   for(int s=0;s<n;s++){var c=guide.Contours[s];for(int k=0;k<c.Vertices.Length;k++)station[s]+=coordinates[support.PhysicalIndices[c.Vertices[k]]]*c.Weights[k];if(s>0&&station[s]<=station[s-1])throw new ArgumentException("Material section coordinates must be ordered");}
   for(int i=0;i<coordinates.Length;i++){int s=0;while(s<n-2&&coordinates[i]>station[s+1])s++;section[i]=s;blend[i]=Math.Max(0,Math.Min(1,(coordinates[i]-station[s])/(station[s+1]-station[s])));}
  }
  static Vec3 Axis(Vec3 proposed,Vec3 normal,Vec3 fallback){proposed-=normal*Vec3.Dot(proposed,normal);if(proposed.LengthSquared<1e-20){proposed=fallback-normal*Vec3.Dot(fallback,normal);if(proposed.LengthSquared<1e-20)proposed=Vec3.Cross(normal,Math.Abs(normal.X)<.8?new Vec3(1,0,0):new Vec3(0,1,0));}return proposed.Normalized;}
  Vec3 Map(int index,Vec3 point)
  {
   int a=section[index],b=a+1;double t=blend[index];Vec3 oldC=oldCentre[a]*(1-t)+oldCentre[b]*t,newC=newCentre[a]*(1-t)+newCentre[b]*t,n0=(oldN[a]*(1-t)+oldN[b]*t).Normalized,n1=(newN[a]*(1-t)+newN[b]*t).Normalized,x0=Axis(oldX[a]*(1-t)+oldX[b]*t,n0,oldX[a]),x1=Axis(newX[a]*(1-t)+newX[b]*t,n1,newX[a]),y0=Vec3.Cross(n0,x0),y1=Vec3.Cross(n1,x1),delta=point-oldC;return newC+x1*Vec3.Dot(delta,x0)+y1*Vec3.Dot(delta,y0)+n1*Vec3.Dot(delta,n0)-point;
  }
  public void Apply(SoftBodySolver solver,Vec3[] physicalBase,Vec3[] fullBase,Vec3[] output)
  {
   for(int s=0;s<guide.Contours.Length;s++){Vec3 a=new Vec3(),b=new Vec3();var c=guide.Contours[s];for(int k=0;k<c.Vertices.Length;k++){int i=c.Vertices[k];a+=physicalBase[i]*c.Weights[k];b+=(physicalBase[i]+solver.Displacement(i))*c.Weights[k];}oldCentre[s]=a;newCentre[s]=b;}
   for(int s=0;s<guide.Contours.Length;s++){int a=Math.Max(0,s-1),b=Math.Min(oldCentre.Length-1,s+1);oldN[s]=(oldCentre[b]-oldCentre[a]).Normalized;newN[s]=(newCentre[b]-newCentre[a]).Normalized;Vec3 x0=new Vec3(),x1=new Vec3();var c=guide.Contours[s];for(int k=0;k<c.Vertices.Length;k++){int i=c.Vertices[k];double w=c.Weights[k]*Math.Cos(angles[i]*2*Math.PI);x0+=(physicalBase[i]-oldCentre[s])*w;x1+=(physicalBase[i]+solver.Displacement(i)-newCentre[s])*w;}oldX[s]=Axis(x0,oldN[s],s==0?new Vec3(1,0,0):oldX[s-1]);newX[s]=Axis(x1,newN[s],s==0?oldX[s]:newX[s-1]);}
   for(int i=0;i<residual.Length;i++){int v=support.PhysicalIndices[i];residual[i]=solver.Displacement(i)-Map(v,fullBase[v]);}
   for(int i=0;i<output.Length;i++){if(physical[i]>=0){output[i]=solver.Displacement(physical[i]);continue;}Vec3 value=Map(i,fullBase[i])*influence[i];for(int j=support.Offsets[i];j<support.Offsets[i+1];j++)value+=residual[support.Indices[j]]*support.Weights[j];output[i]=value;}
  }
 }
}
