using System;
namespace BetterPenetrationNext.Core
{
 // A periodic chart of the supplied open annular mesh, never a replacement
 // visual cylinder. The chart is prepared off-line on a cut copy of the mesh.
 public sealed class AnnularReceiverRig
 {
  readonly int[] inner;
  readonly double[] axial,turns;
  readonly Vec3[] centre,tangent,normal,binormal;
  public readonly Vec3[] Targets;
  readonly Vec3[] offset;
  readonly int[] offsets,neighbors;
  readonly double[] weights;
  readonly bool[] fixedNode;
  readonly int[] lookup;
  readonly double[] support=new double[64],supportX=new double[64],supportY=new double[64];
  public double EnvelopeRadius {get;private set;}
  public AnnularReceiverRig(int vertexCount,int[] innerControls,double[] materialAxial,double[] angleTurns,Vec3[] rest=null,EdgeConstraint[] edges=null,double[] pins=null,int resolution=129)
  {
   if(innerControls.Length!=materialAxial.Length||innerControls.Length!=angleTurns.Length||resolution<3)throw new ArgumentException("Invalid annular chart");
   inner=(int[])innerControls.Clone();axial=(double[])materialAxial.Clone();turns=(double[])angleTurns.Clone();Targets=new Vec3[vertexCount];offset=new Vec3[vertexCount];fixedNode=new bool[vertexCount];lookup=new int[vertexCount];for(int i=0;i<lookup.Length;i++)lookup[i]=-1;for(int i=0;i<inner.Length;i++){fixedNode[inner[i]]=true;lookup[inner[i]]=i;}centre=new Vec3[resolution];tangent=new Vec3[resolution];normal=new Vec3[resolution];binormal=new Vec3[resolution];
   if(edges!=null){offsets=new int[vertexCount+1];foreach(var e in edges){offsets[e.A+1]++;offsets[e.B+1]++;}for(int i=1;i<offsets.Length;i++)offsets[i]+=offsets[i-1];neighbors=new int[offsets[vertexCount]];weights=new double[neighbors.Length];var cursor=(int[])offsets.Clone();foreach(var e in edges){double w=1/Math.Max(1e-7,(rest[e.A]-rest[e.B]).Length);int a=cursor[e.A]++,b=cursor[e.B]++;neighbors[a]=e.B;neighbors[b]=e.A;weights[a]=weights[b]=w;}for(int i=0;i<vertexCount;i++)if(pins[i]>=1)fixedNode[i]=true;}
   for(int i=0;i<support.Length;i++){supportX[i]=Math.Cos(i*2*Math.PI/support.Length);supportY[i]=Math.Sin(i*2*Math.PI/support.Length);}
  }
  Vec3 Point(double u,double turn)
  {
   double t=Math.Max(0,Math.Min(1,u))*(centre.Length-1);int a=Math.Min(centre.Length-2,(int)t);double f=t-a,angle=turn*2*Math.PI,x=Math.Cos(angle),y=Math.Sin(angle),radius=double.PositiveInfinity;for(int j=0;j<support.Length;j++){double dot=supportX[j]*x+supportY[j]*y;if(dot>1e-10)radius=Math.Min(radius,support[j]/dot);}Vec3 n=(normal[a]*(1-f)+normal[a+1]*f).Normalized,b=Vec3.Cross((tangent[a]*(1-f)+tangent[a+1]*f).Normalized,n).Normalized;return centre[a]*(1-f)+centre[a+1]*f+(n*x+b*y)*radius;
  }
  public double FitMaterial(EdgeConstraint[] edges,double[] maximumLength,int iterations)
  {
   double violation=0;
   for(int sweep=0;sweep<iterations;sweep++){
    violation=0;for(int k=0;k<edges.Length;k++){int ei=sweep%2==0?k:edges.Length-1-k;var edge=edges[ei];int a=lookup[edge.A],b=lookup[edge.B];if(a<0||b<0)continue;Vec3 pa=Point(axial[a],turns[a]),pb=Point(axial[b],turns[b]),d=pa-pb;double length=d.Length,gap=length-maximumLength[ei];violation=Math.Max(violation,gap);if(gap<=0||length<1e-15)continue;Vec3 n=d/length;
     double eps=1e-5;Vec3 au=(Point(axial[a]+eps,turns[a])-Point(axial[a]-eps,turns[a]))/(2*eps),bu=(Point(axial[b]+eps,turns[b])-Point(axial[b]-eps,turns[b]))/(2*eps),at=(Point(axial[a],turns[a]+eps)-Point(axial[a],turns[a]-eps))/(2*eps),bt=(Point(axial[b],turns[b]+eps)-Point(axial[b],turns[b]-eps))/(2*eps);
     double ga=axial[a]<=0||axial[a]>=1?0:Vec3.Dot(n,au),gb=axial[b]<=0||axial[b]>=1?0:-Vec3.Dot(n,bu),ta=Vec3.Dot(n,at),tb=-Vec3.Dot(n,bt),den=ga*ga+gb*gb+ta*ta+tb*tb;if(den<1e-30)continue;double dl=-gap*.65/den;
     if(ga!=0)axial[a]=Math.Max(1e-6,Math.Min(1-1e-6,axial[a]+Math.Max(-.002,Math.Min(.002,ga*dl))));if(gb!=0)axial[b]=Math.Max(1e-6,Math.Min(1-1e-6,axial[b]+Math.Max(-.002,Math.Min(.002,gb*dl))));turns[a]+=Math.Max(-.002,Math.Min(.002,ta*dl));turns[b]+=Math.Max(-.002,Math.Min(.002,tb*dl));
    }
    if(violation<1e-9)break;
   }
   for(int i=0;i<inner.Length;i++)Targets[inner[i]]=Point(axial[i],turns[i]);return violation;
  }
  public void CopyChart(double[] u,double[] theta){Array.Copy(axial,u,axial.Length);Array.Copy(turns,theta,turns.Length);}
  static Vec3 Transport(Vec3 vector,Vec3 from,Vec3 to)
  {
   double c=Math.Max(-1,Math.Min(1,Vec3.Dot(from,to)));Vec3 a=Vec3.Cross(from,to);double s=a.Length;if(s<1e-12)return vector;a/=s;return vector*c+Vec3.Cross(a,vector)*s+a*(Vec3.Dot(a,vector)*(1-c));
  }
  public void Capture(AnatomicalPathConstraintSystem path,Vec3[] source,Vec3[] anchors,Vec3 orientation,double margin)
  {
   Array.Copy(anchors,Targets,anchors.Length);EnvelopeRadius=0;for(int j=0;j<support.Length;j++)support[j]=0;
   for(int i=0;i<centre.Length;i++){var q=path.AtDistance(path.Length*i/(centre.Length-1));centre[i]=q.Point;tangent[i]=q.Tangent;
    Vec3 n=i==0?orientation:Transport(normal[i-1],tangent[i-1],tangent[i]);n-=tangent[i]*Vec3.Dot(n,tangent[i]);if(n.LengthSquared<1e-20)n=Vec3.Cross(tangent[i],Math.Abs(tangent[i].X)<.8?new Vec3(1,0,0):new Vec3(0,1,0));normal[i]=n.Normalized;binormal[i]=Vec3.Cross(tangent[i],normal[i]).Normalized;}
   foreach(var p in source){var q=path.Closest(p);double t=path.DistanceAtParameter(q.Parameter)/path.Length*(centre.Length-1);int a=Math.Min(centre.Length-2,(int)t);double f=t-a;Vec3 n=(normal[a]*(1-f)+normal[a+1]*f).Normalized,b=Vec3.Cross(q.Tangent,n).Normalized,d=p-q.Point;double x=Vec3.Dot(d,n),y=Vec3.Dot(d,b);EnvelopeRadius=Math.Max(EnvelopeRadius,Math.Sqrt(x*x+y*y));for(int j=0;j<support.Length;j++)support[j]=Math.Max(support[j],supportX[j]*x+supportY[j]*y);}
   for(int j=0;j<support.Length;j++)support[j]+=margin;
   for(int i=0;i<inner.Length;i++)Targets[inner[i]]=Point(axial[i],turns[i]);
   for(int i=0;i<offset.Length;i++)if(fixedNode[i])offset[i]=Targets[i]-anchors[i];
   if(offsets!=null)for(int sweep=0;sweep<80;sweep++)for(int k=0;k<offset.Length;k++){int i=sweep%2==0?k:offset.Length-1-k;if(fixedNode[i])continue;Vec3 sum=new Vec3();double total=0;for(int j=offsets[i];j<offsets[i+1];j++){sum+=offset[neighbors[j]]*weights[j];total+=weights[j];}if(total>0)offset[i]=sum/total;}
   for(int i=0;i<Targets.Length;i++)Targets[i]=anchors[i]+offset[i];
  }
 }
}
