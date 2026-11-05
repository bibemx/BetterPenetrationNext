using System;
using System.Diagnostics;
namespace BetterPenetrationNext.Core
{
 // Resolves strict edge/face crossings that a finite contact sample set may miss.
 // Source proxy faces must retain the supplied original face order; virtual seam
 // faces appended after that range are deliberately excluded from this pass.
 public sealed class TriangleMeshContactRefiner
 {
  readonly int[] receiverFaces,sourceFaces,overlaps;
  public SourceContactBody Source;public Vec3[] AnimatedSource;
  public double OutwardSign=1,Clearance=.0002,Milliseconds;
  public int BroadphaseQueries,PairTests,Crossings;
  public TriangleMeshContactRefiner(int[] receiverTriangles,int[] sourceTriangles,int maximumSourceFaces){receiverFaces=(int[])receiverTriangles.Clone();sourceFaces=(int[])sourceTriangles.Clone();overlaps=new int[maximumSourceFaces];}
  static bool Segment(Vec3 p,Vec3 q,Vec3 a,Vec3 b,Vec3 c,out double t,out Vec3 bary)
  {
   t=0;bary=new Vec3();Vec3 e1=b-a,e2=c-a,d=q-p,h=Vec3.Cross(d,e2);double det=Vec3.Dot(e1,h);if(Math.Abs(det)<1e-18)return false;Vec3 s=p-a;double u=Vec3.Dot(s,h)/det;if(u<1e-8||u>1-1e-8)return false;Vec3 x=Vec3.Cross(s,e1);double v=Vec3.Dot(d,x)/det;if(v<1e-8||u+v>1-1e-8)return false;t=Vec3.Dot(e2,x)/det;bary=new Vec3(1-u-v,u,v);return t>1e-8&&t<1-1e-8;
  }
  static Vec3 EdgeBary(int edge,double t){return edge==0?new Vec3(1-t,t,0):edge==1?new Vec3(0,1-t,t):new Vec3(t,0,1-t);}
  static bool Cross(Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 e,Vec3 f,out Vec3 receiverBary,out Vec3 sourceBary)
  {
   double t;for(int edge=0;edge<3;edge++){Vec3 p=edge==0?d:edge==1?e:f,q=edge==0?e:edge==1?f:d;if(Segment(p,q,a,b,c,out t,out receiverBary)){sourceBary=EdgeBary(edge,t);return true;}}
   for(int edge=0;edge<3;edge++){Vec3 p=edge==0?a:edge==1?b:c,q=edge==0?b:edge==1?c:a;if(Segment(p,q,d,e,f,out t,out sourceBary)){receiverBary=EdgeBary(edge,t);return true;}}receiverBary=sourceBary=new Vec3();return false;
  }
  Vec3 Inverse(Vec3 point){return Source.Pivot+Source.Rotate(point-Source.Pivot-Source.Offset,true);}
  Vec3 Forward(Vec3 point){return Source.Pivot+Source.Rotate(point-Source.Pivot)+Source.Offset;}
  public int Project(Vec3[] positions,Vec3[] anchors,double[] inverseMass,double maximumOffset,BidirectionalContactSettings settings,double h)
  {
   Crossings=0;var sourceSurface=Source==null?null:Source.Surface as TriangleSurface;if(sourceSurface==null)return 0;long started=Stopwatch.GetTimestamp();
   for(int face=0;face<receiverFaces.Length;face+=3){int ia=receiverFaces[face],ib=receiverFaces[face+1],ic=receiverFaces[face+2];Vec3 a=positions[ia],b=positions[ib],c=positions[ic];var bounds=Bounds3.Empty;bounds.Include(Inverse(a));bounds.Include(Inverse(b));bounds.Include(Inverse(c));BroadphaseQueries++;int count=sourceSurface.CollectTriangles(bounds,overlaps);
    for(int k=0;k<count;k++){int sourceFace=overlaps[k];if(sourceFace>=sourceFaces.Length/3)continue;Vec3 d,e,f;sourceSurface.GetTriangle(sourceFace,out d,out e,out f);d=Forward(d);e=Forward(e);f=Forward(f);Vec3 receiverBary,sourceBary;PairTests++;if(!Cross(a,b,c,d,e,f,out receiverBary,out sourceBary))continue;Crossings++;
     Vec3 n=-Vec3.Cross(b-a,c-a).Normalized*OutwardSign;if(n.LengthSquared<.5)continue;Vec3 gradient=Source.ProjectContactGradient(n);double wa=inverseMass[ia]*settings.TissueDeformationStrength,wb=inverseMass[ib]*settings.TissueDeformationStrength,wc=inverseMass[ic]*settings.TissueDeformationStrength,wr=wa*receiverBary.X*receiverBary.X+wb*receiverBary.Y*receiverBary.Y+wc*receiverBary.Z*receiverBary.Z,ws=Source.InverseMass*gradient.LengthSquared,delta=BidirectionalContactSolver.Multiplier(-Clearance,0,wr,ws,settings.TissueCompliance/(h*h));Vec3 before=Source.Offset;Source.Correct(gradient*(Source.InverseMass*delta));double accepted=Math.Max(0,Vec3.Dot(Source.Offset-before,n));if(wr>1e-20)delta+=Math.Max(0,ws*delta-accepted)/wr;
     positions[ia]=anchors[ia]+Vec3.ClampLength(positions[ia]-anchors[ia]-n*(wa*receiverBary.X*delta),maximumOffset);positions[ib]=anchors[ib]+Vec3.ClampLength(positions[ib]-anchors[ib]-n*(wb*receiverBary.Y*delta),maximumOffset);positions[ic]=anchors[ic]+Vec3.ClampLength(positions[ic]-anchors[ic]-n*(wc*receiverBary.Z*delta),maximumOffset);Source.MarkContact();a=positions[ia];b=positions[ib];c=positions[ic];
    }
   }
   Milliseconds+=(Stopwatch.GetTimestamp()-started)*1000.0/Stopwatch.Frequency;return Crossings;
  }
 }
}
