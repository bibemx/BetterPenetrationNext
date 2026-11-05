using System;
namespace BetterPenetrationNext.Core
{
 public sealed class CoupledChannelGuide
 {
  readonly double[] coordinates,weights;
  readonly Vec3[] controls,sourceControls;
  readonly Vec3[] offsetState;
  public readonly Vec3[] Targets;
  public AnatomicalPathConstraintSystem SharedPath {get;private set;}
  public AnatomicalPathConstraintSystem SourcePath {get;private set;}
  public double MaximumTargetOffset {get;private set;}
  public bool UseCorrectedSource;
  public double EndClearance=.004;
  public CoupledChannelGuide(double[] coordinate,double[] weight,int sections)
  {coordinates=(double[])coordinate.Clone();weights=(double[])weight.Clone();Targets=new Vec3[coordinate.Length];offsetState=new Vec3[coordinate.Length];controls=new Vec3[sections+1];sourceControls=new Vec3[sections];}
  static Vec3 Rotate(Vec3 value,Vec3 from,Vec3 to)
  {
   Vec3 axis=Vec3.Cross(from,to);double sine=axis.Length,cosine=Math.Max(-1,Math.Min(1,Vec3.Dot(from,to)));if(sine<1e-12)return value;axis/=sine;return value*cosine+Vec3.Cross(axis,value)*sine+axis*(Vec3.Dot(axis,value)*(1-cosine));
  }
  public void Capture(AnatomicalPathConstraintSystem reference,SourceSectionRig source,Vec3[] sourceVertices,Vec3[] anchors,double receiverShare,double maximumOffset,double dt)
  {
   for(int i=0;i<sourceControls.Length;i++){double u=(double)i/(sourceControls.Length-1);Vec3 a=reference.AtDistance(reference.Length*u).Point,b=UseCorrectedSource?source.CorrectedCentre(i):source.AnimatedCentre(i);sourceControls[i]=a+(b-a)*(i==0?1:receiverShare);controls[i+1]=sourceControls[i];}
   Vec3 root=UseCorrectedSource?source.CorrectedCentre(0):source.AnimatedCentre(0),rootAxis=((UseCorrectedSource?source.CorrectedCentre(1):source.AnimatedCentre(1))-root).Normalized;double minimum=0;
   foreach(var p in sourceVertices)minimum=Math.Min(minimum,Vec3.Dot(p-root,rootAxis));controls[0]=root+rootAxis*(minimum-EndClearance);
   Vec3 endAxis=(controls[controls.Length-1]-controls[controls.Length-2]).Normalized;double endSupport=0;foreach(var p in sourceVertices)endSupport=Math.Max(endSupport,Vec3.Dot(p-controls[controls.Length-1],endAxis));controls[controls.Length-1]+=endAxis*(endSupport+EndClearance);
   if(SharedPath==null)SharedPath=new AnatomicalPathConstraintSystem(controls,true);else SharedPath.UpdateControls(controls);
   if(SourcePath==null)SourcePath=new AnatomicalPathConstraintSystem(sourceControls,true);else SourcePath.UpdateControls(sourceControls);
   MaximumTargetOffset=0;
   for(int i=0;i<Targets.Length;i++){double u=coordinates[i];var a=reference.AtDistance(reference.Length*u);var b=SharedPath.AtDistance(SharedPath.Length*u);Vec3 desired=b.Point+anchors[i]-a.Point;Vec3 delta=Vec3.ClampLength((desired-anchors[i])*weights[i],maximumOffset);offsetState[i]+=Vec3.ClampLength((delta-offsetState[i])*(1-Math.Exp(-12*dt)),.24*dt);Targets[i]=anchors[i]+offsetState[i];MaximumTargetOffset=Math.Max(MaximumTargetOffset,offsetState[i].Length);}
  }
 }
}
