using System;
namespace BetterPenetrationNext.Core
{
 // A labelled initialization homotopy, not an approximation used in acceptance
 // measurements. Once Ready, contacts use the complete original source mesh.
 public sealed class ReceiverPreparation
 {
  readonly double[] coordinate;readonly Vec3[] controls;readonly Vec3[] proxy;
  double elapsed;
  public bool Ready {get;private set;}
  public double Inflation {get;private set;}
  public AnatomicalPathConstraintSystem Path {get;private set;}
  public Vec3[] SourceOverride {get{return Ready?null:proxy;}}
  public double CurveSeconds=1.5,InflationSeconds=1.5;
  public ReceiverPreparation(Vec3[] sourceReference,Vec3 axis,int sections=17)
  {
   axis=axis.Normalized;coordinate=new double[sourceReference.Length];proxy=new Vec3[sourceReference.Length];controls=new Vec3[sections];double minimum=double.PositiveInfinity,maximum=double.NegativeInfinity;for(int i=0;i<coordinate.Length;i++){coordinate[i]=Vec3.Dot(sourceReference[i],axis);minimum=Math.Min(minimum,coordinate[i]);maximum=Math.Max(maximum,coordinate[i]);}if(maximum-minimum<1e-8)throw new ArgumentException("Collapsed material source axis");for(int i=0;i<coordinate.Length;i++)coordinate[i]=(coordinate[i]-minimum)/(maximum-minimum);
  }
  static double Ease(double t){t=Math.Max(0,Math.Min(1,t));return t*t*(3-2*t);}
  public void Capture(AnatomicalPathConstraintSystem reference,AnatomicalPathConstraintSystem target,Vec3[] actualSource,double dt,double endpointClearance)
  {
   elapsed+=dt;Ready=elapsed>=CurveSeconds+InflationSeconds;double curve=Ease(elapsed/CurveSeconds),inflate=Ease((elapsed-CurveSeconds)/InflationSeconds);Inflation=inflate;
   for(int i=0;i<controls.Length;i++){double u=(double)i/(controls.Length-1);controls[i]=reference.AtDistance(reference.Length*u).Point*(1-curve)+target.AtDistance(target.Length*u).Point*curve;}if(Path==null)Path=new AnatomicalPathConstraintSystem(controls,true);else Path.UpdateControls(controls);if(Ready)return;
   double pad=Math.Min(Path.Length*.2,Math.Max(.002,endpointClearance));for(int i=0;i<proxy.Length;i++){Vec3 axisPoint=Path.AtDistance(pad+coordinate[i]*Math.Max(0,Path.Length-2*pad)).Point;proxy[i]=axisPoint+(actualSource[i]-axisPoint)*inflate;}
  }
 }
}
