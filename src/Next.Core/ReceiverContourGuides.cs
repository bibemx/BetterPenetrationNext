using System;
namespace BetterPenetrationNext.Core
{
 [Serializable]public sealed class ReceiverContour {public int[] Vertices;public double[] Weights;public double Coordinate;}
 [Serializable]public sealed class ReceiverGuideDefinition {public ReceiverContour[] Contours;public int[][] Boundaries;}
 // Centre and open-end plane motors constrain a guide, while the circumferential
 // degrees of freedom remain available to the common contact/strain solver.
 public sealed class ReceiverContourGuides
 {
  readonly ReceiverGuideDefinition definition;readonly Vec3[] targets;
  Vec3 start,end,startNormal,endNormal;
  public SourceContactBody EntryTranslationSource;
  public double Compliance=1e-7;
  public ReceiverContourGuides(ReceiverGuideDefinition definition){this.definition=definition;targets=new Vec3[definition.Contours.Length];}
  public void Capture(AnatomicalPathConstraintSystem path)
  {
   for(int k=0;k<targets.Length;k++)targets[k]=path.AtDistance(path.Length*definition.Contours[k].Coordinate).Point;
   var a=path.AtDistance(0);var b=path.AtDistance(path.Length);start=a.Point;end=b.Point;startNormal=a.Tangent;endNormal=b.Tangent;
  }
  public void Project(Vec3[] positions,double[] inverseMass,double h)
  {
   double alpha=Compliance/(h*h);
   Vec3 motion=EntryTranslationSource==null?new Vec3():EntryTranslationSource.Offset;
   for(int k=0;k<targets.Length;k++){var c=definition.Contours[k];Vec3 centre=new Vec3();double denom=alpha;for(int j=0;j<c.Vertices.Length;j++){int i=c.Vertices[j];double w=c.Weights[j];centre+=positions[i]*w;denom+=inverseMass[i]*w*w;}if(denom<1e-30)continue;double u=Math.Max(0,Math.Min(1,c.Coordinate)),weight=1-u*u*(3-2*u);Vec3 correction=(targets[k]+motion*weight-centre)/denom;for(int j=0;j<c.Vertices.Length;j++){int i=c.Vertices[j];positions[i]+=correction*(inverseMass[i]*c.Weights[j]);}}
   for(int k=0;k<definition.Boundaries.Length;k++){Vec3 point=k==0?start+motion:end,n=k==0?startNormal:endNormal;foreach(int i in definition.Boundaries[k])if(inverseMass[i]>0)positions[i]-=n*(Vec3.Dot(positions[i]-point,n)/(1+alpha));}
  }
 }
}
