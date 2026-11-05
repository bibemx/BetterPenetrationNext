using System;
namespace BetterPenetrationNext.Core
{
 public interface ISourceDeformation { Vec3 Map(Vec3 animated); double MaximumDisplacement { get; } Bounds3 DisplacementBounds { get; } }
 // A neutral kinematic shear guide. Its cross sections retain their shape;
 // centres follow an external 3D route. It is not a constitutive tissue law.
 // Root attachment remains fixed by the guide. Contact can add bounded rigid
 // translation afterwards. Actual render vertices and contact vertices share Map.
 public sealed class SourceRodGuide : ISourceDeformation
 {
  public double Parameter { get; private set; }=-1;
  AnatomicalPathConstraintSystem previousPath;
  readonly Vec3[] target,centres;
  Vec3 root,forward;double length,maximum;bool initialized;
  public double MaximumDisplacement { get { return maximum; } }
  public Bounds3 DisplacementBounds { get; private set; }=Bounds3.Empty;
  public SourceRodGuide(int sections=24){if(sections<2||sections>128)throw new ArgumentException("Invalid rod resolution");target=new Vec3[sections+1];centres=new Vec3[target.Length];}
  public void Reset(){initialized=false;Parameter=-1;previousPath=null;DisplacementBounds=Bounds3.Empty;}
  public void Capture(AnatomicalPathConstraintSystem path,Vec3 animatedRoot,Vec3 animatedTip,double dt,double maximumDisplacement,double response=12){
   if(path==null||!animatedRoot.IsFinite||!animatedTip.IsFinite||!Vec3.Finite(dt)||dt<0||!Vec3.Finite(maximumDisplacement)||maximumDisplacement<0||!Vec3.Finite(response)||response<0)throw new ArgumentException("Invalid rod guide");
   if(!ReferenceEquals(path,previousPath)){Reset();previousPath=path;}
   root=animatedRoot;forward=(animatedTip-root).Normalized;length=(animatedTip-root).Length;maximum=maximumDisplacement;
   if(length<1e-8)throw new ArgumentException("Collapsed animated rod");
   var end=path.Closest(animatedTip,Parameter,.15);Parameter=end.Parameter;double depth=path.DistanceAtParameter(end.Parameter);Vec3 entry=path[0];double outside=(entry-root).Length,total=outside+depth;
   double blend=1-Math.Exp(-response*dt);
   for(int i=0;i<centres.Length;i++){
    double u=(double)i/(centres.Length-1),arc=u*total;Vec3 p=arc<=outside&&outside>1e-12?root+(entry-root)*(arc/outside):path.AtDistance(arc-outside).Point;
    Vec3 animated=root+forward*(u*length);target[i]=Vec3.ClampLength(p-animated,maximum);
    // Filter displacements relative to each frame's fresh animation, never
    // the previous corrected mesh. This survives moving animation baselines.
    if(!initialized)centres[i]=new Vec3();centres[i]=Vec3.ClampLength(centres[i]+(target[i]-centres[i])*blend,maximum);
   }
   centres[0]=new Vec3();
   // Parallel section planes give a rank-one shear map. A positive advance
   // along the animated axis proves local injectivity, even if the prescribed
   // bend radius is smaller than the cross section (rotated disks would fold).
   double scale=1,stepLength=length/(centres.Length-1);
   for(int i=1;i<centres.Length;i++){double advance=Vec3.Dot(centres[i]-centres[i-1],forward);if(advance<0)scale=Math.Min(scale,.95*stepLength/-advance);}
   if(scale<1)for(int i=0;i<centres.Length;i++)centres[i]*=scale;
   initialized=true;
   DisplacementBounds=Bounds3.Empty;
  }
  public void MapVertices(Vec3[] animated,Vec3[] output){
   if(animated.Length!=output.Length)throw new ArgumentException("Source vertex count differs");
   var bounds=Bounds3.Empty;
   for(int i=0;i<animated.Length;i++){output[i]=Map(animated[i]);bounds.Include(output[i]-animated[i]);}
   DisplacementBounds=bounds;
  }
  public Vec3 Map(Vec3 p){
   if(!initialized)return p;
   double s=Vec3.Dot(p-root,forward),u=Math.Max(0,Math.Min(1,s/length))*(centres.Length-1);int a=Math.Min(centres.Length-2,(int)u);double t=u-a;
   Vec3 displacement=centres[a]*(1-t)+centres[a+1]*t;
   return p+displacement;
  }
 }
}
