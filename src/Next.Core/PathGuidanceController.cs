using System;
namespace BetterPenetrationNext.Core
{
    public struct GuidancePose { public Vec3 Translation,RotationVector,Pivot; public double Parameter,Distance; }
    // Independent motion controller. The contact solver never selects a route.
    // A caller supplies a 3D path and an animated pose; contact is applied later.
    public sealed class PathGuidanceController
    {
        public Vec3 Translation { get; private set; }
        public Vec3 RotationVector { get; private set; }
        public double Parameter { get; private set; }=-1;
        AnatomicalPathConstraintSystem lastPath;
        public void Reset() { Translation=RotationVector=new Vec3(); Parameter=-1;lastPath=null; }
        public GuidancePose Step(AnatomicalPathConstraintSystem path,Vec3 animatedPoint,Vec3 forward,double dt,double maximumTranslation,double maximumAngle=.35,double response=12,double angularSpeed=2,bool allowReverse=true)
        {
            if(path==null||!animatedPoint.IsFinite||!forward.IsFinite||forward.LengthSquared<1e-20||!Vec3.Finite(dt)||dt<0||!Vec3.Finite(maximumTranslation)||maximumTranslation<0||!Vec3.Finite(maximumAngle)||maximumAngle<0||!Vec3.Finite(response)||response<0||!Vec3.Finite(angularSpeed)||angularSpeed<0) throw new ArgumentException("Invalid path guidance input");
            if(!ReferenceEquals(lastPath,path)){Reset();lastPath=path;}
            var sample=path.Closest(animatedPoint+Translation,Parameter,.15); Parameter=sample.Parameter;
            double blend=1-Math.Exp(-response*dt);
            Translation=Vec3.ClampLength(Translation+(sample.Point-animatedPoint-Translation)*blend,maximumTranslation);
            Vec3 f=forward.Normalized,t=sample.Tangent; double dot=Math.Max(-1,Math.Min(1,Vec3.Dot(f,t)));
            if(allowReverse&&dot<0) {t=-t;dot=-dot;}
            Vec3 axis=Vec3.Cross(f,t);
            if(axis.LengthSquared<1e-20&&dot<0) axis=Vec3.Cross(f,Math.Abs(f.X)<.8?new Vec3(1,0,0):new Vec3(0,1,0));
            Vec3 desired=axis.Normalized*Math.Min(maximumAngle,Math.Acos(dot));
            RotationVector+=Vec3.ClampLength((desired-RotationVector)*blend,angularSpeed*dt);
            RotationVector=Vec3.ClampLength(RotationVector,maximumAngle);
            return new GuidancePose {Translation=Translation,RotationVector=RotationVector,Pivot=animatedPoint,Parameter=Parameter,Distance=sample.Distance};
        }
        public static void Apply(SourceContactBody source,GuidancePose pose) {source.ApplyGuidance(pose.Translation,pose.RotationVector,pose.Pivot);}
    }
}
