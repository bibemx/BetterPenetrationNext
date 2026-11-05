using System;
namespace BetterPenetrationNext.Core
{
    // Angular guidance about a fixed attachment, independent of local contact
    // deformation. Inputs live in the current parent frame, avoiding world-pose lag.
    public sealed class EntryTargetGuidance
    {
        public Vec3 RotationVector {get;private set;}
        public double RequestedAngle {get;private set;}
        public void Reset(){RotationVector=new Vec3();RequestedAngle=0;}
        public void Advance(Vec3 forward,Vec3 direction,double dt,double maxAngle,double maxSpeed,double response)
        {
            forward=forward.Normalized;direction=direction.Normalized;if(forward.LengthSquared<.5||direction.LengthSquared<.5){Reset();return;}
            double dot=Math.Max(-1,Math.Min(1,Vec3.Dot(forward,direction)));RequestedAngle=Math.Acos(dot);Vec3 axis=Vec3.Cross(forward,direction);
            if(axis.LengthSquared<1e-20&&dot<0)axis=Vec3.Cross(forward,Math.Abs(forward.X)<.8?new Vec3(1,0,0):new Vec3(0,1,0));
            Vec3 target=axis.Normalized*Math.Min(Math.Max(0,maxAngle),RequestedAngle);double alpha=1-Math.Exp(-Math.Max(0,response)*Math.Max(0,dt));
            RotationVector=Vec3.ClampLength(RotationVector+Vec3.ClampLength((target-RotationVector)*alpha,Math.Max(0,maxSpeed)*Math.Max(0,dt)),Math.Max(0,maxAngle));
        }
        public static Vec3 Rotate(Vec3 v,Vec3 rotation)
        {double angle=rotation.Length;if(angle<1e-12)return v;Vec3 n=rotation/angle;return v*Math.Cos(angle)+Vec3.Cross(n,v)*Math.Sin(angle)+n*(Vec3.Dot(n,v)*(1-Math.Cos(angle)));}
    }
}
