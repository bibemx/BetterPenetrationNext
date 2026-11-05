using System;
namespace BetterPenetrationNext.Core
{
    public struct RigidOrganPose
    {
        public Vec3 Pivot,Translation,Rotation;
        public static Vec3 Rotate(Vec3 p,Vec3 rotation)
        {
            double angle=rotation.Length;if(angle<1e-14)return p;Vec3 axis=rotation/angle;
            double c=Math.Cos(angle);return p*c+Vec3.Cross(axis,p)*Math.Sin(angle)+axis*(Vec3.Dot(axis,p)*(1-c));
        }
        public Vec3 Point(Vec3 p){return Pivot+Rotate(p-Pivot,Rotation)+Translation;}
        public Vec3 InversePoint(Vec3 p){return Pivot+Rotate(p-Pivot-Translation,-Rotation);}
        public Vec3 Direction(Vec3 p){return Rotate(p,Rotation);}
    }
    // Art-directed kinematic response, explicitly NOT a physiological model.
    // The anatomical pelvis frame is updated from the animated skeleton.
    public static class StylizedUterusMotion
    {
        public const double MaximumTilt=5*Math.PI/180,ForeAftRatio=.12;
        public static Vec3 ToLocal(Linear3 pelvis,Vec3 world)
        {return new Vec3(Vec3.Dot(world,pelvis.X),Vec3.Dot(world,pelvis.Y),Vec3.Dot(world,pelvis.Z));}
        public static Vec3 Respond(Vec3 current,Vec3 target,double response,double dt)
        {return current+(target-current)*(1-Math.Exp(-Math.Max(0,response)*Math.Max(0,dt)));}
        public static RigidOrganPose Pose(Linear3 pelvis,Vec3 pivot,Vec3 localTranslation,Vec3 localRotation)
        {return new RigidOrganPose{Pivot=pivot,Translation=pelvis.Multiply(localTranslation),Rotation=pelvis.Multiply(localRotation)};}
        public static void Target(Linear3 pelvis,Vec3 tipOffset,Vec3 nativeNormal,Vec3 incoming,double pressure,double maximum,double strength,double groupRadius,out Vec3 localTranslation,out Vec3 localRotation)
        {
            localTranslation=localRotation=new Vec3();if(maximum<=0||pressure<=0||strength<=0)return;
            double load=1-Math.Exp(-Math.Max(0,pressure)*Math.Max(0,strength)/Math.Max(1e-8,maximum));
            double z=Vec3.Dot(tipOffset,pelvis.Z),up=maximum*load;
            localTranslation=new Vec3(0,up,up*ForeAftRatio*Math.Tanh(z/Math.Max(1e-8,maximum)));
            Vec3 a=nativeNormal.Normalized,b=incoming.Normalized,cross=Vec3.Cross(a,b);double angle=Math.Acos(Math.Max(-1,Math.Min(1,Vec3.Dot(a,b))));
            Vec3 world=cross.Normalized*Math.Min(MaximumTilt*load,angle);
            localRotation=ToLocal(pelvis,world);
            Limit(maximum,groupRadius,ref localTranslation,ref localRotation);
        }
        public static void Limit(double maximum,double groupRadius,ref Vec3 translation,ref Vec3 rotation)
        {
            // |T| + r*angle bounds EVERY point's displacement under the rigid
            // pose; scaling both retains shape and respects the existing cap.
            double bound=translation.Length+Math.Max(0,groupRadius)*rotation.Length;
            if(bound>maximum&&bound>0){double scale=Math.Max(0,maximum)/bound;translation*=scale;rotation*=scale;}
        }
    }
}
