using System;
namespace BetterPenetrationNext.Core
{
    // A bounded set of finite surface cuts, including the short approach-side
    // segment outside R0. It is not a capped cylinder or a maximum-radius box.
    public sealed class EntranceContactProxy
    {
        public const int Slices=6;
        public readonly double[] Radii=new double[Slices*ReceiverRadialField.Directions];
        public Vec3 Centre,Axis,Side;public double Depth;public bool Active;
        double maximumRadius;
        public void RefreshBounds(){maximumRadius=0;foreach(double r in Radii)maximumRadius=Math.Max(maximumRadius,r);}
        public double Radius(double axial,double angle)
        {
            if(!Active||Depth<=0||axial<-Depth||axial>Depth*.3)return 0;
            double coordinate=(axial/Depth+1)*(Slices-1)/1.3;int a=Math.Min(Slices-2,Math.Max(0,(int)coordinate)),b=a+1;double t=Math.Max(0,Math.Min(1,coordinate-a));
            double angular=angle*ReceiverRadialField.Directions/(2*Math.PI);angular-=Math.Floor(angular/ReceiverRadialField.Directions)*ReceiverRadialField.Directions;int x=Math.Min(ReceiverRadialField.Directions-1,(int)angular),y=(x+1)%ReceiverRadialField.Directions;double u=angular-x;
            double first=Radii[a*ReceiverRadialField.Directions+x]*(1-u)+Radii[a*ReceiverRadialField.Directions+y]*u,second=Radii[b*ReceiverRadialField.Directions+x]*(1-u)+Radii[b*ReceiverRadialField.Directions+y]*u;
            return first*(1-t)+second*t;
        }
        public Vec3 Correction(Vec3 point,double strength)
        {
            if(!Active||Depth<=0)return new Vec3();Vec3 q=point-Centre;double axial=Vec3.Dot(q,Axis);if(axial<-Depth||axial>Depth*.3)return new Vec3();q-=Axis*axial;
            if(maximumRadius>0&&q.LengthSquared>=maximumRadius*maximumRadius*strength*strength)return new Vec3();double radius=q.Length;if(radius<1e-12)return new Vec3();
            double angle=Math.Atan2(Vec3.Dot(q,Vec3.Cross(Axis,Side)),Vec3.Dot(q,Side)),wanted=Radius(axial,angle)*strength;
            return wanted>radius?q*((wanted-radius)/radius):new Vec3();
        }
    }
}
