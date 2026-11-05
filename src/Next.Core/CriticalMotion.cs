using System;
namespace BetterPenetrationNext.Core
{
    // Exact critically damped tracking of a constant target over one interval.
    // State and velocity live in an animated local frame, never world space.
    public static class CriticalMotion
    {
        public static void Advance(ref Vec3 position,ref Vec3 velocity,Vec3 target,double response,double dt,double speed=double.PositiveInfinity)
        {
            if(dt<=0||response<=0)return;
            Vec3 previous=position,error=position-target,c=velocity+error*response;double decay=Math.Exp(-response*dt);
            Vec3 next=target+(error+c*dt)*decay,nextVelocity=(velocity-c*(response*dt))*decay;
            double maximum=Math.Max(0,speed)*dt;Vec3 change=next-previous;
            if(change.Length>maximum){change=Vec3.ClampLength(change,maximum);next=previous+change;nextVelocity=change/dt;}
            position=next;velocity=Vec3.ClampLength(nextVelocity,Math.Max(0,speed));
        }
    }
}
