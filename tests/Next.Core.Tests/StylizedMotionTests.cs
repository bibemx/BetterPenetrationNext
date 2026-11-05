using System;
using BetterPenetrationNext.Core;

static class StylizedMotionTests
{
    static readonly Linear3 Identity=new Linear3(new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1));
    static void Require(bool condition,string message){if(!condition)throw new Exception(message);}
    static void Near(Vec3 a,Vec3 b,double epsilon,string message){Require((a-b).Length<=epsilon,message+": "+(a-b).Length);}
    static Linear3 Frame(Vec3 rotation){return new Linear3(RigidOrganPose.Rotate(Identity.X,rotation),RigidOrganPose.Rotate(Identity.Y,rotation),RigidOrganPose.Rotate(Identity.Z,rotation));}
    public static void Run(Action<string,Action> test)
    {
        test("Stylized motion saturates and bounds every rigid point",()=>{
            double previous=0;
            foreach(double pressure in new[]{0,.001,.01,.05,1.0,100})
            {
                Vec3 translation,rotation;StylizedUterusMotion.Target(Identity,new Vec3(0,0,.02),new Vec3(0,0,1),new Vec3(0,.4,1),pressure,.035,1,.09,out translation,out rotation);
                Require(translation.Y>=previous-1e-12&&translation.Y>=0,"Saturation reversed the primary motion");previous=translation.Y;
                Require(Math.Abs(translation.Z)<=translation.Y*.12000001&&translation.X==0,"Motion escaped its anatomical cone");
                Require(rotation.Length<=StylizedUterusMotion.MaximumTilt+1e-12,"Tilt exceeded its limit");
                var pose=StylizedUterusMotion.Pose(Identity,new Vec3(3,-2,7),translation,rotation);
                for(int i=0;i<200;i++){double z=1-2.0*i/199,angle=i*2.39996323;Vec3 p=pose.Pivot+new Vec3(Math.Sqrt(1-z*z)*Math.Cos(angle),z,Math.Sqrt(1-z*z)*Math.Sin(angle))*.09;Require((pose.Point(p)-p).Length<=.0350000001,"Rigid point exceeded the configured displacement cap");}
            }
            Require(previous<.035&&previous>.015,"Saturation collapsed the useful travel");
        });
        test("Moving pelvis transports organ response without a world-axis dependency",()=>{
            Vec3 t,r;StylizedUterusMotion.Target(Identity,new Vec3(0,.002,.01),new Vec3(0,0,1),new Vec3(0,.1,1),.02,.035,1,.07,out t,out r);
            Vec3 native=new Vec3(.01,.04,.02),pivot=new Vec3(.1,.2,.3);var reference=StylizedUterusMotion.Pose(Identity,pivot,t,r);
            for(int frame=0;frame<240;frame++)
            {
                var axes=Frame(new Vec3(.9*Math.Sin(frame*.03),frame*.013,.4*Math.Cos(frame*.05)));Vec3 offset=new Vec3(frame*.002,Math.Cos(frame*.04),-3);
                Vec3 translated,rotated;StylizedUterusMotion.Target(axes,axes.Multiply(new Vec3(0,.002,.01)),axes.Z,axes.Multiply(new Vec3(0,.1,1)),.02,.035,1,.07,out translated,out rotated);
                Near(translated,t,1e-12,"Animated pelvis changed local translation");Near(rotated,r,1e-12,"Animated pelvis changed local tilt");
                var posed=StylizedUterusMotion.Pose(axes,axes.Multiply(pivot)+offset,t,r);
                Near(posed.Point(axes.Multiply(native)+offset),axes.Multiply(reference.Point(native))+offset,1e-12,"Organ left its moving pelvis frame");
            }
        });
        test("Rigid organ pose preserves shape and has an exact inverse",()=>{
            var pose=new RigidOrganPose{Pivot=new Vec3(-42,.8,-115),Translation=new Vec3(.002,.025,-.003),Rotation=new Vec3(.03,-.04,.02)};
            Vec3 a=pose.Pivot+new Vec3(.01,.03,.02),b=pose.Pivot+new Vec3(-.02,.04,-.03);
            Require(Math.Abs((pose.Point(a)-pose.Point(b)).Length-(a-b).Length)<1e-12,"Rigid structure stretched");Near(pose.InversePoint(pose.Point(a)),a,1e-12,"Pose inverse lost the original point");
        });
        test("Stylized damping is independent of frame partition and has no overshoot",()=>{
            Vec3 target=new Vec3(0,.025,.002),a=new Vec3(),b=new Vec3();
            for(int i=0;i<60;i++)a=StylizedUterusMotion.Respond(a,target,12,1.0/60);
            for(int i=0;i<144;i++)b=StylizedUterusMotion.Respond(b,target,12,1.0/144);
            Near(a,b,1e-14,"Frame partition changed the response");Require(a.Y>=0&&a.Y<=target.Y,"Contact response overshot");Near(StylizedUterusMotion.Respond(a,target,12,0),a,0,"Zero-time step changed pose");
        });
        test("Inactive rigid recovery follows the moving pelvis and finishes at 4.5 seconds",()=>{
            var phase=new ReceiverContactPhase();phase.Advance(true,1.0/60);phase.Advance(false,0);Vec3 pivot=new Vec3(.1,.2,.3),point=new Vec3(.02,.03,.04),translation=new Vec3(0,.03,.002),rotation=new Vec3(.04,0,0);
            double previous=1;
            for(int i=0;i<=270;i++)
            {
                if(i>0)phase.Advance(false,1.0/60);double weight=phase.Weight(4.5);Require(weight<=previous+1e-12,"Recovery weight increased");previous=weight;
                var axes=Frame(new Vec3(i*.01,0,i*.003));var pose=StylizedUterusMotion.Pose(axes,axes.Multiply(pivot),translation*weight,rotation*weight);
                Vec3 p=axes.Multiply(point);Near(pose.InversePoint(pose.Point(p)),p,1e-12,"Recovery distorted the rigid group");
            }
            Require(previous<1e-12,"Recovery did not finish");
        });
        test("No pressure and disabled travel do not activate stylized motion",()=>{
            foreach(var values in new[]{new[]{0.0,.035,1.0},new[]{.04,0.0,1.0},new[]{.04,.035,0.0}}){Vec3 t,r;StylizedUterusMotion.Target(Identity,new Vec3(0,0,1),Identity.Z,Identity.Y,values[0],values[1],values[2],.09,out t,out r);Near(t,new Vec3(),0,"Unloaded translation");Near(r,new Vec3(),0,"Unloaded rotation");}
        });
        test("Stylized motion reuses value buffers without per-step allocations",()=>{
            Vec3 t,r;for(int i=0;i<100;i++)StylizedUterusMotion.Target(Identity,new Vec3(),Identity.Z,Identity.Y,.02,.035,1,.09,out t,out r);
            long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<1000;i++)StylizedUterusMotion.Target(Identity,new Vec3(),Identity.Z,Identity.Y,.02,.035,1,.09,out t,out r);Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Motion update allocated");
        });
    }
}
