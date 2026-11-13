using System;
using BetterPenetrationNext.Core;
static class SurfaceFairingTests
{
    static void Check(bool ok,string message){if(!ok)throw new Exception(message);}
    public static void Run(Action<string,Action> test)
    {
        var rest=new[]{new Vec3(0,0,0),new Vec3(1,0,0),new Vec3(2,0,0),new Vec3(3,0,0),new Vec3(4,0,0)};var edges=new[]{new EdgeConstraint(0,1),new EdgeConstraint(1,2),new EdgeConstraint(2,3),new EdgeConstraint(3,4)};var identity=new[]{0,1,2,3,4};var mobility=new[]{0.0,1,1,1,0};
        test("Surface fairing reduces an isolated correction spike while retaining both boundary pins",()=>{
            var s=new SurfaceOffsetFairing(rest,edges,identity,mobility);var q=new[]{new Vec3(),new Vec3(),new Vec3(0,.01,0),new Vec3(),new Vec3()};s.Apply(q,4,.35);Check(q[2].Y<.006&&q[1].Y>0&&q[3].Y>0,"Correction was not distributed smoothly");Check(q[0].Length==0&&q[4].Length==0,"Boundary position changed");
        });
        test("Surface fairing preserves a uniform correction and leaves the authored rest surface unchanged",()=>{
            var s=new SurfaceOffsetFairing(rest,edges,identity,mobility);var q=new Vec3[5];for(int i=0;i<q.Length;i++)q[i]=new Vec3(.01,.02,.03);s.Apply(q,4,.35);for(int i=0;i<q.Length;i++){Check((q[i]-new Vec3(.01,.02,.03)).Length<1e-12,"Uniform motion shrank");Check(rest[i].X==i&&rest[i].Y==0,"Rest mesh was modified");}
        });
        test("Surface fairing reuses its cached graph and does not merge independent coincident nodes",()=>{
            var r=new[]{new Vec3(),new Vec3(),new Vec3(1,0,0)};var s=new SurfaceOffsetFairing(r,new[]{new EdgeConstraint(0,2)},new[]{0,1,2},new[]{1.0,1,0});var q=new[]{new Vec3(0,.01,0),new Vec3(0,.07,0),new Vec3()};s.Apply(q,4,.35);Check(q[1].Y==.07,"Unconnected skin identity was merged");long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<100;i++)s.Apply(q,4,.35);Check(GC.GetAllocatedBytesForCurrentThread()-before==0,"Hot fairing allocated buffers");
        });
    }
}
