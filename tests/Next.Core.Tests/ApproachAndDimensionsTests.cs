using System;
using BetterPenetrationNext.Core;
static class ApproachAndDimensionsTests
{
    static void Require(bool value,string message){if(!value)throw new Exception(message);}
    static MeshData Pipe()
    {
        const int rings=25,sides=32,stride=33;var p=new Vec3[rings*stride];var t=new int[(rings-1)*sides*6];int f=0;
        for(int s=0;s<rings;s++)for(int k=0;k<stride;k++){double a=k*Math.PI*2/sides;p[s*stride+k]=new Vec3(.2*Math.Cos(a),.1*Math.Sin(a),s/12.0);}
        for(int s=0;s<rings-1;s++)for(int k=0;k<sides;k++){int a=s*stride+k,b=a+1,c=a+stride,d=c+1;t[f++]=a;t[f++]=b;t[f++]=c;t[f++]=b;t[f++]=d;t[f++]=c;}return new MeshData{Vertices=p,Triangles=t};
    }
    public static void Run(Action<string,Action> test)
    {
        test("Saved source dimensions apply on the first loaded pose without contact",()=>{
            var m=Pipe();var s=new SourceDimensions(new SourceSectionRig(m.Vertices,m.Triangles,new Vec3(),new Vec3(0,0,1)),.7,.9);var output=new Vec3[m.Vertices.Length];
            s.Apply(m.Vertices,new Vec3(),.7,.9,0,12,output);Require(Math.Abs(output[output.Length-1].Z-1.4)<1e-10&&Math.Abs(output[24*33].X-.18)<1e-10,"Saved dimensions waited for contact or transition warm-up");
            for(int i=0;i<33;i++)Require((output[i]-m.Vertices[i]).Length==0,"Preload detached the root");
            s.Apply(m.Vertices,new Vec3(),.7,.9,1.0/60,12,output);Require(Math.Abs(output[output.Length-1].Z-1.4)<1e-10,"Inactive-to-active handoff resized the source twice");
        });
        test("Terminal approach stops with continuous velocity and acceleration",()=>{
            const double end=1,band=.02,h=1e-6;double previous=-1;
            for(int i=0;i<=1000;i++){double free=.94+i*.00012,x=SmoothTerminalApproach.Distance(free,end,band);Require(x>=previous-1e-12&&x<=end,"Approach reversed or overshot the terminal");previous=x;}
            foreach(double boundary in new[]{end-band,end+band})
            {
                Func<double,double> f=x=>SmoothTerminalApproach.Distance(x,end,band);double left=(f(boundary)-f(boundary-h))/h,right=(f(boundary+h)-f(boundary))/h;
                Require(Math.Abs(left-right)<1e-7,"Terminal velocity jumps at the approach boundary");double a=(f(boundary+h)-2*f(boundary)+f(boundary-h))/(h*h);Require(Math.Abs(a)<.01,"Terminal acceleration jumps at the approach boundary");
            }
            Require(SmoothTerminalApproach.Distance(.9,end,band)==.9&&SmoothTerminalApproach.Distance(1.1,end,band)==1,"Free path or final contact position changed");
        });
        test("Critical pose response preserves velocity across a changing target",()=>{
            Vec3 x=new Vec3(),v=new Vec3(),a=new Vec3(),b=new Vec3(),va=new Vec3(),vb=new Vec3();
            for(int i=0;i<60;i++)CriticalMotion.Advance(ref a,ref va,new Vec3(0,.03,0),12,1.0/60);
            for(int i=0;i<144;i++)CriticalMotion.Advance(ref b,ref vb,new Vec3(0,.03,0),12,1.0/144);
            Require((a-b).Length<1e-14&&(va-vb).Length<1e-14,"Critical response depends on frame partition");
            CriticalMotion.Advance(ref x,ref v,new Vec3(0,.03,0),12,1e-6);Require(x.Length<1e-10&&v.Length<1e-4,"A target step produced an instantaneous position jump");
        });
        test("Manual dimensions change real coordinates and preserve the attached base",()=>{
            var m=Pipe();var sections=new SourceSectionRig(m.Vertices,m.Triangles,new Vec3(),new Vec3(0,0,1));var size=new SourceDimensions(sections);var output=new Vec3[m.Vertices.Length];
            for(int i=0;i<180;i++)size.Apply(m.Vertices,new Vec3(),1.2,.8,1.0/60,12,output);
            Require(Math.Abs(size.EffectiveLength/size.NativeLength-1.2)<1e-10&&Math.Abs(size.EffectiveWidth/size.NativeWidth-.8)<1e-10,"Manual dimensions did not reach requested proportions");
            for(int i=0;i<33;i++)Require((output[i]-m.Vertices[i]).Length==0,"Attached root ring moved");Require(Math.Abs(output[output.Length-1].Z-2.4)<1e-9,"Actual tip length did not change");
            Require(Math.Abs(output[24*33].X-.16)<1e-9,"Actual terminal width did not change");
        });
        test("Dimension overrides preserve independent seam identities and do not accumulate",()=>{
            var m=Pipe();var refMesh=(Vec3[])m.Vertices.Clone();var s=new SourceDimensions(new SourceSectionRig(refMesh,m.Triangles,new Vec3(),new Vec3(0,0,1)));var output=new Vec3[m.Vertices.Length];m.Vertices[12*33+32]+=new Vec3(0,.04,0);
            for(int i=0;i<180;i++)s.Apply(m.Vertices,new Vec3(),1.1,1.2,1.0/60,12,output);var accepted=(Vec3[])output.Clone();
            Require((output[12*33]-output[12*33+32]).Length>.03,"Independent seam animation was welded");
            for(int i=0;i<180;i++)s.Apply(m.Vertices,new Vec3(),1.1,1.2,1.0/60,12,output);for(int i=0;i<output.Length;i++)Require((output[i]-accepted[i]).Length<1e-9,"Sizing accumulated displacement on its own previous output");
            for(int i=0;i<300;i++)s.Apply(m.Vertices,new Vec3(),1,1,1.0/60,12,output);for(int i=0;i<output.Length;i++)Require((output[i]-m.Vertices[i]).Length<1e-10,"Card dimensions were not restored");
        });
        test("Animated dimension overrides allocate no per-frame buffers",()=>{
            var m=Pipe();var s=new SourceDimensions(new SourceSectionRig(m.Vertices,m.Triangles,new Vec3(),new Vec3(0,0,1)));var output=new Vec3[m.Vertices.Length];
            for(int i=0;i<20;i++)s.Apply(m.Vertices,new Vec3(),1.1,1.2,1.0/60,12,output);long before=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<60;i++)s.Apply(m.Vertices,new Vec3(),1.1,1.2,1.0/60,12,output);Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Sizing rebuilt structures during animation");
        });
    }
}
