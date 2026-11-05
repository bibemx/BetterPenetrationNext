using System;
using System.Diagnostics;
using System.IO;
using System.Text.Json;
using BetterPenetrationNext.Core;

internal static class LandmarkTests
{
    static void Require(bool value,string message) { if(!value) throw new Exception(message); }
    static void Near(Vec3 actual,Vec3 expected,double tolerance,string message)
    { Require((actual-expected).Length<=tolerance,message+": "+actual+" / "+expected); }

    // An uncapped, asymmetric engineering pipe with independently indexed seam vertices.
    static MeshData Pipe(int rings=13,int sides=32,bool splitSeam=false)
    {
        int stride=sides+(splitSeam?1:0);
        var p=new Vec3[rings*stride];var faces=new int[(rings-1)*sides*6];int f=0;
        for(int r=0;r<rings;r++) for(int s=0;s<stride;s++)
        {
            double angle=2*Math.PI*s/sides,z=2.0*r/(rings-1);
            p[r*stride+s]=new Vec3(.2*Math.Cos(angle),.1*Math.Sin(angle),z);
        }
        for(int r=0;r<rings-1;r++) for(int s=0;s<sides;s++)
        {
            int a=r*stride+s,b=r*stride+(splitSeam?s+1:(s+1)%sides),c=a+stride,d=b+stride;
            faces[f++]=a;faces[f++]=b;faces[f++]=c;faces[f++]=b;faces[f++]=d;faces[f++]=c;
        }
        return new MeshData {Vertices=p,Triangles=faces};
    }
    static Vec3 Rotate(Vec3 v) { return new Vec3(v.Z,v.X,v.Y); }
    static void ExpectArgument(Action body)
    { bool rejected=false;try { body(); }catch(ArgumentException) { rejected=true; }Require(rejected,"Invalid input was accepted"); }
    static int Distinct(Vec3[] values)
    {
        int n=0;for(int i=0;i<values.Length;i++){bool seen=false;for(int j=0;j<i;j++)if((values[i]-values[j]).Length<1e-9)seen=true;if(!seen)n++;}return n;
    }

    public static void Run(Action<string,Action> test)
    {
        test("Connected landmark envelope keeps both open ends finite",()=>{
            var mesh=Pipe(splitSeam:true);var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));Vec3 normal;int span;double blend;
            Require(landmarks.Envelope.Penetration(new Vec3(0,0,1),0,false,out normal,out span,out blend)>.08,"Interior envelope occupancy missing");
            foreach(var p in new[]{new Vec3(0,0,-.01),new Vec3(0,0,2.01),new Vec3(.25,0,1),new Vec3(0,.15,1)})Require(landmarks.Envelope.Penetration(p,0,false,out normal,out span,out blend)==0,"Open envelope extrapolated beyond its geometry");
            Require(landmarks.Envelope.Penetration(new Vec3(0,0,.5),0,true,out normal,out span,out blend)==0,"Head-only query included the shaft");
        });
        test("Connected envelope distance remains continuous across material section boundaries",()=>{
            var mesh=Pipe();var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));Vec3 normal;int span;double blend;
            for(int s=1;s<landmarks.Sections.Length-1;s++)
            {
                double z=landmarks.Sections[s].Centre.Z;
                double a=landmarks.Envelope.Penetration(new Vec3(.08,.02,z-1e-7),0,false,out normal,out span,out blend),b=landmarks.Envelope.Penetration(new Vec3(.08,.02,z+1e-7),0,false,out normal,out span,out blend);
                Require(a>.02&&b>.02&&Math.Abs(a-b)<1e-6,"A cached span boundary created a contact gap");
            }
        });
        test("Envelope refit follows current animation and reuses all buffers",()=>{
            var mesh=Pipe(splitSeam:true);var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));var pose=(Vec3[])mesh.Vertices.Clone();Vec3 normal;int span;double blend;
            for(int i=0;i<pose.Length;i++)pose[i]=Rotate(pose[i])+new Vec3(3,4,5);
            landmarks.Update(pose,new Vec3(0,1,0));double depth=landmarks.Envelope.Penetration(Rotate(new Vec3(0,0,1))+new Vec3(3,4,5),0,false,out normal,out span,out blend);Require(depth>.08,"Envelope used stale reference geometry");
            for(int i=0;i<10;i++){landmarks.Update(pose,new Vec3(0,1,0));landmarks.Envelope.Penetration(new Vec3(4,4,5),0,false,out normal,out span,out blend);}
            long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<50;i++){landmarks.Update(pose,new Vec3(0,1,0));landmarks.Envelope.Penetration(new Vec3(4,4,5),0,false,out normal,out span,out blend);}Require(GC.GetAllocatedBytesForCurrentThread()-before==0,"Envelope animation allocated per-frame structures");
        });
        test("Landmarks retain an open pipe and sample its actual elliptical surface",()=>{
            var mesh=Pipe();var points=(Vec3[])mesh.Vertices.Clone();var faces=(int[])mesh.Triangles.Clone();
            var landmarks=new AdaptiveMeshLandmarks(points,faces,new Vec3(0,0,1));
            Require(new SimulationTopology(points,faces).BoundaryEdges==64,"An open rim changed");
            foreach(var section in landmarks.Sections){Near(section.Centre,new Vec3(0,0,2*Math.Max(1e-6,Math.Min(1-1e-6,section.Coordinate))),1e-10,"Section centre");foreach(var p in section.Radial)Require(p.IsFinite&&Math.Abs(p.X)<=.20000001&&Math.Abs(p.Y)<=.10000001,"Radial point left the surface");}
            for(int i=0;i<points.Length;i++)Near(points[i],mesh.Vertices[i],0,"Input vertex mutation");
            for(int i=0;i<faces.Length;i++)Require(faces[i]==mesh.Triangles[i],"Input topology mutation");
        });
        test("Landmark transverse frames survive a guide parallel to the longitudinal axis",()=>{
            var mesh=Pipe();for(int i=0;i<mesh.Vertices.Length;i++){var p=mesh.Vertices[i];mesh.Vertices[i]=new Vec3(p.X,p.Z,-p.Y);}
            var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,1,0));
            landmarks.Update(mesh.Vertices,new Vec3(0,1,0));
            foreach(var section in landmarks.Sections)Require(Distinct(section.Radial)>=4,"Transverse radial directions collapsed");
        });
        test("Cached landmarks follow rigid rotations and large world translations",()=>{
            var mesh=Pipe();var a=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));
            var pose=new Vec3[mesh.Vertices.Length];var translation=new Vec3(100000,-200000,300000);
            for(int i=0;i<pose.Length;i++)pose[i]=Rotate(mesh.Vertices[i])+translation;
            var b=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));b.Update(pose,Rotate(new Vec3(1,0,0)));
            Near(b.Tip,Rotate(a.Tip)+translation,1e-8,"Rigid tip");Near(b.Base,Rotate(a.Base)+translation,1e-8,"Rigid base");
            for(int s=0;s<a.Sections.Length;s++){Near(b.Sections[s].Centre,Rotate(a.Sections[s].Centre)+translation,1e-8,"Rigid centre");for(int k=0;k<a.Sections[s].Radial.Length;k++)Near(b.Sections[s].Radial[k],Rotate(a.Sections[s].Radial[k])+translation,1e-8,"Rigid radial identity");}
        });
        test("Landmark bindings preserve distinct animation paths at coincident seams",()=>{
            var mesh=Pipe(splitSeam:true);var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));
            var pose=(Vec3[])mesh.Vertices.Clone();for(int r=0;r<13;r++)pose[r*33+32]+=new Vec3(0,.04,0);
            landmarks.Update(pose,new Vec3(1,0,0));bool separate=false;
            foreach(var section in landmarks.Sections)for(int i=0;i<section.Bindings.Length;i++){
                var binding=section.Bindings[i];Near(section.Surface[i],binding.Evaluate(pose),1e-12,"Cached skin identity");
                if((binding.A%33==32||binding.B%33==32)&&section.Surface[i].Y>.02)separate=true;
            }
            Require(separate,"Different seam animation paths were welded");
        });
        test("Animated surface extrema include a newly extended material vertex",()=>{
            var mesh=Pipe();var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));
            var pose=(Vec3[])mesh.Vertices.Clone();int changed=pose.Length-7;pose[changed]+=new Vec3(0,0,.4);
            landmarks.Update(pose,new Vec3(1,0,0));Near(landmarks.Tip,pose[changed],1e-12,"Animated tip ignored current surface");Require(landmarks.TipVertex==changed,"Animated tip identity");
        });
        test("Mechanical tip retains its surface identity when a bent flank becomes an extremum",()=>{
            var mesh=Pipe();var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));
            int bound=landmarks.MaterialTipVertex;var pose=(Vec3[])mesh.Vertices.Clone();int flank=pose.Length-7;pose[flank]+=new Vec3(0,0,.4);
            landmarks.Update(pose,new Vec3(1,0,0));Require(landmarks.TipVertex==flank&&landmarks.MaterialTipVertex==bound,"Diagnostics changed mechanical identity");
            Near(landmarks.MaterialTip,pose[bound],1e-12,"Mechanical tip jumped to a different vertex");
            for(int i=0;i<pose.Length;i++)pose[i]=Rotate(pose[i])+new Vec3(3,4,5);
            landmarks.Update(pose,new Vec3(0,1,0));Near(landmarks.MaterialTip,pose[bound],1e-12,"Animated material tip lost its binding");
        });
        test("Unused vertices do not determine the principal axis or extrema",()=>{
            var mesh=Pipe();var vertices=new Vec3[mesh.Vertices.Length+1];Array.Copy(mesh.Vertices,vertices,mesh.Vertices.Length);vertices[vertices.Length-1]=new Vec3(100000,10000,0);
            var landmarks=new AdaptiveMeshLandmarks(vertices,mesh.Triangles,new Vec3());
            Require(landmarks.Sections[landmarks.Sections.Length-1].Centre.Z-landmarks.Sections[0].Centre.Z>1.99,"Unused outlier changed principal axis");
            foreach(var p in landmarks.Extrema)Require(Math.Abs(p.X)<1,"Unused outlier became an extremum");
        });
        test("Landmark construction rejects malformed or nonfinite geometry",()=>{
            var mesh=Pipe();ExpectArgument(()=>new AdaptiveMeshLandmarks(mesh.Vertices,new int[0],new Vec3(0,0,1)));
            ExpectArgument(()=>new AdaptiveMeshLandmarks(mesh.Vertices,new[]{0,1},new Vec3(0,0,1)));
            ExpectArgument(()=>new AdaptiveMeshLandmarks(mesh.Vertices,new[]{0,1,mesh.Vertices.Length},new Vec3(0,0,1)));
            ExpectArgument(()=>new AdaptiveMeshLandmarks(mesh.Vertices,new[]{0,0,0},new Vec3(0,0,1)));
            ExpectArgument(()=>new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(double.NaN,0,0)));
            ExpectArgument(()=>new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1),16,1,33));
            ExpectArgument(()=>new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1),16,17,10));
            mesh.Vertices[0]=new Vec3(double.PositiveInfinity,0,0);ExpectArgument(()=>new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1)));
        });
        test("Landmark updates reject changed topology size and nonfinite frames",()=>{
            var mesh=Pipe();var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));
            ExpectArgument(()=>landmarks.Update(null,new Vec3(1,0,0)));ExpectArgument(()=>landmarks.Update(new Vec3[3],new Vec3(1,0,0)));
            ExpectArgument(()=>landmarks.Update(mesh.Vertices,new Vec3(double.NaN,0,0)));
            mesh.Vertices[0]=new Vec3(0,double.NaN,0);ExpectArgument(()=>landmarks.Update(mesh.Vertices,new Vec3(1,0,0)));
        });
        test("Collapsed animated sections remain finite without inventing surface extent",()=>{
            var mesh=Pipe();var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));
            var pose=new Vec3[mesh.Vertices.Length];for(int i=0;i<pose.Length;i++)pose[i]=new Vec3(1,2,3);
            landmarks.Update(pose,new Vec3());foreach(var section in landmarks.Sections){Require(section.Tangent.IsFinite,"Nonfinite collapsed tangent");foreach(var p in section.Radial)Near(p,pose[0],1e-12,"Invented collapsed extent");}
        });
        test("Four, eight and sixteen radial landmarks retain cached surface provenance",()=>{
            var mesh=Pipe();foreach(int radial in new[]{4,8,16}){
                var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1),radial);
                foreach(var section in landmarks.Sections){Require(section.Radial.Length==radial,"Wrong radial count");foreach(var p in section.Radial){bool found=false;foreach(var q in section.Surface)if((p-q).Length<1e-12)found=true;Require(found,"Point is not a sampled surface point");}}
            }
        });
        test("Repeated animated landmark evaluation neither recuts triangles nor accumulates drift",()=>{
            var mesh=Pipe();var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));int tests=landmarks.ReferenceTriangleTests;
            var pose=(Vec3[])mesh.Vertices.Clone();for(int frame=0;frame<1000;frame++){
                double offset=Math.Sin(frame*.1);for(int i=0;i<pose.Length;i++){var p=mesh.Vertices[i];pose[i]=p+new Vec3(offset*p.Z*p.Z,0,0);}
                landmarks.Update(pose,new Vec3(1,0,0));foreach(var section in landmarks.Sections)Require(section.Centre.IsFinite,"Animated centre nonfinite");
            }
            landmarks.Update(mesh.Vertices,new Vec3(1,0,0));Require(landmarks.ReferenceTriangleTests==tests,"Animation recut triangles");
            foreach(var section in landmarks.Sections)Require(Math.Abs(section.Centre.X)<1e-10,"Landmark drift");
        });
        test("Warm landmark updates allocate no managed memory",()=>{
            var mesh=Pipe();var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));
            for(int i=0;i<100;i++)landmarks.Update(mesh.Vertices,new Vec3(1,0,0));long before=GC.GetAllocatedBytesForCurrentThread();
            for(int i=0;i<300;i++)landmarks.Update(mesh.Vertices,new Vec3(1,0,0));Require(GC.GetAllocatedBytesForCurrentThread()==before,"Warm update allocated");
        });
    }

    public static void Benchmark(string output)
    {
        const int frames=6000;var mesh=Pipe(37,120,true);var pose=(Vec3[])mesh.Vertices.Clone();
        var watch=Stopwatch.StartNew();var landmarks=new AdaptiveMeshLandmarks(mesh.Vertices,mesh.Triangles,new Vec3(0,0,1));watch.Stop();double setup=watch.Elapsed.TotalMilliseconds;
        var indices=new int[mesh.Vertices.Length];for(int i=0;i<indices.Length;i++)indices[i]=i;
        var baseline=new LandmarkBenchmarkBaseline(landmarks,indices);
        var times=new double[frames];var originalTimes=new double[frames];var orientation=new Vec3(1,0,0);
        for(int i=0;i<2000;i++){baseline.Update(pose,orientation);landmarks.Update(pose,orientation);}
        long before=GC.GetAllocatedBytesForCurrentThread();int initialTriangles=landmarks.ReferenceTriangleTests;
        for(int f=0;f<frames;f++){
            // Alternate execution order, including the identical array animation
            // in both timed steps, to reduce order and background-load bias.
            for(int pass=0;pass<2;pass++){
                bool legacy=((f+pass)&1)==0;long start=Stopwatch.GetTimestamp();double bend=.02*Math.Sin(f*.02);
                for(int i=0;i<pose.Length;i++){var p=mesh.Vertices[i];pose[i]=p+new Vec3(bend*p.Z*p.Z,0,0);}
                if(legacy)baseline.Update(pose,orientation);else landmarks.Update(pose,orientation);
                double milliseconds=(Stopwatch.GetTimestamp()-start)*1000.0/Stopwatch.Frequency;
                if(legacy)originalTimes[f]=milliseconds;else times[f]=milliseconds;
            }
        }
        long allocated=GC.GetAllocatedBytesForCurrentThread()-before;Array.Sort(times);Array.Sort(originalTimes);double total=0,originalTotal=0;
        foreach(var ms in times)total+=ms;foreach(var ms in originalTimes)originalTotal+=ms;int p95=(int)Math.Ceiling(frames*.95)-1;
        var result=new {Context="Neutral open engineering pipe; original Update reproduced on identical cached cuts; .NET array-only steps, not Unity frame time",Vertices=mesh.Vertices.Length,Triangles=mesh.Triangles.Length/3,Sections=landmarks.Sections.Length,Frames=frames,SetupMs=setup,Before=new{MeanMs=originalTotal/frames,P95Ms=originalTimes[p95]},After=new{MeanMs=total/frames,P95Ms=times[p95]},WarmAllocatedBytesCombined=allocated,PerFrameTriangleCuts=landmarks.ReferenceTriangleTests-initialTriangles,Runtime=Environment.Version.ToString(),CpuThreads=Environment.ProcessorCount};
        var json=JsonSerializer.Serialize(result,new JsonSerializerOptions{WriteIndented=true});File.WriteAllText(output,json);Console.WriteLine(json);
    }
}
