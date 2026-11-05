using System;
using System.Diagnostics;
using BetterPenetrationNext.Core;

internal static class Program
{
    sealed class Unbounded : IContactSurface { readonly IContactSurface surface; public Unbounded(IContactSurface value) { surface=value; } public bool TrySample(Vec3 p,double d,out SurfaceSample sample) { return surface.TrySample(p,d,out sample); } }
    static int passed;
    static void Check(bool value,string message) { if(!value) throw new Exception(message); }
    static void Near(double a,double b,double tolerance,string name) { Check(Math.Abs(a-b)<=tolerance,name+": "+a+" / "+b); }
    static void Test(string name,Action body) { body(); Console.WriteLine("PASS "+name); passed++; }
    static double[] Pins(MeshData m,bool edges=true)
    { var p=new double[m.Vertices.Length]; if(edges) for(int i=0;i<p.Length;i++) if(Math.Abs(m.Vertices[i].X)>0.49||Math.Abs(m.Vertices[i].Y)>0.49) p[i]=1; return p; }
    static SoftBodySolver Solver(MeshData m,double[] pins=null) { return new SoftBodySolver(new SimulationTopology(m.Vertices,m.Triangles),pins??Pins(m)); }
    static IContactSurface Obstacle(double z=0.08,double radius=0.22)
    { var m=PrimitiveMeshes.Sphere(32,16,radius); for(int i=0;i<m.Vertices.Length;i++) m.Vertices[i]+=new Vec3(0,0,z); var surface=new TriangleSurface(m.Vertices,m.Triangles,SurfaceKind.ClosedVolume); surface.RobustSign=false; return surface; }
    static void Step(SoftBodySolver s,MeshData m,IContactSurface contact,int frames=120,SoftBodySettings settings=null,PhysicsQuality quality=PhysicsQuality.Balanced)
    { for(int i=0;i<frames;i++) s.Advance(m.Vertices,contact,settings??new SoftBodySettings(),PhysicsPolicy.Resolve(quality),1.0/60,Math.Sqrt(2)); }
    static double Maximum(SoftBodySolver s) { double max=0; for(int i=0;i<s.Positions.Length;i++) max=Math.Max(max,s.SolvedDisplacement(i).Length); return max; }
    static void Main(string[] args)
    {
        try
        {
            if(Array.IndexOf(args,"--bilateral")>=0){BilateralConstraintTests.Run(Test);Console.WriteLine("RESULT "+passed+" bilateral tests passed");return;}
            if(Array.IndexOf(args,"--approach-dimensions")>=0){ApproachAndDimensionsTests.Run(Test);Console.WriteLine("RESULT "+passed+" approach/dimensions tests passed");return;}
            if(Array.IndexOf(args,"--stylized-motion")>=0){StylizedMotionTests.Run(Test);Console.WriteLine("RESULT "+passed+" stylized-motion tests passed");return;}
            if(Array.IndexOf(args,"--inextensible")>=0){InextensibleRigTests.Run(Test);Console.WriteLine("RESULT "+passed+" inextensible-rig tests passed");return;}
            if(Array.IndexOf(args,"--receiver-landmarks")>=0){ReceiverLandmarkTests.Run(Test);Console.WriteLine("RESULT "+passed+" Receiver tests passed");return;}
            int landmarkBenchmark=Array.IndexOf(args,"--landmarks-benchmark");if(landmarkBenchmark>=0){LandmarkTests.Benchmark(args[landmarkBenchmark+1]);return;}
            int landmarkAssets=Array.IndexOf(args,"--landmark-assets");if(landmarkAssets>=0){LandmarkAssetReplay.Run(args[landmarkAssets+1],args[landmarkAssets+2]);return;}
            if(Array.IndexOf(args,"--landmarks")>=0){LandmarkTests.Run(Test);Console.WriteLine("RESULT "+passed+" landmark tests passed");return;}
            if(Array.IndexOf(args,"--rig-worker")>=0){RigWorkerTests.Run(Test);Console.WriteLine("RESULT "+passed+" worker tests passed");return;}
            if(Array.IndexOf(args,"--guidance")>=0) { AnimatedPathTests.Run(Test);Console.WriteLine("RESULT "+passed+" guidance tests passed");return; }
            if(Array.IndexOf(args,"--time-policy")>=0) { TimePolicyTests.Run(Test);Console.WriteLine("RESULT "+passed+" time-policy tests passed");return; }
            if(Array.IndexOf(args,"--section-rig")>=0) { SectionRigTests.Run(Test);Console.WriteLine("RESULT "+passed+" section-rig tests passed");return; }
            Test("Topology preserves open boundaries and builds stretch/bending constraints",()=>
            { var m=PrimitiveMeshes.Grid(12,12,1,1); var t=new SimulationTopology(m.Vertices,m.Triangles); Check(!t.Closed&&t.BoundaryEdges==48&&t.Stretch.Length>0&&t.Bend.Length>0,"bad open topology"); });
            Test("Closed surface has consistent signed volume",()=>
            { var m=PrimitiveMeshes.Sphere(32,16,1); var t=new SimulationTopology(m.Vertices,m.Triangles); Check(t.Closed,"sphere open"); Near(t.SignedVolume(m.Vertices),4*Math.PI/3,0.08,"volume"); });
            Test("Degenerate and nonmanifold simulation geometry is rejected",()=>
            { var m=PrimitiveMeshes.Grid(1,1,1,1); int bad=0; try { new SimulationTopology(m.Vertices,new[]{0,0,1}); } catch(ArgumentException) { bad++; } try { new SimulationTopology(m.Vertices,new[]{0,1,2,0,1,3,0,1,2}); } catch(ArgumentException) { bad++; } Check(bad==2,"invalid geometry accepted"); });
            Test("No-contact equilibrium remains exact for all quality levels",()=>
            { foreach(PhysicsQuality q in Enum.GetValues(typeof(PhysicsQuality))) { var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m); Step(s,m,null,120,null,q); Check(Maximum(s)<1e-10,"drift "+q); } });
            Test("Actual XPBD positions resolve contact while hard pins stay fixed",()=>
            { var m=PrimitiveMeshes.Grid(16,16,1,1); var pins=Pins(m); var s=Solver(m,pins); Step(s,m,Obstacle()); Check(Maximum(s)>0.03&&s.ContactCount>0,"contact inactive"); for(int i=0;i<pins.Length;i++) if(pins[i]==1) Near((s.Positions[i]-m.Vertices[i]).Length,0,1e-12,"hard pin"); });
            Test("Surface restores its rest shape after removal of contact",()=>
            { var m=PrimitiveMeshes.Grid(10,10,1,1); var s=Solver(m); Step(s,m,Obstacle()); double initial=Maximum(s); Step(s,m,null,600); Check(initial>0.03&&Maximum(s)<initial*0.01,"shape did not recover"); });
            Test("Partially pinned nodes have stronger animated-rest attachment",()=>
            { var m=PrimitiveMeshes.Grid(8,8,1,1); var a=Solver(m,new double[m.Vertices.Length]); var pins=new double[m.Vertices.Length]; for(int i=0;i<pins.Length;i++) pins[i]=0.95; var b=Solver(m,pins); var settings=new SoftBodySettings { Gravity=new Vec3(0,0,-1),PinCompliance=1e-6 }; Step(a,m,null,300,settings); Step(b,m,null,300,settings); Check(Maximum(b)<Maximum(a)*0.1,"partial pin stiffness ignored"); });
            Test("Maximum offsets and dynamic velocities remain bounded under deep contact",()=>
            { var m=PrimitiveMeshes.Grid(10,10,1,1); var s=Solver(m); var settings=new SoftBodySettings { MaximumOffsetRatio=0.01,MaximumSpeedRatio=0.3 }; Step(s,m,Obstacle(0,0.4),120,settings); Check(s.SaturatedVertices>0,"saturation absent"); Check(Maximum(s)<=Math.Sqrt(2)*0.010001,"offset exceeded"); foreach(var v in s.Velocities) Check(v.Length<=Math.Sqrt(2)*0.300001,"velocity exceeded"); });
            Test("Fixed step consumes the same sequence for different render partitions",()=>
            { var m=PrimitiveMeshes.Grid(6,6,1,1); var a=Solver(m); var b=Solver(m); var policy=PhysicsPolicy.Resolve(PhysicsQuality.Custom,6,8,120,3); var settings=new SoftBodySettings(); var obstacle=Obstacle(); for(int i=0;i<120;i++) a.Advance(m.Vertices,obstacle,settings,policy,1.0/60,1); for(int i=0;i<240;i++) b.Advance(m.Vertices,obstacle,settings,policy,1.0/120,1); for(int i=0;i<m.Vertices.Length;i++) Near((a.Positions[i]-b.Positions[i]).Length,0,1e-8,"partition"); });
            Test("Long stalls are bounded and lost simulation time is explicit",()=>
            { var m=PrimitiveMeshes.Grid(6,6,1,1); var s=Solver(m); var p=PhysicsPolicy.Resolve(PhysicsQuality.Balanced); s.Advance(m.Vertices,Obstacle(),new SoftBodySettings(),p,5,1); Check(s.LastSubsteps<=p.MaximumSubsteps&&s.DroppedTime>4.9,"unbounded catch-up"); });
            Test("Animated attachment follows translation without accumulating offsets",()=>
            { var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m); Step(s,m,null,1); for(int frame=0;frame<300;frame++) { for(int i=0;i<m.Vertices.Length;i++) m.Vertices[i]+=new Vec3(0.001,0,0); s.Advance(m.Vertices,null,new SoftBodySettings(),PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,Math.Sqrt(2)); } Check(Maximum(s)<1e-9,"attachment lag accumulated"); });
            Test("Scale changes produce scaled contact response",()=>
            { double previous=0; foreach(double scale in new[]{0.01,1.0,100.0}) { var m=PrimitiveMeshes.Grid(10,10,scale,scale); var s=Solver(m,new double[m.Vertices.Length]); var obstacle=Obstacle(0.08*scale,0.22*scale); for(int i=0;i<180;i++) s.Advance(m.Vertices,obstacle,new SoftBodySettings(),PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,Math.Sqrt(2)*scale); double normalized=Maximum(s)/scale; if(previous>0) Near(normalized,previous,0.002,"scale response"); previous=normalized; } });
            Test("Alternating contact motion and direction remain finite over long runs",()=>
            { var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m); for(int f=0;f<1200;f++) { s.Advance(m.Vertices,Obstacle(0.08+0.2*Math.Sin(f*0.03)),new SoftBodySettings(),PhysicsPolicy.Resolve(PhysicsQuality.Performance),1.0/90,Math.Sqrt(2)); foreach(var p in s.Positions) Check(p.IsFinite,"nonfinite simulation"); } Check(Maximum(s)<0.22,"large drift"); });
            Test("Reset clears physical state and visual interpolation",()=>
            { var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m); Step(s,m,Obstacle()); s.Reset(m.Vertices); Near(Maximum(s),0,0,"reset offset"); foreach(var v in s.Velocities) Near(v.Length,0,0,"reset velocity"); for(int i=0;i<m.Vertices.Length;i++) Near(s.Displacement(i).Length,0,0,"reset visual"); });
            Test("Barycentric transfer reproduces a uniform simulation displacement",()=>
            { var m=PrimitiveMeshes.Grid(4,4,1,1); var influence=new double[m.Vertices.Length]; for(int i=0;i<influence.Length;i++) influence[i]=1; var map=SurfaceBinding.Build(m.Vertices,influence,m.Vertices,m.Triangles,0.01); var s=Solver(m,new double[m.Vertices.Length]); Step(s,m,Obstacle(0,0.8),120,new SoftBodySettings { StretchCompliance=0.1,ShapeCompliance=0.1 }); foreach(var b in map) Near((b.Transfer(s)-s.Displacement(b.Vertex)).Length,0,1e-8,"vertex transfer"); });
            Test("Unbound visual vertices are excluded and far bindings fail",()=>
            { var m=PrimitiveMeshes.Grid(4,4,1,1); var influence=new double[m.Vertices.Length]; influence[0]=1; Check(SurfaceBinding.Build(m.Vertices,influence,m.Vertices,m.Triangles,0.1).Length==1,"inactive region changed"); var far=(Vec3[])m.Vertices.Clone(); far[0]+=new Vec3(0,0,5); bool rejected=false; try { SurfaceBinding.Build(far,influence,m.Vertices,m.Triangles,0.1); } catch(ArgumentException) { rejected=true; } Check(rejected,"uncovered visual accepted"); });
            Test("Core hot step reuses all buffers without managed allocation",()=>
            { var m=PrimitiveMeshes.Grid(12,12,1,1); var s=Solver(m); var settings=new SoftBodySettings(); var p=PhysicsPolicy.Resolve(PhysicsQuality.Balanced); var contact=Obstacle(); for(int i=0;i<100;i++) s.Advance(m.Vertices,contact,settings,p,1.0/60,1.4); long before=GC.GetAllocatedBytesForCurrentThread(); for(int i=0;i<100;i++) s.Advance(m.Vertices,contact,settings,p,1.0/60,1.4); long allocated=GC.GetAllocatedBytesForCurrentThread()-before; Check(allocated==0,"allocated "+allocated); });
            Test("Invalid parameters and changed topology fail explicitly",()=>
            { var m=PrimitiveMeshes.Grid(4,4,1,1); var s=Solver(m); int rejected=0; try { s.Advance(m.Vertices,null,new SoftBodySettings { Damping=double.NaN },PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,1); } catch(ArgumentException) { rejected++; } try { s.Reset(new Vec3[2]); } catch(ArgumentException) { rejected++; } Check(rejected==2,"bad input accepted"); });
            Test("Local closed-volume candidates preserve the full-query result",()=>
            { var m=PrimitiveMeshes.Grid(16,16,1,1); var a=Solver(m); var b=Solver(m); var contact=Obstacle(); Step(a,m,contact,180); Step(b,m,new Unbounded(contact),180); double error=0; for(int i=0;i<m.Vertices.Length;i++) error=Math.Max(error,(a.Positions[i]-b.Positions[i]).Length); Check(error<1e-9,"broadphase changed solution: "+error); Check(a.ContactQueries<b.ContactQueries/2,"queries not reduced"); });
            Test("Open shells retain their signed-distance query path",()=>
            { var m=PrimitiveMeshes.Grid(8,8,1,1); var shellMesh=PrimitiveMeshes.Grid(2,2,1,1); var shell=new TriangleSurface(shellMesh.Vertices,shellMesh.Triangles,SurfaceKind.OrientedShell); var a=Solver(m); var b=Solver(m); Step(a,m,shell,60); Step(b,m,new Unbounded(shell),60); for(int i=0;i<m.Vertices.Length;i++) Near((a.Positions[i]-b.Positions[i]).Length,0,1e-10,"shell culling"); });
            Test("Mode transition changes controllers only at zero physical blend",()=>
            { var mode=new ModeTransition(2); mode.Request(0); int switched=0; for(int i=0;i<60;i++) { double previous=mode.Blend; if(mode.Advance(1.0/60,0.2)) { Check(mode.Blend==0,"switch at nonzero blend"); switched++; } Check(Math.Abs(mode.Blend-previous)<=1.0/12+1e-10,"jump in blend"); } Check(switched==1&&mode.Current==0&&mode.Blend==1,"bad transition"); mode.Request(1); for(int i=0;i<4;i++) mode.Advance(1.0/60,0.2); mode.Request(0); for(int i=0;i<30;i++) mode.Advance(1.0/60,0.2); Check(mode.Current==0&&mode.Blend==1,"reversed transition failed"); });
            Test("Rolling profiler reports mean and P95 with a bounded sample window",()=>
            { var window=new PerformanceWindow(100); for(int i=1;i<=100;i++) window.Add(new FrameMeasurement { Total=i }); var s=window.Time(5); Near(s.Mean,50.5,1e-12,"mean"); Near(s.P95,95,0,"p95"); window.Add(new FrameMeasurement { Total=101 }); s=window.Time(5); Near(s.Mean,51.5,1e-12,"window mean"); Near(s.P95,96,0,"window p95"); });
            Test("Managed codec preserves morph arrays and X-Ray exclusions",()=>
            { var d=new SimulationDefinition { XRayExcludedSubmeshes=new[]{0},Morphs=new[]{new SimulationMorph { Name="neutral user morph",Deltas=new[]{new Vec3(1,2,3)},DefaultWeight=25 } } }; var r=JsonCodec.Deserialize<SimulationDefinition>(JsonCodec.Serialize(d)); Check(r.XRayExcludedSubmeshes[0]==0&&r.Morphs[0].Deltas[0].Y==2&&r.Morphs[0].DefaultWeight==25,"codec lost prepared metadata"); });
            Test("Rest-space seam mapping tolerates pose rounding but rejects a separated seam",()=>
            { var rest=new[]{new Vec3(0,0,0),new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,0)}; var weld=new WeldedContactMesh(rest,new[]{0,1,2,3,2,1},1e-6); var posed=(Vec3[])rest.Clone(); for(int i=0;i<posed.Length;i++) posed[i]+=new Vec3(-42.5,1,-115); posed[3]+=new Vec3(0,0,1e-5); weld.Update(posed,6e-5); Check(weld.Positions.Length==3,"seam correspondence lost"); posed[3]+=new Vec3(0,0,.001); bool rejected=false; try { weld.Update(posed,6e-5); } catch(InvalidOperationException) { rejected=true; } Check(rejected,"real seam separation accepted"); bool invalid=false; try { weld.Update(posed,double.NaN); } catch(ArgumentException) { invalid=true; } Check(invalid,"invalid precision accepted"); });
            Test("Coincident vertices with different deformation groups are never welded",()=>
            { var p=new[]{new Vec3(0,0,0),new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,0)}; var weld=new WeldedContactMesh(p,new[]{0,1,2,3,2,1},1e-6,true,new[]{0,0,0,1}); Check(weld.Positions.Length==4,"different skinning paths were merged"); p[3]=new Vec3(0,0,.25); weld.Update(p); Near(weld.Positions[3].Z,.25,0,"independent animated vertex"); });
            Test("Recovery time retains actual particle deformation and returns smoothly after contact",()=>
            { var m=PrimitiveMeshes.Grid(10,10,1,1); var pins=Pins(m); var s=Solver(m,pins); var settings=new SoftBodySettings { RecoverySeconds=4.5 }; Step(s,m,Obstacle(),120,settings); double initial=Maximum(s); Step(s,m,null,30,settings); Check(Maximum(s)>initial*.5,"contact shape snapped back"); for(int i=0;i<pins.Length;i++) if(pins[i]==1) Near(s.SolvedDisplacement(i).Length,0,1e-12,"recovery hard pin"); Step(s,m,null,420,settings); Check(Maximum(s)<initial*.01,"recovery did not finish"); s.Reset(m.Vertices); Step(s,m,null,30,settings); Near(Maximum(s),0,1e-12,"reset retained recovery memory"); });
            Test("New contacts override slow recovery and recovery keeps the allocation-free hot path",()=>
            { var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m); var settings=new SoftBodySettings { RecoverySeconds=4.5 }; var obstacle=Obstacle(); Step(s,m,obstacle,120,settings); Step(s,m,null,15,settings); Step(s,m,obstacle,10,settings); Check(s.ContactCount>0,"new contact delayed by recovery"); long before=GC.GetAllocatedBytesForCurrentThread(); Step(s,m,null,60,settings); Check(GC.GetAllocatedBytesForCurrentThread()-before==0,"recovery allocated managed memory"); bool invalid=false; try { new SoftBodySettings { RecoverySeconds=-1 }.Validate(); } catch(ArgumentException) { invalid=true; } Check(invalid,"invalid recovery time accepted"); });
            Test("Coupled contact shares correction between two free objects",()=>
            { var r=CoupledContactProjection.Project(.1,new Vec3(0,0,1),1,.1,1,1); Check(r.ReceiverDelta.Z>0&&r.SourceDelta.Z<0,"missing reaction"); Near(r.ReceiverDelta.Z-r.SourceDelta.Z,.1,1e-12,"separation"); Near(r.UnresolvedPenetration,0,1e-12,"residual"); });
            Test("Strain-limited tissue returns excess contact correction to source",()=>
            { var p=new Vec3(); var n=new Vec3(0,0,1); var neighbors=new[]{new Vec3(-.1,0,0),new Vec3(.1,0,0)}; var rest=new[]{.1,.1}; double travel=CoupledContactProjection.MaximumAdvance(p,n,p,1,neighbors,rest,.2); Near(travel,Math.Sqrt(.12*.12-.1*.1),1e-12,"strain travel"); var r=CoupledContactProjection.Project(.15,n,1,.01,travel,1); Check(r.ReceiverAtLimit&&r.SourceDelta.Z<-.08,"no limit reaction"); foreach(var q in neighbors) Check((p+r.ReceiverDelta-q).Length<=.120000001,"edge overstretched"); Near(r.UnresolvedPenetration,0,1e-12,"unresolved"); });
            Test("Pinned receiver makes the source absorb contact correction",()=>
            { var r=CoupledContactProjection.Project(.1,new Vec3(0,0,1),0,1,1,1); Near(r.ReceiverDelta.Length,0,0,"pin"); Near(r.SourceDelta.Z,-.1,1e-12,"source reaction"); });
            Test("Incompatible motion limits report residual penetration explicitly",()=>
            { var r=CoupledContactProjection.Project(.1,new Vec3(0,0,1),1,1,.02,.03); Near(r.UnresolvedPenetration,.05,1e-12,"residual"); Check(r.ReceiverAtLimit,"missing strain indicator"); });
            Test("Source axis constraints preserve forbidden directions",()=>
            { var r=CoupledContactProjection.Project(.1,new Vec3(1,0,0),0,1,0,1,new Vec3(0,0,1)); Near(r.SourceDelta.Length,0,0,"axis violation"); Near(r.UnresolvedPenetration,.1,1e-12,"infeasible axis"); });
            Test("Contact projection is symmetric under normal reversal",()=>
            { var a=CoupledContactProjection.Project(.1,new Vec3(0,0,1),1,1,.02,.2); var b=CoupledContactProjection.Project(.1,new Vec3(0,0,-1),1,1,.02,.2); Near((a.SourceDelta+b.SourceDelta).Length,0,1e-12,"source symmetry"); Near((a.ReceiverDelta+b.ReceiverDelta).Length,0,1e-12,"receiver symmetry"); });
            ContactTests.Run(Test);
            ArchitectureTests.Run(Test);
            AnimatedPathTests.Run(Test);
            TimePolicyTests.Run(Test);
            SectionRigTests.Run(Test);
            LandmarkTests.Run(Test);
            InextensibleRigTests.Run(Test);
            ReceiverLandmarkTests.Run(Test);
            RigWorkerTests.Run(Test);
            StylizedMotionTests.Run(Test);
            ApproachAndDimensionsTests.Run(Test);
            BilateralConstraintTests.Run(Test);EntranceAndUvTests.Run(Test);SourceSectionContactTests.Run(Test);
            var bench=PrimitiveMeshes.Grid(24,24,1,1); var solver=Solver(bench); var watch=Stopwatch.StartNew(); Step(solver,bench,Obstacle(),300); watch.Stop(); Console.WriteLine("INFO .NET core only: "+bench.Vertices.Length+" particles, 300 render frames / "+watch.ElapsedMilliseconds+" ms; not KKS FPS");
            Console.WriteLine("RESULT "+passed+" tests passed");
        }
        catch(Exception e) { Console.Error.WriteLine("FAIL "+e); Environment.ExitCode=1; }
    }
}


