using System;
using System.Diagnostics;
using BetterPenetrationNext.Core;

static class ContactTests
{
    static void Check(bool value,string message) { if(!value) throw new Exception(message); }
    static double Max(SoftBodySolver s) { double m=0; for(int i=0;i<s.Positions.Length;i++) m=Math.Max(m,s.SolvedDisplacement(i).Length); return m; }
    static SourceContactBody Source(double z=.12,double radius=.16)
    { var mesh=PrimitiveMeshes.Sphere(20,10,radius); for(int i=0;i<mesh.Vertices.Length;i++) mesh.Vertices[i]+=new Vec3(0,0,z); return new SourceContactBody { Surface=new TriangleSurface(mesh.Vertices,mesh.Triangles,SurfaceKind.ClosedVolume) { RobustSign=false } }; }
    static SoftBodySolver Solver(MeshData m,bool pin=false)
    { var pins=new double[m.Vertices.Length]; if(pin) for(int i=0;i<pins.Length;i++) pins[i]=1; return new SoftBodySolver(new SimulationTopology(m.Vertices,m.Triangles),pins); }
    static BidirectionalContactSettings Settings() { return new BidirectionalContactSettings { TissueCompliance=0,MaxSourceDisplacement=.3,MaxTissueStrain=.25,SourceCorrectionStrength=.1 }; }
    static void Step(SoftBodySolver s,MeshData m,SourceContactBody source,BidirectionalContactSettings c,int frames=60,double dt=1.0/60)
    { for(int f=0;f<frames;f++) { if(source!=null) source.BeginFrame(c,dt); s.Advance(m.Vertices,source,new SoftBodySettings { RecoverySeconds=4.5,MaximumOffsetRatio=.2 },PhysicsPolicy.Resolve(PhysicsQuality.Quality),dt,1.4,c); } }
    public static void Run(Action<string,Action> test)
    {
        test("1.0.4 two-way contact changes actual tissue and shared source coordinates",()=> {
            var m=PrimitiveMeshes.Grid(12,12,1,1); var s=Solver(m); var b=Source(); var c=Settings(); Step(s,m,b,c);
            Check(Max(s)>.0001&&b.Offset.Length>.001,"missing two-sided response"); Check(s.MaximumResidualPenetration<.002,"large contact residual "+s.MaximumResidualPenetration);
            Check(s.MaximumEdgeStrain<=c.MaxTissueStrain+1e-8,"strain cap"); Console.WriteLine("METRIC coupled tissue="+Max(s)+" source="+b.Offset.Length+" residual="+s.MaximumResidualPenetration+" strain="+s.MaximumEdgeStrain);
        });
        test("1.0.4 pinned receiver transfers the contact constraint to source",()=> {
            var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m,true); var b=Source(); Step(s,m,b,Settings());
            Check(Max(s)==0&&b.Offset.Z>.02,"source did not react to pinned mesh"); Check(s.MaximumResidualPenetration<1e-6,"pinned residual");
        });
        test("1.0.4 infeasible displacement limits expose real residual",()=> {
            var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m,true); var b=Source(); var c=Settings(); c.MaxSourceDisplacement=.005; Step(s,m,b,c);
            Check(s.MaximumResidualPenetration>.02&&b.Offset.Length<=.005000001,"infeasible contact hidden");
        });
        test("1.0.4 Original OneWay Bidirectional switch restores and bounds state",()=> {
            var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m); var b=Source(); var c=Settings(); c.Mode=ContactMode.OneWay; Step(s,m,b,c); Check(Max(s)>.001&&b.Offset.Length==0,"one-way");
            c.Mode=ContactMode.Bidirectional; Step(s,m,b,c); Check(b.Offset.Length>.001,"bidirectional"); c.Mode=ContactMode.Original; Step(s,m,b,c,1); Check(Max(s)==0&&b.Offset.Length==0,"original restore");
        });
        test("1.0.4 finite smooth recovery finishes at 4.5 s and repeat contact interrupts it",()=> {
            var m=PrimitiveMeshes.Grid(10,10,1,1); var s=Solver(m); var b=Source(); var c=Settings(); c.Mode=ContactMode.OneWay; Step(s,m,b,c); double initial=Max(s);
            Step(s,m,null,c,30); Check(Max(s)>initial*.7,"early recovery snapped"); double halfway=Max(s);
            Step(s,m,b,c,10); Check(Max(s)>halfway,"repeat contact did not interrupt");
            Step(s,m,null,c,271); Check(Max(s)<1e-8,"recovery deadline "+Max(s));
        });
        test("1.0.4 animated baseline and changing morph-like deformation do not accumulate",()=> {
            var m=PrimitiveMeshes.Grid(8,8,1,1); var rest=(Vec3[])m.Vertices.Clone(); var s=Solver(m); var c=Settings();
            for(int f=0;f<500;f++) { for(int i=0;i<rest.Length;i++) m.Vertices[i]=rest[i]+new Vec3(.1*Math.Sin(f*.03),0,.01*Math.Sin(rest[i].X*5+f*.02)); Step(s,m,null,c,1); }
            Check(Max(s)<1e-9,"animated anchor drift");
        });
        test("1.0.4 sampled spline reduces position and angular deviation with bounded correction",()=> {
            var path=new AnatomicalPathConstraintSystem(new[]{new Vec3(0,0,0),new Vec3(0,.5,.05),new Vec3(0,1,.2)}); var b=Source(); var c=Settings(); b.BeginFrame(c,1.0/60);
            Vec3 p=new Vec3(.1,.4,0),forward=new Vec3(.1,1,0); double before=path.Closest(p).Distance;
            for(int i=0;i<30;i++) path.Constrain(b,p,forward,1.0/60);
            Check(path.Closest(p+b.Offset).Distance<before*.1,"path position"); Check(b.RotationVector.Length>0&&b.RotationVector.Length<=.35,"path orientation"); Check(b.Offset.Length<=c.MaxSourceDisplacement,"path bound");
        });
        test("1.0.4 spline continuity survives direction reversal and moving control points",()=> {
            var path=new AnatomicalPathConstraintSystem(new[]{new Vec3(0,0,0),new Vec3(.1,.5,0),new Vec3(0,1,0)}); var b=Source(); var c=Settings(); b.BeginFrame(c,.02); double previous=-1;
            for(int i=0;i<80;i++) { var p=new Vec3(.03,i<40?i*.01:(80-i)*.01,0); Vec3 before=b.Offset; var sample=path.Constrain(b,p,new Vec3(0,i<40?1:-1,0),.02,12,.35,previous); previous=sample.Parameter; Check((b.Offset-before).Length<.05,"path jump"); }
        });
        test("1.0.4 tube shell expands radially locally without topology change",()=> {
            var m=LabScenarios.Tube(24,12,.2,1); var s=Solver(m); var ball=PrimitiveMeshes.Sphere(24,12,.22); var b=new SourceContactBody { Surface=new TriangleSurface(ball.Vertices,ball.Triangles,SurfaceKind.ClosedVolume) { RobustSign=false } }; var c=Settings(); c.Mode=ContactMode.OneWay;
            Step(s,m,b,c,90); double radial=0,ends=0; for(int i=0;i<m.Vertices.Length;i++) { if(Math.Abs(m.Vertices[i].Y)<.1) radial=Math.Max(radial,Math.Sqrt(s.Positions[i].X*s.Positions[i].X+s.Positions[i].Z*s.Positions[i].Z)-.2); if(Math.Abs(m.Vertices[i].Y)>.49) ends=Math.Max(ends,s.SolvedDisplacement(i).Length); }
            Check(radial>.005&&radial>ends,"no local radial expansion"); Check(s.MaximumEdgeStrain<=c.MaxTissueStrain+1e-8,"tube strain"); Check(s.Topology.Triangles.Length==m.Triangles.Length,"topology changed");
        });
        test("1.0.4 all triangle orientations and neighbor extension limits remain valid",()=> {
            var m=PrimitiveMeshes.Grid(14,14,1,1); var s=Solver(m); var c=Settings(); c.MaxTissueStrain=.03; var b=Source(.04,.22); Step(s,m,b,c);
            Check(s.MaximumEdgeStrain<=.03000001,"excess strain");
            for(int t=0;t<m.Triangles.Length;t+=3) { int a=m.Triangles[t],bb=m.Triangles[t+1],cc=m.Triangles[t+2]; Check(Vec3.Dot(Vec3.Cross(s.Positions[bb]-s.Positions[a],s.Positions[cc]-s.Positions[a]),Vec3.Cross(m.Vertices[bb]-m.Vertices[a],m.Vertices[cc]-m.Vertices[a]))>0,"flipped triangle"); }
        });
        test("1.0.4 explicit regional compliance changes measured local response",()=> {
            var m=PrimitiveMeshes.Grid(8,8,1,1); var a=Solver(m); var b=Solver(m); var c=Settings(); c.Mode=ContactMode.OneWay; Step(a,m,Source(),c);
            c.RegionCompliance=new double[m.Vertices.Length]; for(int i=0;i<c.RegionCompliance.Length;i++) c.RegionCompliance[i]=.01; Step(b,m,Source(),c);
            Check(Max(a)>Max(b)*1.5,"regional parameters ignored");
        });
        test("1.0.4 deltaTime partitions and long bounded runs stay finite",()=> {
            foreach(double dt in new[]{1.0/30,1.0/60,1.0/120,.007,.1}) { var m=PrimitiveMeshes.Grid(6,6,1,1); var s=Solver(m); var b=Source(); var c=Settings(); Step(s,m,b,c,150,dt); Check(b.Offset.IsFinite&&b.Offset.Length<=c.MaxSourceDisplacement+1e-12,"source drift"); foreach(var p in s.Positions) Check(p.IsFinite,"NaN/Infinity"); Check(s.MaximumEdgeStrain<=c.MaxTissueStrain+1e-8,"strain dt"); }
        });
        test("1.0.4 coupled solver warmed loop allocates zero managed bytes",()=> {
            var m=PrimitiveMeshes.Grid(8,8,1,1); var s=Solver(m); var b=Source(); var c=Settings(); var settings=new SoftBodySettings(); var policy=PhysicsPolicy.Resolve(PhysicsQuality.Balanced);
            for(int i=0;i<30;i++) { b.BeginFrame(c,.016); s.Advance(m.Vertices,b,settings,policy,.016,1.4,c); }
            long before=GC.GetAllocatedBytesForCurrentThread(); for(int i=0;i<30;i++) { b.BeginFrame(c,.016); s.Advance(m.Vertices,b,settings,policy,.016,1.4,c); } Check(GC.GetAllocatedBytesForCurrentThread()-before==0,"coupled allocation");
        });
        test("1.0.4 closed-source bounds culling matches full coupled contact queries",()=> {
            var m=PrimitiveMeshes.Grid(8,8,1,1); var a=Solver(m); var b=Solver(m); var sa=Source(); var sb=Source(); sb.Surface=new Unbounded(sb.Surface); var c=Settings(); Step(a,m,sa,c); Step(b,m,sb,c);
            Check((sa.Offset-sb.Offset).Length<1e-10,"bounds changed reaction"); for(int i=0;i<a.Positions.Length;i++) Check((a.Positions[i]-b.Positions[i]).Length<1e-10,"bounds changed tissue");
        });
        test("1.0.4 combined rotation and translation stay within the total source correction limit",()=> {
            var b=Source(); var c=Settings(); c.MaxSourceDisplacement=.02; b.BeginFrame(c,.02); b.Pivot=new Vec3(0,0,.12); b.RotationVector=new Vec3(0,1,0); b.Offset=new Vec3(.02,0,0); b.LimitPose();
            foreach(var p in PrimitiveMeshes.Sphere(20,10,.16).Vertices) { var world=p+new Vec3(0,0,.12); Check((b.ToCorrected(world)-world).Length<=.02000001,"total pose cap"); }
        });
        test("1.0.4 high density source and 1225 particle benchmark records real timings",()=> {
            var m=PrimitiveMeshes.Grid(34,34,1,1); var s=Solver(m); var sourceMesh=PrimitiveMeshes.Sphere(160,80,.16); var b=new SourceContactBody { Surface=new TriangleSurface(sourceMesh.Vertices,sourceMesh.Triangles,SurfaceKind.ClosedVolume) { RobustSign=false } }; var c=Settings();
            Step(s,m,b,c,15); var times=new double[100]; var watch=new Stopwatch(); for(int i=0;i<times.Length;i++) { watch.Restart(); Step(s,m,b,c,1); watch.Stop(); times[i]=watch.Elapsed.TotalMilliseconds; }
            Array.Sort(times); double sum=0; foreach(double t in times) sum+=t;
            Console.WriteLine("PERFORMANCE runtime=dotnet6 particles="+m.Vertices.Length+" sourceTriangles="+sourceMesh.Triangles.Length/3+" meanMs="+sum/times.Length+" p95Ms="+times[94]+" residual="+s.MaximumResidualPenetration); Check(s.MaximumEdgeStrain<=c.MaxTissueStrain+1e-8,"dense strain");
        });
    }
    sealed class Unbounded : IContactSurface { readonly IContactSurface surface; public Unbounded(IContactSurface s) { surface=s; } public bool TrySample(Vec3 p,double d,out SurfaceSample s) { return surface.TrySample(p,d,out s); } }
}
