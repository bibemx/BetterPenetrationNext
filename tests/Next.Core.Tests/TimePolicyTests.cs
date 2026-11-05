using System;
using BetterPenetrationNext.Core;

internal static class TimePolicyTests
{
    static void Require(bool value,string message) { if(!value)throw new Exception(message); }
    static SoftBodySettings Free(double maximumStep)
    { return new SoftBodySettings { MaximumAdaptiveStep=maximumStep,Gravity=new Vec3(0,0,1),Damping=0,StretchCompliance=1e12,BendCompliance=1e12,ShapeCompliance=1e12,MaximumOffsetRatio=100,MaximumSpeedRatio=100 }; }
    static SoftBodySolver New(MeshData mesh)
    { return new SoftBodySolver(new SimulationTopology(mesh.Vertices,mesh.Triangles),new double[mesh.Vertices.Length]); }
    static double Maximum(SoftBodySolver solver)
    { double max=0;for(int i=0;i<solver.Positions.Length;i++)max=Math.Max(max,solver.SolvedDisplacement(i).Length);return max; }
    public static void Run(Action<string,Action> test)
    {
        test("1.0.7 65 ms animated frames advance all elapsed time with a bounded two-step budget",()=>{
            var mesh=PrimitiveMeshes.Grid(1,1,1,1);var legacy=New(mesh);var adaptive=New(mesh);var policy=PhysicsPolicy.Resolve(PhysicsQuality.Performance);
            for(int i=0;i<15;i++) { legacy.Advance(mesh.Vertices,null,Free(0),policy,.065,1);adaptive.Advance(mesh.Vertices,null,Free(1.0/30),policy,.065,1); }
            Require(legacy.DroppedTime>.47&&adaptive.DroppedTime<1e-12,"Overflow time was still lost");
            Require(adaptive.LastSubsteps==2&&Math.Abs(adaptive.LastFrameSimulationTime-.065)<1e-12&&adaptive.LastStepDuration<=1.0/30,"Budget or consumed time is wrong");
            Require(Maximum(adaptive)>3*Maximum(legacy),"Actual particle motion did not catch up with the animated elapsed time");
            Console.WriteLine("TIME_POLICY recordedFrameSeconds=.065 legacyLost="+legacy.DroppedTime+" adaptiveLost="+adaptive.DroppedTime+" legacyDisplacement="+Maximum(legacy)+" adaptiveDisplacement="+Maximum(adaptive));
        });
        test("1.0.7 adaptive overflow leaves the original high-FPS fixed-step sequence intact",()=>{
            var mesh=PrimitiveMeshes.Grid(2,2,1,1);var legacy=New(mesh);var adaptive=New(mesh);var policy=PhysicsPolicy.Resolve(PhysicsQuality.Performance);var a=Free(0);var b=Free(1.0/30);
            for(int frame=0;frame<180;frame++){double dt=frame%3==0?1.0/120:1.0/60;legacy.Advance(mesh.Vertices,null,a,policy,dt,1);adaptive.Advance(mesh.Vertices,null,b,policy,dt,1);}
            for(int i=0;i<mesh.Vertices.Length;i++)Require((legacy.Positions[i]-adaptive.Positions[i]).Length<1e-12,"High-FPS coordinates changed");
            Require(adaptive.DroppedTime==0,"High-FPS time was dropped");
        });
        test("1.0.7 long stalls retain an explicit time cap and the new hot path allocates no memory",()=>{
            var mesh=PrimitiveMeshes.Grid(2,2,1,1);var solver=New(mesh);var settings=Free(1.0/30);var policy=PhysicsPolicy.Resolve(PhysicsQuality.Performance);
            solver.Advance(mesh.Vertices,null,settings,policy,1,1);
            Require(solver.LastSubsteps==2&&solver.LastFrameDroppedTime>.93&&solver.LastFrameSimulationTime<=2.0/30+1e-12,"Long stall became unbounded");
            for(int i=0;i<10;i++)solver.Advance(mesh.Vertices,null,settings,policy,.065,1);
            long bytes=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<120;i++)solver.Advance(mesh.Vertices,null,settings,policy,.065,1);
            Require(GC.GetAllocatedBytesForCurrentThread()==bytes,"Adaptive stepping allocated per-frame memory");foreach(var p in solver.Positions)Require(p.IsFinite,"Nonfinite particle");
            bool rejected=false;try{new SoftBodySettings {MaximumAdaptiveStep=double.NaN}.Validate();}catch(ArgumentException){rejected=true;}Require(rejected,"Nonfinite duration accepted");
        });
        test("1.0.7 contact shape recovers in 4.5 elapsed seconds at low FPS",()=>{
            var mesh=PrimitiveMeshes.Grid(6,6,.4,.4);var solver=New(mesh);var obstacle=PrimitiveMeshes.Sphere(12,8,.12);for(int i=0;i<obstacle.Vertices.Length;i++)obstacle.Vertices[i]+=new Vec3(0,0,.05);
            var surface=new TriangleSurface(obstacle.Vertices,obstacle.Triangles,SurfaceKind.ClosedVolume);surface.RobustSign=false;
            var settings=new SoftBodySettings {MaximumAdaptiveStep=1.0/30,RecoverySeconds=4.5};var coupling=new BidirectionalContactSettings {Mode=ContactMode.OneWay,MaxTissueStrain=.5,RecoveryTime=4.5};var policy=PhysicsPolicy.Resolve(PhysicsQuality.Performance);
            for(int i=0;i<45;i++)solver.Advance(mesh.Vertices,surface,settings,policy,.065,1,coupling);
            double initial=Maximum(solver);Require(initial>1e-3,"No actual contact deformation");solver.Advance(mesh.Vertices,null,settings,policy,.05,1,coupling);Require(Maximum(solver)>initial*.8,"Recovery snapped");
            for(int i=0;i<90;i++)solver.Advance(mesh.Vertices,null,settings,policy,.05,1,coupling);
            Require(Maximum(solver)<1e-9,"4.5 second recovery did not finish");
        });
    }
}
