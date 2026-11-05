using System;
using BetterPenetrationNext.Core;
internal static class ArchitectureTests
{
 static void Check(bool b,string message){if(!b)throw new Exception(message);}
 static void Near(Vec3 a,Vec3 b,double e,string message){Check((a-b).Length<=e,message+" error="+(a-b).Length);}
 public static void Run(Action<string,Action> test)
 {
  test("1.0.5 seam proxy preserves independent animated vertices and closes only paired cracks",()=>{
   var v=new[]{new Vec3(),new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1),new Vec3()};var t=new[]{0,2,1,4,1,3,0,3,2,1,2,3};var p=new AnimatedSeamContactProxy(v,t);v[4]=new Vec3(.002,0,0);Check(p.Update(v),"small crack rejected");Near(p.Positions[0],v[0],0,"first vertex changed");Near(p.Positions[4],v[4],0,"second skinning path changed");Check(p.AddedTriangles>0,"no bridge");new TriangleSurface(p.Positions,p.Triangles,SurfaceKind.ClosedVolume);v[4]=new Vec3(1,0,0);Check(!p.Update(v),"oversized seam accepted");
  });
  test("1.0.5 true open rim is never silently capped",()=>{var m=LabScenarios.Tube(12,4,.2,1);bool reject=false;try{new AnimatedSeamContactProxy(m.Vertices,m.Triangles);}catch(ArgumentException){reject=true;}Check(reject,"genuine tube opening closed");});
  test("1.0.5 triangle hints retain exact global nearest distance after large pose changes",()=>{
   var m=PrimitiveMeshes.Sphere(32,16,.2);var surface=new TriangleSurface(m.Vertices,m.Triangles,SurfaceKind.ClosedVolume){RobustSign=false};int hint=0;var random=new Random(19);
   for(int k=0;k<300;k++){var p=new Vec3(random.NextDouble()-.5,random.NextDouble()-.5,random.NextDouble()-.5);SurfaceSample a,b;bool aa=surface.TrySample(p,2,out a),bb=surface.TrySampleHinted(p,2,ref hint,out b);Check(aa==bb&&Math.Abs(a.SignedDistance-b.SignedDistance)<1e-10,"hint skipped nearer surface");}
  });
  test("1.0.5 active list reduces expensive requests and preserves coupled coordinates",()=>{
   var m=PrimitiveMeshes.Grid(24,24,1,1);var ball=PrimitiveMeshes.Sphere(24,12,.12);var surface=new TriangleSurface(ball.Vertices,ball.Triangles,SurfaceKind.ClosedVolume){RobustSign=false};var a=new SoftBodySolver(new SimulationTopology(m.Vertices,m.Triangles),new double[m.Vertices.Length]);var b=new SoftBodySolver(a.Topology,new double[m.Vertices.Length]);var sa=new SourceContactBody{Surface=surface,ReleaseOnlyWhenSeparated=false};var sb=new SourceContactBody{Surface=surface,ReleaseOnlyWhenSeparated=false};var ca=new BidirectionalContactSettings();var cb=new BidirectionalContactSettings{UseActiveContactList=false,UseTriangleHints=false};var settings=new SoftBodySettings();var policy=PhysicsPolicy.Resolve(PhysicsQuality.Performance);
   for(int f=0;f<40;f++){sa.BeginFrame(ca,.0334);sb.BeginFrame(cb,.0334);a.Advance(m.Vertices,sa,settings,policy,.0334,1.4,ca);b.Advance(m.Vertices,sb,settings,policy,.0334,1.4,cb);Near(sa.Offset,sb.Offset,1e-9,"source mismatch");for(int i=0;i<m.Vertices.Length;i++)Near(a.Positions[i],b.Positions[i],1e-9,"particle mismatch");}
   Check(a.ActiveContactChecks<=a.ActiveContactCandidates,"invalid compaction");Check(a.ContactQueries<3*m.Vertices.Length,"expensive calls not culled");Check(a.MaximumResidualPenetration<=b.MaximumResidualPenetration+1e-8,"missed contact");
  });
  test("1.0.5 support persists at damping 3 and 12 until separation, cap .05 is explicit",()=>{
   foreach(double d in new[]{3.0,12.0}){var c=new BidirectionalContactSettings{ContactDamping=d};var body=new SourceContactBody();body.BeginFrame(c,.2);body.Correct(new Vec3(.1,0,0));Check(body.AtLimit,"no saturation flag");body.MarkContact();body.BeginFrame(c,.2);Near(body.Offset,new Vec3(.05,0,0),1e-12,"support erased");body.BeginFrame(c,.2);Check(Math.Abs(body.Offset.X-.05*Math.Exp(-d*.2))<1e-12,"separated release differs");}
  });
  test("1.0.5 independent 3D guide is bounded, smooth and removable without drift",()=>{
   var controls=new[]{new Vec3(0,-1,-.2),new Vec3(.1,0,0),new Vec3(0,1,.2)};var path=new AnatomicalPathConstraintSystem(controls);var guide=new PathGuidanceController();var c=new BidirectionalContactSettings();var body=new SourceContactBody();Vec3 last=new Vec3();
   for(int i=0;i<200;i++){body.BeginFrame(c,.016);var pose=guide.Step(path,new Vec3(.15,0,0),new Vec3(1,0,0),.016,.05,.35,12,1);Check((pose.RotationVector-last).Length<=.016000001,"angular speed exceeded");last=pose.RotationVector;PathGuidanceController.Apply(body,pose);Check(body.Offset.Length<=.050000001&&body.RotationVector.Length<=.350000001,"guide limit");}
   guide.Reset();body.BeginFrame(c,.016);Near(body.Offset,new Vec3(),1e-12,"guide accumulated");Near(body.RotationVector,new Vec3(),0,"guide rotation retained");
  });
  test("1.0.5 guide translation is frame partition invariant and moving path reuses buffers",()=>{
   var controls=new[]{new Vec3(0,-1,0),new Vec3(.1,0,.1),new Vec3(0,1,.2)};var p=new AnatomicalPathConstraintSystem(controls);var a=new PathGuidanceController();var b=new PathGuidanceController();for(int i=0;i<60;i++)a.Step(p,new Vec3(.2,0,0),new Vec3(0,1,0),1.0/60,.05);for(int i=0;i<120;i++)b.Step(p,new Vec3(.2,0,0),new Vec3(0,1,0),1.0/120,.05);Near(a.Translation,b.Translation,2e-4,"partition drift");long bytes=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<100;i++){controls[1].X+=.00001;p.UpdateControls(controls);a.Step(p,new Vec3(.2,0,0),new Vec3(0,1,0),.016,.05);}Check(GC.GetAllocatedBytesForCurrentThread()-bytes==0,"moving guide allocates");
  });
 }
}
