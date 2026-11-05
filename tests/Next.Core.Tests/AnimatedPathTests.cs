using System;
using BetterPenetrationNext.Core;
internal static class AnimatedPathTests
{
 sealed class BoundsOnly:IContactSurface,IContactBounds{public Bounds3 ContactBounds{get;set;}public bool HasVolumeBounds{get{return false;}}public bool TrySample(Vec3 p,double d,out SurfaceSample s){s=new SurfaceSample();return false;}}
 static void Check(bool ok,string message){if(!ok)throw new Exception(message);}
 static SimulationDefinition Definition(){return new SimulationDefinition{VisualMeshSignature="test",BoneNames=new[]{"root","tip"},Morphs=new[]{new SimulationMorph{Name="shape"}}};}
 static AnimatedPathProfile Profile(){return new AnimatedPathProfile{Schema=2,Spline=true,VisualMeshSignature="test",BoneNames=new[]{"root","tip"},MorphNames=new[]{"shape"},PointOffsets=new[]{0,2,4},BoneIndices=new[]{0,1,0,1},Weights=new[]{.4,.6,.8,.2},Rest=new[]{new Vec3(-.1,0,0),new Vec3(.2,.1,0),new Vec3(.3,0,0),new Vec3(.3,.1,0)},MorphDeltas=new[]{new[]{new Vec3(0,0,.01),new Vec3(0,0,.02),new Vec3(0,0,.03),new Vec3(0,0,.04)}}};}
 public static void Run(Action<string,Action> test){
  test("1.0.6 legacy PathFile remains readable and animated profiles reject wrong rig",()=>{
   var old=JsonCodec.Deserialize<AnimatedPathProfile>("{\"Points\":[{\"X\":0,\"Y\":0,\"Z\":0},{\"X\":0,\"Y\":1,\"Z\":0}],\"Spline\":true}");old.Validate(null);Check(!old.Animated&&old.ControlCount==2,"legacy path broken");var p=Profile();p.Validate(Definition());int rejected=0;p.VisualMeshSignature="wrong";try{p.Validate(Definition());}catch(ArgumentException){rejected++;}p=Profile();p.BoneNames[1]="different";try{p.Validate(Definition());}catch(ArgumentException){rejected++;}p=Profile();p.Weights[0]=double.NaN;try{p.Validate(Definition());}catch(ArgumentException){rejected++;}Check(rejected==3,"bad animated binding accepted");
  });
  test("1.0.6 animated path follows bone transforms and morphs with exact aggregate LBS",()=>{
   var p=Profile();p.Validate(Definition());var linear=new[]{new Linear3(new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1)),new Linear3(new Vec3(0,2,0),new Vec3(-1,0,0),new Vec3(0,0,.5))};var translations=new[]{new Vec3(1,2,3),new Vec3(-1,3,1)};var morph=new[]{.7};var output=new Vec3[2];p.Capture(linear,translations,morph,output);Vec3 expected=new Vec3();for(int t=0;t<2;t++)expected+=(linear[p.BoneIndices[t]].Multiply(p.Rest[t]+p.MorphDeltas[0][t]*morph[0])+translations[p.BoneIndices[t]])*p.Weights[t];Check((output[0]-expected).Length<1e-12,"LBS mismatch");var before=output[0];translations[1]+=new Vec3(.03,0,0);p.Capture(linear,translations,morph,output);Check((output[0]-before-new Vec3(.018,0,0)).Length<1e-12,"bone motion lost");before=output[0];morph[0]=.9;p.Capture(linear,translations,morph,output);Check((output[0]-before).Length>1e-4,"morph lost");
   long bytes=GC.GetAllocatedBytesForCurrentThread();for(int f=0;f<300;f++)p.Capture(linear,translations,morph,output);Check(GC.GetAllocatedBytesForCurrentThread()==bytes,"animated path capture allocates");
  });
  test("1.0.6 translation survives a saturated angular request with exact corner budget",()=>{
   var body=new SourceContactBody{Surface=new BoundsOnly{ContactBounds=new Bounds3(new Vec3(-.01,-.17,-.01),new Vec3(.01,.01,.01))}};var settings=new BidirectionalContactSettings();body.BeginFrame(settings,.016);var translation=new Vec3(.024,0,0);body.ApplyGuidance(translation,new Vec3(0,0,.35),new Vec3());Check((body.Offset-translation).Length<1e-12,"rotation erased guide translation");Check(body.RotationVector.Length>0&&body.MaximumCorrection<=.050000000001,"corner budget invalid");Check(body.AtLimit,"saturation missing");
   var random=new Random(106);for(int i=0;i<600;i++){body.Correct(new Vec3(random.NextDouble()-.5,random.NextDouble()-.5,random.NextDouble()-.5)*.02);Check(body.MaximumCorrection<=.050000001,"contact exceeded rotating geometry cap");}body.BeginFrame(settings,.016);Check(body.RotationVector.Length==0,"guide rotation retained after disabled");
  });
  test("1.0.6 shear guide retains root, path and no-allocation vertex mapping",()=>{
   var path=new AnatomicalPathConstraintSystem(new[]{new Vec3(0,0,0),new Vec3(.01,.05,.005),new Vec3(.02,.1,.01)});var rod=new SourceRodGuide();Vec3 root=new Vec3(0,-.05,0),tip=new Vec3(0,.1,0);var mesh=LabScenarios.Tube(16,12,.02,.15);for(int i=0;i<mesh.Vertices.Length;i++)mesh.Vertices[i]+=new Vec3(0,.025,0);var output=new Vec3[mesh.Vertices.Length];
   for(int i=0;i<180;i++)rod.Capture(path,root,tip,1.0/60,.04);rod.MapVertices(mesh.Vertices,output);Check((rod.Map(root)-root).Length==0,"root moved");Check(path.Closest(rod.Map(tip)).Distance<.002,"tip missed path");for(int i=0;i<output.Length;i++)Check((output[i]-mesh.Vertices[i]).Length<=.04000001,"warp cap exceeded");
   long bytes=GC.GetAllocatedBytesForCurrentThread();for(int f=0;f<60;f++){rod.Capture(path,root,tip,.016,.04);rod.MapVertices(mesh.Vertices,output);}Check(GC.GetAllocatedBytesForCurrentThread()==bytes,"guide rebuilt working structures");var body=new SourceContactBody();body.BeginFrame(new BidirectionalContactSettings(),.016);body.ApplyDeformation(rod);body.MaximumContactTravel=.002;body.Correct(new Vec3(.1,0,0));Check(body.Offset.Length<=.002000001&&body.MaximumCorrection<=.050000001,"contact escaped guide or total cap");
   rod.Capture(path,root,tip,.016,.008);rod.MapVertices(mesh.Vertices,output);for(int i=0;i<output.Length;i++)Check((output[i]-mesh.Vertices[i]).Length<=.008000001,"live reduction of source cap retained excessive deformation");
  });
 }
}
