using System;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;
namespace BetterPenetrationNext
{
 public sealed partial class NextNativeTests
 {
  void RunExternalCopyNative()
  {
   var visual=PrimitiveMeshes.Grid(8,8,.1,.1);var simulation=PrimitiveMeshes.Grid(4,4,.1,.1);var renderer=Skin("Neutral copied receiver baseline",visual,Vector3.zero);var original=renderer.sharedMesh;
   var definition=Definition(original,renderer,simulation);var animation=new AnimationAdapter(definition,renderer);animation.Capture(null);var solver=new SoftBodySolver(new SimulationTopology(definition.Rest,definition.Triangles),definition.PinWeights);
   var obstacle=PrimitiveMeshes.Sphere(16,8,.03);for(int i=0;i<obstacle.Vertices.Length;i++)obstacle.Vertices[i]+=AnimationAdapter.ToCore(renderer.transform.position)+new Vec3(0,0,.01);
   var surface=new TriangleSurface(obstacle.Vertices,obstacle.Triangles,SurfaceKind.ClosedVolume);surface.RobustSign=false;var settings=new SoftBodySettings();
   for(int i=0;i<60;i++)solver.Advance(animation.Anchors,surface,settings,PhysicsPolicy.Resolve(PhysicsQuality.Performance),1.0/60,animation.PosedSize);
   using(var dirty=new MeshDeformationBinding(renderer,definition))
   {
    dirty.Apply(solver,.1,1);var vertices=renderer.sharedMesh.vertices;double displacement=0;for(int i=0;i<vertices.Length;i++)displacement=Math.Max(displacement,(vertices[i]-AnimationAdapter.ToUnity(visual.Vertices[i])).magnitude);
    Require(displacement>.001,"1.0.7 external clone fixture contains actual receiver contact offsets");
    var copied=Object.Instantiate(renderer.sharedMesh);assets.Add(copied);renderer.sharedMesh=copied;
    bool rejected=false;try{using(var invalid=new MeshDeformationBinding(renderer,definition)){} }catch(ArgumentException){rejected=true;}
    Require(rejected,"1.0.7 copying a working mesh reproduces prepared signature rejection");
   }
   renderer.sharedMesh=original;
   var clean=new MeshDeformationBinding(renderer,definition);clean.Apply(solver,.1,1);clean.Dispose();
   var external=Object.Instantiate(renderer.sharedMesh);assets.Add(external);var deltas=new Vector3[external.vertexCount];for(int i=0;i<deltas.Length;i++)deltas[i]=new Vector3(0,0,.001f);external.AddBlendShapeFrame("Third party temporary morph",100,deltas,new Vector3[deltas.Length],new Vector3[deltas.Length]);renderer.sharedMesh=external;renderer.SetBlendShapeWeight(0,0);
   using(var rebound=new MeshDeformationBinding(renderer,definition))
   {
    Require(renderer.sharedMesh.GetBlendShapeIndex("Third party temporary morph")>=0&&renderer.GetBlendShapeWeight(0)==0,"1.0.7 clean baseline copy preserves the external morph");
    rebound.Apply(solver,.1,1);Require(rebound.VerifyAppliedPose(solver)<.00005,"1.0.7 clean clone rebind reproduces correct native tissue displacement");
   }
   double error=0;var rest=original.vertices;for(int i=0;i<rest.Length;i++)error=Math.Max(error,(rest[i]-AnimationAdapter.ToUnity(visual.Vertices[i])).magnitude);Require(error<1e-9,"1.0.7 external copy compatibility leaves authored source geometry unchanged");renderer.enabled=false;
  }
 }
}
