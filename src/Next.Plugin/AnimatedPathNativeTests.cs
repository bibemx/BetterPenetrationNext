using System;
using BetterPenetrationNext.Core;
using UnityEngine;
namespace BetterPenetrationNext
{
 public sealed partial class NextNativeTests
 {
  void RunAnimatedPath()
  {
   var grid=PrimitiveMeshes.Grid(2,2,.1,.1);var r=Skin("Neutral bone bound path",grid,Vector3.zero);var original=r.sharedMesh;var d=Definition(original,r,grid);
   var delta=new Vector3[grid.Vertices.Length];var zero=new Vector3[delta.Length];var core=new Vec3[delta.Length];for(int i=0;i<delta.Length;i++){delta[i]=new Vector3(0,0,.01f);core[i]=AnimationAdapter.ToCore(delta[i]);}
   original.AddBlendShapeFrame("Path shape",100,delta,zero,zero);d.Morphs=new[]{new SimulationMorph{Name="Path shape",Deltas=core}};
   var profile=new NextRuntime.PathProfile{Schema=2,Spline=false,VisualMeshSignature=d.VisualMeshSignature,BoneNames=d.BoneNames,MorphNames=new[]{"Path shape"},PointOffsets=new[]{0,1,2},BoneIndices=new[]{0,0},Weights=new[]{1.0,1.0},Rest=new[]{grid.Vertices[0],grid.Vertices[grid.Vertices.Length-1]},MorphDeltas=new[]{new[]{core[0],core[core.Length-1]}}};
   profile=JsonCodec.Deserialize<NextRuntime.PathProfile>(JsonCodec.Serialize(profile));profile.Validate(d);var animation=new AnimationAdapter(d,r);var output=new Vec3[2];var baked=new Mesh();assets.Add(baked);double max=0;var rendererMatrix=r.transform.localToWorldMatrix;
   for(int f=0;f<40;f++){
    r.bones[0].localRotation=Quaternion.Euler(0,f*.4f,0);r.bones[0].localPosition=new Vector3(.01f*Mathf.Sin(f*.1f),0,0);r.SetBlendShapeWeight(0,f*2);animation.Capture(null);animation.CapturePath(profile,output);r.BakeMesh(baked);var actual=baked.vertices;var matrix=AnimationAdapter.BakedToWorld(r);max=Math.Max(max,(output[0]-AnimationAdapter.ToCore(matrix.MultiplyPoint3x4(actual[0]))).Length);max=Math.Max(max,(output[1]-AnimationAdapter.ToCore(matrix.MultiplyPoint3x4(actual[actual.Length-1]))).Length);
   }
   Require(r.transform.localToWorldMatrix==rendererMatrix,"1.0.6 animated guide tested with unchanged renderer matrix");
   Require(max<.00003,"1.0.6 bone and morph bound controls match native BakeMesh","maximumError="+max);
   Require(output[0].Z>100.0001,"1.0.6 path receives actual changing Blend Shape contribution");
   var old=JsonCodec.Deserialize<NextRuntime.PathProfile>("{\"Points\":[{\"X\":0,\"Y\":0,\"Z\":0},{\"X\":0,\"Y\":1,\"Z\":0}],\"Spline\":true}");old.Validate(d);Require(!old.Animated&&old.ControlCount==2,"1.0.6 native legacy PathFile remains compatible");
   using(var stream=typeof(NextRuntime).Assembly.GetManifestResourceStream("BetterPenetrationNext.receiver-path.json"))Require(stream!=null,"1.0.6 model-specific animated guide embedded in shipping DLL");
   r.enabled=false;
  }
 }
}
