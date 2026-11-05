using System;
using BetterPenetrationNext.Core;
using UnityEngine;
namespace BetterPenetrationNext
{
 public sealed partial class NextNativeTests
 {
  void RunSectionRigNative()
  {
   var owner=new GameObject("Neutral guide owner");assets.Add(owner);var other=new GameObject("Neutral unrelated guide owner");assets.Add(other);var target=new GameObject(Core_BetterPenetration.LookTargets.KokanTarget);assets.Add(target);target.transform.SetParent(owner.transform,false);
   Require(!NextRuntime.GuideTargetMatches(null,owner.transform,true),"1.0.7 idle target does not activate a prepared guide");
   Require(NextRuntime.GuideTargetMatches(target.transform,owner.transform,true),"1.0.7 matching target activates its receiver guide");
   Require(!NextRuntime.GuideTargetMatches(target.transform,other.transform,true),"1.0.7 guide never follows another receiver target");
   target.name="Neutral unrelated target";Require(!NextRuntime.GuideTargetMatches(target.transform,owner.transform,true),"1.0.7 unrelated target does not activate channel guide");
   Require(NextRuntime.GuideTargetMatches(null,owner.transform,false),"1.0.7 explicit neutral guide override remains available");
   var data=LabScenarios.Tube(24,16,.01,.12);var renderer=Skin("Neutral independently guided section shell",data,Vector3.zero);var original=renderer.sharedMesh;
   var first=renderer.bones[0];first.localPosition=new Vector3(0,-.06f,0);var last=new GameObject("Neutral section tip bone").transform;last.SetParent(renderer.transform,false);last.localPosition=new Vector3(0,.06f,0);
   renderer.bones=new[]{first,last};original.bindposes=new[]{first.worldToLocalMatrix*renderer.transform.localToWorldMatrix,last.worldToLocalMatrix*renderer.transform.localToWorldMatrix};
   var weights=original.boneWeights;for(int i=0;i<weights.Length;i++){float u=Mathf.Clamp01((float)(data.Vertices[i].Y/.12+.5));weights[i]=new BoneWeight{boneIndex0=0,weight0=1-u,boneIndex1=1,weight1=u};}original.boneWeights=weights;
   var deltas=new Vector3[weights.Length];for(int i=0;i<deltas.Length;i++)deltas[i]=new Vector3(.001f,0,0);original.AddBlendShapeFrame("Section fixture morph",100,deltas,new Vector3[deltas.Length],new Vector3[deltas.Length]);
   var evidence=new double[weights.Length];for(int i=0;i<evidence.Length;i++)evidence[i]=1;
   using(var geometry=new ContactDetection(renderer,evidence))
   {
    var rig=geometry.CreateSectionRig(first,last);var path=new AnatomicalPathConstraintSystem(new[]{AnimationAdapter.ToCore(first.position),AnimationAdapter.ToCore(renderer.transform.position),AnimationAdapter.ToCore(last.position)},false);var body=new SourceContactBody{Surface=geometry,PreserveContactAcrossDeformation=true};var settings=new BidirectionalContactSettings();var baked=new Mesh();assets.Add(baked);double maximum=0,movement=0;
    for(int frame=0;frame<60;frame++)
    {
     first.localRotation=Quaternion.Euler(0,0,20+Mathf.Sin(frame*.1f)*5);last.localRotation=Quaternion.Euler(0,0,-20-Mathf.Sin(frame*.1f)*5);renderer.SetBlendShapeWeight(0,frame);
     geometry.Capture(true,true);rig.Capture(path,geometry.AnimatedVertices,AnimationAdapter.ToCore(first.position),1.0/60,.04);body.BeginFrame(settings,1.0/60);geometry.RefitDeformation(rig);body.ApplyDeformation(rig);body.TranslationAxis=new Vec3(0,1,0);body.MaximumContactTravel=.02;body.Correct(new Vec3(0,.001,0));body.MarkContact();geometry.ApplyCorrection(body);
     renderer.BakeMesh(baked);var vertices=baked.vertices;var basis=AnimationAdapter.BakedToWorld(renderer);
     for(int i=0;i<vertices.Length;i++){var actual=AnimationAdapter.ToCore(basis.MultiplyPoint3x4(vertices[i]));var expected=body.ToCorrectedVertex(i,geometry.AnimatedVertices[i]);maximum=Math.Max(maximum,(actual-expected).Length);movement=Math.Max(movement,(actual-geometry.AnimatedVertices[i]).Length);}
     Require(body.MaximumCorrection<=.05000001,"1.0.7 section rig total source cap "+frame);
    }
    Require(body.Offset.Y>.015,"1.0.8 animated section guidance keeps previous contact support","offset="+body.Offset);
    Require(maximum<.00005&&movement>.002,"1.0.7 indexed section rig matches native two-bone animation and Blend Shapes","maximumNativeError="+maximum+" maximumMovement="+movement);
    Require(!geometry.HasVolumeBounds,"1.0.7 section rig retains actual open rims");SurfaceSample sample;var point=body.ToCorrectedVertex(30,geometry.AnimatedVertices[30]);Require(body.TrySample(point,.1,out sample)&&Math.Abs(sample.SignedDistance)<.00005,"1.0.7 indexed collision and rendered source share coordinates");
    geometry.ReleaseCorrection();Require(renderer.sharedMesh==original&&renderer.bones[0]==first&&renderer.bones[1]==last,"1.0.7 section rig restores original mesh and leaves animation bones attached");
   }
   renderer.enabled=false;
  }
 }
}
