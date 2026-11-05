using System;
using BetterPenetrationNext.Core;
using UnityEngine;
namespace BetterPenetrationNext
{
 public sealed partial class NextNativeTests
 {
  void RunRodNative()
  {
   var data=LabScenarios.Tube(24,12,.015,.12);var r=Skin("Neutral open guided shell",data,Vector3.zero);var original=r.sharedMesh;var normal=original.normals;
   var delta=new Vector3[data.Vertices.Length];var zero=new Vector3[delta.Length];for(int i=0;i<delta.Length;i++)delta[i]=new Vector3(.001f,0,0);original.AddBlendShapeFrame("Guided source shape",100,delta,zero,zero);
   var evidence=new double[data.Vertices.Length];for(int i=0;i<evidence.Length;i++)evidence[i]=1;
   using(var geometry=new ContactDetection(r,evidence)){
    var path=new AnatomicalPathConstraintSystem(new[]{new Vec3(100,99.99,100),new Vec3(100.003,100.02,100.002),new Vec3(100.009,100.06,100.006)});var rod=new SourceRodGuide();var body=new SourceContactBody{Surface=geometry};var settings=new BidirectionalContactSettings();var baked=new Mesh();assets.Add(baked);double maximum=0,motion=0;
    for(int f=0;f<60;f++){
     r.SetBlendShapeWeight(0,f);r.bones[0].localRotation=Quaternion.Euler(0,0,Mathf.Sin(f*.1f)*2);geometry.Capture(true,true);var boneMatrix=r.bones[0].localToWorldMatrix;Vec3 a=AnimationAdapter.ToCore(boneMatrix.MultiplyPoint3x4(new Vector3(0,-.06f,0))),b=AnimationAdapter.ToCore(boneMatrix.MultiplyPoint3x4(new Vector3(0,.06f,0)));
     rod.Capture(path,a,b,1.0/60,.04,12);body.BeginFrame(settings,1.0/60);body.ApplyDeformation(rod);geometry.RefitDeformation(rod);body.TranslationAxis=path.Closest(rod.Map(b)).Tangent;body.MaximumContactTravel=.002;body.Correct(body.TranslationAxis*.001);
     geometry.ApplyCorrection(body);r.BakeMesh(baked);var actual=baked.vertices;var transform=AnimationAdapter.BakedToWorld(r);
     for(int i=0;i<actual.Length;i++){var p=AnimationAdapter.ToCore(transform.MultiplyPoint3x4(actual[i]));maximum=Math.Max(maximum,(p-body.ToCorrected(geometry.AnimatedVertices[i])).Length);motion=Math.Max(motion,(p-geometry.AnimatedVertices[i]).Length);}
     Require(body.MaximumCorrection<=.05000001,"1.0.6 rod total guide plus contact budget "+f);
    }
    Require(maximum<.00005&&motion>.002,"1.0.6 open-shell rod mapping matches native bones and Blend Shapes","maximumNativeError="+maximum+" maximumMovement="+motion);
    Require(!geometry.HasVolumeBounds,"1.0.6 true open tube remains open under rod mapping");
    var probe=rod.Map(geometry.AnimatedVertices[20]);SurfaceSample sample;Require(body.TrySample(probe+body.Offset,.1,out sample)&&Math.Abs(sample.SignedDistance)<.00002,"1.0.6 warped contact representation matches actual guided surface");
    geometry.ReleaseCorrection();Require(r.sharedMesh==original,"1.0.6 disabling rod restores exact original shared mesh");
   }r.enabled=false;
  }
 }
}
