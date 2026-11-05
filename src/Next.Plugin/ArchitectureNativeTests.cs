using System;
using System.Collections;
using BetterPenetrationNext.Core;
using UnityEngine;
namespace BetterPenetrationNext
{
 public sealed partial class NextNativeTests
 {
  IEnumerator RunArchitecture()
  {
   RunAnimatedPath();
   RunSourceNormals();
   RunRodNative();
   RunSectionRigNative();
   RunExternalCopyNative();
   var v=new[]{new Vec3(),new Vec3(1,0,0),new Vec3(0,1,0),new Vec3(0,0,1),new Vec3()};var t=new[]{0,2,1,4,1,3,0,3,2,1,2,3};var r=Skin("Neutral split-weight contact proxy",new MeshData{Vertices=v,Triangles=t},Vector3.zero);
   var mesh=r.sharedMesh;var b=new GameObject("Independent seam bone");b.transform.SetParent(r.transform,false);assets.Add(b);r.bones=new[]{r.bones[0],b.transform};mesh.bindposes=new[]{Matrix4x4.identity,Matrix4x4.identity};var weights=mesh.boneWeights;weights[4]=new BoneWeight{boneIndex0=1,weight0=1};mesh.boneWeights=weights;var evidence=new double[v.Length];for(int i=0;i<evidence.Length;i++)evidence[i]=1;
   using(var geometry=new ContactDetection(r,evidence)){
    b.transform.localPosition=new Vector3(.002f,0,0);geometry.Capture(true);
    Require(geometry.UsesSeamProxy&&geometry.HasVolumeBounds&&geometry.VirtualContactTriangles>0,"1.0.5 native split-weight seam uses bounded proxy");
    Require((geometry.AnimatedVertices[0]-geometry.AnimatedVertices[4]).Length>.001,"1.0.5 native proxy retains distinct independently skinned vertices");
    Require(r.sharedMesh==mesh&&r.sharedMesh.triangles.Length==t.Length,"1.0.5 contact proxy preserves native rendering topology");
    geometry.EnableSeamProxy=false;geometry.Capture(true);Require(!geometry.UsesSeamProxy&&!geometry.HasVolumeBounds,"1.0.5 proxy toggle restores true open shell without rebuilding native mesh");
    geometry.EnableSeamProxy=true;b.transform.localPosition=new Vector3(.8f,0,0);geometry.Capture(true);Require(!geometry.UsesSeamProxy,"1.0.5 oversized animated seam falls back to shell");
   }
   var go=new GameObject("Neutral 3D path guide");assets.Add(go);var guide=go.AddComponent<AnatomicalPathGuide>();guide.ControlPoints=new Transform[3];for(int i=0;i<3;i++){var p=new GameObject("Control "+i);p.transform.SetParent(go.transform,false);p.transform.position=new Vector3(100,99.5f+i*.5f,100+i*.03f);guide.ControlPoints[i]=p.transform;}
   var first=guide.Capture();guide.ControlPoints[1].position+=new Vector3(.01f,0,0);Require(ReferenceEquals(first,guide.Capture()),"1.0.5 moving Unity guide reuses path and sample buffers");
   var controller=new PathGuidanceController();var sourceBody=new SourceContactBody();var settings=new BidirectionalContactSettings();Vec3 last=new Vec3();
   for(int i=0;i<90;i++){sourceBody.BeginFrame(settings,1.0/60);var pose=controller.Step(guide.Capture(),new Vec3(100.03,100,100),new Vec3(.2,1,.1),1.0/60,.05,.35,12,1);Require((pose.RotationVector-last).Length<=1.0/60+1e-8,"1.0.5 guide angular step bounded "+i);last=pose.RotationVector;PathGuidanceController.Apply(sourceBody,pose);if(i%30==0)yield return null;}
   Require(sourceBody.Offset.Length>0&&sourceBody.Offset.Length<=.05000001,"1.0.5 independent native guide applies bounded correction");controller.Reset();sourceBody.BeginFrame(settings,.016);Require(sourceBody.Offset.Length<1e-10&&sourceBody.RotationVector.Length==0,"1.0.5 guide removal clears its contribution without drift");r.enabled=false;
  }
 }
}

