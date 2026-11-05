using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Reflection;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        IEnumerator RunPrepared(string path)
        {
            var definition=UncensorIntegration.LoadDefinition(path); Require(definition!=null,"prepared package metadata loads in KKS Mono");
            byte[] bytes;
            using(var file=File.OpenRead(path)) using(var zip=new ZipArchive(file,ZipArchiveMode.Read))
            { var entry=zip.GetEntry("abdata/chara/next/oo_base_bpn_1.unity3d"); using(var memory=new MemoryStream()) { using(var input=entry.Open()) input.CopyTo(memory); bytes=memory.ToArray(); } }
            var bundle=AssetBundle.LoadFromMemory(bytes); Require(bundle!=null,"user-edited AssetBundle loads in actual Unity 2019.4.9");
            PreparedMaterials materialBinding=null; MeshDeformationBinding preparedBinding=null; GameObject holder=null; GameObject skin=null;
            try
            {
                var prefab=bundle.LoadAsset<GameObject>("p_cf_body_bpn"); Require(prefab!=null,"new uniquely named prepared prefab loads");
                Require(bundle.LoadAsset<GameObject>("p_cf_body_bpn_low")==prefab,"selector low-detail resource name resolves to the same compatible prepared rig");
                SkinnedMeshRenderer originalRenderer=null;
                foreach(var renderer in prefab.GetComponentsInChildren<SkinnedMeshRenderer>(true)) if(renderer.sharedMesh!=null&&renderer.sharedMesh.bindposes.Length==definition.BoneNames.Length&&renderer.sharedMesh.vertexCount>10000) originalRenderer=renderer;
                Require(originalRenderer!=null,"prepared high-detail body renderer has the expected rig"); var mesh=originalRenderer.sharedMesh; definition.Validate(mesh.vertexCount);
                Require(mesh.isReadable&&mesh.blendShapeCount==2&&mesh.bindposes.Length==138,"user morphs, CPU geometry and four custom bones survive packaging");
                var counts=mesh.GetBonesPerVertex(); int maxWeights=0; foreach(byte count in counts) maxWeights=Math.Max(maxWeights,count);
                Require(maxWeights==8,"authored eight-influence vertices survive the native mesh format");
                skin=new GameObject("BPN prepared native skin"); skin.transform.SetParent(root.transform,false); var receiver=skin.AddComponent<SkinnedMeshRenderer>();
                var bones=new Transform[definition.BoneNames.Length]; var poses=mesh.bindposes;
                for(int i=0;i<bones.Length;i++)
                {
                    var go=new GameObject(definition.BoneNames[i]); go.transform.SetParent(skin.transform,false); var matrix=poses[i].inverse;
                    go.transform.localPosition=matrix.GetColumn(3); go.transform.localRotation=matrix.rotation; go.transform.localScale=matrix.lossyScale; bones[i]=go.transform;
                }
                receiver.bones=bones; receiver.rootBone=bones[0]; receiver.sharedMesh=mesh; receiver.updateWhenOffscreen=true;
                var material=new Material(Shader.Find("Standard")); assets.Add(material); receiver.sharedMaterial=material;
                materialBinding=new PreparedMaterials(receiver,path);
                Require(receiver.sharedMaterials.Length==mesh.subMeshCount&&receiver.sharedMaterials[0]==material,"authored internal materials preserve the original external skin slot");
                preparedBinding=new MeshDeformationBinding(receiver,definition); var animation=new AnimationAdapter(definition,receiver);
                var solver=new SoftBodySolver(new SimulationTopology(definition.Rest,definition.Triangles),definition.PinWeights); animation.Capture(null); solver.Reset(animation.Anchors);
                var rest=(Vec3[])animation.Anchors.Clone();
                for(int morph=0;morph<definition.Morphs.Length;morph++)
                {
                    int index=mesh.GetBlendShapeIndex(definition.Morphs[morph].Name); receiver.SetBlendShapeWeight(index,100); animation.Capture(null);
                    double changed=0; for(int i=0;i<rest.Length;i++) changed=Math.Max(changed,(animation.Anchors[i]-rest[i]).Length);
                    Require(changed>1e-5,"authored morph updates physical anchors: "+definition.Morphs[morph].Name,"max="+changed); receiver.SetBlendShapeWeight(index,0);
                }
                animation.Capture(null); solver.Reset(animation.Anchors);
                int free=-1; for(int i=0;i<definition.PinWeights.Length;i++) if(definition.PinWeights[i]<0.01) { free=i; break; }
                Require(free>=0,"prepared proxy contains free physical particles"); var center=animation.Anchors[free];
                var probe=PrimitiveMeshes.Sphere(24,12,animation.PosedSize*0.04); for(int i=0;i<probe.Vertices.Length;i++) probe.Vertices[i]+=center;
                var obstacle=new TriangleSurface(probe.Vertices,probe.Triangles,SurfaceKind.ClosedVolume) { RobustSign=false }; var settings=new SoftBodySettings { PreserveVolume=definition.PreserveVolume };
                for(int frame=0;frame<90;frame++) { animation.Capture(solver); solver.Advance(animation.Anchors,obstacle,settings,PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,animation.PosedSize); preparedBinding.Apply(solver,animation.PosedSize*0.15,3); if(frame%10==0) yield return null; }
                double max=0; for(int i=0;i<rest.Length;i++) max=Math.Max(max,solver.SolvedDisplacement(i).Length);
                Require(max>1e-5,"actual user-mesh simulation responds to a neutral contact probe","maximum="+max);
                Require(preparedBinding.VerifyAppliedPose(solver)<Math.Max(0.0001,animation.PosedSize*0.001),"actual skinned user geometry receives the calculated physical deformation");
                var xray=preparedBinding.XRaySnapshot();
                Require(xray.GetIndexCount(0)==0,"external body submesh is absent from the receiver X-Ray geometry");
                long diagnostic=0; for(int i=1;i<xray.subMeshCount;i++) diagnostic+=xray.GetIndexCount(i);
                Require(diagnostic>0,"receiver X-Ray contains authored internal working geometry");
                var baselineNormals=mesh.normals; var appliedNormals=receiver.sharedMesh.normals; var external=mesh.GetTriangles(0); double outsideError=0;
                foreach(int i in external) outsideError=Math.Max(outsideError,(baselineNormals[i]-appliedNormals[i]).magnitude);
                Require(outsideError<1e-6,"physical normal updates preserve all external-body authored normals","error="+outsideError);
                var rotations=new Quaternion[bones.Length]; for(int i=0;i<bones.Length;i++) rotations[i]=bones[i].localRotation;
                for(int frame=0;frame<30;frame++)
                {
                    for(int i=0;i<bones.Length;i++) bones[i].localRotation=rotations[i]*Quaternion.Euler(0,2*Mathf.Sin(frame*0.1f+i*0.17f),0);
                    animation.Capture(solver); solver.Advance(animation.Anchors,obstacle,settings,PhysicsPolicy.Resolve(PhysicsQuality.Balanced),1.0/60,animation.PosedSize); preparedBinding.Apply(solver,animation.PosedSize*0.15,3);
                    if(frame%10==0) yield return null;
                }
                Require(preparedBinding.VerifyAppliedPose(solver)<0.0003,"eight-weight authored rig remains correct under independently animated bones");
                for(int i=0;i<bones.Length;i++) bones[i].localRotation=rotations[i];
                var benchmark=BenchmarkPrepared(receiver,definition,solver,animation,preparedBinding); while(benchmark.MoveNext()) yield return benchmark.Current;
                // Exercise the installed MBI implementation, not an imitation of it.
                holder=new GameObject("BPN inactive implant fixture"); holder.SetActive(false); var instance=Object.Instantiate(prefab,holder.transform,false);
                Type implant=Type.GetType("ModBoneImplantor.BoneImplantProcess, ModBoneImplantor",true); var components=instance.GetComponentsInChildren(implant,true);
                Require(components.Length==12,"original eleven implants and one new custom-rig implant are retained");
                var destinations=new Dictionary<string,GameObject>(); var srcField=implant.GetField("trfSrc"); var dstField=implant.GetField("trfDst");
                foreach(var component in components)
                {
                    var target=(Transform)dstField.GetValue(component); if(destinations.ContainsKey(target.name)) continue;
                    var go=new GameObject(target.name); go.transform.SetParent(root.transform,false); destinations.Add(target.name,go);
                }
                var type=Type.GetType("ModBoneImplantor.ModBoneImplantor, ModBoneImplantor",true); var method=type.GetMethod("TryImplantBones",BindingFlags.Static|BindingFlags.NonPublic);
                object info=method.Invoke(null,new object[]{instance,destinations}); Require(info!=null,"installed ModBoneImplantor accepts the packaged transplant definitions");
                var implanted=(List<Transform>)info.GetType().GetField("ImplantedBones").GetValue(info); int customCount=0; foreach(var bone in implanted) if(bone.name.StartsWith("MED_",StringComparison.Ordinal)) customCount++;
                Require(customCount==4,"all four custom bones are transplanted by installed ModBoneImplantor");
                var selector=Type.GetType("KK_Plugins.UncensorSelector, KKS_UncensorSelector",true); var dictionary=(IDictionary)selector.GetField("BodyDictionary").GetValue(null);
                for(int frame=0;frame<300&&!dictionary.Contains(UncensorIntegration.PreparedGuid);frame++) yield return null;
                Require(dictionary.Contains(UncensorIntegration.PreparedGuid),"Sideloader and installed UncensorSelector register the new unique selectable body");
                preparedBinding.Dispose(); preparedBinding=null; materialBinding.Dispose(); materialBinding=null;
                Require(receiver.sharedMesh==mesh&&receiver.sharedMaterials.Length==1&&receiver.sharedMaterial==material,"prepared physics and material bindings restore their exact initial state");
                report.checks.Add(new Check { name="real character selection, animation and visual clinical correctness",status="SKIP",detail="Package, native skinning, selector catalogue and actual implant code are checked; no user character/animation or medical validation is certified by the neutral probe." });
            }
            finally { if(preparedBinding!=null) preparedBinding.Dispose(); if(materialBinding!=null) materialBinding.Dispose(); if(skin!=null) Object.Destroy(skin); if(holder!=null) Object.Destroy(holder); bundle.Unload(false); }
        }
    }
}
