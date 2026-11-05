using System;
using System.Collections;
using System.IO;
using System.IO.Compression;
using System.Diagnostics;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;
namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        IEnumerator RunUvNative()
        {
            string archive=null;foreach(string arg in Environment.GetCommandLineArgs())if(arg.StartsWith("--bpn-uv-archive="))archive=arg.Substring(17);
            Require(!string.IsNullOrEmpty(archive),"actual receiver ZIPMOD supplied for UV validation");byte[] bytes;SimulationDefinition d;
            using(var file=File.OpenRead(archive))using(var zip=new ZipArchive(file,ZipArchiveMode.Read)){
                using(var reader=new StreamReader(zip.GetEntry("bpn/reactive/support.json").Open()))d=JsonCodec.Deserialize<SimulationDefinition>(reader.ReadToEnd());
                using(var stream=zip.GetEntry("abdata/chara/next/oo_base_bpn_1.unity3d").Open())using(var memory=new MemoryStream()){stream.CopyTo(memory);bytes=memory.ToArray();}
            }
            Require(d.UvPatch!=null&&d.UvPatch.Duplicates.Length>0,"actual package UV metadata survives native Mono serialization");var bundle=AssetBundle.LoadFromMemory(bytes);Require(bundle!=null,"actual receiver bundle loads for render remap");
            MeshDeformationBinding remap=null;PreparedMaterials materials=null;var skin=new GameObject("Actual receiver neutral UV validation");skin.transform.SetParent(root.transform,false);
            try{
                var prefab=bundle.LoadAsset<GameObject>("p_cf_body_bpn");Mesh mesh=null;foreach(var item in prefab.GetComponentsInChildren<SkinnedMeshRenderer>(true))if(item.sharedMesh!=null&&item.sharedMesh.vertexCount==56130)mesh=item.sharedMesh;
                Require(mesh!=null,"actual prepared receiver geometry is used");var r=skin.AddComponent<SkinnedMeshRenderer>();r.sharedMesh=mesh;var bones=new Transform[d.BoneNames.Length];var poses=mesh.bindposes;
                for(int i=0;i<bones.Length;i++){var go=new GameObject(d.BoneNames[i]);go.transform.SetParent(skin.transform,false);var m=poses[i].inverse;go.transform.localPosition=m.GetColumn(3);go.transform.localRotation=m.rotation;go.transform.localScale=m.lossyScale;bones[i]=go.transform;}
                r.bones=bones;r.rootBone=bones[0];r.updateWhenOffscreen=true;var material=new Material(Shader.Find("Standard"));assets.Add(material);r.sharedMaterial=material;materials=new PreparedMaterials(r,archive);
                Require(NativeSkinContract.Verify(d,r)==8,"expanded exterior metadata matches every native skin identity");
                var originalUv=mesh.uv;var originalTriangles=mesh.triangles;var originalVertices=mesh.vertices;var oldCounts=mesh.GetBonesPerVertex();var oldWeights=mesh.GetAllBoneWeights();
                remap=new MeshDeformationBinding(r,d);var working=r.sharedMesh;var counts=working.GetBonesPerVertex();var weights=working.GetAllBoneWeights();var starts=new int[mesh.vertexCount+1];for(int i=0;i<mesh.vertexCount;i++)starts[i+1]=starts[i]+oldCounts[i];
                int cursor=0,maxWeights=0;bool same=true;for(int i=0;i<working.vertexCount;i++){int src=i<mesh.vertexCount?i:d.UvPatch.Duplicates[i-mesh.vertexCount];same&=counts[i]==oldCounts[src];maxWeights=Math.Max(maxWeights,counts[i]);for(int j=0;j<counts[i];j++){var a=weights[cursor++];var b=oldWeights[starts[src]+j];same&=a.boneIndex==b.boneIndex&&a.weight==b.weight;}}
                Require(same&&maxWeights==8,"UV seam copies preserve every native bone influence","maximumInfluences="+maxWeights);
                Require(working.vertexCount==mesh.vertexCount+d.UvPatch.Duplicates.Length&&working.blendShapeCount==mesh.blendShapeCount,"render seam splits preserve authored Blend Shape set");
                var authoredTangents=mesh.tangents;var mappedTangents=working.tangents;bool sameExterior=true;if(authoredTangents.Length==mesh.vertexCount)foreach(int i in mesh.GetTriangles(0))sameExterior&=authoredTangents[i]==mappedTangents[i];Require(sameExterior,"UV remapping preserves external skin authored tangent basis");
                var animation=new AnimationAdapter(d,r);var offsets=new Vec3[d.Rest.Length];var baked=new Mesh();assets.Add(baked);var savedRotations=new Quaternion[bones.Length];for(int i=0;i<bones.Length;i++)savedRotations[i]=bones[i].localRotation;
                double maximumSeam=0,maximumPose=0;var apply=new double[60];var sw=new Stopwatch();
                for(int frame=0;frame<60;frame++){
                    for(int b=0;b<bones.Length;b++)bones[b].localRotation=savedRotations[b]*Quaternion.Euler(Mathf.Sin(frame*.12f+b)*1.5f,Mathf.Cos(frame*.09f+b*.2f)*2,0);
                    for(int m=0;m<mesh.blendShapeCount;m++)r.SetBlendShapeWeight(m,50+40*Mathf.Sin(frame*.13f+m));animation.Capture(null);
                    for(int i=0;i<offsets.Length;i++)offsets[i]=new Vec3(.0002*Math.Sin(i*.03+frame*.1),.0003*Math.Cos(i*.04),.0002*Math.Sin(frame*.07));
                    sw.Restart();remap.ApplyControlOffsets(offsets,.02,3);sw.Stop();apply[frame]=sw.Elapsed.TotalMilliseconds;
                    r.BakeMesh(baked);var actual=baked.vertices;for(int i=0;i<d.UvPatch.Duplicates.Length;i++)maximumSeam=Math.Max(maximumSeam,(actual[mesh.vertexCount+i]-actual[d.UvPatch.Duplicates[i]]).magnitude);
                    if(frame%10==0)maximumPose=Math.Max(maximumPose,remap.VerifyAppliedControlOffsets(offsets));yield return null;
                }
                Require(maximumSeam<1e-7&&maximumPose<.00003,"actual native bone/morph animation and deformation preserve UV seam continuity","seam="+maximumSeam+" appliedPoseError="+maximumPose);
                var snapshot=remap.XRaySnapshot();var uv=snapshot.uv;var expected=working.uv;bool correct=uv.Length==expected.Length;for(int i=0;correct&&i<uv.Length;i++)correct&=(uv[i]-expected[i]).sqrMagnitude<1e-12;
                Require(correct,"X-Ray snapshot retains remapped atlas coordinates");var renderTriangles=working.triangles;var diagnosticTriangles=snapshot.triangles;var exists=new System.Collections.Generic.HashSet<int>(renderTriangles);foreach(int v in diagnosticTriangles)correct&=exists.Contains(v);Require(correct,"X-Ray uses remapped render vertex identities");
                var now=mesh.uv;bool untouched=now.Length==originalUv.Length&&mesh.vertexCount==originalVertices.Length;for(int i=0;untouched&&i<now.Length;i++)untouched&=now[i]==originalUv[i];Require(untouched,"original receiver mesh remains unchanged");
                RunEntryJunctionNative(r,d,remap);
                remap.Dispose();remap=null;Require(r.sharedMesh==mesh,"UV remap detaches and restores original mesh safely");
                double sum=0;foreach(double x in apply)sum+=x;Array.Sort(apply);report.performance.Add(new PerformanceRow{scope="Actual Receiver ZIPMOD, 60 neutral independent bone/morph poses; mesh application only, not HScene frame time",vertices=working.vertexCount,particles=d.Rest.Length,samples=60,applyMean=sum/60,applyP95=apply[56],maximumAppliedPoseError=maximumPose});
                report.checks.Add(new Check{name="user animation and visual acceptance",status="SKIP",detail="Actual model and native animation checked with neutral transforms; user acceptance pending."});
            }finally{if(remap!=null)remap.Dispose();if(materials!=null)materials.Dispose();Object.Destroy(skin);bundle.Unload(false);}
        }
    }
}
