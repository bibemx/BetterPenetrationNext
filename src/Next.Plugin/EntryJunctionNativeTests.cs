using System;
using System.Collections.Generic;
using System.IO;
using BetterPenetrationNext.Core;
using UnityEngine;
namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        [Serializable]public sealed class JunctionPose
        {
            public Vec3[] SourceReference,Source,Translation;public int[] SourceFaces;public Vec3 SourceReferenceRoot,SourceAxis,SourceRoot;public Linear3 SourceFrame;public Linear3[] Bones;public double[] Morph;
            public double CanalLengthScale,CervixMaxDisplacement,MaxSource,MaxStrain,SourceCompliance,Response,PosedSize;
        }
        static T JunctionResource<T>(string name){using(var stream=typeof(NextRuntime).Assembly.GetManifestResourceStream("BetterPenetrationNext.channel."+name))using(var reader=new StreamReader(stream))return JsonCodec.Deserialize<T>(reader.ReadToEnd());}
        void RunEntryJunctionNative(SkinnedMeshRenderer renderer,SimulationDefinition support,MeshDeformationBinding mapping)
        {
            string path=null;foreach(string arg in Environment.GetCommandLineArgs())if(arg.StartsWith("--bpn-entry-fixture="))path=arg.Substring(20);
            if(path==null){report.checks.Add(new Check{name="recorded entry junction native probe",status="SKIP",detail="No recorded fixture supplied."});return;}
            var pose=JsonCodec.Deserialize<JunctionPose>(File.ReadAllText(path));var physical=JunctionResource<SimulationDefinition>("physical.json");var transfer=JunctionResource<ReceiverSupportTransfer>("transfer.json");var contours=JunctionResource<ReceiverGuideDefinition>("contours.json");var chart=JunctionResource<ReactiveMaterialChart>("chart.json");var mucosal=JunctionResource<ReceiverSupportTransfer>("reactive-transfer.json");
            var bones=renderer.bones;var bindposes=renderer.sharedMesh.bindposes;
            Require(pose.Bones.Length==bones.Length&&pose.Source.Length==pose.SourceReference.Length,"native junction uses recorded real-model geometry and bone palette");
            for(int i=0;i<bones.Length;i++)
            {
                var matrix=Matrix4x4.identity;matrix.SetColumn(0,AnimationAdapter.ToUnity(pose.Bones[i].X));matrix.SetColumn(1,AnimationAdapter.ToUnity(pose.Bones[i].Y));matrix.SetColumn(2,AnimationAdapter.ToUnity(pose.Bones[i].Z));matrix.SetColumn(3,new Vector4((float)pose.Translation[i].X,(float)pose.Translation[i].Y,(float)pose.Translation[i].Z,1));matrix=matrix*bindposes[i].inverse;
                bones[i].position=matrix.GetColumn(3);bones[i].rotation=matrix.rotation;bones[i].localScale=matrix.lossyScale;
            }
            for(int i=0;i<support.Morphs.Length;i++)renderer.SetBlendShapeWeight(renderer.sharedMesh.GetBlendShapeIndex(support.Morphs[i].Name),(float)(pose.Morph[i]*100));
            var animation=new AnimationAdapter(physical,renderer);animation.Capture(null);
            var rig=new ReactiveReceiverRig(physical,support,transfer,contours,pose.SourceReference,pose.SourceFaces,pose.SourceAxis,new SourceSectionRig(pose.SourceReference,pose.SourceFaces,pose.SourceReferenceRoot,pose.SourceAxis),mucosal,chart);
            var input=new ReactiveRigInput(bones.Length,pose.Morph.Length,pose.Source.Length){SourceRoot=pose.SourceRoot,SourceFrame=pose.SourceFrame,HasSource=true,SourceFollowsPath=true,Dt=1.0/60,PosedSize=pose.PosedSize,CanalLengthScale=pose.CanalLengthScale,CervixMaxDisplacement=pose.CervixMaxDisplacement,MaxSource=pose.MaxSource,MaxStrain=pose.MaxStrain,SourceCompliance=pose.SourceCompliance,Response=pose.Response};
            Array.Copy(animation.CapturedLinear,input.Bones,input.Bones.Length);Array.Copy(animation.CapturedTranslation,input.Translation,input.Bones.Length);Array.Copy(animation.CapturedMorphs,input.Morph,input.Morph.Length);Array.Copy(pose.Source,input.Source,pose.Source.Length);
            var result=new ReactiveRigOutput(support.Rest.Length,physical.Rest.Length,input.Source.Length,rig.SectionCount);for(int i=0;i<45;i++)rig.Step(input,result);
            Require(result.ReceiverActive&&result.EntranceRegularizedVertices>0,"production Step activates corrected junction on the actual native Receiver");mapping.ApplyControlOffsets(result.Offsets,.4,1);double error=mapping.VerifyAppliedControlOffsets(result.Offsets);Require(error<.0001,"native skinned mesh receives complete corrected junction offsets","maximumError="+error);
            var representatives=new int[support.Rest.Length];foreach(var b in support.Bindings)representatives[b.A]=b.Vertex;
            var baked=new Mesh();assets.Add(baked);renderer.BakeMesh(baked);var vertices=baked.vertices;var world=new Vec3[support.Rest.Length];var basis=AnimationAdapter.BakedToWorld(renderer);for(int i=0;i<world.Length;i++)world[i]=AnimationAdapter.ToCore(basis.MultiplyPoint3x4(vertices[representatives[i]]));
            var faces=new List<int>();for(int f=0;f<chart.TriangleRegions.Length;f++)if(chart.TriangleRegions[f]==0){faces.Add(support.Triangles[f*3]);faces.Add(support.Triangles[f*3+1]);faces.Add(support.Triangles[f*3+2]);}
            var surface=new TriangleSurface(world,faces.ToArray(),SurfaceKind.OrientedShell);Vec3 centre=new Vec3();foreach(int v in contours.Boundaries[0])centre+=world[transfer.PhysicalIndices[v]]/contours.Boundaries[0].Length;Vec3 normal=(result.Path[1]-result.Path[0]).Normalized,side=animation.CapturedLinear[Array.IndexOf(physical.BoneNames,"MED_Pelvis")].X;side=(side-normal*Vec3.Dot(side,normal)).Normalized;Vec3 up=Vec3.Cross(normal,side);double radius=double.PositiveInfinity;foreach(int v in contours.Boundaries[0]){Vec3 q=world[transfer.PhysicalIndices[v]]-centre;q-=normal*Vec3.Dot(q,normal);radius=Math.Min(radius,q.Length);}radius*=.6;
            int rays=0,blocked=0;for(int x=-7;x<=7;x++)for(int y=-7;y<=7;y++)if(x*x+y*y<=49){Vec3 p=centre+side*(x*radius/7)+up*(y*radius/7);SurfaceSample hit;rays++;if(surface.TryRaycast(p-normal*.02,p+normal*.02,out hit))blocked++;}
            Require(blocked==0,"actual native BakeMesh entrance remains clear of exterior bridging triangles","blocked="+blocked+" / "+rays);
            Require(result.ContactCanalExtension<1e-12&&result.UterusTranslationPelvis.Length==0,"native junction correction preserves local cervical mode without organ-driven extension");
        }
    }
}
