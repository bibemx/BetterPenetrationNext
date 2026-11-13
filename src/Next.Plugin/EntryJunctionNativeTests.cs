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
        static T JunctionResource<T>(string name)
        {
            foreach(string arg in Environment.GetCommandLineArgs())if(arg.StartsWith("--bpn-uv-archive="))
                using(var file=File.OpenRead(arg.Substring(17)))using(var zip=new System.IO.Compression.ZipArchive(file,System.IO.Compression.ZipArchiveMode.Read))
                using(var reader=new StreamReader(zip.GetEntry("bpn/reactive/"+name).Open()))return JsonCodec.Deserialize<T>(reader.ReadToEnd());
            using(var stream=typeof(NextRuntime).Assembly.GetManifestResourceStream("BetterPenetrationNext.channel."+name))using(var reader=new StreamReader(stream))return JsonCodec.Deserialize<T>(reader.ReadToEnd());
        }
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
            PreparedReceiverPackage package=null;foreach(string arg in Environment.GetCommandLineArgs())if(arg.StartsWith("--bpn-uv-archive="))package=PreparedReceiverPackage.Load(arg.Substring(17));
            NativeClosedEntranceBinding closed=null;
            if(package!=null&&package.ClosedEntrance!=null){closed=new NativeClosedEntranceBinding(renderer,support,physical,contours,package.ClosedEntrance);var carrier=input.Bones[Array.IndexOf(support.BoneNames,"MED_Pelvis")];input.EntranceContactDepth=closed.ContactDepth*(carrier.X.Length+carrier.Y.Length+carrier.Z.Length)/3;}
            var result=new ReactiveRigOutput(support.Rest.Length,physical.Rest.Length,input.Source.Length,rig.SectionCount);for(int i=0;i<45;i++)rig.Step(input,result);
            if(closed!=null){for(int i=0;i<60;i++)closed.Apply(result.Physical,input,result,result.Offsets,1.0/60);Require(closed.Constraint.MaximumOffset>.001&&closed.Constraint.MinimumAreaRatio>.029&&closed.Constraint.MaximumStrain<=input.MaxStrain+1e-5,"native SAC lip constraint changes the original surface within material guards","offset="+closed.Constraint.MaximumOffset+" scale="+closed.Constraint.AppliedScale+" area="+closed.Constraint.MinimumAreaRatio+" rejectedArea="+closed.Constraint.RejectedArea+" rejectedStrain="+closed.Constraint.RejectedStrain);}
            Require(result.ReceiverActive&&result.EntranceRegularizedVertices>0,"production Step activates corrected junction on the actual native Receiver");mapping.ApplyControlOffsets(result.Offsets,.4,1);double error=mapping.VerifyAppliedControlOffsets(result.Offsets);Require(error<.0001,"native skinned mesh receives complete corrected junction offsets","maximumError="+error);
            var representatives=new int[support.Rest.Length];foreach(var b in support.Bindings)representatives[b.A]=b.Vertex;
            var baked=new Mesh();assets.Add(baked);renderer.BakeMesh(baked);var vertices=baked.vertices;var world=new Vec3[support.Rest.Length];var basis=AnimationAdapter.BakedToWorld(renderer);for(int i=0;i<world.Length;i++)world[i]=AnimationAdapter.ToCore(basis.MultiplyPoint3x4(vertices[representatives[i]]));
            var faces=new List<int>();for(int f=0;f<chart.TriangleRegions.Length;f++)if(chart.TriangleRegions[f]==0){faces.Add(support.Triangles[f*3]);faces.Add(support.Triangles[f*3+1]);faces.Add(support.Triangles[f*3+2]);}
            var surface=new TriangleSurface(world,faces.ToArray(),SurfaceKind.OrientedShell);Vec3 centre=new Vec3();foreach(int v in contours.Boundaries[0])centre+=world[transfer.PhysicalIndices[v]]/contours.Boundaries[0].Length;Vec3 normal=(result.Path[1]-result.Path[0]).Normalized,side=animation.CapturedLinear[Array.IndexOf(physical.BoneNames,"MED_Pelvis")].X;side=(side-normal*Vec3.Dot(side,normal)).Normalized;Vec3 up=Vec3.Cross(normal,side);double radius=double.PositiveInfinity;foreach(int v in contours.Boundaries[0]){Vec3 q=world[transfer.PhysicalIndices[v]]-centre;q-=normal*Vec3.Dot(q,normal);radius=Math.Min(radius,q.Length);}radius*=.6;
            int rays=0,blocked=0;for(int x=-7;x<=7;x++)for(int y=-7;y<=7;y++)if(x*x+y*y<=49){Vec3 p=centre+side*(x*radius/7)+up*(y*radius/7);SurfaceSample hit;rays++;if(surface.TryRaycast(p-normal*.02,p+normal*.02,out hit))blocked++;}
            Require(blocked==0,"actual native BakeMesh entrance remains clear of exterior bridging triangles","blocked="+blocked+" / "+rays);
            if(closed!=null)
            {
                var actualWorld=new Vec3[vertices.Length];for(int i=0;i<vertices.Length;i++)actualWorld[i]=AnimationAdapter.ToCore(basis.MultiplyPoint3x4(vertices[i]));
                var exterior=new TriangleSurface(actualWorld,renderer.sharedMesh.GetTriangles(0),SurfaceKind.OrientedShell);int skinBlocked=0;
                for(int x=-7;x<=7;x++)for(int y=-7;y<=7;y++)if(x*x+y*y<=49){Vec3 q=centre+side*(x*radius/7)+up*(y*radius/7);SurfaceSample hit;if(exterior.TryRaycast(q-normal*.025,q+normal*.005,out hit))skinBlocked++;}
                Require(skinBlocked==0,"actual SAC exterior uncovers the receiver after contact","blocked="+skinBlocked+" / "+rays);
                var sourceEdges=new HashSet<long>();int crossings=0,checkedEdges=0;
                for(int f=0;f<pose.SourceFaces.Length;f+=3)for(int k=0;k<3;k++){int a=pose.SourceFaces[f+k],b=pose.SourceFaces[f+(k+1)%3];if(a>b){int swap=a;a=b;b=swap;}if(!sourceEdges.Add(((long)a<<32)|(uint)b))continue;Vec3 x=result.SourceWorld[a],y=result.SourceWorld[b];if(((x+y)*.5-centre).Length>radius*4+.02)continue;SurfaceSample hit;checkedEdges++;if(exterior.TryRaycast(x,y,out hit))crossings++;}
                Require(crossings==0,"actual corrected source edges clear the native SAC exterior at the entrance","crossings="+crossings+" / "+checkedEdges);
                input.HasSource=false;for(int frame=0;frame<272;frame++)closed.Apply(result.Physical,input,result,result.Offsets,1.0/60);
                Require(closed.Constraint.MaximumOffset<1e-8,"actual SAC native binding closes to its own animation baseline at RecoveryTime");
            }
            Require(result.ContactCanalExtension<1e-12&&result.UterusTranslationPelvis.Length==0,"native junction correction preserves local cervical mode without organ-driven extension");
        }
    }
}
