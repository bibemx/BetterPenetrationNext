using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;
using System.Xml.Serialization;
using BepInEx.Bootstrap;
using BetterPenetrationNext.Core;
using Core_BetterPenetration;
using HarmonyLib;
using UnityEngine;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests : MonoBehaviour
    {
        [Serializable] public sealed class Check { public string name,status,detail; }
        [Serializable] public sealed class Report
        { public string version="1.0.7-section-rig",unity,dllSha256; public bool completed,passed; public List<Check> checks=new List<Check>(); public List<PerformanceRow> performance=new List<PerformanceRow>(); }
        readonly Report report=new Report(); readonly List<Object> assets=new List<Object>();
        GameObject root; MeshDeformationBinding binding; ContactDetection source; IEnumerator routine; string output;
        void Start()
        {
            report.unity=Application.unityVersion;
            using(var stream=File.OpenRead(typeof(NextRuntime).Assembly.Location)) using(var sha=System.Security.Cryptography.SHA256.Create()) report.dllSha256=BitConverter.ToString(sha.ComputeHash(stream)).Replace("-","");
            foreach(string arg in Environment.GetCommandLineArgs()) if(arg.StartsWith("--bpn-report=")) output=arg.Substring(13);
            if(string.IsNullOrEmpty(output)) output=Path.Combine(Application.persistentDataPath,"Next-native.xml"); routine=Run();
        }
        void Update() { if(routine==null) return; try { if(!routine.MoveNext()) Finish(null); } catch(Exception e) { Finish(e); } }
        IEnumerator Run()
        {
            yield return null;
            if(Array.IndexOf(Environment.GetCommandLineArgs(),"--bpn-uv-only")>=0){report.version="1.0.9-uv-entry-preview";root=new GameObject("Actual receiver UV fixture");Object.DontDestroyOnLoad(root);root.transform.position=new Vector3(100,100,100);var uv=RunUvNative();while(uv.MoveNext())yield return uv.Current;yield break;}
            if(Array.IndexOf(Environment.GetCommandLineArgs(),"--bpn-xray-only")>=0)
            {
                report.version="1.0.9-xray-compositing";root=new GameObject("Neutral nested-surface X-Ray fixture");Object.DontDestroyOnLoad(root);root.transform.position=new Vector3(100,100,100);
                var xray=RunXray();while(xray.MoveNext())yield return xray.Current;yield break;
            }
            if(Array.IndexOf(Environment.GetCommandLineArgs(),"--bpn-implant-only")>=0)
            {
                report.version="1.0.9-real-zipmod-implants";root=new GameObject("Inactive real-asset implant fixture");Object.DontDestroyOnLoad(root);
                var implants=RunBoneImplantNative();while(implants.MoveNext())yield return implants.Current;yield break;
            }
            if(Array.IndexOf(Environment.GetCommandLineArgs(),"--bpn-landmarks-only")>=0)
            {
                report.version="1.0.9-neutral-landmarks";
                root=new GameObject("Neutral animated material landmark fixture");Object.DontDestroyOnLoad(root);root.transform.position=new Vector3(100,100,100);
                var landmarks=RunLandmarkNative();while(landmarks.MoveNext())yield return landmarks.Current;
                if(SystemInfo.graphicsDeviceType!=UnityEngine.Rendering.GraphicsDeviceType.Null){var pixels=RunXray();while(pixels.MoveNext())yield return pixels.Current;}
                else report.checks.Add(new Check{name="X-Ray pixel tests",status="SKIP",detail="No graphics device in this run."});
                report.checks.Add(new Check{name="user scene acceptance",status="SKIP",detail="Neutral animated pipe only; does not certify contact or complete containment of user geometry."});
                yield break;
            }
            if(Array.IndexOf(Environment.GetCommandLineArgs(),"--bpn-defaults-only")>=0)
            {
                report.version="1.0.8-defaults-only";
                string directory=Path.Combine(Path.GetDirectoryName(Path.GetFullPath(output)),"cases");
                int expected=0; foreach(var row in ReleaseDefaults.LoadEmbedded().Profile.Settings) if(row.Section!="Next.PreparedMorphs") expected++;
                Require(NextRuntime.Instance.VerifyDefaultConfiguration()==expected,"actual plugin startup uses every static release default","settings="+expected);
                int result=DefaultProfileChecks.Run(new[]{Path.Combine(directory,"release-defaults.json"),directory},true);
                Require(result==0,"all profile settings, native colors, deletion, Reset and installer values agree");
                report.checks.Add(new Check { name="simulation acceptance",status="SKIP",detail="Configuration-only test; no contact or animation result is certified." });
                yield break;
            }
            if(Array.IndexOf(Environment.GetCommandLineArgs(),"--bpn-source-feedback-only")>=0)
            {
                report.version="1.0.8-source-feedback-only";root=new GameObject("Neutral source feedback fixture");
                RunSectionRigNative();
                report.checks.Add(new Check { name="multilayer anatomy acceptance",status="SKIP",detail="Source feedback test only, no claim about a user scene or complete tissue contact." });
                yield break;
            }
            Require(Chainloader.PluginInfos.ContainsKey("org.betterpenetration.next"),"independent fork loads in BepInEx");
            Require(!Chainloader.PluginInfos.ContainsKey("animal42069.KKSbetterpenetration")&&!Chainloader.PluginInfos.ContainsKey("org.betterpenetration.enhanced"),"original and Enhanced controllers are absent during the isolated Next test");
            Require(NextRuntime.Instance!=null&&NextRuntime.Instance.Mode==NextMode.Original,"no prepared resource preserves Original mode safely");
            var targetMethod=AccessTools.DeclaredMethod(typeof(DanAgent),"SetDanTarget",new[]{typeof(CollisionAgent),typeof(bool)});
            var patches=Harmony.GetPatchInfo(targetMethod);
            Require(patches!=null&&patches.Owners.Contains("org.betterpenetration.next.simulation"),"fork observation is installed on the actual preserved animation method");
            var constraint=AccessTools.DeclaredMethod(typeof(DanAgent),"ConstrainDanToBody"); var constraintPatches=Harmony.GetPatchInfo(constraint);
            Require(constraintPatches==null||!constraintPatches.Owners.Contains("org.betterpenetration.next.simulation"),"source constraint receives no Next bypass patch");
            var agent=(DanAgent)FormatterServices.GetUninitializedObject(typeof(DanAgent)); agent.m_baseDanLength=1;
            agent.m_danOptions=(DanOptions)FormatterServices.GetUninitializedObject(typeof(DanOptions)); agent.m_danOptions.squishThreshold=0.2f; agent.m_danOptions.danLengthSquish=0.5f;
            Require(Math.Abs(agent.GetSquishedDanLength(0.2f)-0.7f)<1e-6,"preserved source squish formula agrees with original BP");
            Tests.SourceSelectionFixture.Run((valid,name)=>Require(valid,name)); yield return null;
            root=new GameObject("BPN neutral native fixture"); Object.DontDestroyOnLoad(root); root.transform.position=new Vector3(100,100,100);
            var animated=RunAnimatedSource(); while(animated.MoveNext()) yield return animated.Current;
            var coupled=RunBidirectional(); while(coupled.MoveNext()) yield return coupled.Current;
            var architecture=RunArchitecture();while(architecture.MoveNext())yield return architecture.Current;
            var grid=PrimitiveMeshes.Grid(40,40,1,1); var simulation=PrimitiveMeshes.Grid(10,10,1,1);
            var receiver=Skin("neutral visual",grid,Vector3.zero); Mesh original=receiver.sharedMesh; Material material=receiver.sharedMaterial;
            var definition=Definition(original,receiver,simulation); string json=JsonCodec.Serialize(definition); var restored=JsonCodec.Deserialize<SimulationDefinition>(json);
            File.WriteAllText(output+".definition.json",json);
            Require(restored!=null&&restored.Schema==1&&restored.BodyGuid==UncensorIntegration.PreparedGuid&&restored.Rest!=null&&restored.Rest.Length==121,"managed metadata fields survive actual KKS Mono serialization","schema="+(restored==null?-1:restored.Schema)+" guid="+(restored==null?"null":restored.BodyGuid)+" rest="+(restored==null||restored.Rest==null?-1:restored.Rest.Length));
            restored.Validate(original.vertexCount);
            Require(restored.Rest.Length==121&&restored.Bindings.Length==grid.Vertices.Length,"simulation and mapping arrays survive actual runtime serialization");
            binding=new MeshDeformationBinding(receiver,restored); var animation=new AnimationAdapter(restored,receiver);
            var solver=new SoftBodySolver(new SimulationTopology(restored.Rest,restored.Triangles),restored.PinWeights); animation.Capture(null); solver.Reset(animation.Anchors);
            Require(receiver.sharedMesh!=original&&receiver.sharedMaterial==material,"private visual mesh preserves native renderer and material");
            var sphere=Skin("neutral contact sphere",PrimitiveMeshes.Sphere(24,12,0.23),new Vector3(0,0,0.08f));
            var evidence=new double[sphere.sharedMesh.vertexCount]; for(int i=0;i<evidence.Length;i++) evidence[i]=1;
            source=new ContactDetection(sphere,evidence); var settings=new SoftBodySettings { MaximumOffsetRatio=0.2 }; var policy=PhysicsPolicy.Resolve(PhysicsQuality.Balanced);
            for(int frame=0;frame<90;frame++) { animation.Capture(solver); source.Capture(); solver.Advance(animation.Anchors,source,settings,policy,1.0/60,animation.PosedSize); binding.Apply(solver,animation.PosedSize*0.2,3); if(frame%15==0) yield return null; }
            double maximum=0; for(int i=0;i<solver.Positions.Length;i++) maximum=Math.Max(maximum,solver.SolvedDisplacement(i).Length);
            Require(maximum>0.02&&solver.ContactCount>0,"native XPBD particles respond to actual posed source geometry","max="+maximum);
            Require(binding.VerifyAppliedPose(solver)<1e-4,"actual native skinned visual output matches barycentric physical correction");
            Require(original==binding.Original&&SimulationDefinition.Signature(grid.Vertices,grid.Triangles)==restored.VisualMeshSignature,"source shared mesh is not modified in place");
            settings.RecoverySeconds=4.5; solver.Advance(animation.Anchors,source,settings,policy,1.0/60,animation.PosedSize);
            for(int frame=0;frame<30;frame++) solver.Advance(animation.Anchors,null,settings,policy,1.0/60,animation.PosedSize);
            binding.Apply(solver,animation.PosedSize*0.2,1); var recovering=receiver.sharedMesh.vertices; var restVertices=original.vertices; double retained=0;
            for(int i=0;i<recovering.Length;i++) retained=Math.Max(retained,(recovering[i]-restVertices[i]).magnitude);
            Require(retained>maximum*.5&&binding.VerifyAppliedPose(solver)<1e-4,"recovery delay remains present in actual skinned geometry after half a second","retained="+retained);
            for(int frame=0;frame<420;frame++) solver.Advance(animation.Anchors,null,settings,policy,1.0/60,animation.PosedSize);
            binding.Apply(solver,animation.PosedSize*0.2,1); recovering=receiver.sharedMesh.vertices; retained=0;
            for(int i=0;i<recovering.Length;i++) retained=Math.Max(retained,(recovering[i]-restVertices[i]).magnitude);
            Require(retained<maximum*.01&&binding.VerifyAppliedPose(solver)<1e-4,"recovered native visual mesh returns without accumulated deformation","retained="+retained);
            settings.RecoverySeconds=0; solver.Reset(animation.Anchors);
            root.transform.localScale=new Vector3(0.7f,1.3f,0.9f);
            for(int frame=0;frame<60;frame++)
            {
                receiver.bones[0].localRotation=Quaternion.Euler(0,Mathf.Sin(frame*0.1f)*9,0);
                animation.Capture(solver); source.Capture(); solver.Advance(animation.Anchors,source,settings,policy,1.0/60,animation.PosedSize); binding.Apply(solver,animation.PosedSize*0.2,3);
                if(frame%10==0) yield return null;
            }
            double nativeError=binding.VerifyAppliedPose(solver);
            Require(nativeError<0.0003,"animated bones and nonuniform proportions preserve physical-to-visual mapping","error="+nativeError+" scale="+receiver.transform.lossyScale);
            receiver.enabled=false; yield return null; Require(binding.Alive&&!binding.Visible,"renderer visibility does not destroy the binding"); receiver.enabled=true;
            receiver.gameObject.SetActive(false); yield return null; Require(binding.Alive&&!binding.Visible,"inactive hierarchy preserves registered geometry"); receiver.gameObject.SetActive(true);
            foreach(PhysicsQuality q in Enum.GetValues(typeof(PhysicsQuality)))
            { animation.Capture(solver); solver.Advance(animation.Anchors,source,settings,PhysicsPolicy.Resolve(q),1.0/60,animation.PosedSize); binding.Apply(solver,animation.PosedSize*0.2,3); Require(binding.VerifyAppliedPose(solver)<0.0003,"quality "+q+" applies real valid geometry"); }
            solver.Reset(animation.Anchors); binding.Apply(solver,animation.PosedSize*0.2,1);
            Require(binding.VerifyAppliedPose(solver)<0.0003,"reset removes physical displacement without accumulated geometry");
            binding.Dispose(); binding=null; Require(receiver.sharedMesh==original&&receiver.sharedMaterial==material,"Original restoration returns exact mesh and material references");
            for(int cycle=0;cycle<5;cycle++) { binding=new MeshDeformationBinding(receiver,definition); binding.Dispose(); binding=null; yield return null; }
            int meshes=0; foreach(var mesh in Resources.FindObjectsOfTypeAll<Mesh>()) if(mesh.name.EndsWith("[BPN private]",StringComparison.Ordinal)) meshes++;
            Require(meshes==0,"repeated bind and restore does not leak private visual meshes");
            string snapshot=output+".diagnostics.txt"; global::Next.RuntimeProbe.Dump(snapshot);
            Require(File.Exists(snapshot)&&File.ReadAllText(snapshot).Contains("XRAY enabled="),"read-only runtime diagnostic export executes in installed KKS Mono");
            Require(receiver.sharedMesh==original&&receiver.sharedMaterial==material,"diagnostic export preserves mesh and material references");
            string package=null; foreach(string arg in Environment.GetCommandLineArgs()) if(arg.StartsWith("--bpn-package=")) package=arg.Substring(14);
            if(package!=null) { var prepared=RunPrepared(package); while(prepared.MoveNext()) yield return prepared.Current; }
            else report.checks.Add(new Check { name="user-prepared package",status="SKIP",detail="No --bpn-package argument supplied." });
            if(SystemInfo.graphicsDeviceType!=UnityEngine.Rendering.GraphicsDeviceType.Null) { var graphics=RunXray(); while(graphics.MoveNext()) yield return graphics.Current; }
            else report.checks.Add(new Check { name="Next X-Ray pixel tests",status="SKIP",detail="Native run uses -nographics." });
            report.checks.Add(new Check { name="real gameplay performance",status="SKIP",detail="A neutral contact probe does not certify gameplay FPS." });
        }
        SimulationDefinition Definition(Mesh visual,SkinnedMeshRenderer renderer,MeshData sim)
        {
            var influence=new double[visual.vertexCount]; for(int i=0;i<influence.Length;i++) influence[i]=1;
            var rest=visual.vertices; var core=new Vec3[rest.Length]; for(int i=0;i<rest.Length;i++) core[i]=AnimationAdapter.ToCore(rest[i]);
            var d=new SimulationDefinition { Rest=sim.Vertices,Triangles=sim.Triangles,PinWeights=new double[sim.Vertices.Length],BoneNames=new[]{renderer.bones[0].name},Bindposes=new float[16],InfluenceOffsets=new int[sim.Vertices.Length+1],BoneIndices=new int[sim.Vertices.Length],BoneWeights=new double[sim.Vertices.Length],MaximumBindingDistance=0.1 };
            var bindpose=visual.bindposes[0]; for(int r=0;r<4;r++) for(int c=0;c<4;c++) d.Bindposes[r*4+c]=bindpose[r,c];
            for(int i=0;i<sim.Vertices.Length;i++) { d.InfluenceOffsets[i]=i; d.BoneWeights[i]=1; d.PinWeights[i]=Math.Abs(sim.Vertices[i].X)>0.49||Math.Abs(sim.Vertices[i].Y)>0.49?1:0; } d.InfluenceOffsets[sim.Vertices.Length]=sim.Vertices.Length;
            d.Bindings=SurfaceBinding.Build(core,influence,sim.Vertices,sim.Triangles,0.1); d.VisualMeshSignature=SimulationDefinition.Signature(core,visual.triangles); return d;
        }
        SkinnedMeshRenderer Skin(string name,MeshData data,Vector3 position)
        {
            var go=new GameObject(name); go.transform.SetParent(root.transform,false); go.transform.localPosition=position;
            var bone=new GameObject(name+" bone"); bone.transform.SetParent(go.transform,false);
            var mesh=new Mesh { name=name }; assets.Add(mesh); var vertices=new Vector3[data.Vertices.Length]; var weights=new BoneWeight[vertices.Length];
            for(int i=0;i<vertices.Length;i++) { vertices[i]=AnimationAdapter.ToUnity(data.Vertices[i]); weights[i]=new BoneWeight { boneIndex0=0,weight0=1 }; }
            mesh.vertices=vertices; mesh.triangles=data.Triangles; mesh.boneWeights=weights; mesh.bindposes=new[]{bone.transform.worldToLocalMatrix*go.transform.localToWorldMatrix}; mesh.RecalculateNormals(); mesh.RecalculateBounds();
            var renderer=go.AddComponent<SkinnedMeshRenderer>(); renderer.sharedMesh=mesh; renderer.bones=new[]{bone.transform}; renderer.rootBone=bone.transform; renderer.updateWhenOffscreen=true; renderer.quality=SkinQuality.Bone4; renderer.localBounds=mesh.bounds;
            var shader=Shader.Find("Standard")??Shader.Find("Hidden/Internal-Colored"); if(shader!=null) { var material=new Material(shader); assets.Add(material); renderer.sharedMaterial=material; } return renderer;
        }
        void Require(bool value,string name,string detail="") { report.checks.Add(new Check { name=name,status=value?"PASS":"FAIL",detail=detail }); if(!value) throw new InvalidOperationException(name); }
        void Finish(Exception e)
        {
            routine=null; report.completed=true; report.passed=e==null;
            if(e!=null) report.checks.Add(new Check { name="runtime exception",status="FAIL",detail=e.ToString() });
            if(binding!=null) binding.Dispose(); if(source!=null) source.Dispose(); if(root!=null) Object.Destroy(root); foreach(var item in assets) if(item!=null) Object.Destroy(item);
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output))); using(var writer=File.CreateText(output)) new XmlSerializer(typeof(Report)).Serialize(writer,report);
            UnityEngine.Debug.Log("BPN_SELFTEST passed="+report.passed+" "+output); Application.Quit(report.passed?0:1);
        }
    }
}
