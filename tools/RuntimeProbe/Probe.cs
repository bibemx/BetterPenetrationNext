using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace Next
{
    // Read-only snapshot: no Harmony patches, transform writes or renderer changes.
    public static class RuntimeProbe
    {
        const BindingFlags Flags=BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance|BindingFlags.Static;
        static object Get(object o,string name)
        { if(o==null) return null; var t=o as Type??o.GetType(); var f=t.GetField(name,Flags); if(f!=null) return f.GetValue(o is Type?null:o); var p=t.GetProperty(name,Flags); return p==null?null:p.GetValue(o is Type?null:o,null); }
        static string PathOf(Transform value)
        { if(value==null) return "<null>"; string path=value.name; while(value.parent!=null) { value=value.parent; path=value.name+"/"+path; } return path; }
        static void Renderer(StringBuilder b,string role,SkinnedMeshRenderer r)
        {
            if(r==null) { b.AppendLine(role+" NONE"); return; }
            b.AppendLine(role+" path="+PathOf(r.transform)+" id="+r.GetInstanceID()+" enabled="+r.enabled+" active="+r.gameObject.activeInHierarchy+" layer="+r.gameObject.layer+" worldBounds="+r.bounds);
            b.AppendLine(" activeSelf="+r.gameObject.activeSelf); for(var parent=r.transform;parent!=null;parent=parent.parent) if(!parent.gameObject.activeSelf) b.AppendLine(" inactiveAncestor="+PathOf(parent));
            var m=r.sharedMesh; b.AppendLine(" mesh="+(m==null?"null":m.name+" vertices="+m.vertexCount+" submeshes="+m.subMeshCount+" readable="+m.isReadable)+" rendererBones="+r.bones.Length);
            int slot=0; foreach(var material in r.sharedMaterials) { if(material!=null) b.AppendLine(" material["+slot+"]="+material.name+" shader="+material.shader.name+" queue="+material.renderQueue+" mainTex="+(material.HasProperty("_MainTex")&&material.mainTexture!=null?material.mainTexture.name:"none")); slot++; }
        }
        static void Candidates(StringBuilder b,object agent)
        {
            if(agent==null) return; var source=Get(agent,"m_danCharacter") as Component; if(source==null) return;
            var points=Get(Get(agent,"m_danPoints"),"danPoints") as IEnumerable; var chain=new List<Transform>(); if(points!=null) foreach(var p in points) { var t=Get(p,"transform") as Transform; if(t!=null) chain.Add(t); }
            b.AppendLine("AGENT_CANDIDATES chainPoints="+chain.Count);
            foreach(var r in source.GetComponentsInChildren<SkinnedMeshRenderer>(true))
            {
                if(r.sharedMesh==null) continue; int matches=0; foreach(var bone in r.bones) foreach(var p in chain) if(bone!=null&&(bone==p||bone.IsChildOf(p))) { matches++; break; }
                if(matches>0) { Renderer(b,"CHAIN_MATCH",r); b.AppendLine(" matchedBones="+matches); }
            }
        }
        public static string Dump(string file)
        {
            var b=new StringBuilder(); Type type=null; foreach(var assembly in AppDomain.CurrentDomain.GetAssemblies()) if(assembly.GetName().Name=="BetterPenetrationNext") { type=assembly.GetType("BetterPenetrationNext.NextRuntime"); b.AppendLine("loadedNext="+assembly.Location); }
            if(type==null) throw new InvalidOperationException("Next is not loaded."); var runtime=Get(type,"Instance");
            b.AppendLine("utc="+DateTime.UtcNow.ToString("o")+" frame="+Time.frameCount+" timeScale="+Time.timeScale+" delta="+Time.deltaTime+" mode="+Get(runtime,"Mode")+" status="+Get(runtime,"Status"));
            var observations=(IDictionary)Get(runtime,"observations");
            var agents=Get(type.Assembly.GetType("Core_BetterPenetration.CoreGame"),"danAgents") as IEnumerable; if(agents!=null) foreach(var agent in agents) Candidates(b,agent);
            foreach(DictionaryEntry entry in observations)
            {
                var observation=entry.Value; var geometry=Get(observation,"Geometry"); var selected=Get(geometry,"Renderer") as SkinnedMeshRenderer;
                var contactBounds=Get(geometry,"ContactBounds"); Renderer(b,"SELECTED_SOURCE",selected); b.AppendLine(" kind="+Get(geometry,"Kind")+" triangles="+Get(geometry,"TriangleCount")+" visible="+Get(geometry,"Visible")+" boundsMin="+Get(contactBounds,"Min")+" boundsMax="+Get(contactBounds,"Max")+" seen="+Get(observation,"LastSeen"));
                b.AppendLine(" deformationGroups="+Get(geometry,"DeformationGroups")+" unweldedFallback="+Get(geometry,"UsesUnweldedFallback"));
                var agent=Get(observation,"Agent"); var points=(IEnumerable)Get(Get(agent,"m_danPoints"),"danPoints"); var chain=new List<Transform>();
                if(points!=null) foreach(var point in points) { var t=Get(point,"transform") as Transform; if(t!=null) chain.Add(t); }
                foreach(var t in chain) b.AppendLine(" chain="+PathOf(t)+" position="+t.position);
                var source=Get(observation,"Source") as Component;
                if(source!=null) foreach(var r in source.GetComponentsInChildren<SkinnedMeshRenderer>(true))
                {
                    if(r.sharedMesh==null) continue; var bones=r.bones; var included=new bool[bones.Length]; bool found=false;
                    for(int i=0;i<bones.Length;i++) foreach(var t in chain) if(bones[i]!=null&&(bones[i]==t||bones[i].IsChildOf(t))) { included[i]=true; found=true; break; }
                    if(!found) continue; Renderer(b,"CANDIDATE",r);
                    try { var counts=r.sharedMesh.GetBonesPerVertex(); var weights=r.sharedMesh.GetAllBoneWeights(); int cursor=0,positive=0; foreach(byte count in counts) { double total=0; for(int j=0;j<count;j++) { var w=weights[cursor++]; if(w.boneIndex>=0&&w.boneIndex<included.Length&&included[w.boneIndex]) total+=w.weight; } if(total>=0.2) positive++; } b.AppendLine(" weightedVertices="+positive+" ratio="+((double)positive/Math.Max(1,r.sharedMesh.vertexCount))); } catch(Exception e) { b.AppendLine(" evidenceError="+e.Message); }
                }
            }
            foreach(DictionaryEntry entry in (IDictionary)Get(runtime,"bodies"))
            {
                var body=entry.Value; Renderer(b,"RECEIVER",Get(Get(body,"Binding"),"Renderer") as SkinnedMeshRenderer); var solver=Get(body,"Solver");
                foreach(string member in new[]{"ContactCount","ActiveContactCandidates","ContactQueries","SaturatedVertices","LastSubsteps","LastFrameDroppedTime"}) b.AppendLine(" solver."+member+"="+Get(solver,member));
                b.AppendLine(" poseVerified="+Get(body,"PoseVerified")+" frames="+Get(body,"SuccessfulFrames"));
            }
            var xray=Get(runtime,"xray"); b.AppendLine("XRAY enabled="+Get(xray,"Enabled")+" target="+Get(xray,"Target")+" lastFrame="+Get(xray,"LastRenderedFrame")+" callbacks="+Get(xray,"CameraCallbacks"));
            b.AppendLine(" includeHiddenSource="+Get(xray,"IncludeHiddenSource"));
            var surfaces=Get(xray,"surfaces") as IDictionary;
            if(surfaces!=null) foreach(DictionaryEntry entry in surfaces)
            {
                var s=entry.Value; var renderer=Get(s,"Renderer") as SkinnedMeshRenderer; Renderer(b,"XRAY_ROLE source="+Get(s,"Source"),renderer); var m=Get(s,"Mesh") as Mesh; b.AppendLine(" overlayMesh="+(m==null?"null":m.name+" vertices="+m.vertexCount+" submeshes="+m.subMeshCount));
                if(m!=null) for(int slot=0;slot<m.subMeshCount;slot++) b.AppendLine(" overlaySubmesh["+slot+"] indices="+m.GetIndexCount(slot));
                var overlayMaterials=Get(s,"Materials") as Material[]; if(overlayMaterials!=null) foreach(var material in overlayMaterials) if(material!=null) b.AppendLine(" overlayMaterial tint="+material.GetColor("_Tint")+" opacity="+material.GetFloat("_Opacity")+" textureBlend="+material.GetFloat("_TextureBlend")+" density="+material.GetFloat("_DotDensity"));
                foreach(var camera in Resources.FindObjectsOfTypeAll<Camera>()) if(camera.gameObject.scene.IsValid()&&renderer!=null) b.AppendLine(" overlayCamera="+PathOf(camera.transform)+" layerIncluded="+((camera.cullingMask&(1<<renderer.gameObject.layer))!=0)+" enabled="+camera.enabled+" stereo="+camera.stereoEnabled);
            }
            foreach(var c in Resources.FindObjectsOfTypeAll<Camera>()) if(c.gameObject.scene.IsValid()) b.AppendLine("CAMERA path="+PathOf(c.transform)+" enabled="+c.enabled+" active="+c.gameObject.activeInHierarchy+" mask="+c.cullingMask+" stereo="+c.stereoEnabled+" target="+(c.targetTexture==null?"screen":c.targetTexture.name));
            Directory.CreateDirectory(System.IO.Path.GetDirectoryName(file)); File.WriteAllText(file,b.ToString(),Encoding.UTF8); return "Read-only snapshot saved: "+file;
        }
    }
}
