using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Reflection;
using UnityEngine;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        IEnumerator RunBoneImplantNative()
        {
            var archives=new List<string>();foreach(string arg in Environment.GetCommandLineArgs())if(arg.StartsWith("--bpn-implant-archive=",StringComparison.Ordinal))archives.Add(arg.Substring(22));
            Require(archives.Count>0,"actual ZIPMOD archives were supplied to the implant check");
            Type processType=Type.GetType("ModBoneImplantor.BoneImplantProcess, ModBoneImplantor",true);
            Type pluginType=Type.GetType("ModBoneImplantor.ModBoneImplantor, ModBoneImplantor",true);
            MethodInfo transplant=pluginType.GetMethod("TryImplantBones",BindingFlags.Static|BindingFlags.NonPublic);
            Require(transplant!=null,"installed BoneImplantor exposes the previously verified transplant entry point",pluginType.Assembly.GetName().Version.ToString());
            foreach(string archive in archives)
            {
                byte[] bytes;bool receiver;string name=Path.GetFileName(archive);
                using(var file=File.OpenRead(archive))using(var zip=new ZipArchive(file,ZipArchiveMode.Read)){
                    receiver=zip.GetEntry("bpn/simulation.json")!=null;ZipArchiveEntry entry=null;
                    foreach(var item in zip.Entries)if(item.FullName.EndsWith(".unity3d",StringComparison.Ordinal)){if(entry!=null)throw new InvalidDataException("Expected one model bundle");entry=item;}
                    Require(entry!=null,"actual model bundle exists: "+name);using(var memory=new MemoryStream()){using(var stream=entry.Open())stream.CopyTo(memory);bytes=memory.ToArray();}
                }
                var bundle=AssetBundle.LoadFromMemory(bytes);Require(bundle!=null,"actual ZIPMOD bundle loads: "+name);
                var holder=new GameObject("Inactive neutral transplant host");holder.SetActive(false);holder.transform.SetParent(root.transform,false);
                try
                {
                    var prefab=bundle.LoadAsset<GameObject>(receiver?"p_cf_body_bpn":"sos_dick");Require(prefab!=null,"actual model prefab resolves: "+name);
                    var instance=Object.Instantiate(prefab,holder.transform,false);var components=instance.GetComponentsInChildren(processType,true);
                    Require(components.Length>0,"actual model retains BoneImplantProcess components: "+name,"components="+components.Length);
                    var destinations=new Dictionary<string,GameObject>();var targetField=processType.GetField("trfDst");
                    foreach(var component in components){var target=(Transform)targetField.GetValue(component);Require(target!=null,"implant destination is present: "+name);if(destinations.ContainsKey(target.name))continue;var go=new GameObject(target.name);go.transform.SetParent(holder.transform,false);destinations.Add(target.name,go);}
                    object info=transplant.Invoke(null,new object[]{instance,destinations});Require(info!=null,"real BoneImplantor accepts actual package definitions: "+name);
                    var implanted=(List<Transform>)info.GetType().GetField("ImplantedBones").GetValue(info);int custom=0;
                    foreach(var bone in implanted){Require(bone!=null&&bone.parent!=null,"transplanted bone has a live parent: "+name);if(bone.name.StartsWith("MED_",StringComparison.Ordinal))custom++;}
                    if(receiver)Require(custom==4,"all four existing custom receiver bones are transplanted: "+name,"custom="+custom);
                    int renderers=0;
                    foreach(var renderer in instance.GetComponentsInChildren<SkinnedMeshRenderer>(true))
                    {
                        if(renderer.sharedMesh==null)continue;renderers++;var bones=renderer.bones;
                        Require(bones.Length==renderer.sharedMesh.bindposes.Length,"native renderer palette matches bindposes: "+name);
                        foreach(var bone in bones)Require(bone!=null,"native renderer retains live bone references: "+name);
                    }
                    Require(renderers>0,"actual transplanted model retains its skinned renderers: "+name,"renderers="+renderers+" transplanted="+implanted.Count);
                }
                finally{Object.Destroy(holder);bundle.Unload(false);}
                yield return null;
            }
            report.checks.Add(new Check{name="animation and complete containment",status="SKIP",detail="Inactive real ZIPMOD bone-transplant validation only; no contact or user-scene acceptance."});
        }
    }
}
