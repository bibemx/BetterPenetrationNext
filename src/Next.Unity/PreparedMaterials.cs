using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext
{
    // The external skin material remains the selector/game-owned material at slot
    // zero. Only additional authored submeshes receive private local materials.
    public sealed class PreparedMaterials : IDisposable
    {
        [Serializable] public sealed class Entry { public string Name,Texture; public float[] Color; }
        [Serializable] public sealed class Contract { public Entry[] Materials; }
        readonly SkinnedMeshRenderer renderer;
        readonly Material[] original,assigned;
        readonly List<Material> created=new List<Material>();
        readonly List<Texture2D> textures=new List<Texture2D>();
        public PreparedMaterials(SkinnedMeshRenderer renderer,string package)
        {
            this.renderer=renderer; original=renderer.sharedMaterials;
            if(original.Length==0||original[0]==null) throw new ArgumentException("Prepared body has no game skin material.");
            using(var file=File.OpenRead(package)) using(var zip=new ZipArchive(file,ZipArchiveMode.Read))
            {
                var descriptor=zip.GetEntry("bpn/materials.json"); if(descriptor==null||descriptor.Length>1024*1024) throw new InvalidDataException("Prepared material contract is absent/too large.");
                Contract contract; using(var reader=new StreamReader(descriptor.Open())) contract=JsonCodec.Deserialize<Contract>(reader.ReadToEnd());
                if(contract==null||contract.Materials==null||contract.Materials.Length!=renderer.sharedMesh.subMeshCount) throw new InvalidDataException("Prepared submesh/material contract differs.");
                assigned=new Material[contract.Materials.Length]; assigned[0]=original[0]; var loaded=new Dictionary<string,Texture2D>(); var shader=Shader.Find("Standard");
                if(shader==null) throw new NotSupportedException("Unity Standard shader unavailable for supplied materials.");
                try
                {
                    for(int i=1;i<assigned.Length;i++)
                    {
                        var entry=contract.Materials[i]; var material=new Material(shader) { name="BPN "+entry.Name }; created.Add(material); assigned[i]=material;
                        var c=entry.Color; material.color=c!=null&&c.Length==4?new Color(c[0],c[1],c[2],c[3]):UnityEngine.Color.white;
                        material.SetFloat("_Glossiness",0.28f);
                        if(!string.IsNullOrEmpty(entry.Texture))
                        {
                            Texture2D texture;
                            if(!loaded.TryGetValue(entry.Texture,out texture))
                            {
                                var image=zip.GetEntry("bpn/"+entry.Texture); if(image==null||image.Length>32*1024*1024) throw new InvalidDataException("Authored texture absent/too large.");
                                byte[] data; using(var memory=new MemoryStream()) { using(var input=image.Open()) input.CopyTo(memory); data=memory.ToArray(); }
                                texture=new Texture2D(2,2,TextureFormat.RGBA32,true); textures.Add(texture); texture.name=entry.Texture;
                                if(!ImageConversion.LoadImage(texture,data,true)) throw new InvalidDataException("Authored texture failed to decode."); loaded.Add(entry.Texture,texture);
                            }
                            material.mainTexture=texture;
                        }
                    }
                    renderer.sharedMaterials=assigned;
                }
                catch { Dispose(); throw; }
            }
        }
        public void Dispose()
        {
            if(renderer!=null&&assigned!=null)
            { var current=renderer.sharedMaterials; bool owns=current.Length==assigned.Length; for(int i=0;owns&&i<current.Length;i++) owns=current[i]==assigned[i]; if(owns) renderer.sharedMaterials=original; }
            foreach(var m in created) if(m!=null) Object.Destroy(m); created.Clear(); foreach(var t in textures) if(t!=null) Object.Destroy(t); textures.Clear();
        }
    }
}
