using System;
using System.Collections.Generic;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext.Tests
{
    public static class SourceSelectionFixture
    {
        public static void Run(Action<bool,string> check)
        {
            var assets=new List<Object>(); var group=new GameObject("neutral source-selection fixture");
            group.transform.position=new Vector3(-42.5f,1,-115); var joint=new GameObject("neutral joint"); joint.transform.SetParent(group.transform,false);
            ContactDetection selected=null;
            try
            {
                var main=Skin(group,joint.transform,"visible open surface",PrimitiveMeshes.Grid(4,4,.4,.8),assets);
                var all=main.sharedMesh.triangles; var first=new int[all.Length/2]; var second=new int[all.Length-first.Length]; Array.Copy(all,first,first.Length); Array.Copy(all,first.Length,second,0,second.Length);
                main.sharedMesh.subMeshCount=2; main.sharedMesh.SetTriangles(first,0); main.sharedMesh.SetTriangles(second,1); main.sharedMaterials=new[]{main.sharedMaterial,main.sharedMaterial};
                var helper=Skin(group,joint.transform,"hidden closed helper",PrimitiveMeshes.Sphere(12,8,.1),assets); helper.gameObject.SetActive(false);
                var chain=new List<Transform> { joint.transform };
                selected=ContactDetection.FromChain(group.transform,chain);
                check(selected!=null&&selected.Renderer==main,"visible open renderer outranks hidden closed helper"); selected.Dispose(); selected=null;
                helper.sharedMaterials=new Material[0]; main.enabled=false;
                selected=ContactDetection.FromChain(group.transform,chain);
                check(selected!=null&&selected.Renderer==main&&!selected.Visible&&selected.Alive,"materialless collision helper is excluded even when the real source is temporarily hidden");
                main.enabled=true; check(selected.Visible&&selected.Alive,"same source binding becomes visible again");
                main.enabled=false; helper.gameObject.SetActive(true);
                check(!ContactDetection.HasVisibleCandidate(group.transform,chain,main),"visible materialless helper cannot trigger a source rebind");
                helper.sharedMaterial=main.sharedMaterial;
                check(ContactDetection.HasVisibleCandidate(group.transform,chain,main),"visible rendered replacement is discovered independently of archive names");
                helper.gameObject.SetActive(false); main.enabled=true;
                selected.Capture(); var snapshot=selected.XRaySnapshot(); check(snapshot!=null&&snapshot.GetIndexCount(0)>0,"selected visible source produces actual overlay geometry");
                check(snapshot.subMeshCount==2&&snapshot.GetIndexCount(1)>0,"source snapshot preserves submesh material routing");
                selected.Dispose(); selected=null; main.enabled=false;
                var accessory=Skin(group,joint.transform,"smaller rendered alternative",PrimitiveMeshes.Sphere(4,2,.1),assets); accessory.gameObject.SetActive(false); helper.sharedMaterials=new Material[0];
                selected=ContactDetection.FromChain(group.transform,chain);
                check(selected!=null&&selected.Renderer==main,"hidden main surface outranks a smaller rendered alternative by weighted coverage");
            }
            finally { if(selected!=null) selected.Dispose(); Object.DestroyImmediate(group); foreach(var asset in assets) if(asset!=null) Object.DestroyImmediate(asset); }
        }
        static SkinnedMeshRenderer Skin(GameObject root,Transform joint,string name,MeshData data,List<Object> assets)
        {
            var go=new GameObject(name); go.transform.SetParent(root.transform,false); var mesh=new Mesh { name=name }; assets.Add(mesh);
            var vertices=new Vector3[data.Vertices.Length]; var weights=new BoneWeight[vertices.Length];
            for(int i=0;i<vertices.Length;i++) { vertices[i]=AnimationAdapter.ToUnity(data.Vertices[i]); weights[i]=new BoneWeight { boneIndex0=0,weight0=1 }; }
            mesh.vertices=vertices; mesh.triangles=data.Triangles; mesh.boneWeights=weights; mesh.bindposes=new[]{joint.worldToLocalMatrix*go.transform.localToWorldMatrix}; mesh.RecalculateNormals(); mesh.RecalculateBounds();
            var renderer=go.AddComponent<SkinnedMeshRenderer>(); renderer.sharedMesh=mesh; renderer.bones=new[]{joint}; renderer.rootBone=joint; renderer.updateWhenOffscreen=true;
            var shader=Shader.Find("Standard"); if(shader==null) throw new InvalidOperationException("Fixture shader unavailable."); var material=new Material(shader); assets.Add(material); renderer.sharedMaterial=material; return renderer;
        }
    }
}
