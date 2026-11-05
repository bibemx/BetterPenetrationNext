using System;
using System.Collections;
using BetterPenetrationNext.Core;
using UnityEngine;
namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        IEnumerator RunAnimatedSource()
        {
            var grid=PrimitiveMeshes.Grid(2,2,.4,.8); int count=grid.Vertices.Length;
            var points=new Vec3[count*2]; var triangles=new int[grid.Triangles.Length*2];
            Array.Copy(grid.Vertices,points,count); Array.Copy(grid.Vertices,0,points,count,count);
            Array.Copy(grid.Triangles,triangles,grid.Triangles.Length);
            for(int i=0;i<grid.Triangles.Length;i++) triangles[grid.Triangles.Length+i]=grid.Triangles[i]+count;
            var renderer=Skin("BPN neutral independently skinned panels",new MeshData { Vertices=points,Triangles=triangles },Vector3.zero);
            var joint=new GameObject("BPN neutral second joint"); joint.transform.SetParent(renderer.transform,false);
            renderer.bones=new[]{renderer.bones[0],joint.transform}; var mesh=renderer.sharedMesh;
            mesh.bindposes=new[]{renderer.bones[0].worldToLocalMatrix*renderer.transform.localToWorldMatrix,joint.transform.worldToLocalMatrix*renderer.transform.localToWorldMatrix};
            var weights=new BoneWeight[points.Length]; var evidence=new double[points.Length];
            for(int i=0;i<weights.Length;i++) { weights[i]=new BoneWeight { boneIndex0=i<count?0:1,weight0=1 }; evidence[i]=1; } mesh.boneWeights=weights;
            ContactDetection geometry=null;
            try
            {
                geometry=new ContactDetection(renderer,evidence); var displayMesh=geometry.XRaySnapshot();
                Require(geometry.DeformationGroups==2,"coincident source panels keep distinct skinning groups");
                joint.transform.localPosition=new Vector3(0,0,.08f); yield return null; geometry.Capture(); var actual=geometry.XRaySnapshot().vertices;
                Require(geometry.Alive&&!geometry.UsesUnweldedFallback&&Math.Abs(actual[count].z-actual[0].z-.08)<.0001,"actual source snapshot follows independently animated bones without losing registration");
                Require(geometry.XRaySnapshot()==displayMesh,"source pose updates reuse the existing overlay mesh"); geometry.Dispose(); geometry=null;
                joint.transform.localPosition=Vector3.zero;
                for(int i=0;i<weights.Length;i++) weights[i].boneIndex0=0; mesh.boneWeights=weights;
                geometry=new ContactDetection(renderer,evidence); displayMesh=geometry.XRaySnapshot();
                for(int i=count;i<weights.Length;i++) weights[i].boneIndex0=1; mesh.boneWeights=weights; joint.transform.localPosition=new Vector3(0,0,.12f);
                yield return null; geometry.Capture(); actual=geometry.XRaySnapshot().vertices;
                Require(geometry.Alive&&geometry.UsesUnweldedFallback&&geometry.Kind==SurfaceKind.OrientedShell,"an in-place skinning edit falls back to real unwelded triangles while retaining the binding");
                Require(geometry.XRaySnapshot()==displayMesh&&Math.Abs(actual[count].z-actual[0].z-.12)<.0001,"seam fallback preserves the overlay instance and its actual deformed vertices"); geometry.Dispose(); geometry=null;
                joint.transform.localPosition=Vector3.zero; for(int i=0;i<weights.Length;i++) weights[i].boneIndex0=0; mesh.boneWeights=weights;
                var deltas=new Vector3[weights.Length]; for(int i=count;i<deltas.Length;i++) deltas[i]=new Vector3(0,0,.07f);
                mesh.AddBlendShapeFrame("neutral independent panel morph",100,deltas,new Vector3[deltas.Length],new Vector3[deltas.Length]); renderer.SetBlendShapeWeight(0,100);
                geometry=new ContactDetection(renderer,evidence); actual=geometry.XRaySnapshot().vertices;
                Require(geometry.DeformationGroups==2&&geometry.Alive&&Math.Abs(actual[count].z-actual[0].z-.07)<.0001,"different positional blend-shape deltas prevent an invalid source weld");
            }
            finally { if(geometry!=null) geometry.Dispose(); renderer.enabled=false; }
        }
    }
}
