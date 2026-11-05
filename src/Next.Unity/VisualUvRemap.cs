using System;
using BetterPenetrationNext.Core;
using Unity.Collections;
using UnityEngine;

namespace BetterPenetrationNext
{
    // One-time render seam split. Every duplicate retains the complete source
    // skin/morph identity and follows its source vertex on every mesh upload.
    public static class VisualUvRemap
    {
        public static T[] Expand<T>(T[] original,int[] duplicates)
        {if(original.Length==0)return original;var result=new T[original.Length+duplicates.Length];Array.Copy(original,result,original.Length);for(int i=0;i<duplicates.Length;i++)result[original.Length+i]=original[duplicates[i]];return result;}
        public static void CopyDuplicates<T>(T[] values,int originalCount,int[] duplicates)
        {for(int i=0;i<duplicates.Length;i++)values[originalCount+i]=values[duplicates[i]];}
        public static void Apply(Mesh original,Mesh working,VisualUvPatch patch)
        {
            if(patch==null)return;patch.Validate(original.vertexCount,original.triangles);
            int[] copies=patch.Duplicates;int count=original.vertexCount,total=count+copies.Length;
            working.ClearBlendShapes();if(total>65535)working.indexFormat=UnityEngine.Rendering.IndexFormat.UInt32;
            working.vertices=Expand(original.vertices,copies);working.normals=Expand(original.normals,copies);working.tangents=Expand(original.tangents,copies);working.colors32=Expand(original.colors32,copies);
            working.uv3=Expand(original.uv3,copies);working.uv4=Expand(original.uv4,copies);
            working.uv2=Expand(original.uv2,copies);var uv=new Vector2[total];for(int i=0;i<total;i++)uv[i]=new Vector2((float)patch.Coordinates[i*2],(float)patch.Coordinates[i*2+1]);working.uv=uv;
            var oldCounts=original.GetBonesPerVertex();var oldWeights=original.GetAllBoneWeights();var offsets=new int[count+1];for(int i=0;i<count;i++)offsets[i+1]=offsets[i]+oldCounts[i];int weightCount=oldWeights.Length;foreach(int i in copies)weightCount+=oldCounts[i];
            var counts=new NativeArray<byte>(total,Allocator.Temp);var weights=new NativeArray<BoneWeight1>(weightCount,Allocator.Temp);
            try{int cursor=0;for(int i=0;i<total;i++){int source=i<count?i:copies[i-count];counts[i]=oldCounts[source];for(int j=offsets[source];j<offsets[source+1];j++)weights[cursor++]=oldWeights[j];}working.SetBoneWeights(counts,weights);}
            // GetBonesPerVertex/GetAllBoneWeights are Allocator.None views owned
            // by the source Mesh. Dispose only the temporary buffers we own.
            finally{counts.Dispose();weights.Dispose();}
            working.ClearBlendShapes();var v=new Vector3[count];var n=new Vector3[count];var t=new Vector3[count];
            for(int shape=0;shape<original.blendShapeCount;shape++)for(int frame=0;frame<original.GetBlendShapeFrameCount(shape);frame++)
            {original.GetBlendShapeFrameVertices(shape,frame,v,n,t);working.AddBlendShapeFrame(original.GetBlendShapeName(shape),original.GetBlendShapeFrameWeight(shape,frame),Expand(v,copies),Expand(n,copies),Expand(t,copies));}
            int at=0;for(int sub=0;sub<original.subMeshCount;sub++){int length=original.GetTriangles(sub).Length;var faces=new int[length];Array.Copy(patch.Triangles,at,faces,0,length);working.SetTriangles(faces,sub,false);at+=length;}
            if(working.tangents.Length==total){working.RecalculateTangents();var tangents=working.tangents;var authored=original.tangents;if(authored.Length==count){foreach(int i in original.GetTriangles(0))tangents[i]=authored[i];working.tangents=tangents;}}working.bounds=original.bounds;
        }
    }
}
