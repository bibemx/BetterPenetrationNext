using System;
using System.IO;
using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    public sealed partial class NextNativeTests
    {
        [Serializable]public sealed class InnerCollarContract
        {
            public int Schema,OuterSubmesh,InnerSubmesh,OuterFaceStart,InnerFaceStart,FaceCount,OriginalVertices;public int[] Sources,InnerTriangles;
        }
        void RunInnerCollarNative(SkinnedMeshRenderer renderer)
        {
            InnerCollarContract contract=null;
            foreach(string arg in Environment.GetCommandLineArgs())if(arg.StartsWith("--bpn-uv-archive="))
                using(var file=File.OpenRead(arg.Substring(17)))using(var zip=new System.IO.Compression.ZipArchive(file,System.IO.Compression.ZipArchiveMode.Read))
                {var entry=zip.GetEntry("bpn/entry-junction.json");if(entry!=null)using(var reader=new StreamReader(entry.Open()))contract=JsonCodec.Deserialize<InnerCollarContract>(reader.ReadToEnd());}
            if(contract==null)return;
            Require(contract.Schema==1&&renderer.sharedMesh.vertexCount==contract.OriginalVertices+contract.Sources.Length,"inner collar has an explicit independent-normal surface contract");
            var baked=new Mesh();assets.Add(baked);renderer.BakeMesh(baked);var positions=baked.vertices;var outer=renderer.sharedMesh.GetTriangles(contract.OuterSubmesh);var inner=renderer.sharedMesh.GetTriangles(contract.InnerSubmesh);
            double maximumGap=0,maximumNormalDot=-1;int faces=0;
            for(int i=0;i<contract.FaceCount;i++)
            {
                int a=(contract.OuterFaceStart+i)*3,b=(contract.InnerFaceStart+i)*3;
                for(int k=0;k<3;k++)maximumGap=Math.Max(maximumGap,(positions[outer[a+k]]-positions[inner[b+2-k]]).magnitude);
                Vector3 x=Vector3.Cross(positions[outer[a+1]]-positions[outer[a]],positions[outer[a+2]]-positions[outer[a]]),y=Vector3.Cross(positions[inner[b+1]]-positions[inner[b]],positions[inner[b+2]]-positions[inner[b]]);
                if(x.sqrMagnitude>1e-20&&y.sqrMagnitude>1e-20){maximumNormalDot=Math.Max(maximumNormalDot,Vec3.Dot(AnimationAdapter.ToCore(x).Normalized,AnimationAdapter.ToCore(y).Normalized));faces++;}
            }
            Require(maximumGap<1e-7,"actual skinning keeps both entry-collar sides spatially joined","maximumGap="+maximumGap);
            Require(faces==contract.FaceCount&&maximumNormalDot<-.9999,"the entire entry collar has a continuous inward-facing side","faces="+faces+" maximumNormalDot="+maximumNormalDot);
            var uv=renderer.sharedMesh.uv;double span=0;for(int i=0;i<contract.FaceCount;i++){int k=(contract.InnerFaceStart+i)*3;for(int j=0;j<3;j++)span=Math.Max(span,(uv[inner[k+j]]-uv[inner[k+(j+1)%3]]).magnitude);}
            Require(span<.05,"inner collar remains inside its adjacent lining texture island","maximumUvEdge="+span);
        }
    }
}
