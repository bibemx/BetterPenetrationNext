using System;
using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    // Loading-time verification against the complete variable-influence Unity
    // stream. The legacy Mesh.boneWeights view can omit the fifth-eighth bone.
    public static class NativeSkinContract
    {
        public static int Verify(SimulationDefinition definition, SkinnedMeshRenderer renderer)
        {
            var mesh=renderer.sharedMesh;
            var counts=mesh.GetBonesPerVertex();
            var all=mesh.GetAllBoneWeights();
            var bones=renderer.bones;
            var offsets=new int[counts.Length+1];
            for(int i=0;i<counts.Length;i++)offsets[i+1]=offsets[i]+counts[i];
            int maximum=0;
            foreach(var binding in definition.Bindings)
            {
                if(binding.A!=binding.B || binding.A!=binding.C || binding.U!=1 || binding.Influence!=1)
                    throw new ArgumentException("Reactive skin contract requires exact material identities.");
                int i=binding.A,start=offsets[binding.Vertex],end=offsets[binding.Vertex+1];
                maximum=Math.Max(maximum,end-start);
                if(end-start!=definition.InfluenceOffsets[i+1]-definition.InfluenceOffsets[i])
                    throw new ArgumentException("Prepared metadata omits native bone influences at vertex "+binding.Vertex+". Update the experimental receiver metadata.");
                for(int j=start;j<end;j++)
                {
                    var actual=all[j];
                    bool found=false;
                    for(int k=definition.InfluenceOffsets[i];k<definition.InfluenceOffsets[i+1];k++)
                        if(bones[actual.boneIndex]!=null && bones[actual.boneIndex].name==definition.BoneNames[definition.BoneIndices[k]] &&
                           Math.Abs(actual.weight-definition.BoneWeights[k])<3e-5){found=true;break;}
                    if(!found)throw new ArgumentException("Prepared/native skin weights differ at vertex "+binding.Vertex+". Different-weight seam identities must remain separate.");
                }
            }
            return maximum;
        }
    }
}
