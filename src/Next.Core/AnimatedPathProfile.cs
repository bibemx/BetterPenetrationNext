using System;
using System.Runtime.Serialization;
namespace BetterPenetrationNext.Core
{
    // Schema 0/1: legacy renderer-local Points. Schema 2: each point is a
    // weighted sum of LBS terms in the prepared bone palette. Aggregation by
    // bone preserves the exact centroid, including its morph deltas; it does
    // not assign averaged skin weights to an invented single rest vertex.
    [DataContract] public class AnimatedPathProfile
    {
        [DataMember] public int Schema;
        [DataMember] public Vec3[] Points;
        [DataMember] public bool Spline=true;
        [DataMember] public string VisualMeshSignature;
        [DataMember] public string[] BoneNames,MorphNames;
        [DataMember] public int[] PointOffsets,BoneIndices;
        [DataMember] public double[] Weights;
        [DataMember] public Vec3[] Rest;
        [DataMember] public Vec3[][] MorphDeltas;
        [OnDeserializing] void Defaults(StreamingContext context) { Spline=true; }
        public bool Animated { get { return Schema==2; } }
        public int ControlCount { get { return Animated?PointOffsets.Length-1:Points.Length; } }
        public void Validate(SimulationDefinition definition)
        {
            if(Schema==0||Schema==1) {
                if(Points==null||Points.Length<2||Points.Length>4096) throw new ArgumentException("Invalid local path controls.");
                foreach(var p in Points) if(!p.IsFinite) throw new ArgumentException("Nonfinite path control.");
                return;
            }
            if(Schema!=2||definition==null||VisualMeshSignature!=definition.VisualMeshSignature) throw new ArgumentException("Animated path does not match receiver geometry.");
            if(BoneNames==null||BoneNames.Length!=definition.BoneNames.Length) throw new ArgumentException("Animated path bone palette differs.");
            for(int i=0;i<BoneNames.Length;i++) if(BoneNames[i]!=definition.BoneNames[i]) throw new ArgumentException("Animated path bone order differs.");
            int morphs=definition.Morphs==null?0:definition.Morphs.Length;
            if(MorphNames==null||MorphNames.Length!=morphs||MorphDeltas==null||MorphDeltas.Length!=morphs) throw new ArgumentException("Animated path morph palette differs.");
            if(PointOffsets==null||PointOffsets.Length<3||PointOffsets.Length>4097||PointOffsets[0]!=0||Rest==null||Rest.Length>100000||BoneIndices==null||Weights==null||BoneIndices.Length!=Rest.Length||Weights.Length!=Rest.Length||PointOffsets[PointOffsets.Length-1]!=Rest.Length) throw new ArgumentException("Invalid animated path terms.");
            for(int m=0;m<morphs;m++) {
                if(MorphNames[m]!=definition.Morphs[m].Name||MorphDeltas[m]==null||MorphDeltas[m].Length!=Rest.Length) throw new ArgumentException("Animated path morph terms differ.");
                foreach(var p in MorphDeltas[m]) if(!p.IsFinite) throw new ArgumentException("Nonfinite path morph.");
            }
            for(int i=0;i<PointOffsets.Length-1;i++) {
                if(PointOffsets[i]<0||PointOffsets[i+1]<=PointOffsets[i]||PointOffsets[i+1]>Rest.Length) throw new ArgumentException("Invalid path term offsets.");
                double sum=0;
                for(int t=PointOffsets[i];t<PointOffsets[i+1];t++) {
                    if(BoneIndices[t]<0||BoneIndices[t]>=BoneNames.Length||!Vec3.Finite(Weights[t])||Weights[t]<0||!Rest[t].IsFinite) throw new ArgumentException("Invalid animated path influence.");
                    sum+=Weights[t];
                }
                if(Math.Abs(sum-1)>1e-6) throw new ArgumentException("Unnormalized path influences.");
            }
        }
        // Arrays are retained by the animation adapter. No delegates, matrices,
        // skinned meshes or samples are allocated while capturing a moving path.
        public void Capture(Linear3[] linear,Vec3[] translation,double[] morphWeights,Vec3[] output)
        {
            for(int i=0;i<output.Length;i++) {
                Vec3 point=new Vec3();
                for(int t=PointOffsets[i];t<PointOffsets[i+1];t++) {
                    Vec3 p=Rest[t]; for(int m=0;m<MorphDeltas.Length;m++) p+=MorphDeltas[m][t]*morphWeights[m];
                    int b=BoneIndices[t]; point+=(linear[b].Multiply(p)+translation[b])*Weights[t];
                }
                output[i]=point;
            }
        }
    }
}
