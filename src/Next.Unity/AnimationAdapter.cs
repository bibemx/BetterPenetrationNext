using System;
using BetterPenetrationNext.Core;
using UnityEngine;

namespace BetterPenetrationNext
{
    // Prepared simulation vertices are skinned directly on CPU; never BakeMesh the
    // whole visual body merely to update a few hundred physical particles.
    public sealed class AnimationAdapter
    {
        readonly SimulationDefinition definition;
        readonly Transform[] bones;
        readonly Matrix4x4[] bindposes,matrices;
        readonly Linear3[] lastJacobians;
        readonly SkinnedMeshRenderer renderer;
        readonly int[] morphIndices;
        readonly double[] morphWeights;
        readonly Linear3[] pathLinear;
        readonly Vec3[] pathTranslation;
        bool initialized;
        public readonly Vec3[] Anchors;
        public double PosedSize { get; private set; }
        public Linear3[] CapturedLinear {get{return pathLinear;}}
        public Vec3[] CapturedTranslation {get{return pathTranslation;}}
        public double[] CapturedMorphs {get{return morphWeights;}}
        public AnimationAdapter(SimulationDefinition value,SkinnedMeshRenderer renderer)
        {
            definition=value; this.renderer=renderer; Anchors=new Vec3[value.Rest.Length]; bones=new Transform[value.BoneNames.Length];
            int morphCount=value.Morphs==null?0:value.Morphs.Length; morphIndices=new int[morphCount]; morphWeights=new double[morphCount];
            for(int i=0;i<morphCount;i++) { morphIndices[i]=renderer.sharedMesh.GetBlendShapeIndex(value.Morphs[i].Name); if(morphIndices[i]<0) throw new ArgumentException("Prepared morph missing: "+value.Morphs[i].Name); }
            matrices=new Matrix4x4[bones.Length]; bindposes=new Matrix4x4[bones.Length]; lastJacobians=new Linear3[Anchors.Length]; pathLinear=new Linear3[bones.Length]; pathTranslation=new Vec3[bones.Length];
            var available=renderer.bones;
            for(int b=0;b<bones.Length;b++)
            {
                foreach(var t in available) if(t!=null&&t.name==value.BoneNames[b]) { if(bones[b]!=null&&bones[b]!=t) throw new ArgumentException("Ambiguous prepared bone: "+value.BoneNames[b]); bones[b]=t; }
                if(bones[b]==null) throw new ArgumentException("Prepared simulation bone is absent: "+value.BoneNames[b]);
                var matrix=new Matrix4x4(); for(int row=0;row<4;row++) for(int col=0;col<4;col++) matrix[row,col]=value.Bindposes[b*16+row*4+col]; bindposes[b]=matrix;
            }
        }
        public void Capture(SoftBodySolver solver)
        {
            for(int b=0;b<bones.Length;b++) { if(bones[b]==null) throw new InvalidOperationException("An implanted simulation bone was destroyed."); matrices[b]=bones[b].localToWorldMatrix*bindposes[b]; pathLinear[b]=Linear(matrices[b]); pathTranslation[b]=ToCore(matrices[b].MultiplyPoint3x4(Vector3.zero)); }
            for(int m=0;m<morphIndices.Length;m++) morphWeights[m]=renderer.GetBlendShapeWeight(morphIndices[m])*0.01f;
            var bounds=Bounds3.Empty;
            for(int i=0;i<Anchors.Length;i++)
            {
                Vec3 position=new Vec3(),x=new Vec3(),y=new Vec3(),z=new Vec3();
                Vec3 rest=definition.Rest[i]; for(int m=0;m<morphIndices.Length;m++) rest+=definition.Morphs[m].Deltas[i]*morphWeights[m];
                for(int j=definition.InfluenceOffsets[i];j<definition.InfluenceOffsets[i+1];j++)
                {
                    var matrix=matrices[definition.BoneIndices[j]]; double w=definition.BoneWeights[j];
                    position+=ToCore(matrix.MultiplyPoint3x4(ToUnity(rest)))*w;
                    x+=new Vec3(matrix.m00,matrix.m10,matrix.m20)*w; y+=new Vec3(matrix.m01,matrix.m11,matrix.m21)*w; z+=new Vec3(matrix.m02,matrix.m12,matrix.m22)*w;
                }
                Anchors[i]=position; bounds.Include(position); var jacobian=new Linear3(x,y,z);
                if(initialized&&solver!=null) solver.Transport(i,lastJacobians[i],jacobian); lastJacobians[i]=jacobian;
            }
            initialized=true; PosedSize=(bounds.Max-bounds.Min).Length; if(PosedSize<1e-8) throw new InvalidOperationException("Collapsed simulation pose.");
        }
        public static Vec3 ToCore(Vector3 v) { return new Vec3(v.x,v.y,v.z); }
        public void CapturePath(AnimatedPathProfile profile,Vec3[] output) { profile.Capture(pathLinear,pathTranslation,morphWeights,output); }
        public static Vector3 ToUnity(Vec3 v) { return new Vector3((float)v.X,(float)v.Y,(float)v.Z); }
        public static Linear3 Linear(Matrix4x4 m) { return new Linear3(new Vec3(m.m00,m.m10,m.m20),new Vec3(m.m01,m.m11,m.m21),new Vec3(m.m02,m.m12,m.m22)); }
        // The installed Unity 2019.4.9 exposes only BakeMesh(Mesh), with scale
        // already baked into its output. Applying localToWorld scale again doubles
        // it. This matrix is verified against native deltas in the KKS fixture.
        public static Matrix4x4 BakedToWorld(SkinnedMeshRenderer renderer)
        { return Matrix4x4.TRS(renderer.transform.position,renderer.transform.rotation,Vector3.one); }
    }
}
