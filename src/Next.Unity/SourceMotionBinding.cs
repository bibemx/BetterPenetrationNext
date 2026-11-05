using System;
using System.Collections.Generic;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext
{
    // Correct the actual selected source vertices after animation, without editing
    // shared assets or competing with upstream IK. Outside the selected region,
    // two adjacency rings blend the correction down to zero.
    public sealed class SourceMotionBinding : IDisposable
    {
        public readonly Mesh Original,Working;
        readonly SkinnedMeshRenderer renderer;
        readonly Vector3[] rest,output,restNormals;
        readonly Transform[] bones;
        readonly Matrix4x4[] bindposes,matrices;
        readonly int[] offsets,indices,active;
        readonly double[] weights,mask;
        readonly Bounds originalBounds;
        bool applied,normalsChanged;
        public SourceMotionBinding(SkinnedMeshRenderer renderer,int[] selectedTriangles)
        {
            this.renderer=renderer; Original=renderer.sharedMesh; originalBounds=renderer.localBounds;
            if(!Original.isReadable) throw new ArgumentException("Source mesh requires readable vertices for reversible correction.");
            rest=Original.vertices; output=(Vector3[])rest.Clone(); restNormals=Original.normals; mask=new double[rest.Length];
            foreach(int i in selectedTriangles) mask[i]=1;
            var triangles=Original.triangles;
            for(int ring=0;ring<2;ring++) {
                var old=(double[])mask.Clone();
                for(int t=0;t<triangles.Length;t+=3) { double m=Math.Max(old[triangles[t]],Math.Max(old[triangles[t+1]],old[triangles[t+2]]))*.5; for(int j=0;j<3;j++) mask[triangles[t+j]]=Math.Max(mask[triangles[t+j]],m); }
            }
            var list=new List<int>(); for(int i=0;i<mask.Length;i++) if(mask[i]>0) list.Add(i); active=list.ToArray();
            bones=renderer.bones; bindposes=Original.bindposes; matrices=new Matrix4x4[bones.Length];
            var counts=Original.GetBonesPerVertex(); var all=Original.GetAllBoneWeights(); offsets=new int[rest.Length+1]; indices=new int[all.Length]; weights=new double[all.Length]; int cursor=0;
            for(int i=0;i<rest.Length;i++) { offsets[i]=cursor; for(int j=0;j<counts[i];j++) { indices[cursor]=all[cursor].boneIndex; weights[cursor]=all[cursor].weight; cursor++; } } offsets[rest.Length]=cursor;
            Working=Object.Instantiate(Original); Working.name=Original.name+" [BPN source motion]"; Working.MarkDynamic();
            var morphs=new float[Original.blendShapeCount]; for(int i=0;i<morphs.Length;i++) morphs[i]=renderer.GetBlendShapeWeight(i);
            renderer.sharedMesh=Working; for(int i=0;i<morphs.Length;i++) renderer.SetBlendShapeWeight(i,morphs[i]);
        }
        public void RestoreBaseline()
        { if(applied&&renderer!=null&&renderer.sharedMesh==Working) { Working.vertices=rest; RestoreNormals(); renderer.localBounds=originalBounds; applied=false; } }
        void RestoreNormals()
        {
            // These are the mesh's authored rest normals, not BakeMesh output.
            // Blend Shape normal deltas remain in Working and Unity adds the
            // current weights once during skinning, after this restoration.
            if(normalsChanged) { Working.normals=restNormals; normalsChanged=false; }
        }
        public void Apply(SourceContactBody body,Vec3[] animatedWorld,double blend=1)
        {
            if(renderer==null||renderer.sharedMesh!=Working) throw new InvalidOperationException("Source renderer replaced.");
            if(animatedWorld.Length!=rest.Length) throw new ArgumentException("Source topology changed.");
            for(int b=0;b<bones.Length;b++) { if(bones[b]==null) throw new InvalidOperationException("Source bone destroyed."); matrices[b]=bones[b].localToWorldMatrix*bindposes[b]; }
            Array.Copy(rest,output,rest.Length);
            foreach(int i in active) {
                Vec3 x=new Vec3(),y=new Vec3(),z=new Vec3();
                for(int j=offsets[i];j<offsets[i+1];j++) { var m=matrices[indices[j]]; double w=weights[j]; x+=new Vec3(m.m00,m.m10,m.m20)*w; y+=new Vec3(m.m01,m.m11,m.m21)*w; z+=new Vec3(m.m02,m.m12,m.m22)*w; }
                Vec3 local; if(!new Linear3(x,y,z).TryInverseMultiply((body.ToCorrectedVertex(i,animatedWorld[i])-animatedWorld[i])*(mask[i]*Math.Max(0,Math.Min(1,blend))),out local)) throw new InvalidOperationException("Singular source skinning Jacobian.");
                output[i]=rest[i]+AnimationAdapter.ToUnity(local);
            }
            Working.vertices=output;
            // Translation preserves selected-region normals. At the blend boundary
            // recalculate only when a correction is present (native operation).
            if(body.Deformation!=null||body.Offset.LengthSquared+body.RotationVector.LengthSquared>1e-20) { Working.RecalculateNormals(); normalsChanged=true; }
            else RestoreNormals();
            float scale=Mathf.Min(Mathf.Abs(renderer.transform.lossyScale.x),Mathf.Abs(renderer.transform.lossyScale.y),Mathf.Abs(renderer.transform.lossyScale.z));
            var bounds=originalBounds; bounds.Expand((float)(2*(body.MaximumDisplacement+originalBounds.size.magnitude*body.RotationVector.Length)/Math.Max(scale,1e-8))); renderer.localBounds=bounds; applied=true;
        }
        public void Dispose()
        {
            if(renderer!=null&&renderer.sharedMesh==Working) {
                var morphs=new float[Working.blendShapeCount]; for(int i=0;i<morphs.Length;i++) morphs[i]=renderer.GetBlendShapeWeight(i);
                renderer.sharedMesh=Original; for(int i=0;i<morphs.Length;i++) renderer.SetBlendShapeWeight(i,morphs[i]); renderer.localBounds=originalBounds;
            }
            if(Working!=null) Object.Destroy(Working);
        }
    }
}
