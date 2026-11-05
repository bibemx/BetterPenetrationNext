using System;
using System.Collections.Generic;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext
{
    // Keeps the native SkinnedMeshRenderer and all original materials. Only a
    // private rest-mesh clone is assigned; bone transforms are never modified.
    public sealed class MeshDeformationBinding : IDisposable
    {
        public readonly SkinnedMeshRenderer Renderer;
        public readonly Mesh Original;
        readonly Mesh working;
        readonly int[] uvDuplicates;
        readonly Vector3[] rest,output;
        readonly Vector3[] restNormals,normalOutput,restNormalSums,normalSums;
        readonly Vector4[] restTangents,tangentOutput;
        readonly int[] normalVertices,normalTriangles;
        readonly bool[] affectedNormals;
        readonly List<Vector3> nativeNormals=new List<Vector3>();
        readonly SimulationDefinition definition;
        readonly Transform[] bones;
        readonly Matrix4x4[] bindposes,boneMatrices;
        readonly int[] offsets,indices;
        readonly double[] weights;
        sealed class SkinGroup { public int Start,End; public Linear3 Inverse; }
        readonly List<SkinGroup> skinGroups=new List<SkinGroup>();
        readonly int[] bindingGroups;
        public int SkinningGroupCount { get { return skinGroups.Count; } }
        public int NormalVertexCount { get { return normalVertices.Length; } }
        readonly float[] morphWeights;
        readonly Bounds originalBounds;
        int frame;
        Mesh diagnosticPose,diagnosticRegion;
        int diagnosticFrame=-1;
        readonly List<Vector3> diagnosticVertices=new List<Vector3>(),diagnosticNormals=new List<Vector3>();
        public bool Alive { get { return Renderer!=null&&Renderer.sharedMesh==working; } }
        public bool OwnsMesh(Mesh mesh) { return mesh!=null&&mesh==working; }
        public bool Visible { get { return Renderer!=null&&Renderer.enabled&&Renderer.gameObject.activeInHierarchy; } }
        public MeshDeformationBinding(SkinnedMeshRenderer renderer,SimulationDefinition data)
        {
            Renderer=renderer; Original=renderer.sharedMesh;
            if(Original==null||!Original.isReadable) throw new ArgumentException("Prepared Visual Mesh must have Read/Write enabled.");
            data.Validate(Original.vertexCount); definition=data; rest=Original.vertices;uvDuplicates=data.UvPatch==null?new int[0]:data.UvPatch.Duplicates; output=VisualUvRemap.Expand(rest,uvDuplicates); originalBounds=renderer.localBounds;
            restNormals=Original.normals; if(restNormals.Length!=rest.Length) throw new ArgumentException("Prepared mesh normals are incomplete."); normalOutput=VisualUvRemap.Expand(restNormals,uvDuplicates); normalSums=new Vector3[rest.Length]; restNormalSums=new Vector3[rest.Length]; affectedNormals=new bool[rest.Length];
            var active=new bool[rest.Length]; foreach(var binding in data.Bindings) active[binding.Vertex]=true;
            var triangles=Original.triangles;
            for(int t=0;t<triangles.Length;t+=3) if(active[triangles[t]]||active[triangles[t+1]]||active[triangles[t+2]]) { affectedNormals[triangles[t]]=affectedNormals[triangles[t+1]]=affectedNormals[triangles[t+2]]=true; }
            if(data.XRayExcludedSubmeshes!=null) foreach(int submesh in data.XRayExcludedSubmeshes) if(submesh>=0&&submesh<Original.subMeshCount) foreach(int vertex in Original.GetTriangles(submesh)) if(!active[vertex]) affectedNormals[vertex]=false;
            var normalIndices=new List<int>(); var normalFaces=new List<int>(); for(int i=0;i<rest.Length;i++) if(affectedNormals[i]) normalIndices.Add(i);
            for(int t=0;t<triangles.Length;t+=3) if(affectedNormals[triangles[t]]||affectedNormals[triangles[t+1]]||affectedNormals[triangles[t+2]]) { normalFaces.Add(triangles[t]); normalFaces.Add(triangles[t+1]); normalFaces.Add(triangles[t+2]); }
            normalVertices=normalIndices.ToArray(); normalTriangles=normalFaces.ToArray(); AccumulateNormals(rest,restNormalSums); foreach(int vertex in normalVertices) restNormalSums[vertex]=restNormalSums[vertex].normalized;
            var core=new Vec3[rest.Length]; for(int i=0;i<rest.Length;i++) core[i]=AnimationAdapter.ToCore(rest[i]);
            if(SimulationDefinition.Signature(core,Original.triangles)!=data.VisualMeshSignature) throw new ArgumentException("Prepared visual mesh signature differs from its simulation mapping.");
            bones=renderer.bones; bindposes=Original.bindposes; if(bones.Length!=bindposes.Length) throw new ArgumentException("Incomplete visual rig."); boneMatrices=new Matrix4x4[bones.Length];
            var counts=Original.GetBonesPerVertex(); var all=Original.GetAllBoneWeights(); offsets=new int[rest.Length+1]; indices=new int[all.Length]; weights=new double[all.Length]; int cursor=0;
            for(int i=0;i<rest.Length;i++) { offsets[i]=cursor; for(int j=0;j<counts[i];j++) { var w=all[cursor]; indices[cursor]=w.boneIndex; weights[cursor]=w.weight; cursor++; } } offsets[rest.Length]=cursor;
            bindingGroups=new int[definition.Bindings.Length]; var groups=new Dictionary<ulong,List<int>>();
            for(int b=0;b<bindingGroups.Length;b++)
            {
                int vertex=definition.Bindings[b].Vertex,start=offsets[vertex],end=offsets[vertex+1]; ulong hash=14695981039346656037UL;
                unchecked { for(int j=start;j<end;j++) { hash=(hash^(ulong)indices[j])*1099511628211UL; hash=(hash^(ulong)BitConverter.DoubleToInt64Bits(weights[j]))*1099511628211UL; } }
                List<int> bucket; if(!groups.TryGetValue(hash,out bucket)) { bucket=new List<int>(); groups.Add(hash,bucket); }
                int found=-1;
                foreach(int candidate in bucket)
                {
                    var group=skinGroups[candidate]; bool same=group.End-group.Start==end-start;
                    for(int j=0;same&&j<end-start;j++) same=indices[start+j]==indices[group.Start+j]&&weights[start+j]==weights[group.Start+j];
                    if(same) { found=candidate; break; }
                }
                if(found<0) { found=skinGroups.Count; skinGroups.Add(new SkinGroup { Start=start,End=end }); bucket.Add(found); } bindingGroups[b]=found;
            }
            morphWeights=new float[Original.blendShapeCount]; for(int i=0;i<morphWeights.Length;i++) morphWeights[i]=renderer.GetBlendShapeWeight(i);
            working=Object.Instantiate(Original); working.name=Original.name+" [BPN private]";
            try{VisualUvRemap.Apply(Original,working,data.UvPatch);}catch{Object.Destroy(working);throw;}
            restTangents=working.tangents;tangentOutput=(Vector4[])restTangents.Clone();
            working.MarkDynamic(); renderer.sharedMesh=working; RestoreMorphWeights();
        }
        void RestoreMorphWeights() { for(int i=0;i<morphWeights.Length;i++) Renderer.SetBlendShapeWeight(i,morphWeights[i]); }
        public void Apply(SoftBodySolver solver,double maximumWorldOffset,int normalsInterval,double blend=1)
        { ApplyInternal(solver,null,maximumWorldOffset,normalsInterval,blend); }
        public void ApplyControlOffsets(Vec3[] controlOffsets,double maximumWorldOffset,int normalsInterval,double blend=1)
        {
            if(controlOffsets==null||controlOffsets.Length!=definition.Rest.Length)throw new ArgumentException("Control offset topology changed");
            ApplyInternal(null,controlOffsets,maximumWorldOffset,normalsInterval,blend);
        }
        void ApplyInternal(SoftBodySolver solver,Vec3[] controlOffsets,double maximumWorldOffset,int normalsInterval,double blend)
        {
            if(!Alive) throw new InvalidOperationException("Visual Renderer was replaced; release this binding.");
            for(int b=0;b<bones.Length;b++) { if(bones[b]==null) throw new InvalidOperationException("Visual skin bone was destroyed."); boneMatrices[b]=bones[b].localToWorldMatrix*bindposes[b]; }
            Array.Copy(rest,output,rest.Length);
            foreach(var group in skinGroups)
            {
                Vec3 x=new Vec3(),y=new Vec3(),z=new Vec3();
                for(int j=group.Start;j<group.End;j++)
                {
                    var m=boneMatrices[indices[j]]; double w=weights[j]; x+=new Vec3(m.m00,m.m10,m.m20)*w; y+=new Vec3(m.m01,m.m11,m.m21)*w; z+=new Vec3(m.m02,m.m12,m.m22)*w;
                }
                Linear3 inverse; if(!new Linear3(x,y,z).TryInverse(out inverse)) throw new InvalidOperationException("Singular visual skinning Jacobian."); group.Inverse=inverse;
            }
            double strength=Math.Max(0,Math.Min(1,blend));
            for(int i=0;i<definition.Bindings.Length;i++) { var binding=definition.Bindings[i];Vec3 world=controlOffsets==null?binding.Transfer(solver):(controlOffsets[binding.A]*binding.U+controlOffsets[binding.B]*binding.V+controlOffsets[binding.C]*binding.W)*binding.Influence; Vec3 local=skinGroups[bindingGroups[i]].Inverse.Multiply(world*strength); output[binding.Vertex]=rest[binding.Vertex]+AnimationAdapter.ToUnity(local); }
            VisualUvRemap.CopyDuplicates(output,rest.Length,uvDuplicates);working.vertices=output;
            if(frame++%Math.Max(1,normalsInterval)==0)
            {
                // Native recomputation wins for large affected regions in measured
                // KKS Mono runs. Restore all unmodified authored normals afterwards.
                if(normalTriangles.Length>12000&&uvDuplicates.Length==0)
                {
                    working.RecalculateNormals(); working.GetNormals(nativeNormals);
                    for(int vertex=0;vertex<rest.Length;vertex++)if(affectedNormals[vertex])TransportNormal(vertex,nativeNormals[vertex]);else normalOutput[vertex]=restNormals[vertex];
                }
                else
                {
                    foreach(int vertex in normalVertices) normalSums[vertex]=Vector3.zero;
                    AccumulateNormals(output,normalSums);
                    foreach(int vertex in normalVertices)TransportNormal(vertex,normalSums[vertex].normalized);
                }
                VisualUvRemap.CopyDuplicates(normalOutput,rest.Length,uvDuplicates);working.normals=normalOutput;
                if(tangentOutput.Length==output.Length){VisualUvRemap.CopyDuplicates(tangentOutput,rest.Length,uvDuplicates);working.tangents=tangentOutput;}
            }
            float scale=Mathf.Min(Mathf.Abs(Renderer.transform.lossyScale.x),Mathf.Abs(Renderer.transform.lossyScale.y),Mathf.Abs(Renderer.transform.lossyScale.z));
            Bounds bounds=originalBounds; bounds.Expand((float)(2*maximumWorldOffset/Math.Max(scale,1e-8))); Renderer.localBounds=bounds;
        }
        void TransportNormal(int vertex,Vector3 geometric)
        {
            if(geometric.sqrMagnitude<1e-12){normalOutput[vertex]=restNormals[vertex];return;}
            var rotation=Quaternion.FromToRotation(restNormalSums[vertex],geometric);normalOutput[vertex]=(rotation*restNormals[vertex]).normalized;
            if(tangentOutput.Length==output.Length){var t=restTangents[vertex];Vector3 q=rotation*new Vector3(t.x,t.y,t.z);tangentOutput[vertex]=new Vector4(q.x,q.y,q.z,t.w);}
        }
        void AccumulateNormals(Vector3[] positions,Vector3[] sums)
        {
            for(int i=0;i<normalTriangles.Length;i+=3)
            {
                int a=normalTriangles[i],b=normalTriangles[i+1],c=normalTriangles[i+2]; Vector3 normal=Vector3.Cross(positions[b]-positions[a],positions[c]-positions[a]);
                if(affectedNormals[a]) sums[a]+=normal; if(affectedNormals[b]) sums[b]+=normal; if(affectedNormals[c]) sums[c]+=normal;
            }
        }
        public double VerifyAppliedPose(SoftBodySolver solver)
        {return VerifyAppliedInternal(solver,null);}
        public double VerifyAppliedControlOffsets(Vec3[] offsets)
        {return VerifyAppliedInternal(null,offsets);}
        double VerifyAppliedInternal(SoftBodySolver solver,Vec3[] offsets)
        {
            var baked=new Mesh();
            try
            {
                Renderer.BakeMesh(baked); var actual=baked.vertices;
                Mesh current=Renderer.sharedMesh; Renderer.sharedMesh=Original;
                try { Renderer.BakeMesh(baked); } finally { Renderer.sharedMesh=current; }
                var baseline=baked.vertices; double error=0;
                foreach(var b in definition.Bindings)
                {
                    Vec3 measured=AnimationAdapter.ToCore(AnimationAdapter.BakedToWorld(Renderer).MultiplyVector(actual[b.Vertex]-baseline[b.Vertex]));
                    Vec3 expected=offsets==null?b.Transfer(solver):(offsets[b.A]*b.U+offsets[b.B]*b.V+offsets[b.C]*b.W)*b.Influence;
                    error=Math.Max(error,(measured-expected).Length);
                }
                return error;
            }
            finally { Object.Destroy(baked); }
        }
        public Mesh XRaySnapshot()
        {
            if(diagnosticPose==null) diagnosticPose=new Mesh { name="BPN diagnostic native pose" };
            if(diagnosticFrame==Time.frameCount) return diagnosticRegion;
            Renderer.BakeMesh(diagnosticPose); diagnosticPose.GetVertices(diagnosticVertices); diagnosticPose.GetNormals(diagnosticNormals);
            if(diagnosticRegion==null)
            {
                diagnosticRegion=Object.Instantiate(diagnosticPose); diagnosticRegion.name="BPN receiver working region"; diagnosticRegion.MarkDynamic();
                var active=new bool[working.vertexCount]; foreach(var b in definition.Bindings) if(b.Influence>0) active[b.Vertex]=true;VisualUvRemap.CopyDuplicates(active,rest.Length,uvDuplicates);
                diagnosticRegion.subMeshCount=Original.subMeshCount;
                for(int submesh=0;submesh<Original.subMeshCount;submesh++)
                {
                    var indices=new List<int>(); var all=working.GetTriangles(submesh); bool excluded=definition.XRayExcludedSubmeshes!=null&&Array.IndexOf(definition.XRayExcludedSubmeshes,submesh)>=0;
                    if(!excluded) for(int i=0;i<all.Length;i+=3) if(active[all[i]]||active[all[i+1]]||active[all[i+2]]) { indices.Add(all[i]); indices.Add(all[i+1]); indices.Add(all[i+2]); }
                    diagnosticRegion.SetTriangles(indices,submesh,false);
                }
            }
            diagnosticRegion.SetVertices(diagnosticVertices); diagnosticRegion.SetNormals(diagnosticNormals); diagnosticRegion.bounds=diagnosticPose.bounds; diagnosticFrame=Time.frameCount; return diagnosticRegion;
        }
        public void Dispose()
        {
            if(Renderer!=null&&Renderer.sharedMesh==working)
            { for(int i=0;i<morphWeights.Length;i++) morphWeights[i]=Renderer.GetBlendShapeWeight(i); Renderer.sharedMesh=Original; RestoreMorphWeights(); Renderer.localBounds=originalBounds; }
            if(working!=null) Object.Destroy(working);
            if(diagnosticPose!=null) Object.Destroy(diagnosticPose); if(diagnosticRegion!=null) Object.Destroy(diagnosticRegion);
        }
    }
}
