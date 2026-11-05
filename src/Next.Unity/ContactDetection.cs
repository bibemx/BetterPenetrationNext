using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using BetterPenetrationNext.Core;
using UnityEngine;
using Object=UnityEngine.Object;

namespace BetterPenetrationNext
{
    public sealed class ContactDetection : IDisposable,IHintedContactSurface,IContactBounds
    {
        public readonly SkinnedMeshRenderer Renderer;
        readonly Mesh original,baked;
        readonly List<Vector3> vertices=new List<Vector3>();
        readonly Vec3[] world;
        readonly Vec3[] guided;
        WeldedContactMesh welded;
        TriangleSurface surface;
        TriangleSurface originalSurface,proxySurface;
        AnimatedSeamContactProxy seamProxy;
        SurfaceKind originalKind;
        public bool EnableSeamProxy=true;
        public double MaximumSeamGapRatio=.05;
        public bool UsesSeamProxy { get; private set; }
        public int VirtualContactTriangles { get { return seamProxy==null?0:seamProxy.AddedTriangles; } }
        public double MaximumSeamGap { get { return seamProxy==null?0:seamProxy.MaximumGap; } }
        readonly int[] drawTriangles;
        readonly int[] selectedVertices;
        readonly int[][] drawSubmeshes;
        Mesh display;
        Mesh displayPose;
        SourceMotionBinding motion;
        public Vec3[] AnimatedVertices { get { return world; } }
        public Vec3[] ReferenceVertices {get;private set;}
        public int[] SelectedTriangles {get{return drawTriangles;}}
        public TriangleSurface ActiveContactSurface {get{return surface;}}
        readonly List<Vector3> normals=new List<Vector3>();
        int displayFrame=-1;
        int capturedFrame=-1;
        public SurfaceKind Kind { get; private set; }
        public bool UsesUnweldedFallback { get; private set; }
        public int DeformationGroups { get; private set; }
        public bool HasVolumeBounds { get { return Kind==SurfaceKind.ClosedVolume; } }
        public Bounds3 ContactBounds { get { return surface.Bounds; } }
        public int TriangleCount { get { return surface.TriangleCount; } }
        public double CaptureMilliseconds { get; private set; }
        public double RefitMilliseconds { get; private set; }
        public bool Alive { get { return Renderer!=null&&Renderer.sharedMesh==(motion==null?original:motion.Working); } }
        public bool Visible { get { return Renderer!=null&&Renderer.enabled&&Renderer.gameObject.activeInHierarchy; } }
        public ContactDetection(SkinnedMeshRenderer renderer,double[] evidence,double threshold=0.2)
        {
            Renderer=renderer; original=renderer.sharedMesh; baked=new Mesh { name="BPN source pose" }; baked.MarkDynamic();
            try
            {
                renderer.BakeMesh(baked); baked.GetVertices(vertices); world=new Vec3[vertices.Count]; guided=new Vec3[vertices.Count];
                var selected=new List<int>(); drawSubmeshes=new int[baked.subMeshCount][];
                for(int slot=0;slot<drawSubmeshes.Length;slot++)
                {
                    var all=baked.GetTriangles(slot); var subset=new List<int>();
                    for(int t=0;t<all.Length;t+=3) if(Math.Max(evidence[all[t]],Math.Max(evidence[all[t+1]],evidence[all[t+2]]))>=threshold)
                    { selected.Add(all[t]); selected.Add(all[t+1]); selected.Add(all[t+2]); subset.Add(all[t]); subset.Add(all[t+1]); subset.Add(all[t+2]); }
                    drawSubmeshes[slot]=subset.ToArray();
                }
                if(selected.Count<12) throw new ArgumentException("Insufficient source contact triangles.");
                drawTriangles=selected.ToArray(); UpdateWorld();
                var used=new bool[world.Length];foreach(int i in drawTriangles)used[i]=true;var unique=new List<int>();for(int i=0;i<used.Length;i++)if(used[i])unique.Add(i);selectedVertices=unique.ToArray();
                // Derive seam correspondence from immutable mesh coordinates,
                // not the current floating-point pose far from the scene origin.
                var reference=new Vec3[world.Length]; var rest=original.isReadable?original.vertices:vertices.ToArray(); for(int i=0;i<reference.Length;i++) reference[i]=AnimationAdapter.ToCore(rest[i]);
                ReferenceVertices=reference;
                var groups=BuildDeformationGroups(original); welded=new WeldedContactMesh(reference,drawTriangles,1e-6,true,groups);
                try { welded.Update(world,PoseTolerance()); }
                catch(InvalidOperationException) { UseUnweldedSurface(); }
                RebuildSurface();
                capturedFrame=Time.frameCount;
            }
            catch { Object.Destroy(baked); throw; }
        }
        void UpdateWorld() { var transform=AnimationAdapter.BakedToWorld(Renderer); for(int i=0;i<world.Length;i++) world[i]=AnimationAdapter.ToCore(transform.MultiplyPoint3x4(vertices[i])); }
        double PoseTolerance() { double scale=1; foreach(var p in world) scale=Math.Max(scale,Math.Max(Math.Abs(p.X),Math.Max(Math.Abs(p.Y),Math.Abs(p.Z)))); return scale*4*1.1920928955078125e-7; }
        int[] BuildDeformationGroups(Mesh mesh)
        {
            // Coincident rest positions are insufficient: different skin weights
            // or positional blend-shape deltas may legitimately separate later.
            var counts=mesh.GetBonesPerVertex(); var weights=mesh.GetAllBoneWeights(); var keys=new StringBuilder[world.Length]; int cursor=0;
            if(counts.Length!=keys.Length) throw new ArgumentException("Source skin weights do not match the mesh.");
            for(int i=0;i<keys.Length;i++)
            {
                var key=keys[i]=new StringBuilder();
                for(int j=0;j<counts[i];j++) { var weight=weights[cursor++]; key.Append(weight.boneIndex).Append(':').Append(weight.weight.ToString("R",CultureInfo.InvariantCulture)).Append(';'); }
            }
            if(mesh.blendShapeCount>0)
            {
                var deltas=new Vector3[keys.Length]; var deltaNormals=new Vector3[keys.Length]; var deltaTangents=new Vector3[keys.Length];
                for(int shape=0;shape<mesh.blendShapeCount;shape++) for(int frame=0;frame<mesh.GetBlendShapeFrameCount(shape);frame++)
                {
                    mesh.GetBlendShapeFrameVertices(shape,frame,deltas,deltaNormals,deltaTangents);
                    for(int i=0;i<keys.Length;i++) keys[i].Append('|').Append(deltas[i].x.ToString("R",CultureInfo.InvariantCulture)).Append(',').Append(deltas[i].y.ToString("R",CultureInfo.InvariantCulture)).Append(',').Append(deltas[i].z.ToString("R",CultureInfo.InvariantCulture));
                }
            }
            var ids=new Dictionary<string,int>(StringComparer.Ordinal); var result=new int[keys.Length];
            for(int i=0;i<keys.Length;i++) { string key=keys[i].ToString(); int id; if(!ids.TryGetValue(key,out id)) { id=ids.Count; ids.Add(key,id); } result[i]=id; }
            DeformationGroups=ids.Count; return result;
        }
        void UseUnweldedSurface() { welded=new WeldedContactMesh(world,drawTriangles,1e-6,false); UsesUnweldedFallback=true; }
        void RebuildSurface()
        {
            seamProxy=null; proxySurface=null;
            try { originalSurface=new TriangleSurface(welded.Positions,welded.Triangles,SurfaceKind.ClosedVolume); originalKind=SurfaceKind.ClosedVolume; originalSurface.RobustSign=false; }
            catch(ArgumentException) {
                originalSurface=new TriangleSurface(welded.Positions,welded.Triangles,SurfaceKind.OrientedShell); originalKind=SurfaceKind.OrientedShell;
                try { seamProxy=new AnimatedSeamContactProxy(welded.RestPositions,welded.Triangles); seamProxy.Update(welded.Positions,MaximumSeamGapRatio); proxySurface=new TriangleSurface(seamProxy.Positions,seamProxy.Triangles,SurfaceKind.ClosedVolume); proxySurface.RobustSign=false; }
                catch(ArgumentException) {seamProxy=null;proxySurface=null;}
            }
            SelectSurface(false);
        }
        void SelectSurface(bool refit)
        {
            UsesSeamProxy=EnableSeamProxy&&seamProxy!=null&&seamProxy.Update(welded.Positions,MaximumSeamGapRatio);
            surface=UsesSeamProxy?proxySurface:originalSurface; Kind=UsesSeamProxy?SurfaceKind.ClosedVolume:originalKind;
            if(refit) surface.Refit(UsesSeamProxy?seamProxy.Positions:welded.Positions);
        }
        public void Capture(bool force=false,bool deferRefit=false)
        {
            if(!Alive) throw new InvalidOperationException("Contact renderer was replaced.");
            if(!force&&capturedFrame==Time.frameCount) return;
            if(motion!=null) motion.RestoreBaseline();
            long start=Stopwatch.GetTimestamp();
            Renderer.BakeMesh(baked); baked.GetVertices(vertices); if(vertices.Count!=world.Length) throw new InvalidOperationException("Source topology changed.");
            UpdateWorld();
            long refit=Stopwatch.GetTimestamp();if(!deferRefit)RefitDeformation(null);long end=Stopwatch.GetTimestamp();capturedFrame=Time.frameCount;
            CaptureMilliseconds=(refit-start)*1000.0/Stopwatch.Frequency; RefitMilliseconds=(end-refit)*1000.0/Stopwatch.Frequency;
        }
        public void RefitDeformation(ISourceDeformation deformation)
        {
            long start=Stopwatch.GetTimestamp();var positions=world;
            if(deformation!=null){var indexed=deformation as IIndexedSourceDeformation;var rod=deformation as SourceRodGuide;if(indexed!=null)indexed.MapVertices(world,guided);else if(rod!=null)rod.MapVertices(world,guided);else for(int i=0;i<world.Length;i++)guided[i]=deformation.Map(world[i]);positions=guided;}
            RefitContactPositions(positions);
            RefitMilliseconds=(Stopwatch.GetTimestamp()-start)*1000.0/Stopwatch.Frequency;
        }
        public void RefitContactPositions(Vec3[] positions)
        {
            if(positions==null||positions.Length!=world.Length)throw new ArgumentException("Source contact topology changed");
            long start=Stopwatch.GetTimestamp();bool rebuild=false;
            try { welded.Update(positions,PoseTolerance()); }
            catch(InvalidOperationException) { UseUnweldedSurface(); rebuild=true; }
            if(rebuild) { welded.Update(positions,PoseTolerance());RebuildSurface(); } else SelectSurface(true);
            RefitMilliseconds=(Stopwatch.GetTimestamp()-start)*1000.0/Stopwatch.Frequency;
        }
        public bool TrySample(Vec3 point,double distance,out SurfaceSample sample) { return surface.TrySample(point,distance,out sample); }
        public SourceSectionRig CreateSectionRig(Transform root,Transform tip)
        {
            int first=Array.IndexOf(Renderer.bones,root),last=Array.IndexOf(Renderer.bones,tip);
            if(first<0||last<0)throw new ArgumentException("Source section rig endpoints are not in its skin palette");
            var bind=original.bindposes;Vec3 referenceRoot=AnimationAdapter.ToCore(bind[first].inverse.MultiplyPoint3x4(Vector3.zero)),referenceTip=AnimationAdapter.ToCore(bind[last].inverse.MultiplyPoint3x4(Vector3.zero));
            var rest=original.vertices;var points=new Vec3[rest.Length];for(int i=0;i<points.Length;i++)points[i]=AnimationAdapter.ToCore(rest[i]);
            return new SourceSectionRig(points,drawTriangles,referenceRoot,referenceTip-referenceRoot);
        }
        public Vec3 ReferenceAxis(Transform root,Transform tip)
        {
            int a=Array.IndexOf(Renderer.bones,root),b=Array.IndexOf(Renderer.bones,tip);if(a<0||b<0)throw new ArgumentException("Source endpoints are missing");var bind=original.bindposes;return AnimationAdapter.ToCore(bind[b].inverse.MultiplyPoint3x4(Vector3.zero)-bind[a].inverse.MultiplyPoint3x4(Vector3.zero));
        }
        public Vec3 LeadingPoint(Vec3 root,Vec3 boneTip) { return SourceSupport.LeadingPoint(world,root,boneTip,selectedVertices); }
        public bool TrySampleHinted(Vec3 point,double distance,ref int hint,out SurfaceSample sample) {return surface.TrySampleHinted(point,distance,ref hint,out sample);}
        public void ApplyCorrection(SourceContactBody body,double blend=1)
        {
            if(motion==null) motion=new SourceMotionBinding(Renderer,drawTriangles);
            motion.Apply(body,world,blend); displayFrame=-1;
        }
        public void RestoreCorrection() { if(motion!=null) motion.RestoreBaseline(); }
        public void ReleaseCorrection() { if(motion!=null) { motion.Dispose(); motion=null; displayFrame=-1; } }
        public Mesh XRaySnapshot()
        {
            Capture();
            if(display==null) { display=Object.Instantiate(baked); display.name="BPN source working region"; display.subMeshCount=drawSubmeshes.Length; for(int i=0;i<drawSubmeshes.Length;i++) display.SetTriangles(drawSubmeshes[i],i,false); display.MarkDynamic(); }
            if(displayFrame!=capturedFrame) {
                if(motion!=null) { if(displayPose==null) displayPose=new Mesh(); Renderer.BakeMesh(displayPose); displayPose.GetVertices(vertices); displayPose.GetNormals(normals); }
                else baked.GetNormals(normals);
                display.SetVertices(vertices); display.SetNormals(normals); display.bounds=motion==null?baked.bounds:displayPose.bounds; displayFrame=capturedFrame;
            }
            return display;
        }
        public void Dispose() { ReleaseCorrection(); if(baked!=null) Object.Destroy(baked); if(display!=null) Object.Destroy(display); if(displayPose!=null) Object.Destroy(displayPose); }
        static bool HasMaterial(SkinnedMeshRenderer renderer)
        { foreach(var material in renderer.sharedMaterials) if(material!=null) return true; return false; }
        public static bool HasVisibleCandidate(Component character,IList<Transform> chain,SkinnedMeshRenderer excluded)
        {
            foreach(var r in character.GetComponentsInChildren<SkinnedMeshRenderer>(true))
            { if(r==excluded||!r.enabled||!r.gameObject.activeInHierarchy||r.sharedMesh==null||!HasMaterial(r)) continue; foreach(var bone in r.bones) foreach(var point in chain) if(bone!=null&&point!=null&&(bone==point||bone.IsChildOf(point))) return true; }
            return false;
        }
        public static ContactDetection FromChain(Component character,IList<Transform> chain)
        {
            ContactDetection best=null; double bestScore=double.NegativeInfinity; bool ambiguous=false;
            try
            {
                foreach(var renderer in character.GetComponentsInChildren<SkinnedMeshRenderer>(true))
                {
                    if(renderer.sharedMesh==null||!HasMaterial(renderer)) continue; var bones=renderer.bones; var included=new bool[bones.Length]; bool found=false;
                    for(int b=0;b<bones.Length;b++) foreach(var point in chain) if(bones[b]!=null&&point!=null&&(bones[b]==point||bones[b].IsChildOf(point))) { included[b]=true; found=true; break; }
                    if(!found) continue;
                    var counts=renderer.sharedMesh.GetBonesPerVertex(); var weights=renderer.sharedMesh.GetAllBoneWeights(); if(counts.Length!=renderer.sharedMesh.vertexCount) continue;
                    var evidence=new double[counts.Length]; int cursor=0,positive=0;
                    for(int i=0;i<counts.Length;i++)
                    { for(int j=0;j<counts[i];j++) { var w=weights[cursor++]; if(w.boneIndex>=0&&w.boneIndex<included.Length&&included[w.boneIndex]) evidence[i]+=w.weight; } if(evidence[i]>=0.2) positive++; }
                    if(positive<4) continue;
                    var candidate=new ContactDetection(renderer,evidence);
                    // Prefer actual rendered coverage, never a closed low-detail
                    // accessory merely because UV seams make the main mesh open.
                    double score=(double)positive/evidence.Length+0.1*Math.Log(positive+1)+(renderer.enabled&&renderer.gameObject.activeInHierarchy?100:0);
                    if(score>bestScore+0.01) { if(best!=null) best.Dispose(); best=candidate; bestScore=score; ambiguous=false; }
                    else { candidate.Dispose(); if(Math.Abs(score-bestScore)<=0.01) ambiguous=true; }
                }
                if(ambiguous) throw new ArgumentException("Multiple source meshes have equivalent topology/weight evidence; a source profile is required.");
                return best;
            }
            catch { if(best!=null) best.Dispose(); throw; }
        }
    }
}
