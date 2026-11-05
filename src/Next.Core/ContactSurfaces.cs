using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    public interface IContactSurface
    {
        bool TrySample(Vec3 point,double searchDistance,out SurfaceSample sample);
    }
    public interface IHintedContactSurface : IContactSurface
    { bool TrySampleHinted(Vec3 point,double distance,ref int triangleHint,out SurfaceSample sample); }
    public interface IHintedCoupledSurface : ICoupledContactSurface
    { bool TryContactHinted(Vec3 point,double distance,ref int triangleHint,out SurfaceSample sample,out SourceContactBody source); }
    // A closed-volume surface cannot penetrate a point outside its AABB. Open
    // shells do not make that promise and retain the full signed-distance query.
    public interface IContactBounds { bool HasVolumeBounds { get; } Bounds3 ContactBounds { get; } }
    public sealed class LocalContactSurface : IHintedCoupledSurface,IContactBounds
    {
        public IContactSurface Surface;
        public Vec3 Center;
        public double Radius=double.PositiveInfinity;
        public bool Enabled=true;
        public double FullStrengthRadiusFraction;
        public bool HasVolumeBounds { get { var b=Surface as IContactBounds; return b!=null&&b.HasVolumeBounds; } }
        public Bounds3 ContactBounds { get { var b=Surface as IContactBounds;return b==null?Bounds3.Unbounded:b.ContactBounds; } }
        public bool TrySample(Vec3 point,double searchDistance,out SurfaceSample sample)
        {
            sample=new SurfaceSample();
            if(!Enabled||(point-Center).LengthSquared>Radius*Radius||!Surface.TrySample(point,searchDistance,out sample)) return false;
            if(!double.IsPositiveInfinity(Radius)&&sample.SignedDistance<0)
            {
                double q=(point-Center).Length/Radius;
                if(q>FullStrengthRadiusFraction)
                { double edge=(q-FullStrengthRadiusFraction)/Math.Max(1e-9,1-FullStrengthRadiusFraction),t=Math.Max(0,1-edge); sample.SignedDistance*=t*t*t*t*(1+4*edge); }
            }
            return true;
        }
        public bool TryContact(Vec3 point,double reach,out SurfaceSample sample,out SourceContactBody source)
        { source=Surface as SourceContactBody; return TrySample(point,reach,out sample); }
        public bool TryContactHinted(Vec3 point,double reach,ref int hint,out SurfaceSample sample,out SourceContactBody source)
        {
            source=Surface as SourceContactBody; sample=new SurfaceSample();
            if(!Enabled||(point-Center).LengthSquared>Radius*Radius) return false;
            var h=Surface as IHintedContactSurface;
            bool found=h==null?Surface.TrySample(point,reach,out sample):h.TrySampleHinted(point,reach,ref hint,out sample);
            if(found&&!double.IsPositiveInfinity(Radius)&&sample.SignedDistance<0) { double q=(point-Center).Length/Radius; if(q>FullStrengthRadiusFraction) {double edge=(q-FullStrengthRadiusFraction)/Math.Max(1e-9,1-FullStrengthRadiusFraction),t=Math.Max(0,1-edge);sample.SignedDistance*=t*t*t*t*(1+4*edge);} }
            return found;
        }
    }
    public sealed class CompositeSurface : IHintedCoupledSurface,IContactBounds
    {
        public readonly List<LocalContactSurface> Surfaces=new List<LocalContactSurface>();
        public bool HasVolumeBounds { get { foreach(var s in Surfaces) if(s.Enabled&&!s.HasVolumeBounds) return false; return true; } }
        public Bounds3 ContactBounds { get { var bounds=Bounds3.Empty; foreach(var s in Surfaces) if(s.Enabled) bounds.Include(s.ContactBounds); return bounds; } }
        public bool TrySample(Vec3 point,double searchDistance,out SurfaceSample sample)
        {
            sample=new SurfaceSample(); bool found=false;
            for(int i=0;i<Surfaces.Count;i++)
            {
                SurfaceSample candidate;
                if(Surfaces[i].TrySample(point,searchDistance,out candidate)&&(!found||candidate.SignedDistance<sample.SignedDistance))
                { sample=candidate; found=true; }
            }
            return found;
        }
        public bool TryContact(Vec3 point,double reach,out SurfaceSample sample,out SourceContactBody source)
        {
            sample=new SurfaceSample(); source=null; bool found=false;
            foreach(var surface in Surfaces) {
                SurfaceSample candidate; SourceContactBody body;
                if(surface.TryContact(point,reach,out candidate,out body)&&(!found||candidate.SignedDistance<sample.SignedDistance)) { sample=candidate; source=body; found=true; }
            }
            return found;
        }
        public bool TryContactHinted(Vec3 point,double reach,ref int hint,out SurfaceSample sample,out SourceContactBody source)
        {
            sample=new SurfaceSample(); source=null; bool found=false; int bestHint=hint;
            foreach(var surface in Surfaces) { SurfaceSample candidate; SourceContactBody body; int local=hint;
                if(surface.TryContactHinted(point,reach,ref local,out candidate,out body)&&(!found||candidate.SignedDistance<sample.SignedDistance)) {sample=candidate;source=body;bestHint=local;found=true;}
            }
            hint=bestHint; return found;
        }
    }
    // A contact mesh obtained only by merging coincident vertices of real geometry.
    // No missing triangles or end caps are synthesized.
    public sealed class WeldedContactMesh
    {
        readonly int[] remap,counts;
        readonly int[] usedVertices;
        readonly double tolerance;
        readonly Vec3[] first;
        public readonly Vec3[] Positions;
        public readonly Vec3[] RestPositions;
        public readonly int[] Triangles;
        public int MappedVertex(int originalVertex) { return remap[originalVertex]; }
        public WeldedContactMesh(Vec3[] positions,int[] triangles,double relativeTolerance,bool weld=true,int[] deformationGroups=null)
        {
            if(positions.Length==0||relativeTolerance<=0) throw new ArgumentException("Invalid weld input.");
            if(deformationGroups!=null&&deformationGroups.Length!=positions.Length) throw new ArgumentException("Invalid deformation groups.");
            Bounds3 bounds=Bounds3.Empty; foreach(var p in positions) bounds.Include(p);
            tolerance=Math.Max((bounds.Max-bounds.Min).Length*relativeTolerance,1e-10);
            remap=new int[positions.Length]; var representatives=new List<Vec3>(); var groups=new List<int>();
            for(int i=0;i<remap.Length;i++) remap[i]=-1;
            var used=new bool[positions.Length]; foreach(int index in triangles) used[index]=true;
            var usedList=new List<int>(); for(int i=0;i<used.Length;i++) if(used[i]) usedList.Add(i); usedVertices=usedList.ToArray();
            var cells=new Dictionary<string,List<int>>();
            foreach(int i in usedVertices)
            {
                Vec3 p=positions[i]; long x=(long)Math.Floor(p.X/tolerance),y=(long)Math.Floor(p.Y/tolerance),z=(long)Math.Floor(p.Z/tolerance);
                int match=-1;
                for(int a=-1;weld&&a<=1&&match<0;a++) for(int b=-1;b<=1&&match<0;b++) for(int c=-1;c<=1&&match<0;c++)
                {
                    List<int> bucket;
                    if(!cells.TryGetValue(Key(x+a,y+b,z+c),out bucket)) continue;
                    foreach(int index in bucket) if((deformationGroups==null||groups[index]==deformationGroups[i])&&(representatives[index]-p).LengthSquared<=tolerance*tolerance) { match=index; break; }
                }
                if(match<0)
                {
                    match=representatives.Count; representatives.Add(p); groups.Add(deformationGroups==null?0:deformationGroups[i]); string key=Key(x,y,z); List<int> bucket;
                    if(!cells.TryGetValue(key,out bucket)) { bucket=new List<int>(); cells.Add(key,bucket); } bucket.Add(match);
                }
                remap[i]=match;
            }
            RestPositions=representatives.ToArray(); Positions=new Vec3[representatives.Count]; first=new Vec3[Positions.Length]; counts=new int[Positions.Length];
            var indices=new List<int>();
            for(int i=0;i<triangles.Length;i+=3)
            {
                int a=remap[triangles[i]],b=remap[triangles[i+1]],c=remap[triangles[i+2]];
                if(a==b||a==c||b==c) continue; indices.Add(a); indices.Add(b); indices.Add(c);
            }
            Triangles=indices.ToArray(); Update(positions);
        }
        static string Key(long x,long y,long z) { return x+":"+y+":"+z; }
        public void Update(Vec3[] positions,double numericalTolerance=0)
        {
            if(positions.Length!=remap.Length) throw new ArgumentException("Contact topology changed.");
            if(!Vec3.Finite(numericalTolerance)||numericalTolerance<0) throw new ArgumentException("Invalid pose precision tolerance.");
            Array.Clear(Positions,0,Positions.Length); Array.Clear(counts,0,counts.Length);
            foreach(int i in usedVertices)
            {
                int index=remap[i];
                if(counts[index]==0) first[index]=positions[i];
                // A seam that separates under animation is no longer a valid welded surface.
                else if((positions[i]-first[index]).Length>Math.Max(tolerance*20,numericalTolerance)) throw new InvalidOperationException("Welded seam separated under animation; select an unwelded shell profile.");
                Positions[index]+=positions[i]; counts[index]++;
            }
            for(int i=0;i<Positions.Length;i++) Positions[i]/=counts[i];
        }
    }
}
