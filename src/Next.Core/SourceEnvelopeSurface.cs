using System;
namespace BetterPenetrationNext.Core
{
    // Connected material loft from real cached surface points. No end caps and
    // no original vertices/weights are welded. Refit buffers are reused.
    public sealed class SourceEnvelopeSurface
    {
        readonly AdaptiveMeshLandmarks owner;readonly Vec3[] points;readonly int sides,first;
        readonly TriangleSurface head,whole;Bounds3 headBounds,wholeBounds;long version;
        public SourceEnvelopeSurface(AdaptiveMeshLandmarks landmarks)
        {
            owner=landmarks;var sections=owner.Sections;sides=sections[0].Radial.Length;points=new Vec3[sections.Length*sides];
            first=0;while(first<sections.Length-1&&sections[first].Coordinate<.75)first++;
            Fill();var faces=new int[(sections.Length-1)*sides*6];int cursor=0;
            for(int s=0;s<sections.Length-1;s++)for(int k=0;k<sides;k++)
            {
                int a=s*sides+k,b=s*sides+(k+1)%sides,c=a+sides,d=b+sides;
                faces[cursor++]=a;faces[cursor++]=b;faces[cursor++]=c;faces[cursor++]=b;faces[cursor++]=d;faces[cursor++]=c;
            }
            whole=new TriangleSurface(points,faces,SurfaceKind.OrientedShell);int count=(sections.Length-1-first)*sides*6;
            if(count>0){var selected=new int[count];Array.Copy(faces,first*sides*6,selected,0,count);head=new TriangleSurface(points,selected,SurfaceKind.OrientedShell);}
            version=owner.PoseVersion;
        }
        void Fill()
        {
            wholeBounds=headBounds=Bounds3.Empty;
            for(int s=0;s<owner.Sections.Length;s++)for(int k=0;k<sides;k++){Vec3 p=owner.Sections[s].Radial[k];points[s*sides+k]=p;wholeBounds.Include(p);if(s>=first)headBounds.Include(p);}
        }
        void Update()
        {if(version==owner.PoseVersion)return;Fill();whole.Refit(points);if(head!=null)head.Refit(points);version=owner.PoseVersion;}
        public double Penetration(Vec3 point,double margin,bool headOnly,out Vec3 normal,out int span,out double blend)
        {
            Update();normal=new Vec3();span=-1;blend=0;var surface=headOnly?head:whole;if(surface==null)return 0;
            var bounds=headOnly?headBounds:wholeBounds;int start=headOnly?first:0;
            if(point.X<bounds.Min.X-margin||point.X>bounds.Max.X+margin||point.Y<bounds.Min.Y-margin||point.Y>bounds.Max.Y+margin||point.Z<bounds.Min.Z-margin||point.Z>bounds.Max.Z+margin||Vec3.Dot(point-owner.Sections[start].Centre,owner.Sections[start].Tangent)<-margin)return 0;
            SurfaceSample sample;if(!surface.TrySample(point,(bounds.Max-bounds.Min).Length+margin,out sample)||sample.SignedDistance>=margin)return 0;
            normal=sample.Normal.Normalized;span=start+sample.Triangle/(sides*2);span=Math.Max(0,Math.Min(owner.Sections.Length-2,span));
            Vec3 a=owner.Sections[span].Centre,e=owner.Sections[span+1].Centre-a;blend=e.LengthSquared>1e-20?Math.Max(0,Math.Min(1,Vec3.Dot(sample.Point-a,e)/e.LengthSquared)):0;
            return margin-sample.SignedDistance;
        }
        public double Penetration(Vec3 point,double margin,out Vec3 normal)
        {int span;double blend;return Penetration(point,margin,true,out normal,out span,out blend);}
    }
}
