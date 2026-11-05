using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    // Separate unilateral contact against actual cervical triangles. The guide
    // has no closed end. No opening, cap or anatomical passage is fabricated.
    public sealed class CervicalSurfaceContact
    {
        readonly TriangleSurface surface;
        readonly int[] receiverFaces;readonly int[] receiverSamples;Vec3[] receiverWorld;
        bool sourceBound;SourceSectionRig sourceSections;public double SourceCoordinate {get;private set;}
        public int[] OpeningIndices;
        Vec3 contactSum;double contactWeight,coordinateSum;
        void Accumulate(double depth,Vec3 normal,double coordinate)
        {if(depth<=0)return;double weight=depth*depth;contactSum+=normal*weight;contactWeight+=weight;coordinateSum+=coordinate*weight;}
        public bool Available {get{return surface!=null;}}
        public int Queries {get;private set;}
        public bool UseLocalSurfaceContact=true;
        public CervicalSurfaceContact(Vec3[] reference,int[] triangles,int[] triangleRegions)
        {
            if(triangleRegions==null||triangleRegions.Length!=triangles.Length/3)return;
            var faces=new List<int>();for(int f=0;f<triangleRegions.Length;f++)if(triangleRegions[f]==3){faces.Add(triangles[f*3]);faces.Add(triangles[f*3+1]);faces.Add(triangles[f*3+2]);}
            receiverFaces=faces.ToArray();receiverSamples=new int[Math.Min(32,faces.Count/3)];for(int i=0;i<receiverSamples.Length;i++)receiverSamples[i]=i*(faces.Count/3)/receiverSamples.Length;
            if(faces.Count>0)surface=new TriangleSurface(reference,receiverFaces,SurfaceKind.OrientedShell);
        }
        public void BindSource(Vec3[] reference,int[] faces,SourceSectionRig sections)
        {
            if(reference==null||faces==null||sections==null||sections.VertexCount!=reference.Length)throw new ArgumentException("Invalid source contact binding");sourceBound=true;sourceSections=sections;
        }
        public void Refit(Vec3[] positions){Queries=0;receiverWorld=positions;if(surface!=null)surface.Refit(positions);}
        public double PenetrationAlongRay(Vec3 point,Vec3 displacement,Vec3 entry,Vec3 canalDirection,double margin,double range,out Vec3 outward)
        {
            outward=new Vec3();if(surface==null)return 0;Queries++;
            Vec3 direction=canalDirection.Normalized,p=point-displacement;
            double reach=Math.Max(range,Math.Max(0,Vec3.Dot(p-entry,direction))+margin);
            SurfaceSample sample;if(!surface.TryRaycast(p-direction*reach,p+direction*margin,out sample))return 0;
            Vec3 n=sample.Normal.Normalized;if(Vec3.Dot(n,direction)>0)n=-n;
            double separation=Vec3.Dot(p-sample.Point,n);if(separation>=margin)return 0;
            outward=n;return margin-separation;
        }
        public double Contact(AdaptiveMeshLandmarks source,Vec3 displacement,Vec3 entry,Vec3 canalDirection,double margin,double range,out Vec3 outward)
        {
            // The axial tip can pass through a real small opening while the
            // surrounding head still hits its actual triangles. Test cached
            // material landmarks, not a fabricated plane or a closed cap.
            Vec3 normal;double depth=PointContact(source.MaterialTip,displacement,entry,canalDirection,margin,range,out outward);
            SourceCoordinate=sourceSections==null?1:sourceSections.MaterialCoordinate(source.MaterialTipVertex);
            contactSum=new Vec3();contactWeight=coordinateSum=0;Accumulate(depth,outward,SourceCoordinate);
            var sections=source.Sections;
            for(int i=Math.Max(0,sections.Length-3);i<sections.Length;i++)for(int k=0;k<sections[i].Radial.Length;k++)
            {
                var point=sections[i].Radial[k];
                double candidate=PointContact(point,displacement,entry,canalDirection,margin,range,out normal);
                Accumulate(candidate,normal,Coordinate(sections[i].Bindings[sections[i].RadialBindings[k]]));
                if(candidate>depth){depth=candidate;outward=normal;SourceCoordinate=Coordinate(sections[i].Bindings[sections[i].RadialBindings[k]]);}
            }
            if(UseLocalSurfaceContact&&sourceBound&&receiverWorld!=null)
            {
                foreach(int triangle in receiverSamples)
                {
                    int t=triangle*3;Vec3 point=(receiverWorld[receiverFaces[t]]+receiverWorld[receiverFaces[t+1]]+receiverWorld[receiverFaces[t+2]])/3+displacement;
                    Queries++;int span;double blend;double candidate=source.Envelope.Penetration(point,margin,true,out normal,out span,out blend);if(candidate>0)Accumulate(candidate,-normal,Coordinate(sections[span].Bindings[0])*(1-blend)+Coordinate(sections[span+1].Bindings[0])*blend);if(candidate>depth){depth=candidate;outward=-normal;SourceCoordinate=Coordinate(sections[span].Bindings[0])*(1-blend)+Coordinate(sections[span+1].Bindings[0])*blend;}
                }
                // Only REAL os rim vertices, never its virtual centre. Sampling
                // the finite whole source here also catches a crossed head when
                // the opening already overlaps a shaft section at high speed.
                if(OpeningIndices!=null)for(int k=1;k<OpeningIndices.Length;k++)if(OpeningIndices[k]>=0)
                {
                    Queries++;int span;double blend;Vec3 point=receiverWorld[OpeningIndices[k]]+displacement;
                    double candidate=source.Envelope.Penetration(point,margin,false,out normal,out span,out blend);
                    if(candidate>0)Accumulate(candidate,-normal,Coordinate(sections[span].Bindings[0])*(1-blend)+Coordinate(sections[span+1].Bindings[0])*blend);
                    if(candidate>depth){depth=candidate;outward=-normal;SourceCoordinate=Coordinate(sections[span].Bindings[0])*(1-blend)+Coordinate(sections[span+1].Bindings[0])*blend;}
                }
            }
            if(contactWeight>0){outward=contactSum.LengthSquared>1e-28?contactSum.Normalized:-canalDirection.Normalized;SourceCoordinate=coordinateSum/contactWeight;}
            return depth;
        }
        double Coordinate(SurfaceLandmark p){return sourceSections==null?1:sourceSections.MaterialCoordinate(p.A)*(1-p.Blend)+sourceSections.MaterialCoordinate(p.B)*p.Blend;}
        public double PenetrationFromEntry(Vec3 point,Vec3 displacement,Vec3 entry,double margin,out Vec3 outward)
        {
            outward=new Vec3();if(surface==null)return 0;Queries++;
            // A tilted source can leave the terminal tangent's parallel rays
            // completely outside the cervix. Its actual entry-to-point sweep
            // still crosses the real finite surface. Transform both endpoints
            // into the unshifted organ's frame; no plane or cap is fabricated.
            Vec3 direction=(point-entry).Normalized;if(direction.LengthSquared<.5)return 0;
            Vec3 p=point-displacement;SurfaceSample sample;
            if(!surface.TryRaycast(entry-displacement,p+direction*Math.Max(0,margin),out sample))return 0;
            Vec3 n=sample.Normal.Normalized;if(Vec3.Dot(n,direction)>0)n=-n;
            double separation=Vec3.Dot(p-sample.Point,n);if(separation>=margin)return 0;
            outward=n;return margin-separation;
        }
        double PointContact(Vec3 point,Vec3 displacement,Vec3 entry,Vec3 canalDirection,double margin,double range,out Vec3 outward)
        {
            if(UseLocalSurfaceContact)
            {
                outward=new Vec3();if(surface==null)return 0;Queries++;SurfaceSample sample;
                if(!surface.TrySample(point-displacement,range,out sample)||sample.SignedDistance>=margin)return 0;
                outward=sample.Normal.Normalized;return margin-sample.SignedDistance;
            }
            double swept=PenetrationFromEntry(point,displacement,entry,margin,out outward);
            if(swept>0)return swept;
            return PenetrationAlongRay(point,displacement,entry,canalDirection,margin,range,out outward);
        }
        public double Penetration(Vec3 point,Vec3 displacement,Vec3 canalDirection,double margin,double range,out Vec3 outward)
        {
            outward=new Vec3();if(surface==null)return 0;Queries++;SurfaceSample sample;
            if(!surface.TrySample(point-displacement,range,out sample))return 0;
            Vec3 n=sample.Normal.Normalized;double facing=Vec3.Dot(n,canalDirection);
            if(Math.Abs(facing)<.25)return 0;if(facing>0)n=-n;
            double separation=Vec3.Dot(point-displacement-sample.Point,n);if(separation>=margin)return 0;
            outward=n;return margin-separation;
        }
    }
}
