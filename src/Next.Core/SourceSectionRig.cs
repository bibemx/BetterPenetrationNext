using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    // Indexed material coordinates preserve each independently skinned vertex.
    // Sections are sampled from the evaluated animation before correction.
    public interface IIndexedSourceDeformation : ISourceDeformation
    {
        Vec3 MapVertex(int index,Vec3 animated);
        void MapVertices(Vec3[] animated,Vec3[] output);
    }
    public sealed class SourceSectionRig : IIndexedSourceDeformation
    {
        sealed class Section { public int[] Vertices;public double[] Weights; }
        readonly Section[] sections;
        readonly double[] coordinates;
        readonly Vec3[] animatedCentres,offsets;
        readonly Vec3[] materialOffsets;
        readonly int[] triangles;
        readonly SourceSectionAreaConstraints areaConstraints;
        bool initialized,mappedOffsetsValid;
        AnatomicalPathConstraintSystem previousPath;
        double maximum;
        public double Parameter { get; private set; }=-1;
        public double GeometryLimitScale { get; private set; }=1;
        public double MaximumDisplacement { get { return maximum; } }
        public Bounds3 DisplacementBounds { get; private set; }=Bounds3.Empty;
        public int SectionCount { get { return sections.Length; } }
        public int VertexCount { get { return coordinates.Length; } }
        public int AreaConstraintChecks {get;private set;}
        public Vec3 AnimatedCentre(int index) { return animatedCentres[index]; }
        public Vec3 CorrectedCentre(int index) { return animatedCentres[index]+offsets[index]*GeometryLimitScale; }
        public double MaterialCoordinate(int index) { return coordinates[index]; }
        public SourceSectionRig(Vec3[] reference,int[] selectedTriangles,Vec3 referenceRoot,Vec3 referenceAxis,int intervals=24)
        {
            if(reference==null||selectedTriangles==null||intervals<2||intervals>128||!referenceRoot.IsFinite||referenceAxis.LengthSquared<1e-12)throw new ArgumentException("Invalid source section reference");
            Vec3 axis=referenceAxis.Normalized;double length=0;triangles=(int[])selectedTriangles.Clone();
            foreach(int vertex in triangles){if(vertex<0||vertex>=reference.Length)throw new ArgumentException("Source triangle index out of range");length=Math.Max(length,Vec3.Dot(reference[vertex]-referenceRoot,axis));}
            if(length<1e-8)throw new ArgumentException("Collapsed source material axis");
            coordinates=new double[reference.Length];materialOffsets=new Vec3[reference.Length];for(int i=0;i<coordinates.Length;i++)coordinates[i]=Math.Max(0,Math.Min(1,Vec3.Dot(reference[i]-referenceRoot,axis)/length));
            sections=new Section[intervals+1];animatedCentres=new Vec3[sections.Length];offsets=new Vec3[sections.Length];
            areaConstraints=new SourceSectionAreaConstraints(coordinates,triangles,sections.Length);
            for(int station=0;station<sections.Length;station++)
            {
                double plane=length*Math.Min(1-1e-6,(double)station/intervals);var coefficients=new Dictionary<int,double>();double total=0;
                for(int face=0;face<triangles.Length;face+=3)
                {
                    int a0=0,b0=0,a1=0,b1=0,count=0;double t0=0,t1=0;Vec3 p0=new Vec3(),p1=new Vec3();
                    for(int edge=0;edge<3;edge++)
                    {
                        int a=triangles[face+edge],b=triangles[face+(edge+1)%3];double sa=Vec3.Dot(reference[a]-referenceRoot,axis),sb=Vec3.Dot(reference[b]-referenceRoot,axis);
                        if(!((sa<=plane&&sb>plane)||(sb<=plane&&sa>plane)))continue;
                        double t=(plane-sa)/(sb-sa);Vec3 point=reference[a]*(1-t)+reference[b]*t;
                        if(count++==0){a0=a;b0=b;t0=t;p0=point;}else{a1=a;b1=b;t1=t;p1=point;}
                    }
                    if(count!=2)continue;double weight=(p1-p0).Length*.5;if(weight<1e-16)continue;
                    Add(coefficients,a0,weight*(1-t0));Add(coefficients,b0,weight*t0);Add(coefficients,a1,weight*(1-t1));Add(coefficients,b1,weight*t1);total+=2*weight;
                }
                if(total<1e-14)
                {
                    // An exact open root plane may lie outside the actual mesh.
                    // Nearest material slice is a sampling fallback, never a cap.
                    double nearest=double.PositiveInfinity;foreach(int v in triangles)nearest=Math.Min(nearest,Math.Abs(Vec3.Dot(reference[v]-referenceRoot,axis)-plane));
                    foreach(int v in triangles)if(Math.Abs(Vec3.Dot(reference[v]-referenceRoot,axis)-plane)<=nearest+length*1e-7&&!coefficients.ContainsKey(v)){coefficients.Add(v,1);total++;}
                }
                if(total<=0)throw new ArgumentException("No material samples for source section");
                var section=new Section{Vertices=new int[coefficients.Count],Weights=new double[coefficients.Count]};int index=0;
                foreach(var entry in coefficients){section.Vertices[index]=entry.Key;section.Weights[index++]=entry.Value/total;}sections[station]=section;
            }
        }
        static void Add(Dictionary<int,double> values,int index,double value){double previous;values.TryGetValue(index,out previous);values[index]=previous+value;}
        public void Reset(){initialized=false;mappedOffsetsValid=false;Parameter=-1;previousPath=null;DisplacementBounds=Bounds3.Empty;Array.Clear(offsets,0,offsets.Length);}
        public void CaptureAnimation(Vec3[] animated,Vec3 animatedRoot)
        {
            if(animated==null||animated.Length!=coordinates.Length||!animatedRoot.IsFinite)throw new ArgumentException("Invalid source animation pose");
            for(int i=0;i<sections.Length;i++){Vec3 origin=animated[sections[i].Vertices[0]],centre=new Vec3();var section=sections[i];for(int j=0;j<section.Vertices.Length;j++)centre+=(animated[section.Vertices[j]]-origin)*section.Weights[j];animatedCentres[i]=origin+centre;}
            animatedCentres[0]=animatedRoot;
        }
        public void Capture(AnatomicalPathConstraintSystem path,Vec3[] animated,Vec3 animatedRoot,double dt,double maximumDisplacement,double response=12)
        {
            if(path==null||animated==null||animated.Length!=coordinates.Length||!animatedRoot.IsFinite||!Vec3.Finite(dt)||dt<0||!Vec3.Finite(maximumDisplacement)||maximumDisplacement<0||!Vec3.Finite(response)||response<0)throw new ArgumentException("Invalid source section pose");
            if(!ReferenceEquals(previousPath,path)){Reset();previousPath=path;}maximum=maximumDisplacement;
            for(int i=0;i<sections.Length;i++){Vec3 centre=new Vec3();var section=sections[i];for(int j=0;j<section.Vertices.Length;j++)centre+=animated[section.Vertices[j]]*section.Weights[j];if(!centre.IsFinite)throw new ArgumentException("Nonfinite animated source section");animatedCentres[i]=centre;}
            animatedCentres[0]=animatedRoot;
            var end=path.Closest(animatedCentres[sections.Length-1],Parameter,.15);Parameter=end.Parameter;
            Vec3 entry=path[0];double outside=(entry-animatedRoot).Length,total=outside+path.DistanceAtParameter(Parameter),blend=1-Math.Exp(-response*dt);
            for(int i=0;i<offsets.Length;i++)
            {
                double arc=total*i/(offsets.Length-1);Vec3 target=arc<=outside&&outside>1e-12?animatedRoot+(entry-animatedRoot)*(arc/outside):path.AtDistance(arc-outside).Point;
                Vec3 delta=Vec3.ClampLength(target-animatedCentres[i],maximum);
                offsets[i]=Vec3.ClampLength((initialized?offsets[i]:new Vec3())+(delta-(initialized?offsets[i]:new Vec3()))*blend,maximum);
            }
            offsets[0]=new Vec3();initialized=true;mappedOffsetsValid=false;GeometryLimitScale=1;DisplacementBounds=Bounds3.Empty;
        }
        Vec3 RawOffset(int index){double u=coordinates[index]*(offsets.Length-1);int a=Math.Min(offsets.Length-2,(int)u);double t=u-a;return offsets[a]*(1-t)+offsets[a+1]*t;}
        Vec3 Offset(int index){return (mappedOffsetsValid?materialOffsets[index]:RawOffset(index))*GeometryLimitScale;}
        public Vec3 MapVertex(int index,Vec3 animated){if(index<0||index>=coordinates.Length)throw new ArgumentOutOfRangeException("index");return initialized?animated+Offset(index):animated;}
        public Vec3 Map(Vec3 point)
        {
            if(!initialized)return point;double best=double.PositiveInfinity;Vec3 result=point;
            for(int i=0;i<animatedCentres.Length-1;i++){Vec3 edge=animatedCentres[i+1]-animatedCentres[i];double t=edge.LengthSquared<1e-20?0:Math.Max(0,Math.Min(1,Vec3.Dot(point-animatedCentres[i],edge)/edge.LengthSquared));double distance=(point-animatedCentres[i]-edge*t).LengthSquared;if(distance<best){best=distance;result=point+(offsets[i]*(1-t)+offsets[i+1]*t)*GeometryLimitScale;}}
            return result;
        }
        bool Valid(Vec3[] animated,double scale)
        {
            for(int t=0;t<triangles.Length;t+=3){int a=triangles[t],b=triangles[t+1],c=triangles[t+2];Vec3 ab=animated[b]-animated[a],ac=animated[c]-animated[a],normal=Vec3.Cross(ab,ac);double area=normal.LengthSquared;if(area<1e-24)continue;Vec3 deformed=Vec3.Cross(ab+(materialOffsets[b]-materialOffsets[a])*scale,ac+(materialOffsets[c]-materialOffsets[a])*scale);if(Vec3.Dot(deformed,normal)<area*.01)return false;}return true;
        }
        public void MapVertices(Vec3[] animated,Vec3[] output)
        {
            if(animated.Length!=coordinates.Length||output.Length!=coordinates.Length)throw new ArgumentException("Source section vertex count differs");
            for(int i=0;i<materialOffsets.Length;i++)materialOffsets[i]=initialized?RawOffset(i):new Vec3();mappedOffsetsValid=true;
            GeometryLimitScale=1;AreaConstraintChecks=0;
            if(initialized){
                GeometryLimitScale=SourceDeformationLimiter.Limit(animated,materialOffsets,triangles);
                if(GeometryLimitScale<.999999){
                    areaConstraints.Project(animated,offsets,maximum);
                    AreaConstraintChecks=areaConstraints.LastChecks;
                    for(int i=0;i<materialOffsets.Length;i++)materialOffsets[i]=RawOffset(i);
                    GeometryLimitScale=SourceDeformationLimiter.Limit(animated,materialOffsets,triangles);
                }
            }
            var bounds=Bounds3.Empty;for(int i=0;i<animated.Length;i++){output[i]=MapVertex(i,animated[i]);bounds.Include(output[i]-animated[i]);}DisplacementBounds=bounds;
        }
    }
}
