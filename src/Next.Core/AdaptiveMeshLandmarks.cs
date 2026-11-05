using System;
using System.Collections.Generic;

namespace BetterPenetrationNext.Core
{
    public struct SurfaceLandmark
    {
        public int A,B;public double Blend;
        public Vec3 Evaluate(Vec3[] vertices){return vertices[A]*(1-Blend)+vertices[B]*Blend;}
    }
    public sealed class MeshLandmarkSection
    {
        public readonly SurfaceLandmark[] Bindings;
        public readonly int[] Segments;
        public readonly Vec3[] Surface,Radial;
        public readonly int[] RadialBindings;
        internal readonly double[] ProjectedX,ProjectedY;
        public Vec3 Centre,Tangent;
        public readonly double Coordinate;
        public MeshLandmarkSection(SurfaceLandmark[] bindings,double coordinate,int radial,int[] segments=null){Bindings=bindings;Segments=segments??new int[0];Coordinate=coordinate;Surface=new Vec3[bindings.Length];Radial=new Vec3[radial];RadialBindings=new int[radial];ProjectedX=new double[bindings.Length];ProjectedY=new double[bindings.Length];}
    }
    // Material cross sections are cut ONCE on the real triangles. Per-frame
    // evaluation follows their cached vertex/edge skin and morph identities.
    // Open rims remain open; no artificial source cap or vertex weld is added.
    public sealed class AdaptiveMeshLandmarks
    {
        public readonly MeshLandmarkSection[] Sections;
        public readonly Vec3[] Extrema=new Vec3[6];
        public Vec3 Tip,Base;public int TipVertex,BaseVertex;
        public Vec3 MaterialTip,MaterialBase;public readonly int MaterialTipVertex,MaterialBaseVertex;
        public Vec3[] CurrentVertices {get;private set;}public long PoseVersion {get;private set;}
        public readonly SourceEnvelopeSurface Envelope;
        public long CachedPointEvaluations;public int ReferenceTriangleTests;
        readonly double[] dx,dy;readonly int[] selected;
        readonly int vertexCount;readonly Vec3 referenceAxis;
        public AdaptiveMeshLandmarks(Vec3[] reference,int[] triangles,Vec3 axis,int radial=16,int minimumSections=17,int maximumSections=33)
        {
            if(reference==null||triangles==null||reference.Length<3||triangles.Length<3||triangles.Length%3!=0||radial<4||radial>16||minimumSections<2||maximumSections<minimumSections||!axis.IsFinite)throw new ArgumentException("Invalid landmark mesh or sampling settings");
            vertexCount=reference.Length;
            var used=new bool[reference.Length];
            foreach(int i in triangles){if(i<0||i>=reference.Length)throw new ArgumentException("Landmark triangle index is outside the vertex buffer");if(!reference[i].IsFinite)throw new ArgumentException("Nonfinite landmark reference vertex");used[i]=true;}
            bool hasArea=false;for(int i=0;i<triangles.Length;i+=3){var n=Vec3.Cross(reference[triangles[i+1]]-reference[triangles[i]],reference[triangles[i+2]]-reference[triangles[i]]);if(!n.IsFinite)throw new ArgumentException("Reference geometry exceeds the numeric range");if(n.LengthSquared>0)hasArea=true;}if(!hasArea)throw new ArgumentException("Landmark mesh has no surface area");
            var list=new List<int>();for(int i=0;i<used.Length;i++)if(used[i])list.Add(i);selected=list.ToArray();
            axis=axis.Normalized;if(axis.LengthSquared<.5)axis=PrincipalAxis(reference,selected);referenceAxis=axis;
            double minimum=double.PositiveInfinity,maximum=double.NegativeInfinity;foreach(int i in selected){double u=Vec3.Dot(reference[i],axis);if(u<minimum){minimum=u;BaseVertex=i;}if(u>maximum){maximum=u;TipVertex=i;}}if(maximum-minimum<1e-9)throw new ArgumentException("Collapsed material axis");
            MaterialTipVertex=TipVertex;MaterialBaseVertex=BaseVertex;
            var sections=new List<MeshLandmarkSection>();for(int s=0;s<minimumSections;s++)sections.Add(Cut(reference,triangles,axis,minimum,maximum,(double)s/(minimumSections-1),radial));
            // Add stations where the supplied geometry changes width rapidly.
            for(int s=0;s<sections.Count-1&&sections.Count<maximumSections;s++)
            {
                double a=ReferenceRadius(sections[s],reference),b=ReferenceRadius(sections[s+1],reference);
                if(Math.Abs(a-b)>Math.Max(.0001,Math.Max(a,b)*.3)&&sections[s+1].Coordinate-sections[s].Coordinate>1.0/(minimumSections*4))sections.Insert(s+1,Cut(reference,triangles,axis,minimum,maximum,(sections[s].Coordinate+sections[s+1].Coordinate)*.5,radial));
            }
            Sections=sections.ToArray();dx=new double[radial];dy=new double[radial];for(int i=0;i<radial;i++){dx[i]=Math.Cos(i*2*Math.PI/radial);dy[i]=Math.Sin(i*2*Math.PI/radial);}Update(reference,new Vec3(1,0,0));Envelope=new SourceEnvelopeSurface(this);
        }
        static Vec3 PrincipalAxis(Vec3[] p,int[] indices)
        {
            Vec3 origin=p[indices[0]],c=new Vec3();foreach(int i in indices)c+=(p[i]-origin)/indices.Length;
            Vec3 diagonal=new Vec3();foreach(int i in indices){Vec3 d=p[i]-origin-c;diagonal+=new Vec3(d.X*d.X,d.Y*d.Y,d.Z*d.Z);}
            Vec3 x=diagonal.X>=diagonal.Y&&diagonal.X>=diagonal.Z?new Vec3(1,0,0):diagonal.Y>=diagonal.Z?new Vec3(0,1,0):new Vec3(0,0,1);
            for(int k=0;k<24;k++){Vec3 q=new Vec3();foreach(int i in indices){Vec3 d=p[i]-origin-c;q+=d*Vec3.Dot(d,x);}x=q.Normalized;}return x;
        }
        static double ReferenceRadius(MeshLandmarkSection section,Vec3[] vertices){Vec3 c=new Vec3();foreach(var b in section.Bindings)c+=b.Evaluate(vertices)/section.Bindings.Length;double r=0;foreach(var b in section.Bindings)r=Math.Max(r,(b.Evaluate(vertices)-c).Length);return r;}
        MeshLandmarkSection Cut(Vec3[] reference,int[] triangles,Vec3 axis,double min,double max,double coordinate,int radial)
        {
            double t=Math.Max(1e-6,Math.Min(1-1e-6,coordinate)),plane=min+(max-min)*t;var points=new List<SurfaceLandmark>();var seen=new Dictionary<long,int>();var segments=new List<int>();
            for(int f=0;f<triangles.Length;f+=3){ReferenceTriangleTests++;int first=-1,second=-1;for(int j=0;j<3;j++){int a=triangles[f+j],b=triangles[f+(j+1)%3];double x=Vec3.Dot(reference[a],axis),y=Vec3.Dot(reference[b],axis);if(!((x<=plane&&y>plane)||(y<=plane&&x>plane)))continue;long key=((long)Math.Min(a,b)<<32)|(uint)Math.Max(a,b);int index;if(!seen.TryGetValue(key,out index)){index=points.Count;seen.Add(key,index);points.Add(new SurfaceLandmark{A=a,B=b,Blend=(plane-x)/(y-x)});}if(first<0)first=index;else second=index;}if(first>=0&&second>=0){segments.Add(first);segments.Add(second);}}
            if(points.Count==0){int best=0;double distance=double.PositiveInfinity;foreach(int i in selected){double d=Math.Abs(Vec3.Dot(reference[i],axis)-plane);if(d<distance){distance=d;best=i;}}points.Add(new SurfaceLandmark{A=best,B=best});}
            return new MeshLandmarkSection(points.ToArray(),coordinate,radial,segments.ToArray());
        }
        static Vec3 Transverse(Vec3 normal,Vec3 orientation)
        {
            Vec3 x=orientation-normal*Vec3.Dot(orientation,normal);
            if(x.LengthSquared<=Math.Max(1,orientation.LengthSquared)*1e-20)
            {
                // The least parallel cardinal axis also works for vertical tangents.
                Vec3 seed=Math.Abs(normal.X)<=Math.Abs(normal.Y)&&Math.Abs(normal.X)<=Math.Abs(normal.Z)?new Vec3(1,0,0):Math.Abs(normal.Y)<=Math.Abs(normal.Z)?new Vec3(0,1,0):new Vec3(0,0,1);
                x=seed-normal*Vec3.Dot(seed,normal);
            }
            return x.Normalized;
        }
        public void Update(Vec3[] vertices,Vec3 orientation)
        {
            if(vertices==null||vertices.Length!=vertexCount||!orientation.IsFinite)throw new ArgumentException("Invalid landmark animation frame");
            foreach(int i in selected)if(!vertices[i].IsFinite)throw new ArgumentException("Nonfinite landmark animation vertex");
            CurrentVertices=vertices;PoseVersion++;
            CachedPointEvaluations=0;
            for(int s=0;s<Sections.Length;s++){var section=Sections[s];Vec3 origin=section.Bindings[0].Evaluate(vertices),c=new Vec3();for(int i=0;i<section.Surface.Length;i++){section.Surface[i]=section.Bindings[i].Evaluate(vertices);CachedPointEvaluations++;}double total=0;for(int i=0;i<section.Segments.Length;i+=2){Vec3 a=section.Surface[section.Segments[i]],b=section.Surface[section.Segments[i+1]];double length=(b-a).Length;c+=((a-origin)+(b-origin))*(length*.5);total+=length;}if(total>1e-20)c/=total;else for(int i=0;i<section.Surface.Length;i++)c+=(section.Surface[i]-origin)/section.Surface.Length;section.Centre=origin+c;}
            Vec3 span=Sections[Sections.Length-1].Centre-Sections[0].Centre,z=span.LengthSquared>1e-24?span.Normalized:referenceAxis;
            Vec3 xLocal=Transverse(z,orientation),yLocal=Vec3.Cross(z,xLocal),basis=Sections[0].Centre;
            int xMinIndex=selected[0],xMaxIndex=selected[0],yMinIndex=selected[0],yMaxIndex=selected[0],zMinIndex=BaseVertex,zMaxIndex=TipVertex;
            double xMin=Vec3.Dot(vertices[xMinIndex]-basis,xLocal),xMax=xMin,yMin=Vec3.Dot(vertices[yMinIndex]-basis,yLocal),yMax=yMin,zMin=Vec3.Dot(vertices[zMinIndex]-basis,z),zMax=Vec3.Dot(vertices[zMaxIndex]-basis,z);
            double tolerance=Math.Max(1,span.Length)*1e-10;
            foreach(int i in selected)
            {
                Vec3 delta=vertices[i]-basis;double px=Vec3.Dot(delta,xLocal),py=Vec3.Dot(delta,yLocal),pz=Vec3.Dot(delta,z);
                if(px<xMin-tolerance){xMin=px;xMinIndex=i;}if(px>xMax+tolerance){xMax=px;xMaxIndex=i;}
                if(py<yMin-tolerance){yMin=py;yMinIndex=i;}if(py>yMax+tolerance){yMax=py;yMaxIndex=i;}
                if(pz<zMin-tolerance){zMin=pz;zMinIndex=i;}if(pz>zMax+tolerance){zMax=pz;zMaxIndex=i;}
            }
            Extrema[0]=vertices[xMinIndex];Extrema[1]=vertices[xMaxIndex];Extrema[2]=vertices[yMinIndex];Extrema[3]=vertices[yMaxIndex];Extrema[4]=vertices[zMinIndex];Extrema[5]=vertices[zMaxIndex];BaseVertex=zMinIndex;TipVertex=zMaxIndex;
            Tip=vertices[TipVertex];Base=vertices[BaseVertex];
            // Extrema remain useful diagnostics. Mechanical endpoints retain
            // their material identities when a bent flank becomes farther away.
            MaterialTip=vertices[MaterialTipVertex];MaterialBase=vertices[MaterialBaseVertex];
            for(int s=0;s<Sections.Length;s++)
            {
                var section=Sections[s];Vec3 tangent=Sections[Math.Min(Sections.Length-1,s+1)].Centre-Sections[Math.Max(0,s-1)].Centre,n=tangent.LengthSquared>1e-24?tangent.Normalized:z;section.Tangent=n;Vec3 x=Transverse(n,orientation),y=Vec3.Cross(n,x);
                // Project each cached surface point once, then reuse its two
                // coordinates for every radial support query.
                for(int i=0;i<section.Surface.Length;i++){Vec3 delta=section.Surface[i]-section.Centre;section.ProjectedX[i]=Vec3.Dot(delta,x);section.ProjectedY[i]=Vec3.Dot(delta,y);}
                for(int k=0;k<section.Radial.Length;k++){double best=double.NegativeInfinity;int point=0;for(int i=0;i<section.Surface.Length;i++){double d=section.ProjectedX[i]*dx[k]+section.ProjectedY[i]*dy[k];if(d>best){best=d;point=i;}}section.Radial[k]=section.Surface[point];section.RadialBindings[k]=point;}
            }
        }
    }
}
