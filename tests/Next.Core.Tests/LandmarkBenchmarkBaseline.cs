using System;
using BetterPenetrationNext.Core;

// The pre-change Update algorithm evaluated against the identical cached cuts.
// This is a component baseline, not a reproduction of a Unity render frame.
internal sealed class LandmarkBenchmarkBaseline
{
    readonly MeshLandmarkSection[] sections;
    readonly Vec3[] extrema=new Vec3[6];
    readonly int[] selected;
    readonly double[] dx,dy;
    readonly int tipVertex,baseVertex;
    public LandmarkBenchmarkBaseline(AdaptiveMeshLandmarks reference,int[] vertexIndices)
    {
        sections=new MeshLandmarkSection[reference.Sections.Length];
        for(int i=0;i<sections.Length;i++){
            var section=reference.Sections[i];sections[i]=new MeshLandmarkSection(section.Bindings,section.Coordinate,section.Radial.Length);
        }
        selected=vertexIndices;tipVertex=reference.TipVertex;baseVertex=reference.BaseVertex;
        int count=sections[0].Radial.Length;dx=new double[count];dy=new double[count];
        for(int i=0;i<count;i++){dx[i]=Math.Cos(i*2*Math.PI/count);dy[i]=Math.Sin(i*2*Math.PI/count);}
    }
    public void Update(Vec3[] vertices,Vec3 orientation)
    {
        Vec3 tip=vertices[tipVertex],basis=vertices[baseVertex];for(int k=0;k<6;k++)extrema[k]=vertices[selected[0]];
        Vec3 z=(tip-basis).Normalized,xLocal=orientation-z*Vec3.Dot(orientation,z);
        if(xLocal.LengthSquared<1e-18)xLocal=Vec3.Cross(z,new Vec3(0,1,0));
        xLocal=xLocal.Normalized;Vec3 yLocal=Vec3.Cross(z,xLocal);
        foreach(int i in selected){var v=vertices[i];for(int axis=0;axis<3;axis++){
            Vec3 direction=axis==0?xLocal:axis==1?yLocal:z;double d=Vec3.Dot(v-basis,direction);
            if(d<Vec3.Dot(extrema[axis*2]-basis,direction))extrema[axis*2]=v;
            if(d>Vec3.Dot(extrema[axis*2+1]-basis,direction))extrema[axis*2+1]=v;
        }}
        for(int s=0;s<sections.Length;s++){
            var section=sections[s];Vec3 c=new Vec3();for(int i=0;i<section.Surface.Length;i++){
                section.Surface[i]=section.Bindings[i].Evaluate(vertices);c+=section.Surface[i]/section.Surface.Length;
            }section.Centre=c;
        }
        for(int s=0;s<sections.Length;s++){
            var section=sections[s];Vec3 n=(sections[Math.Min(sections.Length-1,s+1)].Centre-sections[Math.Max(0,s-1)].Centre).Normalized;
            section.Tangent=n;Vec3 x=orientation-n*Vec3.Dot(orientation,n);
            if(x.LengthSquared<1e-20)x=Vec3.Cross(n,new Vec3(0,1,0));x=x.Normalized;Vec3 y=Vec3.Cross(n,x);
            for(int k=0;k<section.Radial.Length;k++){
                Vec3 direction=x*dx[k]+y*dy[k];double best=double.NegativeInfinity;Vec3 point=section.Surface[0];
                foreach(var v in section.Surface){double d=Vec3.Dot(v-section.Centre,direction);if(d>best){best=d;point=v;}}
                section.Radial[k]=point;
            }
        }
    }
}
