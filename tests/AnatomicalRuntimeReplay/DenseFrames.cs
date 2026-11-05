using System;
using System.Collections.Generic;
using BetterPenetrationNext.Core;
static partial class Program
{
    // Interpolated recorded-input fixture, explicitly NOT additional measured
    // Runtime frames. Separates solver discontinuities from capture cadence.
    static List<Frame> DenseFrames(List<Frame> source)
    {
        var result=new List<Frame>();result.Add(source[0]);
        for(int f=1;f<source.Count;f++)
        {
            var a=source[f-1];var b=source[f];var previous=source[Math.Max(0,f-2)];var next=source[Math.Min(source.Count-1,f+1)];int count=Math.Max(1,(int)Math.Ceiling((b.Time-a.Time)*120));
            for(int j=1;j<=count;j++)
            {
                double t=j/(double)count,dt=b.Time-a.Time,ha=dt/(b.Time-previous.Time),hb=dt/(next.Time-a.Time);var n=new Frame{Id=result.Count,Time=a.Time+dt*t,Dt=dt/count,Size=a.Size+(b.Size-a.Size)*t,Settings=b.Settings,SourceLengthScale=b.SourceLengthScale,SourceWidthScale=b.SourceWidthScale,Root=Cubic(previous.Root,a.Root,b.Root,next.Root,t,ha,hb),Offset=Lerp(a.Offset,b.Offset,t),SourceFrame=Cubic(previous.SourceFrame,a.SourceFrame,b.SourceFrame,next.SourceFrame,t,ha,hb),Source=new Vec3[b.Source.Length],Solved=b.Solved,Linear=new Linear3[b.Linear.Length],Translation=new Vec3[b.Translation.Length],Morph=new double[b.Morph.Length]};
                for(int i=0;i<n.Source.Length;i++)n.Source[i]=Cubic(previous.Source[i],a.Source[i],b.Source[i],next.Source[i],t,ha,hb);
                for(int i=0;i<n.Linear.Length;i++){n.Linear[i]=Cubic(previous.Linear[i],a.Linear[i],b.Linear[i],next.Linear[i],t,ha,hb);n.Translation[i]=Cubic(previous.Translation[i],a.Translation[i],b.Translation[i],next.Translation[i],t,ha,hb);}
                for(int i=0;i<n.Morph.Length;i++)n.Morph[i]=a.Morph[i]+(b.Morph[i]-a.Morph[i])*t;result.Add(n);
            }
        }
        Console.WriteLine("INTERPOLATED_CAPTURE_FIXTURE frames="+result.Count+" original="+source.Count);return result;
    }
    static Vec3 Lerp(Vec3 a,Vec3 b,double t){return a*(1-t)+b*t;}
    static Linear3 Lerp(Linear3 a,Linear3 b,double t){return new Linear3(Lerp(a.X,b.X,t),Lerp(a.Y,b.Y,t),Lerp(a.Z,b.Z,t));}
    static Vec3 Cubic(Vec3 previous,Vec3 a,Vec3 b,Vec3 next,double t,double ha,double hb)
    {double t2=t*t,t3=t2*t;return a*(2*t3-3*t2+1)+(b-previous)*(ha*(t3-2*t2+t))+b*(-2*t3+3*t2)+(next-a)*(hb*(t3-t2));}
    static Linear3 Cubic(Linear3 p,Linear3 a,Linear3 b,Linear3 n,double t,double ha,double hb)
    {return new Linear3(Cubic(p.X,a.X,b.X,n.X,t,ha,hb),Cubic(p.Y,a.Y,b.Y,n.Y,t,ha,hb),Cubic(p.Z,a.Z,b.Z,n.Z,t,ha,hb));}
}
