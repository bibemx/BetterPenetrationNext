using System;
namespace BetterPenetrationNext.Core
{
    public struct FrameMeasurement { public double Capture,Refit,Solve,Apply,XRay,Total; public int Vertices,Contacts,Pairs,Collections; }
    public struct TimeSummary { public double Mean,P95; }
    public sealed class PerformanceWindow
    {
        readonly FrameMeasurement[] samples;
        readonly double[] scratch;
        int cursor,count;
        public int Count { get { return count; } }
        public PerformanceWindow(int capacity=240) { if(capacity<20) throw new ArgumentException("Profile window too short."); samples=new FrameMeasurement[capacity]; scratch=new double[capacity]; }
        public void Add(FrameMeasurement value) { samples[cursor]=value; cursor=(cursor+1)%samples.Length; count=Math.Min(count+1,samples.Length); }
        public TimeSummary Time(int category)
        {
            double sum=0;
            for(int i=0;i<count;i++) { var s=samples[i]; double value=category==0?s.Capture:category==1?s.Refit:category==2?s.Solve:category==3?s.Apply:category==4?s.XRay:s.Total; scratch[i]=value; sum+=value; }
            Array.Sort(scratch,0,count); return count==0?new TimeSummary():new TimeSummary { Mean=sum/count,P95=scratch[(int)Math.Ceiling(count*0.95)-1] };
        }
    }
}
