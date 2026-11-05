using System;

namespace BetterPenetrationNext.Core
{
    // A continuous section field replaces sharp differences between individual
    // loaded/unloaded sections. No smoothing operation merges skin identities.
    public sealed class ReceiverSectionField
    {
        readonly double[] radii,slopes,arc,lower;
        public ReceiverSectionField(int count){radii=new double[count];slopes=new double[count];arc=new double[count];lower=new double[count];}
        public void Update(Vec3[] centres,double[] values)
        {
            if(centres.Length!=radii.Length||values.Length!=radii.Length)throw new ArgumentException("Section field dimensions differ");
            arc[0]=0;for(int i=0;i<radii.Length;i++){radii[i]=lower[i]=Math.Max(0,values[i]);if(i>0)arc[i]=arc[i-1]+(centres[i]-centres[i-1]).Length;}
            for(int pass=0;pass<2;pass++)
            {
                for(int i=1;i<radii.Length;i++)radii[i]=Math.Max(radii[i],radii[i-1]-.75*(arc[i]-arc[i-1]));
                for(int i=radii.Length-2;i>=0;i--)radii[i]=Math.Max(radii[i],radii[i+1]-.75*(arc[i+1]-arc[i]));
                for(int i=1;i<radii.Length-1;i++)radii[i]=Math.Max(lower[i],(radii[i-1]+radii[i]*2+radii[i+1])*.25);
            }
            for(int i=0;i<radii.Length;i++)
            {
                if(i==0)slopes[i]=radii[1]-radii[0];else if(i==radii.Length-1)slopes[i]=radii[i]-radii[i-1];
                else{double a=radii[i]-radii[i-1],b=radii[i+1]-radii[i];slopes[i]=a*b<=0?0:2*a*b/(a+b);}
            }
        }
        public double Radius(int a,double t)
        {
            double t2=t*t,t3=t2*t;double r=radii[a]*(2*t3-3*t2+1)+slopes[a]*(t3-2*t2+t)+radii[a+1]*(-2*t3+3*t2)+slopes[a+1]*(t3-t2);
            return Math.Max(Math.Min(radii[a],radii[a+1]),Math.Min(Math.Max(radii[a],radii[a+1]),r));
        }
        public static Vec3 Centre(Vec3[] p,int a,double t)
        {Vec3 b=p[a],c=p[a+1],v=(c-p[Math.Max(0,a-1)])*.5,w=(p[Math.Min(p.Length-1,a+2)]-b)*.5;double t2=t*t,t3=t2*t;return b*(2*t3-3*t2+1)+v*(t3-2*t2+t)+c*(-2*t3+3*t2)+w*(t3-t2);}
    }
}
