using System;
namespace BetterPenetrationNext.Core
{
    // Finite local compression of the existing cervical mesh. Its attachment,
    // uterine cavity and other organs remain on their animated baseline.
    public sealed class LocalCervixCompression
    {
        readonly double[] weights;readonly bool[] cervical;
        double amount,velocity,released;
        public double Amount {get{return amount;}}
        public double Maximum {get;private set;}
        public LocalCervixCompression(Vec3[] rest,int[] regions,Vec3 opening,Vec3 normal)
        {
            weights=new double[rest.Length];cervical=new bool[rest.Length];double extent=0;
            for(int i=0;i<rest.Length;i++)if(regions[i]==3)extent=Math.Max(extent,Vec3.Dot(rest[i]-opening,normal));
            for(int i=0;i<rest.Length;i++)if(regions[i]==3){cervical[i]=true;double t=Math.Max(0,Math.Min(1,Vec3.Dot(rest[i]-opening,normal)/Math.Max(1e-9,extent)));weights[i]=1-t*t*(3-2*t);}
        }
        public void Seed(double value){amount=Math.Max(0,value);velocity=0;}
        public void Advance(Vec3[] animated,Vec3 opening,Vec3 normal,double demand,double userLimit,double strength,double dt,double response,bool active,bool beganRelease,double releaseWeight)
        {
            double extent=0;for(int i=0;i<weights.Length;i++)if(cervical[i])extent=Math.Max(extent,Vec3.Dot(animated[i]-opening,normal));
            // A positive axial Jacobian is retained throughout compression.
            // smoothstep's maximum derivative is 1.5: travel <= .6 depth.
            Maximum=Math.Min(Math.Max(0,userLimit),Math.Max(0,extent)*.6);
            if(beganRelease)released=amount;
            if(active){Vec3 p=new Vec3(amount,0,0),v=new Vec3(velocity,0,0);CriticalMotion.Advance(ref p,ref v,new Vec3(Math.Min(Maximum,Math.Max(0,demand)*Math.Max(0,strength)),0,0),response,Math.Min(.25,Math.Max(0,dt)));amount=Math.Max(0,Math.Min(Maximum,p.X));velocity=p.X==amount?v.X:0;}
            else{amount=Math.Min(Maximum,released*Math.Max(0,Math.Min(1,releaseWeight)));velocity=0;}
        }
        public Vec3 Point(int index,Vec3 animated,Vec3 normal){return animated+normal*(amount*weights[index]);}
    }
}
