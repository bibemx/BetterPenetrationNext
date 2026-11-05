using System;
namespace BetterPenetrationNext.Core
{
    // Independent material sectors: an eccentric load must not dilate the
    // opposite wall. Cached profiles interpolate continuously around the seam
    // and along the canal. No vertex, skin weight or topology is merged.
    public sealed class ReceiverRadialField
    {
        public const int Directions=16;
        readonly double[][] required,history,values;
        readonly ReceiverSectionField[] fields;
        readonly int count;
        readonly double padding=1/Math.Cos(2*Math.PI/Directions);
        public ReceiverRadialField(int sections)
        {
            count=sections;required=new double[Directions][];history=new double[Directions][];values=new double[Directions][];fields=new ReceiverSectionField[Directions];
            for(int k=0;k<Directions;k++){required[k]=new double[count];history[k]=new double[count];values[k]=new double[count];fields[k]=new ReceiverSectionField(count);}
        }
        public void Begin(double[] baseline)
        {if(baseline.Length!=count)throw new ArgumentException("Radial field dimensions changed");for(int k=0;k<Directions;k++)Array.Copy(baseline,required[k],count);}
        public void Include(int section,Vec3 radial,Vec3 x,Vec3 y,double clearance)
        {
            double a=Vec3.Dot(radial,x),b=Vec3.Dot(radial,y),r=Math.Sqrt(a*a+b*b);if(r<1e-12)return;
            double angle=Math.Atan2(b,a)*Directions/(2*Math.PI);if(angle<0)angle+=Directions;
            // A negative sub-ulp angle can round to exactly Directions after
            // wrapping. It belongs to sector zero, not an extra array entry.
            if(angle>=Directions)angle=0;
            int first=(int)angle,second=(first+1)%Directions;double wanted=r*padding+Math.Max(0,clearance);
            required[first][section]=Math.Max(required[first][section],wanted);required[second][section]=Math.Max(required[second][section],wanted);
        }
        public void Update(Vec3[] centres,double dt,double response,bool currentContact=false)
        {
            double alpha=1-Math.Exp(-Math.Max(0,dt)*Math.Max(0,response));
            for(int k=0;k<Directions;k++)
            {
                for(int s=0;s<count;s++){history[k][s]=currentContact?required[k][s]:history[k][s]+(required[k][s]-history[k][s])*alpha;values[k][s]=Math.Max(required[k][s],history[k][s]);}
                fields[k].Update(centres,values[k]);
            }
        }
        public double Radius(int section,double t,double radians)
        {
            double coordinate=radians*Directions/(2*Math.PI);coordinate-=Math.Floor(coordinate/Directions)*Directions;
            int first=Math.Min(Directions-1,(int)coordinate),second=(first+1)%Directions;double blend=coordinate-first;
            return fields[first].Radius(section,t)*(1-blend)+fields[second].Radius(section,t)*blend;
        }
    }
}
