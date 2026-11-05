using System;
namespace BetterPenetrationNext.Core
{
    // A live, oriented aperture. The authored contour is retained, including
    // non-circular cross sections; no mesh welding or new cap is introduced.
    public sealed class OrientedEntryFrame
    {
        readonly Vec3[] polygon;
        public Vec3 Centre,Normal,X,Y;
        public double MinimumRadius {get;private set;}
        public OrientedEntryFrame(int count){if(count<3)throw new ArgumentException("Entry contour required");polygon=new Vec3[count];}
        public void Update(Vec3[] world,int[] indices,Vec3 centre,Vec3 next,Vec3 orientation)
        {
            if(indices.Length!=polygon.Length)throw new ArgumentException("Entry topology changed");
            Centre=centre;Normal=(next-centre).Normalized;
            X=(orientation-Normal*Vec3.Dot(orientation,Normal)).Normalized;
            if(X.LengthSquared<.5)X=Vec3.Cross(Normal,Math.Abs(Normal.X)<.8?new Vec3(1,0,0):new Vec3(0,1,0)).Normalized;
            Y=Vec3.Cross(Normal,X);MinimumRadius=double.PositiveInfinity;
            for(int i=0;i<indices.Length;i++){Vec3 p=world[indices[i]]-centre;polygon[i]=new Vec3(Vec3.Dot(p,X),Vec3.Dot(p,Y),0);}
            for(int i=0;i<polygon.Length;i++)MinimumRadius=Math.Min(MinimumRadius,Closest(new Vec3(),polygon[i],polygon[(i+1)%polygon.Length]).Length);
        }
        static Vec3 Closest(Vec3 p,Vec3 a,Vec3 b){Vec3 e=b-a;return a+e*(e.LengthSquared<1e-24?0:Math.Max(0,Math.Min(1,Vec3.Dot(p-a,e)/e.LengthSquared)));}
        Vec3 Project(Vec3 p){p-=Centre;return new Vec3(Vec3.Dot(p,X),Vec3.Dot(p,Y),0);}
        public bool Contains(Vec3 world,double radius=0)
        {
            Vec3 p=Project(world);bool inside=false;double closest=double.PositiveInfinity;
            for(int i=0,j=polygon.Length-1;i<polygon.Length;j=i++)
            {
                Vec3 a=polygon[j],b=polygon[i];closest=Math.Min(closest,(p-Closest(p,a,b)).LengthSquared);
                if((a.Y>p.Y)!=(b.Y>p.Y)&&p.X<(b.X-a.X)*(p.Y-a.Y)/(b.Y-a.Y)+a.X)inside=!inside;
            }
            return inside||closest<=radius*radius;
        }
        public Vec3 DesiredDirection(Vec3 pivot)
        {
            // A fixed base can align perfectly with the plane normal only if
            // its normal ray intersects the aperture. Otherwise target an
            // inset point of its actual contour, favouring safe admission.
            double depth=-Vec3.Dot(pivot-Centre,Normal);
            if(depth<=1e-8)return (Centre-pivot).Normalized;
            Vec3 projected=pivot+Normal*depth;
            if(Contains(projected))return Normal;
            Vec3 p=Project(projected),best=new Vec3();double distance=double.PositiveInfinity;
            for(int i=0;i<polygon.Length;i++){Vec3 q=Closest(p,polygon[i],polygon[(i+1)%polygon.Length]);double d=(p-q).LengthSquared;if(d<distance){distance=d;best=q;}}
            return (Centre+(X*best.X+Y*best.Y)*.5-pivot).Normalized;
        }
        public bool Intersects(AdaptiveMeshLandmarks source)
        {
            var s=source.Sections;
            for(int i=0;i<s.Length-1;i++)
            {
                double a=Vec3.Dot(s[i].Centre-Centre,Normal),b=Vec3.Dot(s[i+1].Centre-Centre,Normal);
                if((a<0&&b<0)||(a>0&&b>0))continue;
                double t=Math.Abs(b-a)<1e-12?0:Math.Max(0,Math.Min(1,-a/(b-a))),ra=0,rb=0;
                foreach(var p in s[i].Radial)ra=Math.Max(ra,(p-s[i].Centre).Length);
                foreach(var p in s[i+1].Radial)rb=Math.Max(rb,(p-s[i+1].Centre).Length);
                if(Contains(s[i].Centre*(1-t)+s[i+1].Centre*t,ra*(1-t)+rb*t))return true;
            }
            return false;
        }
    }
}
