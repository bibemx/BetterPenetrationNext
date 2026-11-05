using System;

namespace BetterPenetrationNext.Core
{
    public struct PathSample { public Vec3 Point,Tangent; public double Parameter,Distance; }
    // A sampled Catmull-Rom or piecewise-linear centreline, rebuilt only when
    // authoring points move. No assumptions about openings or closed anatomy.
    public sealed class AnatomicalPathConstraintSystem
    {
        readonly Vec3[] samples;
        readonly double[] distances;
        public double Length { get { return distances[distances.Length-1]; } }
        readonly bool spline;
        readonly int subdivisions,controlCount;
        public int Count { get { return samples.Length; } }
        public Vec3 this[int i] { get { return samples[i]; } }
        public AnatomicalPathConstraintSystem(Vec3[] controls,bool spline=true,int subdivisions=16)
        {
            if(controls==null||controls.Length<2||subdivisions<1||subdivisions>128) throw new ArgumentException("Path requires at least two finite points.");
            foreach(var p in controls) if(!p.IsFinite) throw new ArgumentException("Non-finite path.");
            this.spline=spline;this.subdivisions=subdivisions;controlCount=controls.Length;
            int steps=spline?subdivisions:1; samples=new Vec3[(controls.Length-1)*steps+1];
            distances=new double[samples.Length];
            UpdateControls(controls);
        }
        public void UpdateControls(Vec3[] controls)
        {
            if(controls==null||controls.Length!=controlCount) throw new ArgumentException("Control count changed; rebuild path.");
            foreach(var p in controls)if(!p.IsFinite)throw new ArgumentException("Non-finite path.");
            int steps=spline?subdivisions:1;
            for(int i=0;i<controls.Length-1;i++) for(int j=0;j<steps;j++)
            {
                double t=(double)j/steps; Vec3 a=controls[Math.Max(0,i-1)],b=controls[i],c=controls[i+1],d=controls[Math.Min(controls.Length-1,i+2)];
                samples[i*steps+j]=spline?(b*2+(c-a)*t+(a*2-b*5+c*4-d)*(t*t)+(-a+b*3-c*3+d)*(t*t*t))*.5:b*(1-t)+c*t;
            }
            samples[samples.Length-1]=controls[controls.Length-1];
            double length=0; distances[0]=0; for(int i=1;i<samples.Length;i++) { length+=(samples[i]-samples[i-1]).Length; distances[i]=length; }
            if(length<1e-8) throw new ArgumentException("Collapsed path.");
        }
        public double DistanceAtParameter(double parameter) {
            double index=Math.Max(0,Math.Min(1,parameter))*(samples.Length-1);int a=Math.Min(samples.Length-2,(int)index);
            return distances[a]+(distances[a+1]-distances[a])*(index-a);
        }
        public PathSample AtDistance(double distance) {
            double d=Math.Max(0,Math.Min(Length,distance));int lo=0,hi=samples.Length-1;
            while(hi-lo>1){int mid=(lo+hi)/2;if(distances[mid]<d)lo=mid;else hi=mid;}
            double segment=distances[hi]-distances[lo],t=segment>1e-20?(d-distances[lo])/segment:0;
            return new PathSample{Point=samples[lo]*(1-t)+samples[hi]*t,Tangent=(samples[hi]-samples[lo]).Normalized,Parameter=(lo+t)/(samples.Length-1)};
        }
        public PathSample Closest(Vec3 position,double previous=-1,double continuityWindow=1)
        {
            if(!position.IsFinite) throw new ArgumentException("Invalid path query.");
            var best=new PathSample { Distance=double.PositiveInfinity };
            for(int i=0;i<samples.Length-1;i++)
            {
                Vec3 edge=samples[i+1]-samples[i]; double length=edge.LengthSquared; if(length<1e-24) continue;
                double t=Math.Max(0,Math.Min(1,Vec3.Dot(position-samples[i],edge)/length)); double parameter=(i+t)/(samples.Length-1);
                if(previous>=0&&Math.Abs(parameter-previous)>continuityWindow) continue;
                Vec3 point=samples[i]+edge*t; double distance=(position-point).Length;
                if(distance<best.Distance) best=new PathSample { Point=point,Tangent=edge.Normalized,Distance=distance,Parameter=parameter };
            }
            if(double.IsPositiveInfinity(best.Distance)) return Closest(position); return best;
        }
        public PathSample Constrain(SourceContactBody source,Vec3 animatedPoint,Vec3 animatedForward,double dt,double strength=12,double maximumAngle=.35,double previous=-1)
        {
            var sample=Closest(animatedPoint+source.Offset,previous,.15);
            double blend=1-Math.Exp(-Math.Max(0,strength)*Math.Max(0,dt));
            source.Offset=Vec3.ClampLength(source.Offset+(sample.Point-animatedPoint-source.Offset)*blend,source.MaximumDisplacement);
            Vec3 forward=animatedForward.Normalized,axis=Vec3.Cross(forward,sample.Tangent); double dot=Math.Max(-1,Math.Min(1,Vec3.Dot(forward,sample.Tangent)));
            // Keep tangent direction continuous through a reversing animated input.
            if(dot<0) { sample.Tangent=-sample.Tangent; axis=-axis; dot=-dot; }
            source.Pivot=animatedPoint;
            var desired=axis.Normalized*Math.Min(Math.Max(0,maximumAngle),Math.Acos(dot));
            source.RotationVector=source.RotationVector*(1-blend)+desired*blend; source.LimitPose();
            return sample;
        }
    }
}
