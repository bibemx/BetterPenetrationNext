using System;

namespace BetterPenetrationNext.Core
{
    // A reduced, root-attached rod with a separate positional guide. Material
    // links retain their current animated lengths. Each
    // cross-section is transported by one rigid rotation, without radial scale.
    public sealed class InextensibleSourceRig
    {
        struct Rotation
        {
            public Vec3 V;public double W;
            public static Rotation Identity {get{return new Rotation{W=1};}}
            public Vec3 Apply(Vec3 p){Vec3 c=Vec3.Cross(V,p);return p+c*(2*W)+Vec3.Cross(V,c)*2;}
            public static Rotation Between(Vec3 a,Vec3 b)
            {
                a=a.Normalized;b=b.Normalized;if(a.LengthSquared<.5||b.LengthSquared<.5)return Identity;
                double d=Math.Max(-1,Math.Min(1,Vec3.Dot(a,b)));
                if(d<-.999999){Vec3 axis=Vec3.Cross(a,Math.Abs(a.X)<.8?new Vec3(1,0,0):new Vec3(0,1,0)).Normalized;return new Rotation{V=axis};}
                double w=Math.Sqrt((1+d)*.5);return new Rotation{V=Vec3.Cross(a,b)/(2*w),W=w};
            }
            public static Rotation Blend(Rotation a,Rotation b,double t)
            {
                double d=Vec3.Dot(a.V,b.V)+a.W*b.W;if(d<0){b.V=-b.V;b.W=-b.W;d=-d;}
                double x=1-t,y=t;if(d<.9995){double angle=Math.Acos(Math.Min(1,d)),s=Math.Sin(angle);x=Math.Sin((1-t)*angle)/s;y=Math.Sin(t*angle)/s;}
                var r=new Rotation{V=a.V*x+b.V*y,W=a.W*x+b.W*y};double n=Math.Sqrt(r.V.LengthSquared+r.W*r.W);r.V/=n;r.W/=n;return r;
            }
            public static Rotation SectionBlend(Rotation a,Rotation b,double t)
            {
                if(Vec3.Dot(a.V,b.V)+a.W*b.W<0){b.V=-b.V;b.W=-b.W;}
                var q=new Rotation{V=a.V*(1-t)+b.V*t,W=a.W*(1-t)+b.W*t};double norm=Math.Sqrt(q.V.LengthSquared+q.W*q.W);q.V/=norm;q.W/=norm;return q;
            }
        }
        readonly SourceSectionRig sections;
        readonly Vec3[] previousDirections;
        readonly Rotation[] frames;
        readonly double[] lengths,arc;
        readonly Vec3[] guideSeed,guideSeedAnimated;Linear3 guideSeedFrame;bool hasGuideSeed;int transverseVertex=-1;
        public Linear3 InputFrame;public Linear3 GuidanceFrame {get;private set;}
        double curvatureActivation=1;
        readonly int[] vertexSpans;readonly double[] vertexBlends;
        public readonly double[] SectionRadii;
        public readonly Vec3[] Animated,Solved;
        public double MaximumDisplacement {get;private set;}
        public double BudgetBlend {get;private set;}
        public double MaximumTurn {get;private set;}
        public int Count {get{return Animated.Length;}}
        public bool StablePathGuidance=true;
        public bool PositionGuidance=true;public int EndpointVertex=-1;
        public bool ContinuousTerminalOrientation=true;
        public bool ContinuousTerminalStop=true;
        public bool DistributedTerminalApproach=true;
        public double IntendedDistance {get;private set;}public double GuideLength {get;private set;}public double TerminalPressure {get;private set;}public double EndpointError {get;private set;}
        public InextensibleSourceRig(SourceSectionRig sections)
        {
            if(sections==null)throw new ArgumentNullException("sections");this.sections=sections;int n=sections.SectionCount;
            Animated=new Vec3[n];Solved=new Vec3[n];guideSeed=new Vec3[n];guideSeedAnimated=new Vec3[n];previousDirections=new Vec3[n-1];frames=new Rotation[n];lengths=new double[n-1];arc=new double[n];
            SectionRadii=new double[n];vertexSpans=new int[sections.VertexCount];vertexBlends=new double[vertexSpans.Length];for(int i=0;i<vertexSpans.Length;i++){double u=sections.MaterialCoordinate(i)*(Count-1);vertexSpans[i]=Math.Min(Count-2,(int)u);vertexBlends[i]=u-vertexSpans[i];}
        }
        void Capture(Vec3[] vertices,Vec3 root)
        {
            sections.CaptureAnimation(vertices,root);arc[0]=0;
            for(int i=0;i<Count;i++){Animated[i]=sections.AnimatedCentre(i);if(i>0){lengths[i-1]=(Animated[i]-Animated[i-1]).Length;arc[i]=arc[i-1]+lengths[i-1];}}
            Linear3 inverse;if(InputFrame.TryInverse(out inverse))GuidanceFrame=InputFrame;
            else
            {
                Vec3 z=(Animated[Count-1]-Animated[0]).Normalized;
                if(transverseVertex<0){double far=-1;for(int i=0;i<vertices.Length;i++){Vec3 d=vertices[i]-Animated[0];double distance=(d-z*Vec3.Dot(d,z)).LengthSquared;if(distance>far){far=distance;transverseVertex=i;}}}
                Vec3 x=vertices[transverseVertex]-Animated[0];x=(x-z*Vec3.Dot(x,z)).Normalized;
                GuidanceFrame=new Linear3(x,Vec3.Cross(z,x).Normalized,z);
            }
        }
        public void SetGuideSeed(Vec3[] animated,Vec3[] guided,Linear3 frame)
        {if(animated.Length!=Count||guided.Length!=Count)throw new ArgumentException("Guide seed dimensions differ");Array.Copy(animated,guideSeedAnimated,Count);Array.Copy(guided,guideSeed,Count);guideSeedFrame=frame;hasGuideSeed=true;}
        public void ClearGuideSeed(){hasGuideSeed=false;}
        void SaveGuide(){Array.Copy(Animated,guideSeedAnimated,Count);Array.Copy(Solved,guideSeed,Count);guideSeedFrame=GuidanceFrame;hasGuideSeed=true;}
        bool StartFromGuideSeed(double bendRadius)
        {
            Linear3 inverse;if(!hasGuideSeed||!guideSeedFrame.TryInverse(out inverse))return false;
            Solved[0]=Animated[0];
            for(int i=0;i<Count-1;i++)
            {
                Vec3 a=GuidanceFrame.Multiply(inverse.Multiply(guideSeedAnimated[i+1]-guideSeedAnimated[i])).Normalized,b=GuidanceFrame.Multiply(inverse.Multiply(guideSeed[i+1]-guideSeed[i])).Normalized;
                Vec3 direction=Rotation.Between(a,b).Apply((Animated[i+1]-Animated[i]).Normalized);
                if(i==0)direction=(Animated[1]-Animated[0]).Normalized;else direction=Turn((Solved[i]-Solved[i-1]).Normalized,direction,CurvatureLimit(i,bendRadius));
                Solved[i+1]=Solved[i]+direction*lengths[i];
            }
            return true;
        }
        public static PathSample Extended(AnatomicalPathConstraintSystem path,double distance)
        {
            var p=path.AtDistance(distance);if(distance<0)p.Point+=p.Tangent*distance;else if(distance>path.Length)p.Point+=p.Tangent*(distance-path.Length);return p;
        }
        static Vec3 Turn(Vec3 from,Vec3 to,double maximum)
        {
            from=from.Normalized;to=to.Normalized;if(from.LengthSquared<.5)return to;if(to.LengthSquared<.5)return from;
            double angle=Math.Acos(Math.Max(-1,Math.Min(1,Vec3.Dot(from,to))));return Rotation.Blend(Rotation.Identity,Rotation.Between(from,to),angle>1e-12?Math.Min(1,maximum/angle):1).Apply(from).Normalized;
        }
        double CurvatureLimit(int link,double bendRadius)
        {
            double limit=Math.Min(.3,lengths[link]/Math.Max(1e-8,bendRadius));
            if(link>0)
            {
                Vec3 a=(Animated[link]-Animated[link-1]).Normalized,b=(Animated[link+1]-Animated[link]).Normalized;
                // Never straighten an existing animated material corner as
                // guidance becomes infinitesimally nonzero. That created a
                // finite entry jump even with a continuous contact weight.
                double native=Math.Acos(Math.Max(-1,Math.Min(1,Vec3.Dot(a,b))));limit+=Math.Max(0,native-limit)*(1-curvatureActivation);
            }
            return limit;
        }
        public void Guide(AnatomicalPathConstraintSystem path,Vec3[] vertices,Vec3 root,double radius,double strength,double dt,double response,double maximum,double activation=-1,bool stopAtEnd=false)
        {
            curvatureActivation=Math.Max(0,Math.Min(1,activation<0?strength:activation));
            Capture(vertices,root);Solved[0]=root;var entry=path.AtDistance(0);
            // Signed distance carries the animated axial degree of freedom even
            // beyond either end of a finite guide. A cervix is a separate surface.
            double progress=Vec3.Dot(root-entry.Point,entry.Tangent);
            IntendedDistance=progress+arc[Count-1];GuideLength=path.Length;TerminalPressure=stopAtEnd?Math.Max(0,IntendedDistance-GuideLength):0;EndpointError=0;
            double bendRadius=Math.Max(radius*2.5,arc[Count-1]*.12);
            for(int i=0;i<Count-1;i++)
            {
                Vec3 native=(Animated[i+1]-Animated[i]).Normalized,goal=Extended(path,progress+arc[i+1]).Point-Solved[i];
                if(StablePathGuidance)
                {
                    // Finite arc look-ahead cannot reverse when an animated
                    // section passes a nearby positional target. Remove axial
                    // error: guidance must not latch the tip to a path sample.
                    double look=Math.Max(radius*2,arc[Count-1]/(Count-1)*3),distance=progress+arc[i];
                    var here=Extended(path,distance);Vec3 guideDirection=(Extended(path,distance+look).Point-here.Point).Normalized;
                    Vec3 lateral=here.Point-Solved[i];lateral-=guideDirection*Vec3.Dot(lateral,guideDirection);
                    goal=guideDirection*look+Vec3.ClampLength(lateral,look*.5);
                }
                double ramp=1-Math.Exp(-arc[i]/Math.Max(radius*3,arc[Count-1]*.1));
                double engagement=Math.Max(0,Math.Min(1,(Vec3.Dot(Animated[i+1]-entry.Point,entry.Tangent)+radius)/Math.Max(1e-8,2*radius)));
                Vec3 direction=Turn(native,goal,(StablePathGuidance?Math.PI*Math.Max(0,Math.Min(1,strength)):Math.Min(.45,Math.PI*Math.Max(0,Math.Min(1,strength))))*ramp*engagement);
                if(i==0)direction=native;
                else if(strength>0)direction=Turn((Solved[i]-Solved[i-1]).Normalized,direction,CurvatureLimit(i,bendRadius));
                previousDirections[i]=direction;Solved[i+1]=Solved[i]+direction*lengths[i];
            }
            if(PositionGuidance&&strength>0&&curvatureActivation>0)
            {
                // A direction-only pass need not meet the oriented terminal.
                // Match an ANIMATED arc sample, clamping only at a verified Rn.
                // Withdrawal immediately changes this sample below Rn; no latch.
                double targetDistance=TargetDistance(path,radius,stopAtEnd);Vec3 desired=Extended(path,targetDistance).Point;
                BuildFrames();Vec3 tip=EndpointVertex>=0?Map(EndpointVertex,vertices[EndpointVertex]):Solved[Count-1];
                desired=tip+(desired-tip)*curvatureActivation;
                bool warm=StartFromGuideSeed(bendRadius);BuildFrames();tip=EndpointVertex>=0?Map(EndpointVertex,vertices[EndpointVertex]):Solved[Count-1];
                // A binary switch at Rn imposed a new endpoint orientation in
                // one frame. Follow the C1 guide tangent throughout approach.
                Vec3 direction=stopAtEnd&&(ContinuousTerminalOrientation||IntendedDistance>=path.Length)?Extended(path,targetDistance).Tangent:new Vec3();
                ReactAtCoordinate(desired-tip,1,vertices,radius,maximum,12,true,direction,warm?8:32);tip=EndpointVertex>=0?Map(EndpointVertex,vertices[EndpointVertex]):Solved[Count-1];EndpointError=(desired-tip).Length;SaveGuide();
                return;
            }
            LimitBudget(vertices,maximum);BuildFrames();SaveGuide();
        }
        public void RefreshEndpoint(AnatomicalPathConstraintSystem path,Vec3[] vertices,double radius,double maximum,bool stopAtEnd)
        {
            GuideLength=path.Length;TerminalPressure=stopAtEnd?Math.Max(0,IntendedDistance-GuideLength):0;
            double targetDistance=TargetDistance(path,radius,stopAtEnd);Vec3 desired=Extended(path,targetDistance).Point;
            BuildFrames();Vec3 tip=EndpointVertex>=0?Map(EndpointVertex,vertices[EndpointVertex]):Solved[Count-1];
            ReactAtCoordinate((desired-tip)*curvatureActivation,1,vertices,radius,maximum,4,true,stopAtEnd&&(ContinuousTerminalOrientation||IntendedDistance>=path.Length)?Extended(path,targetDistance).Tangent:new Vec3(),8);
            tip=EndpointVertex>=0?Map(EndpointVertex,vertices[EndpointVertex]):Solved[Count-1];EndpointError=(desired-tip).Length;SaveGuide();
        }
        double TargetDistance(AnatomicalPathConstraintSystem path,double radius,bool stop)
        {
            // The stop must be resolved over a bend-sized material interval,
            // not a single tip link crossed in less than one fast frame. The
            // compact C2 map still releases immediately with axial animation.
            double band=DistributedTerminalApproach?Math.Min(path.Length*.3,Math.Max(radius*1.5,arc[Count-1]*.12)):Math.Max(lengths[lengths.Length-1]*.5,Math.Min(radius*.5,arc[Count-1]*.04));
            return !stop?IntendedDistance:!ContinuousTerminalStop?Math.Min(IntendedDistance,path.Length):SmoothTerminalApproach.Distance(IntendedDistance,path.Length,band);
        }
        public void ReactAtTip(Vec3 delta,Vec3[] vertices,double radius,double maximum)
        {ReactAtCoordinate(delta,1,vertices,radius,maximum);}
        public void ReactAtCoordinate(Vec3 delta,double coordinate,Vec3[] vertices,double radius,double maximum,int iterations=12,bool endpointFit=false,Vec3 endDirection=new Vec3(),int ccdPasses=32)
        {
            int end=Math.Max(1,Math.Min(Count-1,(int)Math.Round(Math.Max(0,Math.Min(1,coordinate))*(Count-1))));
            for(int i=end;i<Count-1;i++)previousDirections[i]=(Solved[i+1]-Solved[i]).Normalized;
            Vec3 oldTangent=(Solved[end]-Solved[end-1]).Normalized,target=Solved[end]+delta;double bendRadius=Math.Max(radius*2.5,arc[Count-1]*.12);
            for(int iteration=0;iteration<iterations;iteration++)
            {
                Solved[end]=target;
                for(int i=end-1;i>=0;i--)Solved[i]=Solved[i+1]+(Solved[i]-Solved[i+1]).Normalized*lengths[i];
                Solved[0]=Animated[0];
                for(int i=0;i<end;i++){Vec3 direction=(Solved[i+1]-Solved[i]).Normalized;if(i==0)direction=(Animated[1]-Animated[0]).Normalized;else direction=Turn((Solved[i]-Solved[i-1]).Normalized,direction,CurvatureLimit(i,bendRadius));Solved[i+1]=Solved[i]+direction*lengths[i];}
            }
            bool oriented=endDirection.LengthSquared>.5;Vec3 approach=target-endDirection.Normalized*lengths[end-1];
            if(endpointFit)for(int pass=0;pass<ccdPasses;pass++)
            {
                bool improved=false;
                for(int joint=end-1;joint>=1;joint--)
                {
                    Vec3 pivot=Solved[joint],outgoing=(Solved[joint+1]-pivot).Normalized,incoming=(pivot-Solved[joint-1]).Normalized;
                    Vec3 from=Solved[end]-pivot,to=target-pivot;
                    if(oriented&&joint<end-1){Vec3 a=Solved[end-1]-pivot,b=approach-pivot;Vec3 cross=Vec3.Cross(from,to)+Vec3.Cross(a,b);double dot=Vec3.Dot(from,to)+Vec3.Dot(a,b);double angle=Math.Atan2(cross.Length,dot);Vec3 axis=cross.Normalized;var objective=new Rotation{V=axis*Math.Sin(angle*.5),W=Math.Cos(angle*.5)};from=outgoing;to=objective.Apply(outgoing);}
                    var goalRotation=Rotation.Between(from,to);Vec3 proposed=goalRotation.Apply(outgoing);
                    var coneRotation=Rotation.Between(proposed,Turn(incoming,proposed,CurvatureLimit(joint,bendRadius)));
                    Vec3 tip=pivot+coneRotation.Apply(goalRotation.Apply(Solved[end]-pivot));
                    double oldError=(Solved[end]-target).LengthSquared,newError=(tip-target).LengthSquared;
                    if(oriented&&joint<end-1){Vec3 before=Solved[end-1]-approach,after=pivot+coneRotation.Apply(goalRotation.Apply(Solved[end-1]-pivot))-approach;oldError+=before.LengthSquared;newError+=after.LengthSquared;}
                    if(newError>=oldError-1e-18)continue;
                    for(int i=joint+1;i<=end;i++)Solved[i]=pivot+coneRotation.Apply(goalRotation.Apply(Solved[i]-pivot));improved=true;
                }
                if(!improved)break;
            }
            var rotation=Rotation.Between(oldTangent,(Solved[end]-Solved[end-1]).Normalized);
            for(int i=end;i<Count-1;i++){Vec3 direction=rotation.Apply(previousDirections[i]);direction=Turn((Solved[i]-Solved[i-1]).Normalized,direction,CurvatureLimit(i,bendRadius));Solved[i+1]=Solved[i]+direction*lengths[i];}
            LimitBudget(vertices,maximum);BuildFrames();
        }
        public void RestoreSolution(Vec3[] saved,Vec3[] vertices,double budgetBlend)
        {
            if(saved.Length!=Count)throw new ArgumentException("Rod snapshot dimensions differ");
            Array.Copy(saved,Solved,Count);BudgetBlend=budgetBlend;MaximumDisplacement=Measure(vertices);
        }
        // Reapply a published solution to the latest native animation. This
        // avoids replacing animated axial motion with a worker's older pose.
        public void Rebase(Vec3[] vertices,Vec3 root,Vec3[] oldAnimated,Vec3[] oldSolved,Linear3 oldFrame,Linear3 currentFrame,double maximum)
        {
            Capture(vertices,root);Solved[0]=root;Linear3 inverse;if(!oldFrame.TryInverse(out inverse))throw new ArgumentException("Singular source frame");
            bool hasCorrection=false;for(int i=0;i<Count;i++)hasCorrection|=(oldSolved[i]-oldAnimated[i]).LengthSquared>1e-18;
            if(!hasCorrection){Array.Copy(Animated,Solved,Count);MaximumDisplacement=0;BudgetBlend=1;BuildFrames();return;}
            double radius=0;for(int i=0;i<vertices.Length;i++){int a=vertexSpans[i];double t=vertexBlends[i];Vec3 e=Animated[a+1]-Animated[a],d=vertices[i]-Animated[a]-e*t;radius=Math.Max(radius,(d-e.Normalized*Vec3.Dot(d,e.Normalized)).Length);}
            double bendRadius=Math.Max(radius*2.5,arc[Count-1]*.12);
            for(int i=0;i<Count-1;i++)
            {
                Vec3 a=currentFrame.Multiply(inverse.Multiply(oldAnimated[i+1]-oldAnimated[i])).Normalized,b=currentFrame.Multiply(inverse.Multiply(oldSolved[i+1]-oldSolved[i])).Normalized;
                Vec3 direction=Rotation.Between(a,b).Apply((Animated[i+1]-Animated[i]).Normalized).Normalized;
                if(i==0)direction=(Animated[1]-Animated[0]).Normalized;
                else direction=Turn((Solved[i]-Solved[i-1]).Normalized,direction,CurvatureLimit(i,bendRadius));
                Solved[i+1]=Solved[i]+direction*lengths[i];
            }
            LimitBudget(vertices,maximum);BuildFrames();
        }
        public void ApplyDirectionDelta(Vec3[] oldGuided,Vec3[] oldSolved,Linear3 oldFrame,Linear3 currentFrame,Vec3[] vertices,double maximum)
        {
            bool corrected=false;for(int i=0;i<Count;i++)corrected|=(oldSolved[i]-oldGuided[i]).LengthSquared>1e-18;
            if(!corrected)return;Linear3 inverse;if(!oldFrame.TryInverse(out inverse))return;
            for(int i=0;i<Count-1;i++)previousDirections[i]=(Solved[i+1]-Solved[i]).Normalized;
            double radius=0;for(int i=0;i<vertices.Length;i++){int a=vertexSpans[i];double t=vertexBlends[i];Vec3 e=Animated[a+1]-Animated[a],d=vertices[i]-Animated[a]-e*t;radius=Math.Max(radius,(d-e.Normalized*Vec3.Dot(d,e.Normalized)).Length);}
            double bendRadius=Math.Max(radius*2.5,arc[Count-1]*.12);Solved[0]=Animated[0];
            for(int i=0;i<Count-1;i++)
            {
                Vec3 a=currentFrame.Multiply(inverse.Multiply(oldGuided[i+1]-oldGuided[i])).Normalized,b=currentFrame.Multiply(inverse.Multiply(oldSolved[i+1]-oldSolved[i])).Normalized;
                Vec3 direction=Rotation.Between(a,b).Apply(previousDirections[i]).Normalized;
                if(i==0)direction=(Animated[1]-Animated[0]).Normalized;
                else direction=Turn((Solved[i]-Solved[i-1]).Normalized,direction,CurvatureLimit(i,bendRadius));
                Solved[i+1]=Solved[i]+direction*lengths[i];
            }
            LimitBudget(vertices,maximum);BuildFrames();
        }
        void BuildFrames()
        {
            MaximumTurn=0;
            for(int i=0;i<Count;i++)
            {
                Vec3 a=Animated[Math.Min(Count-1,i+1)]-Animated[Math.Max(0,i-1)],b=Solved[Math.Min(Count-1,i+1)]-Solved[Math.Max(0,i-1)];frames[i]=Rotation.Between(a,b);
                if(i>0&&i<Count-1)MaximumTurn=Math.Max(MaximumTurn,Math.Acos(Math.Max(-1,Math.Min(1,Vec3.Dot((Solved[i]-Solved[i-1]).Normalized,(Solved[i+1]-Solved[i]).Normalized)))));
            }
            frames[0]=Rotation.Identity;
        }
        Vec3 Map(int index,Vec3 point)
        {
            double coordinate=sections.MaterialCoordinate(index);if(coordinate<=1e-8)return point;
            int a=vertexSpans[index];double t=vertexBlends[index];
            Vec3 c=Animated[a]*(1-t)+Animated[a+1]*t,q=Solved[a]*(1-t)+Solved[a+1]*t;
            return q+Rotation.SectionBlend(frames[a],frames[a+1],t).Apply(point-c);
        }
        double Measure(Vec3[] vertices)
        {BuildFrames();double max=0;for(int i=0;i<vertices.Length;i++)max=Math.Max(max,(Map(i,vertices[i])-vertices[i]).Length);return max;}
        void LimitBudget(Vec3[] vertices,double maximum)
        {
            if(!Vec3.Finite(maximum)||maximum<0)throw new ArgumentException("Invalid source displacement budget");
            BudgetBlend=1;double distance=Measure(vertices);if(distance<=maximum){MaximumDisplacement=distance;return;}
            // Reduce all angular corrections together, reintegrating exact link
            // lengths. Per-vertex clamping would crush the cross sections.
            for(int i=0;i<Count-1;i++)previousDirections[i]=(Solved[i+1]-Solved[i]).Normalized;
            double lo=0,hi=1;
            for(int attempt=0;attempt<12;attempt++)
            {
                double blend=(lo+hi)*.5;Integrate(blend);distance=Measure(vertices);if(distance>maximum)hi=blend;else lo=blend;
            }
            BudgetBlend=lo;Integrate(lo);MaximumDisplacement=Measure(vertices);
        }
        void Integrate(double blend)
        {Solved[0]=Animated[0];for(int i=0;i<Count-1;i++){Vec3 native=(Animated[i+1]-Animated[i]).Normalized,dir=Rotation.Blend(Rotation.Identity,Rotation.Between(native,previousDirections[i]),blend).Apply(native);Solved[i+1]=Solved[i]+dir*lengths[i];}}
        public void MapVertices(Vec3[] vertices,Vec3[] output)
        {if(vertices.Length!=output.Length)throw new ArgumentException("Source mapping dimensions differ");for(int i=0;i<vertices.Length;i++)output[i]=Map(i,vertices[i]);}
        public void MeasureRadii(Vec3[] corrected)
        {
            Array.Clear(SectionRadii,0,SectionRadii.Length);
            for(int i=0;i<corrected.Length;i++){if(sections.MaterialCoordinate(i)<=1e-8)continue;int a=vertexSpans[i];double t=vertexBlends[i];Vec3 e=Solved[a+1]-Solved[a],n=e.Normalized,d=corrected[i]-(Solved[a]+e*t);double radius=(d-n*Vec3.Dot(d,n)).Length;SectionRadii[a]=Math.Max(SectionRadii[a],radius);SectionRadii[a+1]=Math.Max(SectionRadii[a+1],radius);}
        }
    }
}
